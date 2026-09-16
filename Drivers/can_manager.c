#include "can_if.h"
#include "gps.h"
#include "rti.h"
#include "gsm_rx.h"
//#include "StateMachine.h"

/* ============================================================
 *  CONFIG
 * ============================================================ */
#define STARTUP_ACK_TIMEOUT_SEC   5U
#define DEV_COUNT 8U

/* ============================================================
 *  STARTUP ACK TRACKING
 * ============================================================ */
uint8_t  startup_in_progress;
uint32_t startup_start_time;

uint32_t ack_ok_mask;
uint32_t ack_miss_mask;
uint8_t mandatory_missing;

/* ============================================================
 *  HEARTBEAT TRACKING
 * ============================================================ */
uint8_t  hb_in_progress;
uint32_t hb_start_time;
uint32_t last_hb_tx_time;
uint32_t hb_ok_mask;
uint32_t hb_miss_mask;
uint8_t hb_mandatory_missing;

/* ============================================================
 *  DEVICE TABLE
 * ============================================================ */
typedef struct
{
    uint8_t     index;
    uint32_t    startup_ack_id;
    uint32_t    hb_ack_id;
    const char *name;
    uint8_t     mandatory;
    uint8_t     dev_index;
} can_device_t;

static const can_device_t can_devices[] =
{
    { 0, 0x81U, 0x0C1U, "RADIO1",       1U , 0},
    { 1, 0x82U, 0x0C2U, "RADIO2",       1U , 0},

    { 2, 0x83U, 0x0C3U, "EI1",          1U , 1},
    { 3, 0x84U, 0x0C4U, "EI2",          1U , 1},

    { 4, 0x85U, 0x0C5U, "DATA LOGGER",  1U , 2},

    { 5, 0x86U, 0x0C6U, "NMS",          1U , 3},
    { 6, 0x87U, 0x0C7U, "KMS",          1U , 3},

    { 7, 0x88U, 0x0C8U, "ADJ STCAS1",   1U , 4},
    { 8, 0x89U, 0x0C9U, "ADJ STCAS2",   1U , 4},

    { 9, 0x8AU, 0x0CAU, "INPUT CARD1",  1U , 5},
    {10, 0x8BU, 0x0CBU, "INPUT CARD2",  1U , 5},

    {11, 0x8CU, 0x0CCU, "RIU",          1U , 6},

    {12, 0x8DU, 0x0CDU, "SMOCIP",       1U , 7},

    {13, 0x8EU, 0x0CEU, "INPUT CARD3",  1U , 5},

};

uint8_t dev_count[DEV_COUNT];

// uint8_t device_count[10];
uint8_t system_faulty_flag;

#define NUM_CAN_DEVICES (sizeof(can_devices) / sizeof(can_devices[0]))
uint8_t  hb_ack_bitmap[NUM_CAN_DEVICES];
uint8_t  ack_bitmap[NUM_CAN_DEVICES];

volatile uint32_t gps1_firmware_checksum = 0U;
volatile uint32_t gps2_firmware_checksum = 0U;

volatile uint8_t gps1_checksum_valid = 0U;
volatile uint8_t gps2_checksum_valid = 0U;
/* ============================================================
 *  CARD FIRMWARE CHECKSUMS
 * ============================================================ */
uint32_t peripheral_firmware_checksum[NUM_CAN_DEVICES];

volatile uint32_t comm_card1_checksum = 0U;
volatile uint32_t comm_card2_checksum = 0U;
volatile uint32_t mvi_card_checksum = 0U;
volatile uint32_t input_card_checksum = 0U;
volatile uint32_t riu_checksum = 0U;

/* ============================================================
 *  CRC32 - IEEE 802.3 To combine checksums
 * ============================================================ */

static uint32_t CRC32_Calculate(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;
    uint8_t j;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint32_t)buf[i];

        for (j = 0U; j < 8U; j++)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFU;
}

static void checksum32_to_bytes(uint32_t checksum, uint8_t *buf)
{
    buf[0] = (uint8_t)(checksum >> 24);
    buf[1] = (uint8_t)(checksum >> 16);
    buf[2] = (uint8_t)(checksum >> 8);
    buf[3] = (uint8_t)checksum;
}

static void update_card_checksums(void)
{
    uint8_t card1_data[16];
    uint8_t card2_data[16];
    uint8_t mvi_data[16];
    uint8_t input_card_data[12]; //! Depends
//    uint8_t riu_data[4];

    uint32_t radio1_checksum;
    uint32_t radio2_checksum;
    uint32_t ei1_checksum;
    uint32_t ei2_checksum;
    uint32_t adjstcas1_checksum;
    uint32_t adjstcas2_checksum;
    uint32_t smocip_checksum;

    uint32_t datalogger_checksum;
    uint32_t nms_checksum;
    uint32_t kms_checksum;

    uint32_t input_card1_checksum;
    uint32_t input_card2_checksum;
    uint32_t input_card3_checksum;

//    uint32_t riu_checksum;

    /* ========================================================
     * PERIPHERAL CHECKSUMS
     * ======================================================== */

    radio1_checksum = peripheral_firmware_checksum[0];
    radio2_checksum = peripheral_firmware_checksum[1];

    ei1_checksum = peripheral_firmware_checksum[2];
    ei2_checksum = peripheral_firmware_checksum[3];

    datalogger_checksum = peripheral_firmware_checksum[4];

    nms_checksum = peripheral_firmware_checksum[5];
    kms_checksum = peripheral_firmware_checksum[6];

    adjstcas1_checksum = peripheral_firmware_checksum[7];
    adjstcas2_checksum = peripheral_firmware_checksum[8];

    input_card1_checksum = peripheral_firmware_checksum[9];
    input_card2_checksum = peripheral_firmware_checksum[10];
    input_card3_checksum = peripheral_firmware_checksum[13];

    riu_checksum = peripheral_firmware_checksum[11];

    smocip_checksum = peripheral_firmware_checksum[12];

    /* ========================================================
     * COMM CARD 1
     *
     * RADIO1 -> GPS1 -> EI1 -> ADJ STCAS1
     * ======================================================== */

    if (ack_bitmap[0] && gps1_checksum_valid && ack_bitmap[2] && ack_bitmap[7])
    {
        checksum32_to_bytes(radio1_checksum, card1_data + 0);

        checksum32_to_bytes(gps1_frame.firmware_checksum, card1_data + 4);

        checksum32_to_bytes(ei1_checksum, card1_data + 8);

        checksum32_to_bytes(adjstcas1_checksum, card1_data + 12);

        comm_card1_checksum = CRC32_Calculate(card1_data, 16U);
    }

    /* ========================================================
     * COMM CARD 2
     *
     * RADIO2 -> GPS2 -> EI2 -> ADJ STCAS2
     * ======================================================== */

    if (ack_bitmap[1] && gps2_checksum_valid && ack_bitmap[3] && ack_bitmap[8])
    {
        checksum32_to_bytes(radio2_checksum, card2_data + 0);

        checksum32_to_bytes(gps2_frame.firmware_checksum, card2_data + 4);

        checksum32_to_bytes(ei2_checksum, card2_data + 8);

        checksum32_to_bytes(adjstcas2_checksum, card2_data + 12);

        comm_card2_checksum = CRC32_Calculate(card2_data, 16U);
    }

    /* ========================================================
     * MVI CARD
     *
     * DATA LOGGER -> NMS -> KMS -> SMOCIP
     * ======================================================== */

    if (ack_bitmap[4] && ack_bitmap[5] && ack_bitmap[6] && ack_bitmap[12])
    {
        checksum32_to_bytes(datalogger_checksum, mvi_data + 0);

        checksum32_to_bytes(nms_checksum,        mvi_data + 4);

        checksum32_to_bytes(kms_checksum,        mvi_data + 8);

        checksum32_to_bytes(smocip_checksum,     mvi_data + 12);

        mvi_card_checksum = CRC32_Calculate(mvi_data, 16U);
    }

    /* ========================================================
     * INPUT CARD
     * INPUT CARD1 -> INPUT CARD2 -> INPUT CARD3
     * ======================================================== */

    if (ack_bitmap[9] && ack_bitmap[10] && ack_bitmap[13])
    {
        checksum32_to_bytes(input_card1_checksum, input_card_data + 0);

        checksum32_to_bytes(input_card2_checksum, input_card_data + 4);

        checksum32_to_bytes(input_card3_checksum, input_card_data + 8);

        input_card_checksum = CRC32_Calculate(input_card_data, 12U);
    }

    /* ========================================================
     * RIU
     * Only one RIU is currently present
     * ======================================================== */

    if (ack_bitmap[11])
    {
        riu_checksum = peripheral_firmware_checksum[11];
    }
}

/* ============================================================
 *  INIT
 * ============================================================ */
void can_manager_init(void)
{
    uint8_t i;
    for (i = 0U; i < NUM_CAN_DEVICES; i++)
    {
        ack_bitmap[i] = 0U;
        hb_ack_bitmap[i] = 0U;
        peripheral_firmware_checksum[i] = 0U;
    }
    send_cpu_startup_can();
    startup_start_time   = seconds_uptime;
    startup_in_progress  = 1U;
}

/* ============================================================
 *  STARTUP STATE
 * ============================================================ */
uint8_t can_startup_in_progress(void)
{
    return startup_in_progress;
}

void can_manager_handle_ack(uint32_t can_id, uint8_t *data)
{
    uint8_t i;
    for (i = 0U; i < NUM_CAN_DEVICES; i++)
    {
        if (can_devices[i].startup_ack_id == can_id)
        {
            ack_bitmap[can_devices[i].index] = 1U;
            dev_count[can_devices[i].dev_index]++;

            /* Extract peripheral firmware checksum
             *
             * Byte 0 = Startup ACK message type
             * Byte 1 = CRC [31:24]
             * Byte 2 = CRC [23:16]
             * Byte 3 = CRC [15:8]
             * Byte 4 = CRC [7:0]
             */
            peripheral_firmware_checksum[can_devices[i].index] = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[4];

            break;
        }
    }
}

/* ============================================================
 *  HEARTBEAT HANDLING
 * ============================================================ */
void can_manager_handle_hb_ack(uint32_t can_id)
{
    uint8_t i;
    for (i = 0U; i < NUM_CAN_DEVICES; i++)
    {
        if (can_devices[i].hb_ack_id == can_id)
        {
            hb_ack_bitmap[can_devices[i].index] = 1U;
            dev_count[can_devices[i].dev_index]++;
            break;
        }
    }
}

/* ============================================================
 *  1 SECOND SCHEDULER
 * ============================================================ */
void can_scheduler_1s_tick(void)
{
    uint8_t i;
    /* ---- Periodic CPU TIME ---- */
    send_cpu_time_can();
    /* ---- Heartbeat (every 5 seconds) ---- */
    if ((seconds_uptime - last_hb_tx_time) >= HEARTBEAT_PERIOD_SEC)
    {
        for(i = 0; i < DEV_COUNT; i++)
        {
            dev_count[i] = 0;
        }
        send_cpu_heartbeat_can();
        last_hb_tx_time = seconds_uptime;
        for (i = 0U; i < NUM_CAN_DEVICES; i++)
        {
            hb_ack_bitmap[i] = 0U;
        }
        hb_start_time  = seconds_uptime;
        hb_in_progress = 1U;
    }
    /* ---- GSM FSM ---- */
    gsm_start_poll_1s();
    gsm_manager_process();
}

/* ============================================================
 *  STARTUP + HEARTBEAT POLLING
 * ============================================================ */
int can_manager_poll_startup(void)
{
    uint8_t i;
    /* ---------------- STARTUP PHASE ---------------- */
    if (startup_in_progress)
    {
        if ((seconds_uptime - startup_start_time) >= STARTUP_ACK_TIMEOUT_SEC)
        {
            startup_in_progress = 0U;
            ack_ok_mask = 0U;
            ack_miss_mask = 0U;
            mandatory_missing = 0U;
            for (i = 0U; i < NUM_CAN_DEVICES; i++)
            {
                uint8_t idx = can_devices[i].index;
                if (ack_bitmap[idx])
                {
                    ack_ok_mask |= (1U << i);
                }
                else
                {
                    ack_miss_mask |= (1U << i);
                    if (can_devices[i].mandatory)
                    {
                        mandatory_missing = 1U;
                    }
                }
            }
            /* Calculate combined Card checksums */
            update_card_checksums();
            for (i = 0; i < DEV_COUNT; i++)
            {
                if(i == 3)
                {
                    dev_count[i] = 0;
                    continue;
                }

                if(dev_count[i] == 0)
                {
                    system_faulty_flag = 1;
                }

                dev_count[i] = 0;
            }
//            if(system_faulty_flag == 0)
//            {
//                input_write.raw_flags[0] &= ~(1U << 4);
//                input_write.raw_flags[0] |= (1U << 5);
//            }
//            else
//            {
//                input_write.raw_flags[0] |= (1U << 4);
//                input_write.raw_flags[0] &= ~(1U << 5);
//            }
//            system_faulty_flag = 0;
        }
        return 0;   /* startup not complete */
    }

    /* ---------------- HEARTBEAT PHASE ---------------- */
    if (hb_in_progress)
    {
        if ((seconds_uptime - hb_start_time) >= HEARTBEAT_ACK_TIMEOUT)
        {
            hb_in_progress = 0U;
            hb_ok_mask = 0U;
            hb_miss_mask = 0U;
            hb_mandatory_missing = 0U;
            for (i = 0U; i < NUM_CAN_DEVICES; i++)
            {
                uint8_t idx = can_devices[i].index;
                if (hb_ack_bitmap[idx])
                {
                    hb_ok_mask |= (1U << i);
                }
                else
                {
                    hb_miss_mask |= (1U << i);
                    if (can_devices[i].mandatory)
                    {
                        hb_mandatory_missing = 1U;
                    }
                }
            }
            for (i = 0; i < DEV_COUNT; i++)
            {
                if(i == 3)
                {
                    dev_count[i] = 0;
                    continue;
                }
                if(dev_count[i] == 0)
                {
                    system_faulty_flag = 1;
                }
                dev_count[i] = 0;
            }
//            if(system_faulty_flag == 0)
//            {
//                input_write.raw_flags[0] &= ~(1U << 4);
//                input_write.raw_flags[0] |= (1U << 5);
//            }
//            else
//            {
//                input_write.raw_flags[0] |= (1U << 4);
//                input_write.raw_flags[0] &= ~(1U << 5);
//            }
//            system_faulty_flag = 0;
        }
    }

    return 1;   /* normal operation */
}
