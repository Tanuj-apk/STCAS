#include "can_if.h"
#include "gps.h"
#include "rti.h"
#include "kms.h"
#include "input_card.h"

/* ============================================================
 *  CONFIG
 * ============================================================ */
#define STARTUP_ACK_TIMEOUT_SEC   5U

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
    uint32_t    startup_ack_id;
    uint32_t    hb_ack_id;
    const char *name;
    uint8_t     mandatory;
    uint8_t     input_card_number;
} can_device_t;

typedef enum
{
    CAN_DEV_RADIO1 = 0,
    CAN_DEV_RADIO2,
    CAN_DEV_EI1,
    CAN_DEV_EI2,
    CAN_DEV_DATA_LOGGER,
    CAN_DEV_NMS,
    CAN_DEV_KMS,
    CAN_DEV_ADJ_STCAS1,
    CAN_DEV_ADJ_STCAS2,
    CAN_DEV_INPUT_CARD1,
    CAN_DEV_INPUT_CARD2,
    CAN_DEV_RIU,
    CAN_DEV_SMOCIP,
    CAN_DEV_INPUT_CARD3,
    CAN_DEV_INPUT_CARD4,
    CAN_DEV_INPUT_CARD5,
    CAN_DEV_INPUT_CARD6,
    CAN_DEV_INPUT_CARD7,
    CAN_DEV_INPUT_CARD8,
    CAN_DEV_INPUT_CARD9,
    CAN_DEV_INPUT_CARD10,
    CAN_DEV_INPUT_CARD11,
    CAN_DEV_INPUT_CARD12,
    CAN_DEV_INPUT_CARD13,
    CAN_DEV_INPUT_CARD14,
    CAN_DEV_INPUT_CARD15,
    CAN_DEV_INPUT_CARD16,
    CAN_DEVICE_COUNT
} can_device_index_t;

static const can_device_t can_devices[CAN_DEVICE_COUNT] =
{
    {0x84U, 0x0C1U, "RADIO1",      1U,  0U},
    {0x85U, 0x0C2U, "RADIO2",      1U,  0U},

    {0x86U, 0x0C3U, "EI1",         1U,  0U},
    {0x87U, 0x0C4U, "EI2",         1U,  0U},

    {0x88U, 0x0C5U, "DATA LOGGER", 1U,  0U},

    {0x89U, 0x0C6U, "NMS",         1U,  0U},
    {0x8AU, 0x0C7U, "KMS",         1U,  0U},

    {0x8BU, 0x0C8U, "ADJ STCAS1",  1U,  0U},
    {0x8CU, 0x0C9U, "ADJ STCAS2",  1U,  0U},

    {0x8DU, 0x0CAU, "INPUT CARD1", 1U,  1U},
    {0x8EU, 0x0CBU, "INPUT CARD2", 1U,  2U},

    {0x8FU, 0x0CCU, "RIU",         1U,  0U},

    {0x90U, 0x0CDU, "SMOCIP",      1U,  0U},

    {0x91U, 0x0CEU, "INPUT CARD3", 1U,  3U},
    {0x92U, 0x0CFU, "INPUT CARD4", 1U,  4U},
    {0x93U, 0x0D0U, "INPUT CARD5", 1U,  5U},
    {0x94U, 0x0D1U, "INPUT CARD6", 1U,  6U},
    {0x95U, 0x0D2U, "INPUT CARD7", 1U,  7U},
    {0x96U, 0x0D3U, "INPUT CARD8", 1U,  8U},
    {0x97U, 0x0D4U, "INPUT CARD9", 1U,  9U},
    {0x98U, 0x0D5U, "INPUT CARD10", 1U, 10U},
    {0x99U, 0x0D6U, "INPUT CARD11", 1U, 11U},
    {0x9AU, 0x0D7U, "INPUT CARD12", 1U, 12U},
    {0x9BU, 0x0D8U, "INPUT CARD13", 1U, 13U},
    {0x9CU, 0x0D9U, "INPUT CARD14", 1U, 14U},
    {0x9DU, 0x0DAU, "INPUT CARD15", 1U, 15U},
    {0x9EU, 0x0DBU, "INPUT CARD16", 1U, 16U}
};

static const uint8_t input_card_device_indices[INPUT_CARD_MAX_COUNT] =
{
    CAN_DEV_INPUT_CARD1,  CAN_DEV_INPUT_CARD2,
    CAN_DEV_INPUT_CARD3,  CAN_DEV_INPUT_CARD4,
    CAN_DEV_INPUT_CARD5,  CAN_DEV_INPUT_CARD6,
    CAN_DEV_INPUT_CARD7,  CAN_DEV_INPUT_CARD8,
    CAN_DEV_INPUT_CARD9,  CAN_DEV_INPUT_CARD10,
    CAN_DEV_INPUT_CARD11, CAN_DEV_INPUT_CARD12,
    CAN_DEV_INPUT_CARD13, CAN_DEV_INPUT_CARD14,
    CAN_DEV_INPUT_CARD15, CAN_DEV_INPUT_CARD16
};

uint8_t system_faulty_flag;

#define NUM_CAN_DEVICES ((uint8_t)CAN_DEVICE_COUNT)
uint8_t  hb_ack_bitmap[NUM_CAN_DEVICES];
uint8_t  ack_bitmap[NUM_CAN_DEVICES];

static uint8_t can_device_is_active(uint8_t device_index)
{
    uint8_t input_card_number;

    input_card_number = can_devices[device_index].input_card_number;
    return (uint8_t)((input_card_number == 0U) ||
                     (input_card_number <= STCAS_INPUT_CARD_COUNT));
}

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
    uint8_t input_checksum_data[INPUT_CARD_MAX_COUNT * 4U];
    uint8_t input_cards_ready;
    uint8_t input_card_index;
    uint8_t device_index;

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

    /* ========================================================
     * PERIPHERAL CHECKSUMS
     * ======================================================== */

    radio1_checksum = peripheral_firmware_checksum[CAN_DEV_RADIO1];
    radio2_checksum = peripheral_firmware_checksum[CAN_DEV_RADIO2];

    ei1_checksum = peripheral_firmware_checksum[CAN_DEV_EI1];
    ei2_checksum = peripheral_firmware_checksum[CAN_DEV_EI2];

    datalogger_checksum =
        peripheral_firmware_checksum[CAN_DEV_DATA_LOGGER];

    nms_checksum = peripheral_firmware_checksum[CAN_DEV_NMS];
    kms_checksum = peripheral_firmware_checksum[CAN_DEV_KMS];

    adjstcas1_checksum =
        peripheral_firmware_checksum[CAN_DEV_ADJ_STCAS1];
    adjstcas2_checksum =
        peripheral_firmware_checksum[CAN_DEV_ADJ_STCAS2];

    smocip_checksum = peripheral_firmware_checksum[CAN_DEV_SMOCIP];

    comm_card1_checksum = 0U;
    comm_card2_checksum = 0U;
    mvi_card_checksum = 0U;
    input_card_checksum = 0U;
    riu_checksum = 0U;

    /* ========================================================
     * COMM CARD 1
     *
     * RADIO1 -> GPS1 -> EI1 -> ADJ STCAS1
     * ======================================================== */

    if (ack_bitmap[CAN_DEV_RADIO1] && gps1_checksum_valid &&
        ack_bitmap[CAN_DEV_EI1] && ack_bitmap[CAN_DEV_ADJ_STCAS1])
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

    if (ack_bitmap[CAN_DEV_RADIO2] && gps2_checksum_valid &&
        ack_bitmap[CAN_DEV_EI2] && ack_bitmap[CAN_DEV_ADJ_STCAS2])
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

    if (ack_bitmap[CAN_DEV_DATA_LOGGER] && ack_bitmap[CAN_DEV_NMS] &&
        ack_bitmap[CAN_DEV_KMS] && ack_bitmap[CAN_DEV_SMOCIP])
    {
        checksum32_to_bytes(datalogger_checksum, mvi_data + 0);

        checksum32_to_bytes(nms_checksum,        mvi_data + 4);

        checksum32_to_bytes(kms_checksum,        mvi_data + 8);

        checksum32_to_bytes(smocip_checksum,     mvi_data + 12);

        mvi_card_checksum = CRC32_Calculate(mvi_data, 16U);
    }

    /* ========================================================
     * INPUT CARDS
     * CRC covers all cards configured for this station, in
     * ascending card-number order.
     * ======================================================== */
    input_cards_ready = 1U;
    for (input_card_index = 0U;
         input_card_index < STCAS_INPUT_CARD_COUNT;
         input_card_index++)
    {
        device_index = input_card_device_indices[input_card_index];
        if (ack_bitmap[device_index] == 0U)
        {
            input_cards_ready = 0U;
            break;
        }

        checksum32_to_bytes(peripheral_firmware_checksum[device_index],
                            input_checksum_data +
                            ((uint16_t)input_card_index * 4U));
    }

    if (input_cards_ready != 0U)
    {
        input_card_checksum =
            CRC32_Calculate(input_checksum_data,
                            (uint32_t)STCAS_INPUT_CARD_COUNT * 4U);
    }

    /* ========================================================
     * RIU
     * Only one RIU is currently present
     * ======================================================== */

    if (ack_bitmap[CAN_DEV_RIU])
    {
        riu_checksum = peripheral_firmware_checksum[CAN_DEV_RIU];
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
    ack_ok_mask = 0U;
    ack_miss_mask = 0U;
    hb_ok_mask = 0U;
    hb_miss_mask = 0U;
    mandatory_missing = 0U;
    hb_mandatory_missing = 0U;
    system_faulty_flag = 0U;
    hb_in_progress = 0U;
    last_hb_tx_time = seconds_uptime;
    startup_start_time   = seconds_uptime;
    startup_in_progress  = 1U;
    send_cpu_startup_can();
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
        if ((can_device_is_active(i) != 0U) &&
            (can_devices[i].startup_ack_id == can_id))
        {
            ack_bitmap[i] = 1U;

            /* Extract peripheral firmware checksum
             *
             * Byte 0 = Startup ACK message type
             * Byte 1 = CRC [31:24]
             * Byte 2 = CRC [23:16]
             * Byte 3 = CRC [15:8]
             * Byte 4 = CRC [7:0]
             */
            peripheral_firmware_checksum[i] =
                ((uint32_t)data[1] << 24) |
                ((uint32_t)data[2] << 16) |
                ((uint32_t)data[3] << 8) |
                (uint32_t)data[4];

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
        if ((can_device_is_active(i) != 0U) &&
            (can_devices[i].hb_ack_id == can_id))
        {
            hb_ack_bitmap[i] = 1U;
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
        for (i = 0U; i < NUM_CAN_DEVICES; i++)
        {
            hb_ack_bitmap[i] = 0U;
        }
        hb_start_time  = seconds_uptime;
        hb_in_progress = 1U;
        last_hb_tx_time = seconds_uptime;
        send_cpu_heartbeat_can();
    }
    /* ---- KMS authentication transaction ---- */
    kms_process_1s();
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
                if (can_device_is_active(i) == 0U)
                {
                    continue;
                }

                if (ack_bitmap[i])
                {
                    ack_ok_mask |= ((uint32_t)1U << i);
                }
                else
                {
                    ack_miss_mask |= ((uint32_t)1U << i);
                    if (can_devices[i].mandatory)
                    {
                        mandatory_missing = 1U;
                    }
                }
            }
            /* Calculate combined Card checksums */
            update_card_checksums();
            system_faulty_flag = mandatory_missing;
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
                if (can_device_is_active(i) == 0U)
                {
                    continue;
                }

                if (hb_ack_bitmap[i])
                {
                    hb_ok_mask |= ((uint32_t)1U << i);
                }
                else
                {
                    hb_miss_mask |= ((uint32_t)1U << i);
                    if (can_devices[i].mandatory)
                    {
                        hb_mandatory_missing = 1U;
                    }
                }
            }
            system_faulty_flag = hb_mandatory_missing;
        }
    }

    return 1;   /* normal operation */
}
