#include "can_if.h"
#include "gps.h"
#include "rti.h"
#include "gsm_rx.h"
//#include "StateMachine.h"

/* ============================================================
 *  CONFIG
 * ============================================================ */
#define STARTUP_ACK_TIMEOUT_SEC   5U
#define DEV_COUNT 9U

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

    { 2, 0x83U, 0x0C3U, "RFID1",        1U , 1},
    { 3, 0x84U, 0x0C4U, "RFID2",        1U , 1},

    { 4, 0x85U, 0x0C5U, "DATA LOGGER",  1U , 2},

    { 5, 0x86U, 0x0C6U, "GSM1",         1U , 3},
    { 6, 0x87U, 0x0C7U, "GSM2",         1U , 3},

    { 7, 0x88U, 0x0C8U, "BIU1",         1U , 4},
    { 8, 0x89U, 0x0C9U, "BIU2",         1U , 4},

    { 9, 0x8AU, 0x0CAU, "INPUT CARD1",  1U , 5},
    {10, 0x8BU, 0x0CBU, "INPUT CARD2",  1U , 5},

    {11, 0x8CU, 0x0CCU, "OUTPUT CARD1", 1U , 6},
    {12, 0x8DU, 0x0CDU, "OUTPUT CARD2", 1U , 6},

    {13, 0x8EU, 0x0CEU, "COUNTER CARD", 1U , 7},

    {14, 0x8FU, 0x0CFU, "DMI1",         1U , 8},
    {15, 0x90U, 0x0D0U, "DMI2",         1U , 8},
};

uint8_t dev_count[DEV_COUNT];

// uint8_t device_count[10];
uint8_t system_faulty_flag;

#define NUM_CAN_DEVICES (sizeof(can_devices) / sizeof(can_devices[0]))
uint8_t  hb_ack_bitmap[NUM_CAN_DEVICES];
uint8_t  ack_bitmap[NUM_CAN_DEVICES];

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
