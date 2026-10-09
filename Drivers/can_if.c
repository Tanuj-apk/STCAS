#include "can_if.h"
#include "can.h"
#include "gps.h"
#include "rfid_rx.h"
#include "kms.h"
#include "sci.h"
#include "radio.h"
#include <stdio.h>
#include "SMOCIP.h"
#include "NMS.h"
#include "datalogging.h"

/* ============================================================
 *  CPU STARTUP PAYLOAD CONSTANTS
 * ============================================================ */
#define CPU_STARTUP_MSG_TYPE  0x01U
#define CPU_HW_VERSION        0x02U
#define CPU_SW_MAJOR          0x01U
#define CPU_SW_MINOR          0x04U
#define STARTUP_MODE_NORMAL   0x00U

/* ============================================================
 *  RX QUEUE
 * ============================================================ */
volatile can_rx_queue_entry_t can_rx_queue[CAN_RX_QUEUE_SIZE];
volatile uint8_t can_rx_head = 0U;
volatile uint8_t can_rx_tail = 0U;
volatile uint32_t can_rx_queue_overflow = 0U;
volatile uint32_t can_rx_message_lost = 0U;
volatile uint32_t can_rx_read_failures = 0U;

volatile uint32_t can1_tx_reject_count = 0U;
volatile uint32_t can2_tx_reject_count = 0U;
volatile uint32_t can_dual_tx_reject_count = 0U;

/* ============================================================
 *  HELPERS
 * ============================================================ */
static inline uint16_t unpack_u16_le(const uint8_t *b)
{
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static inline void pack_u32_le(uint8_t *buf, uint32_t v)
{
    buf[0] = (uint8_t)(v & 0xFFu);
    buf[1] = (uint8_t)((v >> 8) & 0xFFu);
    buf[2] = (uint8_t)((v >> 16) & 0xFFu);
    buf[3] = (uint8_t)((v >> 24) & 0xFFu);
}

uint8_t can_transmit_redundant(uint32_t message_box, const uint8_t data[8])
{
    uint8_t accepted = 0U;

    if (canTransmit(canREG1, message_box, data) != 0U)
    {
        accepted |= CAN_TX_ACCEPTED_CAN1;
    }
    else
    {
        can1_tx_reject_count++;
    }

    if (canTransmit(canREG2, message_box, data) != 0U)
    {
        accepted |= CAN_TX_ACCEPTED_CAN2;
    }
    else
    {
        can2_tx_reject_count++;
    }

    if (accepted == 0U)
    {
        can_dual_tx_reject_count++;
    }

    return accepted;
}

uint32_t can_get_local_tx_id(uint32_t base_can_id)
{
    uint32_t device_offset = 0U;

    if ((g_device_id >= CPU_DEVICE_ID_MIN) &&
        (g_device_id <= CPU_DEVICE_ID_MAX))
    {
        device_offset = (uint32_t)(g_device_id - CPU_DEVICE_ID_MIN);
    }

    return base_can_id + device_offset;
}

/* ============================================================
 *  CPU-SPECIFIC CAN ARBITRATION IDS
 * ============================================================ */
void can_configure_device_ids(void)
{
    uint32_t device_offset;

    if ((g_device_id < CPU_DEVICE_ID_MIN) ||
        (g_device_id > CPU_DEVICE_ID_MAX))
    {
        g_device_id = CPU_DEVICE_ID_MIN;
    }

    device_offset = (uint32_t)(g_device_id - CPU_DEVICE_ID_MIN);

    canUpdateID(canREG1, canMESSAGE_BOX1,
                0x60000000U | (CPU_TIME_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX1,
                0x60000000U | (CPU_TIME_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX2,
                0x60000000U | (CPU_STARTUP_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX2,
                0x60000000U | (CPU_STARTUP_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX4,
                0x60000000U | (CPU_HEARTBEAT_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX4,
                0x60000000U | (CPU_HEARTBEAT_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX9,
                0x60000000U | (CPU_UNIVERSAL_ACK_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX9,
                0x60000000U | (CPU_UNIVERSAL_ACK_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX7,
                0x60000000U | (KMS_QUERY_TX_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX7,
                0x60000000U | (KMS_QUERY_TX_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX12,
                0x60000000U | (RADIO1_TX_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX12,
                0x60000000U | (RADIO1_TX_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX13,
                0x60000000U | (RADIO2_TX_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX13,
                0x60000000U | (RADIO2_TX_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX16,
                0x60000000U | (DATA_LOGGER_TX_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX16,
                0x60000000U | (DATA_LOGGER_TX_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX20,
                0x60000000U | (NMS_TX_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX20,
                0x60000000U | (NMS_TX_CAN_ID + device_offset));

    canUpdateID(canREG1, canMESSAGE_BOX21,
                0x60000000U | (SMOCIP_TX_CAN_ID + device_offset));
    canUpdateID(canREG2, canMESSAGE_BOX21,
                0x60000000U | (SMOCIP_TX_CAN_ID + device_offset));
}

/* ============================================================
 *  CPU TIME CAN TX
 * ============================================================ */
void send_cpu_time_can(void)
{
    uint8_t tx_buf[8] = {0};
    pack_u32_le(tx_buf, cpu_time_sec);
    uint8_t flags = 0;

    if (cpu_time_valid)    flags |= (1u << 0);
    if (fallback_active)   flags |= (1u << 1);
    if (current_sel == GPS_SEL_GPS1) flags |= (1u << 2);
    if (current_sel == GPS_SEL_GPS2) flags |= (1u << 3);

    tx_buf[4] = flags;

    (void)can_transmit_redundant(canMESSAGE_BOX1, tx_buf);

}

/* ============================================================
 *  CPU STARTUP CAN TX
 * ============================================================ */
void send_cpu_startup_can(void)
{
    uint8_t tx_buf[8];
    tx_buf[0] = CPU_STARTUP_MSG_TYPE;
    tx_buf[1] = g_device_id;
    tx_buf[2] = CPU_HW_VERSION;
    tx_buf[3] = CPU_SW_MAJOR;
    tx_buf[4] = CPU_SW_MINOR;
    tx_buf[5] = STARTUP_MODE_NORMAL;
    tx_buf[6] = 0x00U;
    tx_buf[7] = 0x00U;

    (void)can_transmit_redundant(canMESSAGE_BOX2, tx_buf);
}

/* ============================================================
 *  CPU UNIVERSAL ACK TX
 * ============================================================ */
void send_cpu_universal_ack(uint16_t peripheral_can_id, uint8_t action_type, uint8_t ack_status)
{
    uint8_t tx_buf[8] = {0};

    /* Byte 0-1 : PERIPHERAL_CAN_ID */
    tx_buf[0] = (uint8_t)((peripheral_can_id >> 8) & 0xFFU);  // MSB
    tx_buf[1] = (uint8_t)(peripheral_can_id & 0xFFU);         // LSB

    /* Byte 2 : ACTION_TYPE */
    tx_buf[2] = action_type;

    /* Byte 3 : ACK_STATUS */
    tx_buf[3] = ack_status;

    /* Bytes 4-7 : RESERVED = 0 */

    (void)can_transmit_redundant(canMESSAGE_BOX9, tx_buf);
}

/* ============================================================
 *  RX ROUTING ENTRY POINT
 * ============================================================ */
void can_if_process_rx(uint32_t can_id, uint8_t *data, can_source_t can_source)
{
    /* ---------- STARTUP ACK ---------- */
    if (can_startup_in_progress())
    {
        if ((can_id >= PERIPH_ACK_BASE_ID) && (can_id <= PERIPH_ACK_MAX_ID) && (data[0] == MSG_TYPE_PERIPH_ACK))
        {
            can_manager_handle_ack(can_id, data);
            return;
        }
    }
    /* ---------- HEARTBEAT ACK ---------- */
    if ((can_id >= PERIPH_HB_ACK_BASE_ID) && (can_id <= PERIPH_HB_ACK_MAX_ID))
    {
        if (unpack_u16_le(data) == MSG_TYPE_HB_ACK)
        {
            can_manager_handle_hb_ack(can_id);
            return;
        }
    }
    /* ---------- RFID RX ---------- */
    if ((can_id == 0x120U) || (can_id == 0x121U))
    {
        rfid_rx_handle(can_id, data);
        return;
    }
    /* ---------- KMS RX ---------- */
    if ((can_id & KMS_RX_CAN_ID_MASK) == KMS_RX_CAN_ID_BASE)
    {
        kms_rx_handle(can_id, data);
        return;
    }
    /* ---------- INPUT CARD RX ---------- */
    //! 0x150-0x15F - Field Input Card status range
    if ((can_id & INPUT_CARD_RX_MASK) == INPUT_CARD_RX_ID)
    {
        input_card_rx_handler(can_id, data, can_source);
        return;
    }
    /* ---- RADIO AAP RX ---- */
    //! 0x148 - Radio TIVA 1
    //! 0x149 - Radio TIVA 2
    if ((can_id & RADIO_AAP_RX_MASK) == RADIO_AAP_RX_BASE_ID)
    {
        uint16_t ack_can_id;

        ack_can_id = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

        if (((can_id == RADIO1_ACK_CAN_ID) &&
             (ack_can_id ==
              (uint16_t)can_get_local_tx_id(RADIO1_TX_CAN_ID))) ||
            ((can_id == RADIO2_ACK_CAN_ID) &&
             (ack_can_id ==
              (uint16_t)can_get_local_tx_id(RADIO2_TX_CAN_ID))))
        {
            radio_ack_rx_handle(can_id, data);
        }
        else
        {
            radio_rx_handle(can_id, data);
        }
        return;
    }
    //! 0x234 - SMOCIP
    if (can_id == SMOCIP_RX_ID)
    {
        uint16_t ack_can_id;

        ack_can_id = ((uint16_t)data[0] << 8) |
                     (uint16_t)data[1];

        if ((can_id == SMOCIP_ACK_CAN_ID) &&
            (ack_can_id ==
             (uint16_t)can_get_local_tx_id(SMOCIP_TX_CAN_ID)) &&
            (data[2] == ACK_ACTION_SMOCIP))
        {
            smocip_ack_rx_handle(can_id, data);
        }
        else
        {
            smocip_rx_handle(data, can_source);
        }

        return;
    }
    /* ---------- DATALOGGER ACK RX ---------- */
    //! 0x214
    if (can_id == DATA_LOGGER_ACK_CAN_ID)
    {
        data_logger_ack_rx_handle(can_id, data);
        return;
    }
    /* ---------- NMS ACK RX ---------- */
    //! 0x224
    if (can_id == NMS_ACK_CAN_ID)
    {
        nms_ack_rx_handle(can_id, data);
        return;
    }
}

/* ============================================================
 *  CPU HEARTBEAT CAN TX
 * ============================================================ */
void send_cpu_heartbeat_can(void)
{
    uint8_t tx_buf[8] = {0};

    tx_buf[0] = (uint8_t)(MSG_TYPE_CPU_HEARTBEAT & 0xFFU);
    tx_buf[1] = (uint8_t)((MSG_TYPE_CPU_HEARTBEAT >> 8) & 0xFFU);
    tx_buf[2] = g_device_id;
    tx_buf[3] = 0x01U;   /* CPU_STATE = RUN */

    (void)can_transmit_redundant(canMESSAGE_BOX4, tx_buf);
}

static inline void can_rx_queue_push_isr(canBASE_t *node, uint32_t messageBox)
{
    can_source_t can_source;
    uint8_t next_head;
    uint32_t read_status;

    if (node == canREG1)
    {
        can_source = CAN_SOURCE_1;
    }
    else if (node == canREG2)
    {
        can_source = CAN_SOURCE_2;
    }
    else
    {
        return;
    }

    next_head = (uint8_t)((can_rx_head + 1U) % CAN_RX_QUEUE_SIZE);

    if (next_head == can_rx_tail)
    {
        uint8_t discarded_data[8];

        /* Clear NewDat even when the software queue is full. */
        read_status = canGetData(node, messageBox, discarded_data);
        can_rx_queue_overflow++;

        if (read_status == 3U)
        {
            can_rx_message_lost++;
        }
        else if (read_status == 0U)
        {
            can_rx_read_failures++;
        }
        return;
    }

    /* Read the identifier before canGetData clears NewDat. */
    can_rx_queue[can_rx_head].id = canGetID(node, messageBox);
    can_rx_queue[can_rx_head].can_source = can_source;
    read_status = canGetData(node, messageBox,
                             (uint8_t *)can_rx_queue[can_rx_head].data);

    if (read_status == 0U)
    {
        can_rx_read_failures++;
        return;
    }

    if (read_status == 3U)
    {
        can_rx_message_lost++;
    }

    /* Publish the entry only after all fields have been written. */
    can_rx_head = next_head;
}

/* ============================================================
 *  CAN RX ISR CALLBACK
 * ============================================================ */
void canMessageNotification(canBASE_t *node, uint32_t messageBox)
{
    switch (messageBox)
    {
    case canMESSAGE_BOX3:
    case canMESSAGE_BOX5:
    case canMESSAGE_BOX6:
    case canMESSAGE_BOX8:
    case canMESSAGE_BOX10:
    case canMESSAGE_BOX14:
    case canMESSAGE_BOX22:
    case canMESSAGE_BOX24:
    case canMESSAGE_BOX25:
        can_rx_queue_push_isr(node, messageBox);
        break;

    default:
        break;
    }
}

static uint8_t can_rx_queue_pop(uint32_t *id, uint8_t *data,
                                can_source_t *can_source)
{
    uint8_t current_tail;
    uint8_t i;

    if (can_rx_tail == can_rx_head)
    {
        return 0U;
    }

    current_tail = can_rx_tail;
    *id = can_rx_queue[current_tail].id;
    *can_source = can_rx_queue[current_tail].can_source;

    for (i = 0U; i < 8U; i++)
    {
        data[i] = can_rx_queue[current_tail].data[i];
    }

    can_rx_tail = (uint8_t)((current_tail + 1U) % CAN_RX_QUEUE_SIZE);
    return 1U;
}

void can_if_process_rx_pending(void)
{
    uint32_t id;
    can_source_t can_source;
    uint8_t data[8];
    uint8_t count = 0U;

    while ((count < CAN_RX_PROCESS_LIMIT) &&
           can_rx_queue_pop(&id, data, &can_source))
    {
        can_if_process_rx(id, data, can_source);
        count++;
    }
}

/* ============================================================
 *  DEBUG
 * ============================================================ */
void debug_print_can_payload(void)
{
    uint8_t dbg[128];

    uint32_t len = sprintf((char *)dbg,
        "CAN TX -> time=%lu valid=%u fb=%u sel=%u\r\n",
        (unsigned long)cpu_time_sec,
        (unsigned)cpu_time_valid,
        (unsigned)fallback_active,
        (unsigned)current_sel);

    sciSend(GPS2_SCI, len, dbg);
}
