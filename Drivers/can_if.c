#include "can_if.h"
#include "can.h"
#include "gps.h"
#include "rfid_rx.h"
#include "gsm_rx.h"
#include "sci.h"
#include "radio.h"
#include <stdio.h>
#include "SMOCIP.h"
//#include "StateMachine.h"
//#include "dmi_can.h"

/* ============================================================
 *  CPU STARTUP PAYLOAD CONSTANTS
 * ============================================================ */
#define CPU_STARTUP_MSG_TYPE  0x01U
#define CPU_ID_VALUE          0x01U
#define CPU_HW_VERSION        0x02U
#define CPU_SW_MAJOR          0x01U
#define CPU_SW_MINOR          0x04U
#define STARTUP_MODE_NORMAL   0x00U

/* ============================================================
 *  RX BUFFERS
 * ============================================================ */
static uint8_t rx_data_startup[8];
static uint8_t rx_data_heartbeat[8];
static uint8_t rx_data_rfid[8];
static uint8_t rx_data_gsm[8];
static uint8_t rx_data_input_card[8];
static uint8_t rx_data_radio[8];
static uint32_t rx_id;
static uint8_t rx_dmi_pilot[8];
static uint8_t rx_data_smocip[8];

/* ============================================================
 *  TX BUFFERS
 * ============================================================ */
uint8_t tx_data_log[18] = {0x56, 0x7F, 0x5A, 0x00, 0xFF, 0xFF, 0xF3, 0x27, 0xFF, 0xFF, 0xF2, 0xAA, 0xAA, 0xAD, 0xD8, 0x00, 0x00, 0x00};

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

    canTransmit(canREG1, canMESSAGE_BOX1, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX1, tx_buf);

}

/* ============================================================
 *  CPU STARTUP CAN TX
 * ============================================================ */
void send_cpu_startup_can(void)
{
    uint8_t tx_buf[8];
    tx_buf[0] = CPU_STARTUP_MSG_TYPE;
    tx_buf[1] = CPU_ID_VALUE;
    tx_buf[2] = CPU_HW_VERSION;
    tx_buf[3] = CPU_SW_MAJOR;
    tx_buf[4] = CPU_SW_MINOR;
    tx_buf[5] = STARTUP_MODE_NORMAL;
    tx_buf[6] = 0x00U;
    tx_buf[7] = 0x00U;

    canTransmit(canREG1, canMESSAGE_BOX2, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX2, tx_buf);
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

    canTransmit(canREG1, canMESSAGE_BOX9, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX9, tx_buf);
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
    /* ---------- CAN RX PILOT DATA ---------- */
    if ((can_id == 0x182U) || (can_id == 0x183U))
    {
//        dmi_rx_pilot_handle(can_id, data);
        return;
    }
    /* ---------- GSM RX ---------- */
    if ((can_id & GSM_AUTH_KEY_RX_MASK) == GSM_AUTH_KEY_RX_ID)
    {
        gsm_rx_handle(can_id, data);
        return;
    }
    /* ---------- INPUT CARD RX ---------- */
    //! 0x150,151,152 - Input Cards
    if ((can_id & INPUT_CARD_RX_MASK) == INPUT_CARD_RX_ID)
    {
        input_card_rx_handler(can_id, data, can_source);
        return;
    }
    /* ---- RADIO AAP RX ---- */
    //! 0X142- RADIO TIVA 1 
    //! 0X143- RADIO TIVA 2
    if ((can_id & RADIO_AAP_RX_MASK) == RADIO_AAP_RX_BASE_ID)
    {
        radio_rx_handle(can_id, data);
        return;
    }
    //! 0x221 - SMOCIP
    if (can_id == SMOCIP_RX_ID) 
    {
        smocip_rx_handle(data, can_source);
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
    tx_buf[2] = CPU_ID_VALUE;
    tx_buf[3] = 0x01U;   /* CPU_STATE = RUN */

    canTransmit(canREG1, canMESSAGE_BOX4, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX4, tx_buf);
}

/* ============================================================
 *  GSM START REQUEST TX
 * ============================================================ */
void send_gsm_start_req(uint8_t gsm_id, uint8_t action)
{
    uint8_t tx_buf[8] = {0};

    tx_buf[0] = MSG_TYPE_GSM_START_REQ;
    tx_buf[1] = CPU_ID_VALUE;
    tx_buf[2] = gsm_id;
    tx_buf[3] = action;

    canTransmit(canREG1, canMESSAGE_BOX7, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX7, tx_buf);
}

void send_Counter_Change_req(uint8_t flagSet)
{
    uint8_t tx_buf[8] = {0};

    tx_buf[0] = flagSet;
    //tx_buf[0] |= (FutureUse << 5);
    //txbuf[1] to txbuf[7] reserved
    canTransmit(canREG1, canMESSAGE_BOX15, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX15, tx_buf);
}

void send_Data_Log(uint8_t count)
{
    uint8_t tx_buf[8] = {0};
    uint8_t seq_total = 0x03, pkt_type = 0x0A;
    tx_buf[0] = (pkt_type & 0x0F) | ((seq_total << 4) & 0xF0);
    tx_buf[1] = ((seq_total >> 4) & 0x03)| ((count & 0x3F) << 2);
    if(count == 2)
    {
        tx_data_log[15]++;
    }
    tx_buf[2] = tx_data_log[count*6 + 0];
    tx_buf[3] = tx_data_log[count*6 + 1];
    tx_buf[4] = tx_data_log[count*6 + 2];
    tx_buf[5] = tx_data_log[count*6 + 3];
    tx_buf[6] = tx_data_log[count*6 + 4];
    tx_buf[7] = tx_data_log[count*6 + 5];

    if(tx_data_log[15] == 0xFF)
    {
        tx_data_log[15] = 0;
    }
    canTransmit(canREG1, canMESSAGE_BOX16, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX16, tx_buf);
}

/* ============================================================
 *  CAN RX ISR CALLBACK
 * ============================================================ */
void canMessageNotification(canBASE_t *node, uint32_t messageBox)
{
//    uint32_t rx_id;
    can_source_t can_source;

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

    if (messageBox == canMESSAGE_BOX19)
    {

        canGetData(node, messageBox, rx_dmi_pilot);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_dmi_pilot, can_source);

    }
    else if (messageBox == canMESSAGE_BOX3)
    {
        canGetData(node, messageBox, rx_data_startup);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_data_startup, can_source);
    }
    else if (messageBox == canMESSAGE_BOX5)
    {
        canGetData(node, messageBox, rx_data_heartbeat);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_data_heartbeat, can_source);
    }
    else if (messageBox == canMESSAGE_BOX6)
    {
        canGetData(node, messageBox, rx_data_rfid);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_data_rfid, can_source);
    }
    else if (messageBox == canMESSAGE_BOX8)
    {
        canGetData(node, messageBox, rx_data_gsm);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_data_gsm, can_source);
    }
    //! 0x150,151,152 - Primary Input Card
    else if (messageBox == canMESSAGE_BOX10) 
    {
        canGetData(node, messageBox, rx_data_input_card);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_data_input_card, can_source);
    }

    //! 0X142- RADIO TIVA 1 
    //! 0X143- RADIO TIVA 2
    else if (messageBox == canMESSAGE_BOX14)
    {
        canGetData(node, messageBox, rx_data_radio);
        rx_id = canGetID(node, messageBox);

        can_if_process_rx(rx_id, rx_data_radio, can_source);
    } 
    //! 0x221 - SMOCIP
    else if (messageBox == canMESSAGE_BOX21) 
    {
      canGetData(node, messageBox, rx_data_smocip);
      rx_id = canGetID(node, messageBox);

      can_if_process_rx(rx_id, rx_data_smocip, can_source);
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
