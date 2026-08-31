#include "SMOCIP.h"
#include "can.h"
#include <string.h>

smocip_tx_t smocip_tx;
smocip_rx_t smocip_rx;

/* 18-byte payload */
static uint8_t smocip_payload[18];

typedef struct {
  uint8_t data[8];
  uint8_t valid;
} smocip_can_frame_ctx_t;

static smocip_can_frame_ctx_t smocip_can1;
static smocip_can_frame_ctx_t smocip_can2;

//! ============ TEST DATA ==================
void smocip_test_data_init(void) {
  /* Station ID = "12345" */
  smocip_tx.station_id[0] = '1';
  smocip_tx.station_id[1] = '2';
  smocip_tx.station_id[2] = '3';
  smocip_tx.station_id[3] = '4';
  smocip_tx.station_id[4] = '5';

  /* KMS Key Index = 0x1234 */
  smocip_tx.kms_key_index = 0x1234U;

  /* TSR Count = 0x0056 */
  smocip_tx.tsr_count = 0x0056U;

  /*
   * Status:
   *
   * Bit 0 = SMOCIP
   * Bit 1 = STN_SOS_GEN
   * Bit 2 = SOS_CANCEL
   * Bit 3 = SOS_ACK
   *
   * 0x0B = 1011
   *
   * SMOCIP       = 1
   * STN_SOS_GEN  = 1
   * SOS_CANCEL   = 0
   * SOS_ACK      = 1
   */
  smocip_tx.status_byte = 0x0BU;

  /* Application checksum = AA BB CC DD EE FF */
  smocip_tx.app_checksum[0] = 0xAAU;
  smocip_tx.app_checksum[1] = 0xBBU;
  smocip_tx.app_checksum[2] = 0xCCU;
  smocip_tx.app_checksum[3] = 0xDDU;
  smocip_tx.app_checksum[4] = 0xEEU;
  smocip_tx.app_checksum[5] = 0xFFU;
}
//! =================================================

void smocip_build_payload(void) 
{
  smocip_payload[0] = smocip_tx.station_id[0];
  smocip_payload[1] = smocip_tx.station_id[1];
  smocip_payload[2] = smocip_tx.station_id[2];
  smocip_payload[3] = smocip_tx.station_id[3];
  smocip_payload[4] = smocip_tx.station_id[4];

  smocip_payload[5] = (uint8_t)(smocip_tx.kms_key_index >> 8);

  smocip_payload[6] = (uint8_t)(smocip_tx.kms_key_index);

  smocip_payload[7] = (uint8_t)(smocip_tx.tsr_count >> 8);
  smocip_payload[8] = (uint8_t)(smocip_tx.tsr_count);

  smocip_payload[9] = 0x00; /* Reserved */
  smocip_payload[10] = smocip_tx.status_byte;

  smocip_payload[11] = smocip_tx.app_checksum[0];
  smocip_payload[12] = smocip_tx.app_checksum[1];
  smocip_payload[13] = smocip_tx.app_checksum[2];
  smocip_payload[14] = smocip_tx.app_checksum[3];
  smocip_payload[15] = smocip_tx.app_checksum[4];
  smocip_payload[16] = smocip_tx.app_checksum[5];

  smocip_payload[17] = 0x00; /* Future */
}

void smocip_send_can(uint8_t seq_index) 
{
  uint8_t tx_buf[8] = {0};

  tx_buf[0] = (SMOCIP_PKT_TYPE & 0x0FU) | ((SMOCIP_SEQ_TOTAL << 4) & 0xF0U);

  tx_buf[1] = ((SMOCIP_SEQ_TOTAL >> 4) & 0x03U) | ((seq_index & 0x3FU) << 2);

  tx_buf[2] = smocip_payload[(seq_index * 6U) + 0];
  tx_buf[3] = smocip_payload[(seq_index * 6U) + 1];
  tx_buf[4] = smocip_payload[(seq_index * 6U) + 2];
  tx_buf[5] = smocip_payload[(seq_index * 6U) + 3];
  tx_buf[6] = smocip_payload[(seq_index * 6U) + 4];
  tx_buf[7] = smocip_payload[(seq_index * 6U) + 5];

  canTransmit(canREG1, canMESSAGE_BOX20, tx_buf);
  canTransmit(canREG2, canMESSAGE_BOX20, tx_buf);
}

void smocip_rx_handle(uint8_t *data, can_source_t can_source)
{
  uint8_t pkt_type;
  uint8_t seq_total;
  uint8_t seq_index;

  /* =========================================================
   * CAN1 / CAN2 REDUNDANCY CHECK
   * ========================================================= */

  if (can_source == CAN_SOURCE_1) 
  {
    /* If this exact frame was already received on CAN2,
     * this is the redundant copy.
     */
    if (smocip_can2.valid && memcmp(smocip_can2.data, data, 8U) == 0) 
    {
      return;
    }

    /* New CAN1 frame - save it */
    memcpy(smocip_can1.data, data, 8U);
    smocip_can1.valid = 1U;
  } 
  else if (can_source == CAN_SOURCE_2) 
  {
    /* If this exact frame was already received on CAN1,
     * this is the redundant copy.
     */
    if (smocip_can1.valid && memcmp(smocip_can1.data, data, 8U) == 0) 
    {
      return;
    }

    /* New CAN2 frame - save it */
    memcpy(smocip_can2.data, data, 8U);
    smocip_can2.valid = 1U;
  } 
  else 
  {
    return;
  }

  pkt_type = data[0] & 0x0F;

  seq_total = ((data[1] & 0x03) << 4) | ((data[0] & 0xF0) >> 4);

  seq_index = (data[1] & 0xFC) >> 2;

  if (seq_total != 0U)
    return;

  if (seq_index != 0U)
    return;

  smocip_rx.station_id[0] = data[2];
  smocip_rx.station_id[1] = data[3];
  smocip_rx.station_id[2] = data[4];
  smocip_rx.station_id[3] = data[5];
  smocip_rx.station_id[4] = data[6];
  smocip_rx.station_id[5] = '\0';

  smocip_rx.status_byte = data[7];

  smocip_rx.smocip = (data[7] >> 0) & 1U;
  smocip_rx.stn_sos_gen = (data[7] >> 1) & 1U;
  smocip_rx.sos_cancel = (data[7] >> 2) & 1U;
  smocip_rx.sos_ack = (data[7] >> 3) & 1U;

  smocip_rx.valid = 1U;
  send_cpu_universal_ack((uint16_t)SMOCIP_RX_ID, ACK_ACTION_SMOCIP, CPU_ACK_OK);
}