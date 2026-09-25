#include "SMOCIP.h"
#include "can.h"
#include <string.h>
#include "gps.h"

smocip_tx_t smocip_tx;
smocip_rx_t smocip_rx;

/* 31-byte payload */
static uint8_t smocip_payload[31];

typedef struct {
  uint8_t data[8];
  uint8_t valid;
} smocip_can_frame_ctx_t;

static smocip_can_frame_ctx_t smocip_can1;
static smocip_can_frame_ctx_t smocip_can2;

volatile uint8_t smocip_ack_received_flag = 0U;
volatile uint8_t smocip_ack_action = 0U;
volatile uint8_t smocip_ack_status = 0U;

static smocip_transaction_t smocip_transaction_queue[SMOCIP_TRANSACTION_QUEUE_SIZE];

static uint8_t smocip_queue_head  = 0U;
static uint8_t smocip_queue_tail  = 0U;
static uint8_t smocip_queue_count = 0U;

static uint8_t smocip_queue_is_full(void)
{
    return (smocip_queue_count >= SMOCIP_TRANSACTION_QUEUE_SIZE);
}
static smocip_transaction_t *smocip_get_transaction(void)
{
    if (smocip_queue_count == 0U)
    {
        return NULL;
    }

    return &smocip_transaction_queue[smocip_queue_head];
}

void smocip_transaction_start(uint8_t pkt_type, const uint8_t *payload, uint16_t payload_len)
{
    smocip_transaction_t *transaction;

    if (payload == NULL)
    {
        return;
    }

    if (payload_len == 0U)
    {
        return;
    }

    if (payload_len > SMOCIP_MAX_PAYLOAD_LEN)
    {
        return;
    }

    if (smocip_queue_is_full())
    {
        return;
    }

    transaction = &smocip_transaction_queue[smocip_queue_tail];

    transaction->pkt_type    = pkt_type;
    transaction->payload_len = payload_len;

    memcpy(transaction->payload, payload, payload_len);

    transaction->seq_total =
        (uint8_t)((payload_len + SMOCIP_PAYLOAD_BYTES - 1U) /
                  SMOCIP_PAYLOAD_BYTES);

    if (transaction->seq_total == 0U)
    {
        return;
    }

    if (transaction->seq_total > SMOCIP_MAX_FRAGMENTS)
    {
        return;
    }

    transaction->seq_index  = 0U;
    transaction->retry_count = 0U;
    transaction->active     = 0U;
    transaction->start_time = 0U;

    smocip_queue_tail++;

    if (smocip_queue_tail >= SMOCIP_TRANSACTION_QUEUE_SIZE)
    {
        smocip_queue_tail = 0U;
    }

    smocip_queue_count++;

    /*
     * First transaction becomes active.
     */
    if (smocip_queue_count == 1U)
    {
        transaction->active = 1U;
    }
}

static void smocip_pop_transaction(void)
{
    smocip_transaction_t *transaction;

    transaction = smocip_get_transaction();

    if (transaction == NULL)
    {
        return;
    }

    transaction->active = 0U;

    smocip_queue_head++;

    if (smocip_queue_head >= SMOCIP_TRANSACTION_QUEUE_SIZE)
    {
        smocip_queue_head = 0U;
    }

    smocip_queue_count--;

    /*
     * Activate next transaction.
     */
    if (smocip_queue_count > 0U)
    {
        transaction = smocip_get_transaction();

        transaction->seq_index   = 0U;
        transaction->retry_count = 0U;
        transaction->start_time  = 0U;
        transaction->active      = 1U;
    }
}

static void smocip_transaction_send_next_fragment(smocip_transaction_t *transaction)
{
    uint8_t tx_buf[8];
    uint16_t payload_offset;
    uint8_t i;

    if (transaction == NULL)
    {
        return;
    }

    if (transaction->seq_index >= transaction->seq_total)
    {
        return;
    }

    memset(tx_buf, 0, sizeof(tx_buf));

    /*
     * Byte 0:
     *
     * Bits 7-4 : Sequence total LSB
     * Bits 3-0 : Packet type
     */
    tx_buf[0] =
        (uint8_t)(((transaction->seq_total & 0x0FU) << 4) |
                  (transaction->pkt_type & 0x0FU));

    /*
     * Byte 1:
     *
     * Bits 7-2 : Sequence index
     * Bits 1-0 : Sequence total MSB
     */
    tx_buf[1] =
        (uint8_t)(((transaction->seq_index & 0x3FU) << 2) |
                  ((transaction->seq_total >> 4) & 0x03U));

    payload_offset =
        (uint16_t)transaction->seq_index *
        SMOCIP_PAYLOAD_BYTES;

    for (i = 0U; i < SMOCIP_PAYLOAD_BYTES; i++)
    {
        if ((payload_offset + i) < transaction->payload_len)
        {
            tx_buf[2U + i] =
                transaction->payload[payload_offset + i];
        }
        else
        {
            tx_buf[2U + i] = 0U;
        }
    }

    /*
     * Redundant transmission on CAN1 and CAN2.
     */
    canTransmit(canREG1, canMESSAGE_BOX21, tx_buf);
    canTransmit(canREG2, canMESSAGE_BOX21, tx_buf);

    transaction->seq_index++;

    /*
     * Complete SMOCIP packet transmitted.
     * Now wait for ACK.
     */
    if (transaction->seq_index >= transaction->seq_total)
    {
        transaction->seq_index = 0U;
        transaction->start_time = system_ms;
    }
}

void smocip_ack_received(uint8_t ack_status)
{
    smocip_transaction_t *transaction;

    transaction = smocip_get_transaction();

    if (transaction == NULL)
    {
        return;
    }

    if (transaction->active == 0U)
    {
        return;
    }

    if (ack_status == CPU_ACK_OK)
    {
        smocip_pop_transaction();
    }
}

void smocip_ack_process(void)
{
    smocip_transaction_t *transaction;

    if (smocip_queue_count == 0U)
    {
        return;
    }

    transaction = smocip_get_transaction();

    if (transaction == NULL)
    {
        return;
    }

    /*
     * Make sure current transaction is active.
     */
    if (transaction->active == 0U)
    {
        transaction->active      = 1U;
        transaction->seq_index   = 0U;
        transaction->retry_count = 0U;
        transaction->start_time  = 0U;

        return;
    }

    /*
     * Still transmitting fragments.
     */
    if (transaction->start_time == 0U)
    {
        return;
    }

    /*
     * Waiting for ACK.
     */
    if ((system_ms - transaction->start_time) >=
        SMOCIP_ACK_TIMEOUT_MS)
    {
        if (transaction->retry_count <
            SMOCIP_ACK_MAX_RETRIES)
        {
            transaction->retry_count++;

            /*
             * Retransmit complete SMOCIP packet.
             */
            transaction->seq_index  = 0U;
            transaction->start_time = 0U;
        }
        else
        {
            /*
             * Final failure.
             *
             * SMOCIP fault handling can be added here.
             */
            smocip_pop_transaction();
        }
    }
}

void smocip_tx_process(void)
{
    smocip_transaction_t *transaction;

    if (smocip_queue_count == 0U)
    {
        return;
    }

    transaction = smocip_get_transaction();

    if ((transaction != NULL) &&
        (transaction->active != 0U) &&
        (transaction->start_time == 0U))
    {
        smocip_transaction_send_next_fragment(transaction);
    }
}

//! ============ TEST DATA ==================
void smocip_test_data_init(void)
{
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
  smocip_tx.comm_card1_checksum = comm_card1_checksum;
  smocip_tx.comm_card2_checksum = comm_card2_checksum;
  smocip_tx.mvi_card_checksum    = mvi_card_checksum;
  smocip_tx.input_card_checksum  = input_card_checksum;
  smocip_tx.riu_checksum         = riu_checksum;
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

    smocip_payload[9] = 0x00U;   /* Reserved */
    smocip_payload[10] = smocip_tx.status_byte;

    /* COMM CARD 1 CHECKSUM */
    smocip_payload[11] = (uint8_t)(smocip_tx.comm_card1_checksum >> 24);
    smocip_payload[12] = (uint8_t)(smocip_tx.comm_card1_checksum >> 16);
    smocip_payload[13] = (uint8_t)(smocip_tx.comm_card1_checksum >> 8);
    smocip_payload[14] = (uint8_t)(smocip_tx.comm_card1_checksum);

    /* COMM CARD 2 CHECKSUM */
    smocip_payload[15] = (uint8_t)(smocip_tx.comm_card2_checksum >> 24);
    smocip_payload[16] = (uint8_t)(smocip_tx.comm_card2_checksum >> 16);
    smocip_payload[17] = (uint8_t)(smocip_tx.comm_card2_checksum >> 8);
    smocip_payload[18] = (uint8_t)(smocip_tx.comm_card2_checksum);

    /* MVI CARD CHECKSUM */
    smocip_payload[19] = (uint8_t)(smocip_tx.mvi_card_checksum >> 24);
    smocip_payload[20] = (uint8_t)(smocip_tx.mvi_card_checksum >> 16);
    smocip_payload[21] = (uint8_t)(smocip_tx.mvi_card_checksum >> 8);
    smocip_payload[22] = (uint8_t)(smocip_tx.mvi_card_checksum);

    /* INPUT CARD CHECKSUM */
    smocip_payload[23] = (uint8_t)(smocip_tx.input_card_checksum >> 24);
    smocip_payload[24] = (uint8_t)(smocip_tx.input_card_checksum >> 16);
    smocip_payload[25] = (uint8_t)(smocip_tx.input_card_checksum >> 8);
    smocip_payload[26] = (uint8_t)(smocip_tx.input_card_checksum);

    /* RIU CHECKSUM */
    smocip_payload[27] = (uint8_t)(smocip_tx.riu_checksum >> 24);
    smocip_payload[28] = (uint8_t)(smocip_tx.riu_checksum >> 16);
    smocip_payload[29] = (uint8_t)(smocip_tx.riu_checksum >> 8);
    smocip_payload[30] = (uint8_t)(smocip_tx.riu_checksum);
}

//void smocip_send_can(uint8_t seq_index)
//{
//  uint8_t tx_buf[8] = {0};
//
//  tx_buf[0] = (SMOCIP_PKT_TYPE & 0x0FU) | ((SMOCIP_SEQ_TOTAL << 4) & 0xF0U);
//
//  tx_buf[1] = ((SMOCIP_SEQ_TOTAL >> 4) & 0x03U) | ((seq_index & 0x3FU) << 2);
//
//  tx_buf[2] = smocip_payload[(seq_index * 6U) + 0];
//  tx_buf[3] = smocip_payload[(seq_index * 6U) + 1];
//  tx_buf[4] = smocip_payload[(seq_index * 6U) + 2];
//  tx_buf[5] = smocip_payload[(seq_index * 6U) + 3];
//  tx_buf[6] = smocip_payload[(seq_index * 6U) + 4];
//  tx_buf[7] = smocip_payload[(seq_index * 6U) + 5];
//
//  canTransmit(canREG1, canMESSAGE_BOX21, tx_buf);
//  canTransmit(canREG2, canMESSAGE_BOX21, tx_buf);
//}
void smocip_send(void)
{
    smocip_build_payload();

    smocip_transaction_start(SMOCIP_PKT_TYPE, smocip_payload, SMOCIP_MAX_PAYLOAD_LEN);
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

void smocip_ack_rx_handle(uint32_t can_id, uint8_t *data)
{
    uint16_t ack_can_id;
    uint8_t action_type;
    uint8_t ack_status;

    ack_can_id = ((uint16_t)data[0] << 8) |
                 (uint16_t)data[1];

    if (ack_can_id != SMOCIP_TX_CAN_ID)
    {
        return;
    }

    action_type = data[2];
    ack_status = data[3];

    smocip_ack_received_flag = 1U;
    smocip_ack_action   = action_type;
    smocip_ack_status   = ack_status;

    smocip_ack_received(ack_status);
}
