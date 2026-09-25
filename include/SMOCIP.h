#ifndef SMOCIP_H_
#define SMOCIP_H_

#include <stdint.h>
#include "can_if.h"

#define SMOCIP_PKT_TYPE    0x01U
#define SMOCIP_SEQ_TOTAL   0x05U

#define SMOCIP_TX_CAN_ID    0x0230U
#define SMOCIP_RX_ID        0x0231U
#define SMOCIP_ACK_CAN_ID   0x0230U

#define ACK_ACTION_SMOCIP SMOCIP_PKT_TYPE
typedef struct
{
    char     station_id[5];      // "12345"
    uint16_t kms_key_index;
    uint16_t tsr_count;
    uint8_t  status_byte;

    uint32_t comm_card1_checksum;
    uint32_t comm_card2_checksum;
    uint32_t mvi_card_checksum;
    uint32_t input_card_checksum;
    uint32_t riu_checksum;
} smocip_tx_t;

extern smocip_tx_t smocip_tx;

typedef struct {
  char station_id[6];

  uint8_t smocip;
  uint8_t stn_sos_gen;
  uint8_t sos_cancel;
  uint8_t sos_ack;

  uint8_t status_byte;
  uint8_t valid;
} smocip_rx_t;

#define SMOCIP_PAYLOAD_BYTES          6U
#define SMOCIP_MAX_PAYLOAD_LEN        31U
#define SMOCIP_MAX_FRAGMENTS          6U

#define SMOCIP_ACK_TIMEOUT_MS         100U
#define SMOCIP_ACK_MAX_RETRIES        3U

#define SMOCIP_TRANSACTION_QUEUE_SIZE 8U

typedef struct
{
    uint8_t  pkt_type;
    uint16_t payload_len;
    uint8_t  payload[SMOCIP_MAX_PAYLOAD_LEN];

    uint8_t  seq_total;
    uint8_t  seq_index;

    uint8_t  retry_count;
    uint8_t  active;

    uint32_t start_time;

} smocip_transaction_t;

extern smocip_rx_t smocip_rx;
void smocip_rx_handle(uint8_t *data, can_source_t can_source);

void smocip_build_payload(void);
//void smocip_send_can(uint8_t seq_index);
void smocip_send(void);
void smocip_ack_process(void);
void smocip_tx_process(void);
//! TEST
void smocip_test_data_init(void);
void smocip_ack_rx_handle(uint32_t can_id, uint8_t *data);

extern volatile uint32_t comm_card1_checksum;
extern volatile uint32_t comm_card2_checksum;
extern volatile uint32_t mvi_card_checksum;
extern volatile uint32_t input_card_checksum;
extern volatile uint32_t riu_checksum;

#endif
