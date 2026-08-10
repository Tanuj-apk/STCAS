#ifndef SMOCIP_H_
#define SMOCIP_H_

#include <stdint.h>

#define SMOCIP_PKT_TYPE    0x01U
#define SMOCIP_SEQ_TOTAL   0x03U

#define SMOCIP_RX_ID 0x221U

typedef struct
{
    char     station_id[5];      // "12345"
    uint16_t kms_key_index;
    uint16_t tsr_count;
    uint8_t  status_byte;
    uint8_t  app_checksum[6];
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

extern smocip_rx_t smocip_rx;
void smocip_rx_handle(uint8_t *data);

void smocip_build_payload(void);
void smocip_send_can(uint8_t seq_index);

#endif
