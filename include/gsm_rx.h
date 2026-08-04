#ifndef GSM_RX_H
#define GSM_RX_H

#include <stdint.h>

/* ================= GSM DEFINES ================= */

#define GSM_AUTH_KEY_BITS      128
#define GSM_AUTH_KEY_BYTES     16
#define GSM_MAX_FRAGMENTS      4

#define MSG_TYPE_GSM_AUTH_KEY   0x31U
#define MSG_TYPE_GSM_START_ACK  0x32U

#define GSM_START_ACK_OK        0x00U
#define GSM_START_ACK_BUSY      0x01U
#define GSM_START_ACK_REJECTED  0x02U
#define GSM_START_ACK_HW_FAULT  0x03U

#define GSM_START_ACK_TIMEOUT_SEC   2U
#define GSM_AUTH_KEY_TIMEOUT_SEC    3U
#define GSM_MAX_RETRIES_PER_UNIT    2

/* GSM FSM */
typedef enum
{
    GSM_FSM_IDLE = 0,
    GSM_FSM_WAIT_ACK,
    GSM_FSM_ACK_OK,
    GSM_FSM_ACK_REJECTED,
    GSM_FSM_ACK_TIMEOUT
} gsm_start_state_t;

/* GSM faults */
#define GSM_FAULT_NONE            0x00U
#define GSM_FAULT_GSM1_FAILED     0x01U
#define GSM_FAULT_GSM2_FAILED     0x02U
#define GSM_FAULT_AUTH_TIMEOUT    0x04U
#define GSM_FAULT_NO_GSM_AVAIL    0x08U

extern uint8_t gsm_fault_flags;

/* APIs */
void gsm_rx_handle(uint32_t can_id, uint8_t *data);

void gsm_start_request(uint8_t gsm_id);
void gsm_start_poll_1s(void);
void gsm_manager_process(void);

uint8_t gsm_auth_key_available(void);
const uint8_t *gsm_get_auth_key(void);

uint8_t gsm_start_ack_received(void);
uint8_t gsm_get_start_ack_status(void);
uint8_t gsm_get_start_ack_gsm_id(void);
void gsm_clear_start_ack(void);

#endif
