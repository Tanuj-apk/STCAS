#ifndef KMS_H_
#define KMS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define KMS_MAX_FRAGMENTS             11U
#define KMS_FRAGMENT_PAYLOAD_SIZE      6U
#define KMS_MAX_DATA_BYTES            66U
#define KMS_RX_TIMEOUT_SEC             1U
#define KMS_QUERY_TIMEOUT_SEC          2U
#define KMS_QUERY_MAX_RETRIES          2U

#define KMS_MSG_UNIQUE_SET_ID          0x01U
#define KMS_MSG_IDENTIFICATION_ACK     0x02U
#define KMS_MSG_AUTHENTICATION_KEYS    0x03U
#define KMS_MSG_OTP                    0x04U

void kms_init(void);
bool kms_send_auth_key_query(void);
void kms_rx_handle(uint32_t can_id, uint8_t *data);
void kms_process_1s(void);

extern volatile uint32_t kms_unique_set_id;
extern volatile uint8_t kms_identification_ack;
extern volatile uint8_t kms_identification_ack_valid;
extern volatile uint8_t kms_authentication_keys_valid;
extern volatile uint8_t kms_otp_valid;
extern volatile uint8_t kms_key_validity_start_time[4];
extern volatile uint8_t kms_key_set_end_time[4];
extern volatile uint8_t kms_key_1[16];
extern volatile uint8_t kms_key_2[16];
extern volatile uint8_t kms_otp[4];
extern volatile uint32_t kms_rx_timeout_count;
extern volatile uint32_t kms_rx_invalid_count;
extern volatile uint8_t kms_query_pending;
extern volatile uint8_t kms_query_retry_count;
extern volatile uint32_t kms_query_failure_count;

#ifdef __cplusplus
}
#endif

#endif /* KMS_H_ */
