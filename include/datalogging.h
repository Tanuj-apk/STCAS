#ifndef DATALOGGING_H_
#define DATALOGGING_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Data Logger transport used by STCAS.  Record contents are intentionally
 * opaque here: station-specific event producers serialize their own record
 * and queue it for the common CAN fragmentation/ACK transport. */
#define DATA_LOGGER_PACKET_TYPE          0x0AU
#define DATA_LOGGER_QUEUE_SIZE           16U
#define DATA_LOGGER_MAX_RECORD_SIZE      64U
#define DATA_LOGGER_CAN_FRAME_SIZE       8U
#define DATA_LOGGER_CAN_PAYLOAD_SIZE     6U
#define DATA_LOGGER_MAX_FRAGMENTS        63U
#define DATA_LOGGER_ACK_TIMEOUT_MS       100U
#define DATA_LOGGER_ACK_MAX_RETRIES      3U
#define DATA_LOGGER_ACK_OK               0x00U

typedef bool (*data_logger_can_tx_callback_t)(
    uint32_t can_id,
    const uint8_t data[DATA_LOGGER_CAN_FRAME_SIZE]);

void data_logger_init(data_logger_can_tx_callback_t tx_callback);
bool data_logger_queue_record(const uint8_t *record, uint16_t length);
uint16_t data_logger_queue_count(void);
uint8_t data_logger_process_tx(uint8_t max_frames);
void data_logger_process_ack(void);
void data_logger_ack_rx_handle(uint32_t can_id, uint8_t *data);

extern volatile uint8_t data_logger_ack_received;
extern volatile uint8_t data_logger_ack_action;
extern volatile uint8_t data_logger_ack_status;
extern volatile uint32_t data_logger_queue_full_count;
extern volatile uint32_t data_logger_tx_failure_count;
extern volatile uint32_t data_logger_ack_timeout_count;

#ifdef __cplusplus
}
#endif

#endif /* DATALOGGING_H_ */
