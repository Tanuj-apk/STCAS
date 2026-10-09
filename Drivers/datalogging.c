#include "datalogging.h"

#include <stddef.h>
#include <string.h>

#include "can_if.h"
#include "gps.h"

typedef struct
{
    uint16_t length;
    uint8_t data[DATA_LOGGER_MAX_RECORD_SIZE];
} data_logger_record_t;

typedef struct
{
    data_logger_record_t records[DATA_LOGGER_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
} data_logger_queue_t;

typedef struct
{
    data_logger_queue_t queue;
    data_logger_can_tx_callback_t tx_callback;
    bool tx_active;
    bool tx_waiting_ack;
    uint8_t tx_total_fragments;
    uint8_t tx_fragment_index;
    uint8_t tx_retry_count;
    uint16_t tx_record_length;
    uint8_t tx_record[DATA_LOGGER_MAX_RECORD_SIZE];
    uint32_t tx_start_time;
} data_logger_context_t;

static data_logger_context_t data_logger_context;

volatile uint8_t data_logger_ack_received;
volatile uint8_t data_logger_ack_action;
volatile uint8_t data_logger_ack_status;
volatile uint32_t data_logger_queue_full_count;
volatile uint32_t data_logger_tx_failure_count;
volatile uint32_t data_logger_ack_timeout_count;

static uint16_t data_logger_next_index(uint16_t index)
{
    index++;
    if (index >= DATA_LOGGER_QUEUE_SIZE)
    {
        index = 0U;
    }
    return index;
}

static bool data_logger_queue_pop(data_logger_record_t *record)
{
    if ((record == NULL) ||
        (data_logger_context.queue.tail == data_logger_context.queue.head))
    {
        return false;
    }

    memcpy(record,
           &data_logger_context.queue.records[data_logger_context.queue.tail],
           sizeof(*record));
    data_logger_context.queue.tail =
        data_logger_next_index(data_logger_context.queue.tail);
    return true;
}

static uint8_t data_logger_make_byte0(uint8_t total_fragments)
{
    return (uint8_t)((((total_fragments) & 0x0FU) << 4U) |
                     (DATA_LOGGER_PACKET_TYPE & 0x0FU));
}

static uint8_t data_logger_make_byte1(uint8_t total_fragments,
                                     uint8_t fragment_index)
{
    return (uint8_t)(((fragment_index & 0x3FU) << 2U) |
                     ((total_fragments >> 4U) & 0x03U));
}

void data_logger_init(data_logger_can_tx_callback_t tx_callback)
{
    memset(&data_logger_context, 0, sizeof(data_logger_context));
    data_logger_context.tx_callback = tx_callback;

    data_logger_ack_received = 0U;
    data_logger_ack_action = 0U;
    data_logger_ack_status = 0U;
    data_logger_queue_full_count = 0U;
    data_logger_tx_failure_count = 0U;
    data_logger_ack_timeout_count = 0U;
}

bool data_logger_queue_record(const uint8_t *record, uint16_t length)
{
    uint16_t next_head;
    data_logger_record_t *destination;

    if ((record == NULL) || (length == 0U) ||
        (length > DATA_LOGGER_MAX_RECORD_SIZE))
    {
        return false;
    }

    next_head = data_logger_next_index(data_logger_context.queue.head);
    if (next_head == data_logger_context.queue.tail)
    {
        data_logger_queue_full_count++;
        return false;
    }

    destination =
        &data_logger_context.queue.records[data_logger_context.queue.head];
    destination->length = length;
    memcpy(destination->data, record, length);
    data_logger_context.queue.head = next_head;
    return true;
}

uint16_t data_logger_queue_count(void)
{
    if (data_logger_context.queue.head >= data_logger_context.queue.tail)
    {
        return data_logger_context.queue.head - data_logger_context.queue.tail;
    }

    return (uint16_t)(DATA_LOGGER_QUEUE_SIZE -
                      data_logger_context.queue.tail +
                      data_logger_context.queue.head);
}

uint8_t data_logger_process_tx(uint8_t max_frames)
{
    uint8_t frames_sent = 0U;

    if ((max_frames == 0U) || (data_logger_context.tx_callback == NULL))
    {
        return 0U;
    }

    if (!data_logger_context.tx_active)
    {
        data_logger_record_t record;

        if (!data_logger_queue_pop(&record))
        {
            return 0U;
        }

        data_logger_context.tx_record_length = record.length;
        memcpy(data_logger_context.tx_record, record.data, record.length);
        data_logger_context.tx_total_fragments =
            (uint8_t)((record.length + DATA_LOGGER_CAN_PAYLOAD_SIZE - 1U) /
                      DATA_LOGGER_CAN_PAYLOAD_SIZE);
        data_logger_context.tx_fragment_index = 0U;
        data_logger_context.tx_retry_count = 0U;
        data_logger_context.tx_waiting_ack = false;
        data_logger_context.tx_start_time = 0U;
        data_logger_context.tx_active = true;
    }

    if (data_logger_context.tx_waiting_ack)
    {
        return 0U;
    }

    while (data_logger_context.tx_active &&
           !data_logger_context.tx_waiting_ack &&
           (frames_sent < max_frames))
    {
        uint8_t frame[DATA_LOGGER_CAN_FRAME_SIZE] = {0U};
        uint16_t offset;
        uint16_t bytes_remaining;
        uint8_t bytes_to_send;

        frame[0] =
            data_logger_make_byte0(data_logger_context.tx_total_fragments);
        frame[1] =
            data_logger_make_byte1(data_logger_context.tx_total_fragments,
                                   data_logger_context.tx_fragment_index);

        offset = (uint16_t)(data_logger_context.tx_fragment_index *
                            DATA_LOGGER_CAN_PAYLOAD_SIZE);
        bytes_remaining =
            (uint16_t)(data_logger_context.tx_record_length - offset);
        bytes_to_send = (bytes_remaining > DATA_LOGGER_CAN_PAYLOAD_SIZE) ?
                        DATA_LOGGER_CAN_PAYLOAD_SIZE :
                        (uint8_t)bytes_remaining;
        memcpy(&frame[2], &data_logger_context.tx_record[offset], bytes_to_send);

        if (!data_logger_context.tx_callback(
                can_get_local_tx_id(DATA_LOGGER_TX_CAN_ID), frame))
        {
            data_logger_tx_failure_count++;
            break;
        }

        frames_sent++;
        data_logger_context.tx_fragment_index++;

        if (data_logger_context.tx_fragment_index >=
            data_logger_context.tx_total_fragments)
        {
            data_logger_context.tx_waiting_ack = true;
            data_logger_context.tx_start_time = system_ms;
        }
    }

    return frames_sent;
}

void data_logger_process_ack(void)
{
    if (!data_logger_context.tx_active ||
        !data_logger_context.tx_waiting_ack)
    {
        return;
    }

    if ((system_ms - data_logger_context.tx_start_time) <
        DATA_LOGGER_ACK_TIMEOUT_MS)
    {
        return;
    }

    data_logger_ack_timeout_count++;
    if (data_logger_context.tx_retry_count <
        DATA_LOGGER_ACK_MAX_RETRIES)
    {
        data_logger_context.tx_retry_count++;
        data_logger_context.tx_fragment_index = 0U;
        data_logger_context.tx_waiting_ack = false;
        data_logger_context.tx_start_time = 0U;
    }
    else
    {
        data_logger_context.tx_active = false;
        data_logger_context.tx_waiting_ack = false;
        data_logger_context.tx_retry_count = 0U;
        data_logger_context.tx_start_time = 0U;
        data_logger_tx_failure_count++;
    }
}

void data_logger_ack_rx_handle(uint32_t can_id, uint8_t *data)
{
    uint16_t acknowledged_can_id;

    if ((data == NULL) || (can_id != DATA_LOGGER_ACK_CAN_ID))
    {
        return;
    }

    acknowledged_can_id = ((uint16_t)data[0] << 8U) |
                          (uint16_t)data[1];
    if (acknowledged_can_id !=
        (uint16_t)can_get_local_tx_id(DATA_LOGGER_TX_CAN_ID))
    {
        return;
    }

    data_logger_ack_received = 1U;
    data_logger_ack_action = data[2];
    data_logger_ack_status = data[3];

    if (data[3] == DATA_LOGGER_ACK_OK)
    {
        data_logger_context.tx_active = false;
        data_logger_context.tx_waiting_ack = false;
        data_logger_context.tx_retry_count = 0U;
        data_logger_context.tx_start_time = 0U;
    }
}
