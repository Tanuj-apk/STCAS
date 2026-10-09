#include "kms.h"

#include <stddef.h>
#include <string.h>

#include "can.h"
#include "can_if.h"
#include "gps.h"

typedef struct
{
    uint8_t active;
    uint8_t message_type;
    uint8_t total_fragments;
    uint16_t received_mask;
    uint16_t data_length;
    uint8_t data[KMS_MAX_DATA_BYTES];
    uint32_t start_time;
} kms_rx_context_t;

static kms_rx_context_t kms_rx_context;

volatile uint32_t kms_unique_set_id;
volatile uint8_t kms_identification_ack;
volatile uint8_t kms_identification_ack_valid;
volatile uint8_t kms_authentication_keys_valid;
volatile uint8_t kms_otp_valid;
volatile uint8_t kms_key_validity_start_time[4];
volatile uint8_t kms_key_set_end_time[4];
volatile uint8_t kms_key_1[16];
volatile uint8_t kms_key_2[16];
volatile uint8_t kms_otp[4];
volatile uint32_t kms_rx_timeout_count;
volatile uint32_t kms_rx_invalid_count;
volatile uint8_t kms_query_pending;
volatile uint8_t kms_query_retry_count;
volatile uint32_t kms_query_failure_count;

static uint32_t kms_query_start_time;

static void kms_reset_rx(void)
{
    memset(&kms_rx_context, 0, sizeof(kms_rx_context));
}

static bool kms_transmit_auth_key_query(void)
{
    uint8_t frame[8] = {0U};

    frame[0] = KMS_MSG_UNIQUE_SET_ID;
    return (can_transmit_redundant(canMESSAGE_BOX7, frame) != 0U);
}

static uint8_t kms_get_total_fragments(const uint8_t data[8])
{
    return (uint8_t)((((uint8_t)(data[0] >> 4U)) & 0x0FU) |
                     ((data[1] & 0x03U) << 4U));
}

static void kms_copy_from_rx(volatile uint8_t *destination,
                             uint16_t source_offset,
                             uint16_t length)
{
    uint16_t i;

    for (i = 0U; i < length; i++)
    {
        destination[i] = kms_rx_context.data[source_offset + i];
    }
}

static bool kms_apply_complete_message(void)
{
    switch (kms_rx_context.message_type)
    {
    case KMS_MSG_UNIQUE_SET_ID:
        if (kms_rx_context.data_length < 4U)
        {
            return false;
        }
        kms_unique_set_id =
            (uint32_t)kms_rx_context.data[0] |
            ((uint32_t)kms_rx_context.data[1] << 8U) |
            ((uint32_t)kms_rx_context.data[2] << 16U) |
            ((uint32_t)kms_rx_context.data[3] << 24U);
        return true;

    case KMS_MSG_IDENTIFICATION_ACK:
        if (kms_rx_context.data_length < 1U)
        {
            return false;
        }
        kms_identification_ack = kms_rx_context.data[0];
        kms_identification_ack_valid = 1U;
        return true;

    case KMS_MSG_AUTHENTICATION_KEYS:
        if (kms_rx_context.data_length < 40U)
        {
            return false;
        }
        kms_copy_from_rx(kms_key_validity_start_time, 0U, 4U);
        kms_copy_from_rx(kms_key_set_end_time, 4U, 4U);
        kms_copy_from_rx(kms_key_1, 8U, 16U);
        kms_copy_from_rx(kms_key_2, 24U, 16U);
        kms_authentication_keys_valid = 1U;
        return true;

    case KMS_MSG_OTP:
        if (kms_rx_context.data_length < 4U)
        {
            return false;
        }
        kms_copy_from_rx(kms_otp, 0U, 4U);
        kms_otp_valid = 1U;
        return true;

    default:
        return false;
    }
}

void kms_init(void)
{
    kms_reset_rx();
    kms_unique_set_id = 0U;
    kms_identification_ack = 0U;
    kms_identification_ack_valid = 0U;
    kms_authentication_keys_valid = 0U;
    kms_otp_valid = 0U;
    memset((void *)kms_key_validity_start_time, 0,
           sizeof(kms_key_validity_start_time));
    memset((void *)kms_key_set_end_time, 0, sizeof(kms_key_set_end_time));
    memset((void *)kms_key_1, 0, sizeof(kms_key_1));
    memset((void *)kms_key_2, 0, sizeof(kms_key_2));
    memset((void *)kms_otp, 0, sizeof(kms_otp));
    kms_rx_timeout_count = 0U;
    kms_rx_invalid_count = 0U;
    kms_query_pending = 0U;
    kms_query_retry_count = 0U;
    kms_query_failure_count = 0U;
    kms_query_start_time = 0U;
}

bool kms_send_auth_key_query(void)
{
    if (!kms_transmit_auth_key_query())
    {
        kms_query_failure_count++;
        return false;
    }

    kms_query_pending = 1U;
    kms_query_retry_count = 0U;
    kms_query_start_time = seconds_uptime;
    return true;
}

void kms_rx_handle(uint32_t can_id, uint8_t *data)
{
    uint8_t message_type;
    uint8_t total_fragments;
    uint8_t fragment_index;
    uint16_t fragment_bit;
    uint16_t offset;
    uint16_t expected_mask;

    if ((data == NULL) ||
        ((can_id != KMS_RX_CAN_ID_1) && (can_id != KMS_RX_CAN_ID_2)))
    {
        kms_rx_invalid_count++;
        return;
    }

    message_type = data[0] & 0x0FU;
    total_fragments = kms_get_total_fragments(data);
    fragment_index = (data[1] >> 2U) & 0x3FU;

    if ((total_fragments == 0U) ||
        (total_fragments > KMS_MAX_FRAGMENTS) ||
        (fragment_index >= total_fragments))
    {
        kms_rx_invalid_count++;
        return;
    }

    if (!kms_rx_context.active)
    {
        if (fragment_index != 0U)
        {
            kms_rx_invalid_count++;
            return;
        }

        kms_reset_rx();
        kms_rx_context.active = 1U;
        kms_rx_context.message_type = message_type;
        kms_rx_context.total_fragments = total_fragments;
        kms_rx_context.start_time = seconds_uptime;
    }
    else if ((kms_rx_context.message_type != message_type) ||
             (kms_rx_context.total_fragments != total_fragments))
    {
        kms_rx_invalid_count++;
        kms_reset_rx();
        return;
    }

    fragment_bit = (uint16_t)1U << fragment_index;
    if ((kms_rx_context.received_mask & fragment_bit) != 0U)
    {
        return;
    }

    offset = (uint16_t)fragment_index * KMS_FRAGMENT_PAYLOAD_SIZE;
    if ((offset + KMS_FRAGMENT_PAYLOAD_SIZE) > KMS_MAX_DATA_BYTES)
    {
        kms_rx_invalid_count++;
        kms_reset_rx();
        return;
    }

    memcpy(&kms_rx_context.data[offset], &data[2],
           KMS_FRAGMENT_PAYLOAD_SIZE);
    kms_rx_context.received_mask |= fragment_bit;
    if (kms_rx_context.data_length <
        (offset + KMS_FRAGMENT_PAYLOAD_SIZE))
    {
        kms_rx_context.data_length =
            offset + KMS_FRAGMENT_PAYLOAD_SIZE;
    }

    expected_mask = (uint16_t)(((uint16_t)1U << total_fragments) - 1U);
    if (kms_rx_context.received_mask == expected_mask)
    {
        uint8_t status = kms_apply_complete_message() ?
                         CPU_ACK_OK : CPU_ACK_INVALID;

        send_cpu_universal_ack((uint16_t)can_id,
                               ACK_ACTION_ACCESS_AUTH,
                               status);
        if (status != CPU_ACK_OK)
        {
            kms_rx_invalid_count++;
        }
        else
        {
            kms_query_pending = 0U;
            kms_query_retry_count = 0U;
            kms_query_start_time = 0U;
        }
        kms_reset_rx();
    }
}

void kms_process_1s(void)
{
    if (kms_rx_context.active &&
        ((seconds_uptime - kms_rx_context.start_time) >
         KMS_RX_TIMEOUT_SEC))
    {
        kms_rx_timeout_count++;
        kms_reset_rx();
    }

    if (kms_query_pending &&
        ((seconds_uptime - kms_query_start_time) >=
         KMS_QUERY_TIMEOUT_SEC))
    {
        if (kms_query_retry_count < KMS_QUERY_MAX_RETRIES)
        {
            kms_query_retry_count++;
            (void)kms_transmit_auth_key_query();
            kms_query_start_time = seconds_uptime;
        }
        else
        {
            kms_query_pending = 0U;
            kms_query_retry_count = 0U;
            kms_query_start_time = 0U;
            kms_query_failure_count++;
        }
    }
}
