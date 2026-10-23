#include "input_card.h"

#include <string.h>

#include "datalogging.h"

input_card_data_t input_card_data[INPUT_CARD_MAX_COUNT];

typedef struct
{
    uint8_t data[8];
    uint8_t valid;
} input_card_can_frame_ctx_t;

static input_card_can_frame_ctx_t input_card_can1[INPUT_CARD_MAX_COUNT];
static input_card_can_frame_ctx_t input_card_can2[INPUT_CARD_MAX_COUNT];
static uint8_t input_card_snapshot_valid[INPUT_CARD_MAX_COUNT];
static uint8_t input_card_dedup_initialized[INPUT_CARD_MAX_COUNT];
static uint8_t input_card_last_transaction_id[INPUT_CARD_MAX_COUNT];
static uint8_t input_card_expected_transaction_id[INPUT_CARD_MAX_COUNT];
static uint8_t input_card_last_payload[INPUT_CARD_MAX_COUNT][6];

static uint8_t field_input_values[STATION_CONFIG_MAX_FIELD_INPUTS];
static uint8_t field_input_value_valid[STATION_CONFIG_MAX_FIELD_INPUTS];
static uint16_t active_station_id;
static uint32_t active_config_revision;
static uint8_t active_config_seen;

volatile uint32_t input_card_duplicate_count;
volatile uint32_t input_card_conflict_count;

static void input_card_clear_runtime_state(void)
{
    memset(input_card_data, 0, sizeof(input_card_data));
    memset(input_card_can1, 0, sizeof(input_card_can1));
    memset(input_card_can2, 0, sizeof(input_card_can2));
    memset(input_card_snapshot_valid, 0, sizeof(input_card_snapshot_valid));
    memset(input_card_dedup_initialized, 0,
           sizeof(input_card_dedup_initialized));
    memset(input_card_last_transaction_id, 0,
           sizeof(input_card_last_transaction_id));
    memset(input_card_expected_transaction_id, 0,
           sizeof(input_card_expected_transaction_id));
    memset(input_card_last_payload, 0, sizeof(input_card_last_payload));
    memset(field_input_values, 0, sizeof(field_input_values));
    memset(field_input_value_valid, 0, sizeof(field_input_value_valid));
}

static bool input_card_sync_configuration(void)
{
    station_config_header_t header;

    if ((station_config_get_state() != STATION_CONFIG_STATE_VALID) ||
        (station_config_get_header(&header) != STATION_CONFIG_OK))
    {
        if (active_config_seen != 0U)
        {
            input_card_clear_runtime_state();
        }
        active_config_seen = 0U;
        active_station_id = 0U;
        active_config_revision = 0U;
        return false;
    }

    if ((active_config_seen == 0U) ||
        (active_station_id != header.station_id) ||
        (active_config_revision != header.revision))
    {
        input_card_clear_runtime_state();
        active_station_id = header.station_id;
        active_config_revision = header.revision;
        active_config_seen = 1U;
    }

    return true;
}

void input_card_init(void)
{
    input_card_clear_runtime_state();
    active_station_id = 0U;
    active_config_revision = 0U;
    active_config_seen = 0U;
    input_card_duplicate_count = 0U;
    input_card_conflict_count = 0U;
}

uint16_t input_card_get_configured_card_bitmap(void)
{
    uint16_t count;
    uint16_t index;
    uint16_t bitmap = 0U;
    station_config_field_input_t input;

    if ((!input_card_sync_configuration()) ||
        (station_config_get_field_input_count(&count) != STATION_CONFIG_OK))
    {
        return 0U;
    }

    for (index = 0U; index < count; index++)
    {
        if (station_config_get_field_input_by_index(index, &input) !=
            STATION_CONFIG_OK)
        {
            return 0U;
        }

        bitmap |= (uint16_t)((uint16_t)1U << (input.card_index - 1U));
    }

    return bitmap;
}

bool input_card_is_configured(uint8_t card_number)
{
    uint16_t bitmap;

    if ((card_number == 0U) || (card_number > INPUT_CARD_MAX_COUNT))
    {
        return false;
    }

    bitmap = input_card_get_configured_card_bitmap();
    return ((bitmap & ((uint16_t)1U << (card_number - 1U))) != 0U);
}

static uint8_t input_card_get_index(uint32_t can_id)
{
    uint8_t card_number;

    if ((can_id < INPUT_CARD_CAN_ID_BASE) ||
        (can_id >= (INPUT_CARD_CAN_ID_BASE + INPUT_CARD_MAX_COUNT)))
    {
        return 0xFFU;
    }

    card_number = (uint8_t)((can_id - INPUT_CARD_CAN_ID_BASE) + 1U);
    if (!input_card_is_configured(card_number))
    {
        return 0xFFU;
    }

    return (uint8_t)(card_number - 1U);
}

static void input_card_update_logical_inputs(
    uint8_t card_number,
    uint32_t inputs,
    uint32_t previous_inputs,
    uint32_t changed_inputs,
    uint8_t previous_snapshot_valid)
{
    uint16_t count;
    uint16_t config_index;
    uint8_t value;
    uint8_t previous_value;
    station_config_field_input_t input;

    if (station_config_get_field_input_count(&count) != STATION_CONFIG_OK)
    {
        return;
    }

    for (config_index = 0U; config_index < count; config_index++)
    {
        if (station_config_get_field_input_by_index(config_index, &input) !=
            STATION_CONFIG_OK)
        {
            return;
        }

        if (input.card_index != card_number)
        {
            continue;
        }

        value = (uint8_t)((inputs >> input.channel_index) & 0x01U);
        previous_value =
            (uint8_t)((previous_inputs >> input.channel_index) & 0x01U);
        field_input_values[config_index] = value;
        field_input_value_valid[config_index] = 1U;

        if ((previous_snapshot_valid != 0U) &&
            ((changed_inputs &
              ((uint32_t)1U << input.channel_index)) != 0U))
        {
            (void)Log_Event_FieldInput(input.logical_id,
                                       previous_value,
                                       value);
        }
    }
}

void input_card_rx_handler(
    uint32_t can_id,
    uint8_t *data,
    can_source_t can_source)
{
    uint8_t index;
    uint8_t seq_total;
    uint8_t seq_index;
    uint8_t transaction_id;
    uint32_t inputs;
    uint32_t previous_inputs;
    uint32_t changed_inputs;

    if ((data == NULL) || (!input_card_sync_configuration()))
    {
        return;
    }

    seq_total = (uint8_t)(((data[0] >> 4) & 0x0FU) |
                          ((data[1] & 0x03U) << 4));
    seq_index = (uint8_t)((data[1] >> 2) & 0x3FU);
    transaction_id = data[0] & 0x0FU;

    if ((seq_total != INPUT_CARD_SEQ_TOTAL) ||
        (seq_index != INPUT_CARD_SEQ_INDEX))
    {
        return;
    }

    index = input_card_get_index(can_id);
    if (index == 0xFFU)
    {
        return;
    }

    if (can_source == CAN_SOURCE_1)
    {
        if ((input_card_can2[index].valid != 0U) &&
            (memcmp(input_card_can2[index].data, data, 8U) == 0))
        {
            return;
        }
        memcpy(input_card_can1[index].data, data, 8U);
        input_card_can1[index].valid = 1U;
    }
    else if (can_source == CAN_SOURCE_2)
    {
        if ((input_card_can1[index].valid != 0U) &&
            (memcmp(input_card_can1[index].data, data, 8U) == 0))
        {
            return;
        }
        memcpy(input_card_can2[index].data, data, 8U);
        input_card_can2[index].valid = 1U;
    }
    else
    {
        return;
    }

    if (input_card_dedup_initialized[index] == 0U)
    {
        input_card_dedup_initialized[index] = 1U;
    }
    else if (transaction_id == input_card_last_transaction_id[index])
    {
        if (memcmp(input_card_last_payload[index], &data[2], 6U) == 0)
        {
            input_card_duplicate_count++;
            send_cpu_universal_ack((uint16_t)can_id, 0U, CPU_ACK_OK);
        }
        else
        {
            input_card_conflict_count++;
        }
        return;
    }
    else if (transaction_id != input_card_expected_transaction_id[index])
    {
        input_card_conflict_count++;
        return;
    }

    input_card_last_transaction_id[index] = transaction_id;
    input_card_expected_transaction_id[index] =
        (uint8_t)((transaction_id + 1U) & 0x0FU);
    memcpy(input_card_last_payload[index], &data[2], 6U);

    inputs = ((uint32_t)data[2]) |
             ((uint32_t)data[3] << 8) |
             ((uint32_t)data[4] << 16) |
             ((uint32_t)data[5] << 24);
    previous_inputs = input_card_data[index].inputs;
    changed_inputs = previous_inputs ^ inputs;
    input_card_data[index].inputs = inputs;

    send_cpu_universal_ack((uint16_t)can_id, 0U, CPU_ACK_OK);

    input_card_update_logical_inputs((uint8_t)(index + 1U),
                                     inputs,
                                     previous_inputs,
                                     changed_inputs,
                                     input_card_snapshot_valid[index]);
    input_card_snapshot_valid[index] = 1U;
}

bool field_input_try_get_value(uint16_t logical_id, uint8_t *value)
{
    uint16_t count;
    uint16_t index;
    station_config_field_input_t input;

    if ((value == NULL) || (!input_card_sync_configuration()) ||
        (station_config_get_field_input_count(&count) != STATION_CONFIG_OK))
    {
        return false;
    }

    for (index = 0U; index < count; index++)
    {
        if (station_config_get_field_input_by_index(index, &input) !=
            STATION_CONFIG_OK)
        {
            return false;
        }

        if (input.logical_id == logical_id)
        {
            if (field_input_value_valid[index] == 0U)
            {
                return false;
            }
            *value = field_input_values[index];
            return true;
        }
    }

    return false;
}

bool field_input_try_get_value_by_index(
    uint16_t config_index,
    uint16_t *logical_id,
    uint8_t *value)
{
    station_config_field_input_t input;

    if ((logical_id == NULL) || (value == NULL) ||
        (!input_card_sync_configuration()) ||
        (station_config_get_field_input_by_index(config_index, &input) !=
         STATION_CONFIG_OK) ||
        (field_input_value_valid[config_index] == 0U))
    {
        return false;
    }

    *logical_id = input.logical_id;
    *value = field_input_values[config_index];
    return true;
}

static bool field_input_decode_state(
    station_input_encoding_t encoding,
    uint8_t raw_value,
    field_input_state_t *state)
{
    if (state == NULL)
    {
        return false;
    }

    raw_value = (uint8_t)(raw_value & 0x01U);

    switch (encoding)
    {
        case STATION_INPUT_ENCODING_ONE_ASSERTED:
            *state = (raw_value != 0U) ?
                FIELD_INPUT_STATE_ASSERTED :
                FIELD_INPUT_STATE_DEASSERTED;
            break;

        case STATION_INPUT_ENCODING_ZERO_ASSERTED:
            *state = (raw_value == 0U) ?
                FIELD_INPUT_STATE_ASSERTED :
                FIELD_INPUT_STATE_DEASSERTED;
            break;

        case STATION_INPUT_ENCODING_ONE_NORMAL:
            *state = (raw_value != 0U) ?
                FIELD_INPUT_STATE_NORMAL : FIELD_INPUT_STATE_REVERSE;
            break;

        case STATION_INPUT_ENCODING_ONE_CLEAR:
            *state = (raw_value != 0U) ?
                FIELD_INPUT_STATE_CLEAR : FIELD_INPUT_STATE_OCCUPIED;
            break;

        case STATION_INPUT_ENCODING_ONE_OFF:
            *state = (raw_value != 0U) ?
                FIELD_INPUT_STATE_OFF : FIELD_INPUT_STATE_ON;
            break;

        default:
            return false;
    }

    return true;
}

bool field_input_try_get_state(
    uint16_t logical_id,
    field_input_state_t *state)
{
    uint8_t raw_value;
    station_config_field_input_t input;

    if ((state == NULL) ||
        (station_config_find_field_input_by_id(logical_id, &input) !=
         STATION_CONFIG_OK) ||
        (!field_input_try_get_value(logical_id, &raw_value)))
    {
        return false;
    }

    return field_input_decode_state(input.encoding, raw_value, state);
}

bool field_input_try_get_state_by_index(
    uint16_t config_index,
    uint16_t *logical_id,
    field_input_state_t *state)
{
    uint8_t raw_value;
    station_config_field_input_t input;

    if ((logical_id == NULL) || (state == NULL) ||
        (station_config_get_field_input_by_index(config_index, &input) !=
         STATION_CONFIG_OK) ||
        (!field_input_try_get_value_by_index(config_index,
                                              logical_id,
                                              &raw_value)))
    {
        return false;
    }

    return field_input_decode_state(input.encoding, raw_value, state);
}
