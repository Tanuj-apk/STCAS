#ifndef INPUT_CARD_H
#define INPUT_CARD_H

#include <stdbool.h>
#include <stdint.h>

#include "can_if.h"
#include "station_config.h"

#define INPUT_CARD_CAN_ID_BASE 0x150U
#define INPUT_CARD_MAX_COUNT   STATION_CONFIG_MAX_INPUT_CARDS
#define INPUT_CARD_CHANNELS    STATION_CONFIG_INPUT_CHANNELS_PER_CARD

#define INPUT_CARD_SEQ_TOTAL   1U
#define INPUT_CARD_SEQ_INDEX   0U

typedef struct
{
    uint32_t inputs;
} input_card_data_t;

/*
 * Application-facing meaning of a configured raw field-input bit.  A state is
 * returned only after the corresponding physical input card has supplied a
 * valid snapshot.
 */
typedef enum
{
    FIELD_INPUT_STATE_DEASSERTED = 0U,
    FIELD_INPUT_STATE_ASSERTED,
    FIELD_INPUT_STATE_NORMAL,
    FIELD_INPUT_STATE_REVERSE,
    FIELD_INPUT_STATE_CLEAR,
    FIELD_INPUT_STATE_OCCUPIED,
    FIELD_INPUT_STATE_OFF,
    FIELD_INPUT_STATE_ON
} field_input_state_t;

extern input_card_data_t input_card_data[INPUT_CARD_MAX_COUNT];
extern volatile uint32_t input_card_duplicate_count;
extern volatile uint32_t input_card_conflict_count;

/*
 * Clears received input state.  A validated station configuration must be
 * active before any input-card frame is accepted.
 */
void input_card_init(void);

void input_card_rx_handler(
    uint32_t can_id,
    uint8_t *data,
    can_source_t can_source);

/* Returns a bit for each physical input-card number configured at station. */
uint16_t input_card_get_configured_card_bitmap(void);
bool input_card_is_configured(uint8_t card_number);

/*
 * Raw-value access used by event logging and diagnostics.  A value is
 * available only after the configured physical channel has been received at
 * least once.  Callers must not interpret unavailable as zero or OFF.
 */
bool field_input_try_get_value(uint16_t logical_id, uint8_t *value);
bool field_input_try_get_value_by_index(
    uint16_t config_index,
    uint16_t *logical_id,
    uint8_t *value);

/*
 * Decoded access for application logic.  The active station configuration
 * converts the raw bit to ASSERTED/DEASSERTED, NORMAL/REVERSE,
 * CLEAR/OCCUPIED, or OFF/ON according to the input's encoding.
 */
bool field_input_try_get_state(
    uint16_t logical_id,
    field_input_state_t *state);
bool field_input_try_get_state_by_index(
    uint16_t config_index,
    uint16_t *logical_id,
    field_input_state_t *state);

#endif /* INPUT_CARD_H */
