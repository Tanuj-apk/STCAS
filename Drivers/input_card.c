#include "input_card.h"

input_card_data_t input_card_data[INPUT_CARD_COUNT]; // Raw data received from Input Cards
field_input_t
    field_inputs[FIELD_INPUT_COUNT]; // Current value of each field-input key,
                                     // will eventually store Key + Value
input_mapping_t input_mapping[FIELD_INPUT_COUNT]; // will eventually tell us
                                                  // Card + Channel → Key

static uint8_t input_card_get_index(uint32_t can_id) 
{
    if (can_id == INPUT_CARD1_CAN_ID) 
    {
        return 0U;
    } 
    else if (can_id == INPUT_CARD2_CAN_ID) 
    {
        return 1U;
    } 
    else if (can_id == INPUT_CARD3_CAN_ID) 
    {
        return 2U;
    }

    return 0xFFU;
}

static void field_input_set_value(uint16_t key, uint8_t value) 
{
    field_inputs[key - 1U].key = key;
    field_inputs[key - 1U].value = value;
}
input_mapping_t input_mapping[FIELD_INPUT_COUNT] = 
{
    /* Card 1 : Channels 0-31 */

    {1U, 0U, KEY_D01_HECR},
    {1U, 1U, KEY_D01_DECR},
    {1U, 2U, KEY_D01_HHECR},
    {1U, 3U, KEY_ID01_HECR},
    {1U, 4U, KEY_ID01_DECR},
    {1U, 5U, KEY_ID01_HHECR},
    {1U, 6U, KEY_S01_RECR},
    {1U, 7U, KEY_S01_HECR},
    {1U, 8U, KEY_S01_DECR},
    {1U, 9U, KEY_ID02_HECR},
    {1U, 10U, KEY_ID02_DECR},
    {1U, 11U, KEY_ID02_HHECR},
    {1U, 12U, KEY_S02_DECR},
    {1U, 13U, KEY_S02_HECR},
    {1U, 14U, KEY_S02_RECR},
    {1U, 15U, KEY_S04_DECR},
    {1U, 16U, KEY_S04_HECR},
    {1U, 17U, KEY_S04_RECR},
    {1U, 18U, KEY_S06_HECR},
    {1U, 19U, KEY_S06_RECR},
    {1U, 20U, KEY_UP_IBS_RECR},
    {1U, 21U, KEY_UP_IBS_DECR},
    {1U, 22U, KEY_S05_RECR},
    {1U, 23U, KEY_S05_HECR},
    {1U, 24U, KEY_ID11_HECR},
    {1U, 25U, KEY_ID11_DECR},
    {1U, 26U, KEY_S11_RECR},
    {1U, 27U, KEY_S11_DECR},
    {1U, 28U, KEY_S08_DECR},
    {1U, 29U, KEY_S08_RECR},
    {1U, 30U, KEY_S07_RECR},
    {1U, 31U, KEY_S07_HECR},

    /* Card 2 : Channels 0-31 */

    {2U, 0U, KEY_S09_D11_RECR},
    {2U, 1U, KEY_S09_D11_HECR},
    {2U, 2U, KEY_S09_D11_DECR},
    {2U, 3U, KEY_S09_D11_HHECR},
    {2U, 4U, KEY_S03_RECR},
    {2U, 5U, KEY_S03_HECR},
    {2U, 6U, KEY_S03_DECR},
    {2U, 7U, KEY_S03_HHECR},
    {2U, 8U, KEY_D02_HECR},
    {2U, 9U, KEY_D02_DECR},
    {2U, 10U, KEY_D02_HHECR},
    {2U, 11U, KEY_101A},
    {2U, 12U, KEY_104A},
    {2U, 13U, KEY_104B},
    {2U, 14U, KEY_102B},
    {2U, 15U, KEY_102A},
    {2U, 16U, KEY_106B},
    {2U, 17U, KEY_103B},
    {2U, 18U, KEY_103A},
    {2U, 19U, KEY_106A},
    {2U, 20U, KEY_101B},
    {2U, 21U, KEY_105B},
    {2U, 22U, KEY_105A},
    {2U, 23U, KEY_C01T},
    {2U, 24U, KEY_04AT},
    {2U, 25U, KEY_04BT},
    {2U, 26U, KEY_104AT},
    {2U, 27U, KEY_H01T},
    {2U, 28U, KEY_101BT},
    {2U, 29U, KEY_09T},
    {2U, 30U, KEY_03AT},
    {2U, 31U, KEY_03BT},

    /* Card 3 : Channels 0-28 */

    {3U, 0U, KEY_104BT},
    {3U, 1U, KEY_102BT},
    {3U, 2U, KEY_03_07T},
    {3U, 3U, KEY_08T},
    {3U, 4U, KEY_H02T},
    {3U, 5U, KEY_04_06T},
    {3U, 6U, KEY_102AT},
    {3U, 7U, KEY_106BT},
    {3U, 8U, KEY_02BT},
    {3U, 9U, KEY_02AT},
    {3U, 10U, KEY_103BT},
    {3U, 11U, KEY_103AT},
    {3U, 12U, KEY_01BT},
    {3U, 13U, KEY_01AT},
    {3U, 14U, KEY_105T},
    {3U, 15U, KEY_106AT},
    {3U, 16U, KEY_101AT},
    {3U, 17U, KEY_C02T},
    {3U, 18U, KEY_11AC},
    {3U, 19U, KEY_SH43},
    {3U, 20U, KEY_SH41},
    {3U, 21U, KEY_SH42},
    {3U, 22U, KEY_C01},
    {3U, 23U, KEY_C02},
    {3U, 24U, KEY_RS1},
    {3U, 25U, KEY_RS2},
    {3U, 26U, KEY_RS3},
    {3U, 27U, KEY_LC8},
    {3U, 28U, KEY_LC9}
};

void input_card_rx_handler(uint32_t can_id, uint8_t *data) 
{
    uint8_t index;

    uint8_t pkt_type;
    uint8_t seq_total;
    uint8_t seq_index;

    uint32_t inputs;
    uint8_t card;
    uint8_t channel;
    uint16_t key;

    pkt_type = data[0] & 0x0F;

    seq_total = ((data[0] >> 4) & 0x0F) | ((data[1] & 0x03) << 4);

    seq_index = (data[1] >> 2) & 0x3F;

    if (seq_total != 1U) 
    {
        return;
    }

    if (seq_index != 0U) 
    {
        return;
    }

    index = input_card_get_index(can_id);

    if (index == 0xFFU) 
    {
        return;
    }

    input_card_data[index].inputs =
        ((uint32_t)data[2]) | ((uint32_t)data[3] << 8) |
        ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 24);

    card = index + 1U;

    for (uint16_t i = 0U; i < FIELD_INPUT_COUNT; i++) 
    {
        if (input_mapping[i].card != card) 
        {
            continue;
        }

        channel = input_mapping[i].channel;
        key = input_mapping[i].key;

        /*
        * Extract the corresponding channel bit.
        */
        field_input_set_value(key, (uint8_t)((inputs >> channel) & 0x01U));
    }
}

uint8_t field_input_get_value(uint16_t key) 
{
    return field_inputs[key - 1U].value;
}