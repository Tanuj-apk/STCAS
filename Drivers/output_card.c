#include "output_card.h"
#include "can.h"

/* ============================================================
 * INTERNAL STATE
 * ============================================================ */

/* Shadow register for outputs */
static uint16_t output_shadow = 0U;

/* ============================================================
 * INTERNAL TX HELPER
 * ============================================================ */

static void output_card_tx(uint16_t bits)
{
    uint8_t tx_buf[8] = {0};

    tx_buf[0] = MSG_TYPE_OUTPUT_CARD_CMD;

    /* Outputs 1–8 */
    tx_buf[1] = (uint8_t)(bits & 0xFFU);

    /* Outputs 9–16 */
    tx_buf[2] = (uint8_t)((bits >> 8) & 0xFFU);

    /* Bytes 3–7 reserved = 0 */

    canTransmit(canREG1, canMESSAGE_BOX11, tx_buf);
}

/* ============================================================
 * API IMPLEMENTATION
 * ============================================================ */

void output_card_set_bit(uint8_t bit)
{
    if (bit >= 16U)
        return;

    output_shadow |= (1U << bit);
}

void output_card_clear_bit(uint8_t bit)
{
    if (bit >= 16U)
        return;

    output_shadow &= ~(1U << bit);
}

void output_card_set_raw(uint16_t bits)
{
    output_shadow = bits;
}

uint16_t output_card_get_raw(void)
{
    return output_shadow;
}

void output_card_send(void)
{
    output_card_tx(output_shadow);
}

void output_card_all_off(void)
{
    output_shadow = 0U;
    output_card_tx(output_shadow);
}
