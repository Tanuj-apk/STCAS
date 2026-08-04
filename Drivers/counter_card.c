#include "counter_card.h"
#include "can.h"

/* ============================================================
 * INTERNAL STATE
 * ============================================================ */

/* Shadow register for outputs */
static uint16_t counter_status = 0U;

/* ============================================================
 * INTERNAL TX HELPER
 * ============================================================ */

static void counter_card_tx(uint8_t status)
{
    uint8_t tx_buf[8] = {0};

    /* Byte 0 contains all valid data */
    tx_buf[0] = status;

    /* Bytes 3–7 reserved = 0 */

    canTransmit(canREG1, canMESSAGE_BOX15, tx_buf);
}

/* ============================================================
 * API IMPLEMENTATION
 * ============================================================ */

void counter_card_set_bit(uint8_t bit)
{
    if (bit >= 4U)
        return;

    counter_status |= (1U << bit);
}

void counter_card_clear_bit(uint8_t bit)
{
    if (bit >= 4U)
        return;

    counter_status &= ~(1U << bit);
}

uint8_t counter_card_get_status(void)
{
    return counter_status;
}

void counter_card_send(void)
{
    counter_card_tx(counter_status);
}

void counter_card_clear_all(void)
{
    counter_status = 0U;
    counter_card_tx(counter_status);
}
