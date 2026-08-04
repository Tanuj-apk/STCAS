#ifndef COUNTER_CARD_H
#define COUNTER_CARD_H

#include <stdint.h>

/* ============================================================
 * CAN DEFINES
 * ============================================================ */

#define COUNTER_CARD_CAN_ID        0x170U

/* ============================================================
 * OUTPUT BIT DEFINITIONS
 * ============================================================ */

#define COUNTER_SOS                0
#define COUNTER_BRAKE              1
#define COUNTER_OVERRIDE_SELECT    2
#define COUNTER_BIU_ISOLATION      3
#define COUNTER_TRIP_MODE          4

/* ============================================================
 * API
 * ============================================================ */

void counter_card_set_bit(uint8_t bit);
void counter_card_clear_bit(uint8_t bit);
uint8_t counter_card_get_status(void);
void counter_card_send(void);
void counter_card_clear_all(void);

#endif /* COUNTER_CARD_H */
