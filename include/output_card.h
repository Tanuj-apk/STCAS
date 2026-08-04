#ifndef OUTPUT_CARD_H
#define OUTPUT_CARD_H

#include <stdint.h>

/* ============================================================
 * CAN DEFINES
 * ============================================================ */

#define OUTPUT_CARD_CAN_ID        0x170U
#define MSG_TYPE_OUTPUT_CARD_CMD  0x50U

/* ============================================================
 * OUTPUT BIT DEFINITIONS
 * ============================================================ */

/* OUTPUT_BITS_L (Byte 1) */
#define OUT_NORMAL_BRAKE              0
#define OUT_FULL_SERVICE_BRAKE        1
#define OUT_EMERGENCY_BRAKE_1         2
#define OUT_EMERGENCY_BRAKE_2         3
#define OUT_LIGHT_ENGINE_BRAKE        4
#define OUT_HORN                      5
#define OUT_TRACTION_CUTOFF           6
#define OUT_PVEF_DISABLE_RELAY        7

/* OUTPUT_BITS_H (Byte 2) */
#define OUT_ACTIVE_CAB1               8
#define OUT_ACTIVE_CAB2               9
#define OUT_VEB_ISO_CAB1              10
#define OUT_VEB_ISO_CAB2              11
#define OUT_VEB_COIL_STATUS_CAB1      12
#define OUT_VEB_COIL_STATUS_CAB2      13
#define OUT_KAVACH_ISO_STATUS         14
#define OUT_TACHO_MONITOR_SUPPLY      15

/* ============================================================
 * API
 * ============================================================ */

/* Set / clear individual outputs */
void output_card_set_bit(uint8_t bit);
void output_card_clear_bit(uint8_t bit);

/* Write full 16-bit output bitmap */
void output_card_set_raw(uint16_t bits);

/* Get current CPU output shadow */
uint16_t output_card_get_raw(void);

/* Transmit current output state on CAN */
void output_card_send(void);

/* Force all outputs OFF (safety) */
void output_card_all_off(void);

#endif /* OUTPUT_CARD_H */
