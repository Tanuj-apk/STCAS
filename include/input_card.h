#ifndef INPUT_CARD_H
#define INPUT_CARD_H

#include <stdint.h>

/* ================= CAN FILTER ================= */

#define INPUT_CARD_RX_ID      0x150U
#define INPUT_CARD_RX_MASK    0x7FCU   /* accepts 0x150�0x151 */

/* ================= CARD DEFINITIONS ================= */

#define INPUT_CARD_COUNT      2U

typedef enum
{
    INPUT_CARD_PRIMARY = 0,
    INPUT_CARD_REDUNDANT = 1
} input_card_id_t;

/* ================= INPUT BIT DEFINITIONS (32-bit unified) ================= */

/* -------- 0-15 (Old Card 1) -------- */
#define IN_COMMON_ACK_CAB1            0
#define IN_CANCEL_CAB1                1
#define IN_SOS_CAB1                   2
#define IN_LEADING_CAB1               3
#define IN_NORMAL_BRAKE_RELAY         4
#define IN_FULL_SERVICE_BRAKE_FBK     5
#define IN_EMERGENCY_BRAKE_FBK        6
#define IN_TRACTION_CUTOFF_FBK        7
#define IN_ACTIVE_CAB1                8
#define IN_ACTIVE_CAB2                9
#define IN_FORWARD_HANDLE_CAB1        10
#define IN_REVERSE_HANDLE_CAB1        11
#define IN_FORWARD_HANDLE_CAB2        12
#define IN_REVERSE_HANDLE_CAB2        13
#define IN_HORN_CUTOUT_CAB1           14
#define IN_HORN_CUTOUT_CAB2           15

/* -------- 16-31 (Old Card 2) -------- */
#define IN_COMMON_ACK_CAB2            16
#define IN_CANCEL_CAB2                17
#define IN_SOS_CAB2                   18
#define IN_LEADING_CAB2               19
#define IN_VEB_ISO_CAB1               20
#define IN_VEB_ISO_CAB2               21
#define IN_VEB_COIL_CAB1              22
#define IN_VEB_COIL_CAB2              23
#define IN_KAVACH_ISO_FBK             24
#define IN_IRU_NORMAL_CAB1            25
#define IN_IRU_NORMAL_CAB2            26
#define IN_TRACTION_CUTOFF_STATUS     27
#define IN_LEBU_FBK                   28
#define IN_KAVACH_SERVICE_STATUS      29
#define IN_TACHO_SUPPLY               30
#define IN_SPARE                      31

//! Added by Tanuj
//? Handle position used for Reverser conition
#define HANDLE_NEUTRAL   0
#define HANDLE_FORWARD   1
#define HANDLE_REVERSE   2
#define HANDLE_INVALID   3

extern uint8_t handle_state;
extern uint8_t reverse_active;
extern uint32_t reverse_start_time;

/* ================= API ================= */

void input_card_rx_handle(uint32_t can_id, uint8_t *data);

uint8_t  input_card_is_valid(input_card_id_t card);
uint32_t input_card_get_raw_bits(input_card_id_t card);
uint8_t  input_card_is_bit_set(input_card_id_t card, uint8_t bit);

#endif /* INPUT_CARD_H */
