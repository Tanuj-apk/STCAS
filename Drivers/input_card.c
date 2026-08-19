#include "input_card.h"
//#include "StateMachine.h"
#include "gps.h"
/* ================= INTERNAL CONTEXT ================= */
//! Added by Tanuj
uint8_t reverse_active = 0;
uint32_t reverse_start_time = 0;

typedef struct
{
    uint32_t inputs;
    uint8_t  valid;
    uint8_t  seq;
} input_card_ctx_t;

static input_card_ctx_t input_cards[INPUT_CARD_COUNT];

/* ================= INTERNAL HELPERS ================= */
/*
 * This function maps a received CAN ID to the corresponding
 * input card index used internally.
 */
static uint8_t card_index_from_can_id(uint32_t can_id)
{
    return (uint8_t)(can_id - INPUT_CARD_RX_ID); // 0x150 ? 0, 0x151 ? 1
}

/* ================= RX HANDLER ================= */

void input_card_rx_handle(uint32_t can_id, uint8_t *data)
{
/*
* CAN PAYLOAD FORMAT (INPUT CARD)
*
* Byte 0 : Card ID (1 = Primary, 2 = Redundant)
* Byte 1 : Input bits 0–7
?    Bit 0	COMMON/ACK-CAB1.DMI
?    Bit 1	CANCEL-CAB1.DMI
?    Bit 2	SOS-CAB1.DMI
?    Bit 3	LEADING/NON-LEADING-CAB1
?    Bit 4	NORMAL BRAKE RELAY FBK
?    Bit 5	FULL SERVICE BRAKE FBK
?    Bit 6	EMERGENCY BRAKE FBK
?    Bit 7	TRACTION CUTOFF FBK
* Byte 2 : Input bits 8–15
?    Bit 8	ACTIVE-CAB1
?    Bit 9	ACTIVE-CAB2
?    Bit 10	FORWARD HANDLE-CAB1
?    Bit 11	REVERSE HANDLE-CAB1
?    Bit 12	FORWARD HANDLE-CAB2
?    Bit 13	REVERSE HANDLE-CAB2
?    Bit 14	HORN CUTOUT FBK-CAB1
?    Bit 15	HORN CUTOUT FBK-CAB2
* Byte 3 : Input bits 16–23
?    Bit 16	COMMON/ACK-CAB2.DMI
?    Bit 17	CANCEL-CAB2.DMI
?    Bit 18	SOS-CAB2.DMI
?    Bit 19	LEADING/NON-LEADING-CAB2
?    Bit 20	VEB ISO CUTOUT-CAB1
?    Bit 21	VEB ISO CUTOUT-CAB2
?    Bit 22	VEB COIL FBK-CAB1
?    Bit 23	VEB COIL FBK-CAB2
* Byte 4 : Input bits 24–31
?    Bit 24	KAVACH ISOLATION FBK
?    Bit 25	IRU NORMAL FBK-CAB1
?    Bit 26	IRU NORMAL FBK-CAB2
?    Bit 27	TRACTION CUTOFF STATUS
?    Bit 28	LIGHT ENGINE BRAKE FBK
?    Bit 29	KAVACH SERVICE STATUS
?    Bit 30	TACHO POWER SUPPLY
?    Bit 31	SPARE
* Byte 5 : Sequence Counter
* Byte 6 : Health / Status
* Byte 7 : Reserved
*/
    uint8_t idx;

    /* Map CAN ID to internal card index */
    idx = card_index_from_can_id(can_id);

    /* Safety: ignore invalid index */
    if (idx >= INPUT_CARD_COUNT)
        return;

    /* Validate card identity (payload vs CAN ID) */
    if (data[0] != (idx + 1U))
        return;

    /* Extract 32-bit input value (little-endian) */
    uint32_t in =
    (uint32_t)data[1] |
    ((uint32_t)data[2] << 8) |
    ((uint32_t)data[3] << 16) |
    ((uint32_t)data[4] << 24);

    input_cards[idx].inputs = in;

    /* Store sequence counter */
    input_cards[idx].seq = data[5];

    /* Mark this card as valid (data received) */
    input_cards[idx].valid = 1;

    /* =========================================
    * UPDATE GLOBAL INPUT FLAGS (PRIMARY ONLY)
    * ========================================= */

    if (idx == INPUT_CARD_PRIMARY)
    {
        /* -------- LEADING / NON-LEADING -------- */

//        uint8_t cab1_leading  = (in >> IN_LEADING_CAB1) & 1U;
//        uint8_t cab2_leading = (in >> IN_LEADING_CAB2) & 1U;

//        /* Clear previous bits (important) */
//        input_write.raw_flags[0] &= ~((1U << 2) | (1U << 3));
//
//        if (cab1_leading && cab2_leading)
//        {
//            input_write.raw_flags[0] |= (1U << 3);   // Condition 4
//        }
//        else if (!cab1_leading && !cab2_leading)
//        {
//            input_write.raw_flags[0] |= (1U << 2);   // Condition 3
//        }
//        else
//        {
//            // TODO: fault handling
//            // mismatch ? handle later
//        }

        /* -------- CAB OCCUPANCY LOGIC -------- */

//        uint8_t cab1_active  = (in >> IN_ACTIVE_CAB1) & 1U;
//        uint8_t cab2_active = (in >> IN_ACTIVE_CAB2) & 1U;
//        static uint8_t prev_active_cab = 0;

        /* Clear relevant bits */
//        input_write.raw_flags[0] &= ~((1U << 6) | (1U << 7) | (1U << 8));
//
//        /* Case 1: No CAB active */
//        if (!cab1_active && !cab2_active)
//        {
//            input_write.raw_flags[0] |= (1U << 6); // Condition 7
//            prev_active_cab = 0;
//        }
//
//        /* Case 2: Exactly one CAB active */
//        else if (cab1_active ^ cab2_active)
//        {
//            input_write.raw_flags[0] |= (1U << 7); // Condition 8
//
//            uint8_t current_cab = cab1_active ? 1 : 2;
//
//            /* Detect CAB change */
//            if (prev_active_cab != 0 && prev_active_cab != current_cab)
//            {
//                input_write.raw_flags[0] |= (1U << 8);  // CAB changed Condition 9
//            }
//
//            prev_active_cab = current_cab;
//        }

//        /* Case 3: Both active ? Fault */
//        else
//        {
//            // TODO: fault handling
//            // Example:
//            // system_faults |= FAULT_BOTH_CABS_ACTIVE;
//        }

        //!Added by Tanuj
//        uint8_t fwd_cab1 = (in >> IN_FORWARD_HANDLE_CAB1) & 1U;
//        uint8_t rev_cab1 = (in >> IN_REVERSE_HANDLE_CAB1) & 1U;
//
//        uint8_t fwd_cab2 = (in >> IN_FORWARD_HANDLE_CAB2) & 1U;
//        uint8_t rev_cab2 = (in >> IN_REVERSE_HANDLE_CAB2) & 1U;

//        uint8_t fwd = 0;
//        uint8_t rev = 0;

//        if (cab1_active && !cab2_active)
//        {
//            fwd = fwd_cab1;
//            rev = rev_cab1;
//        }
//        else if (cab2_active && !cab1_active)
//        {
//            fwd = fwd_cab2;
//            rev = rev_cab2;
//        }
//        else
//        {
//            fwd = 0;
//            rev = 0;
//        }

//        uint8_t handle_state;
//
//        if (fwd == 0 && rev == 0)
//        {
//            handle_state = HANDLE_NEUTRAL;
//        }
//        else if (fwd == 1 && rev == 0)
//        {
//            handle_state = HANDLE_FORWARD;
//        }
//        else if (fwd == 0 && rev == 1)
//        {
//            handle_state = HANDLE_REVERSE;
//            reverse_active = 1;
//        }
//        else
//        {
//            handle_state = HANDLE_INVALID;  // both 1 - fault
//        }
//        if (handle_state != HANDLE_REVERSE && reverse_active == 1)
//        {
//            reverse_active = 0;
//            input_write.raw_flags[1] |= (1U << 8);  // Reverse condition
//        }

        /* =========================================
        * KAVACH ISOLATION LOGIC
        * ========================================= */

//        uint8_t kavach_iso = (in >> IN_KAVACH_ISO_FBK) & 1U;

//        /* Clear relevant bits first */
//        input_write.raw_flags[0] &= ~((1U << 0) | (1U << 1));
//
//        if (kavach_iso)
//        {
//            input_write.raw_flags[0] |= (1U << 1);   // Condition 2: isolated
//        }
//        else
//        {
//            input_write.raw_flags[0] |= (1U << 0);   // Condition 1: not isolated
//        }
    }
}

/* ================= QUERY API ================= */
/*
 * Returns whether at least one valid CAN frame has been received from the specified input card.
 *         1 ? valid data available
 *         0 ? no data received yet
 */
//! Currently Unused
uint8_t input_card_is_valid(input_card_id_t card)
{
    if (card >= INPUT_CARD_COUNT)
        return 0;

    return input_cards[card].valid;
}
/*
 * Returns the latest snapshot of all 32 digital inputs received from the specified card.
*/
//! Currently Unused
uint32_t input_card_get_raw_bits(input_card_id_t card)
{
    if (card >= INPUT_CARD_COUNT)
        return 0;

    return input_cards[card].inputs;
}

/*
 * This function checks whether a particular input signal
 * (bit position) is active (1) or inactive (0).
 */
//! Currently Unused
uint8_t input_card_is_bit_set(input_card_id_t card, uint8_t bit)
{
    if (card >= INPUT_CARD_COUNT || bit >= 32)
        return 0;

    return (input_cards[card].inputs >> bit) & 0x01U;
}
