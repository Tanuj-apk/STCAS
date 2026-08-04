#ifndef INCLUDE_STATEMACHINE_H_
#define INCLUDE_STATEMACHINE_H_

#include <stdint.h>
#include <stdbool.h>
/*
 * CONDITION NUMBERING RULE:
 *
 * Condition numbers start from 1 (NOT 0)
 *
 * Internally:
 *   condition n ? bit (n-1)
 *
 * This is why all macros use (n-1)
 */

/**
 * @brief Condition mask structure
 *
 * Represents up to 128 boolean conditions using bitfields.
 *
 * Layout:
 *   bits[0] ? Conditions 1 to 64
 *   bits[1] ? Conditions 65 to 128
 *
 * Each condition is stored as a single bit:
 *   1 ? condition is TRUE
 *   0 ? condition is FALSE
 *
 * Example:
 *   Condition 1  ? bits[0], bit 0
 *   Condition 10 ? bits[0], bit 9
 *   Condition 65 ? bits[1], bit 0
 */
typedef struct
{
    uint64_t bits[2];   // supports 128 conditions (1 bit per condition)
} cond_mask_t;

/**
 * @brief Set a condition bit (mark condition as TRUE)
 *
 * @param m Condition mask
 * @param n Condition number (1�128)
 *
 * Operation:
 *   - Finds correct 64-bit word
 *   - Sets corresponding bit to 1
 *
 * Example:
 *   C_SET(mask, 10); ? sets condition 10
 * 
*    C_CLR(mask, 10); ? clears condition 10
 */
#define C_SET(m, n)   ((m).bits[((n)-1)/64] |=  (1ULL << (((n)-1)%64)))
#define C_CLR(m, n)   ((m).bits[((n)-1)/64] &= ~(1ULL << (((n)-1)%64)))

/**
 * @brief Test if a condition is TRUE or FALSE
 *
 * @param m Condition mask
 * @param n Condition number (1�128)
 *
 * @return 1 if condition is TRUE, 0 if FALSE
 *
 * Example:
 *   if (C_TEST(mask, 10)) ? checks condition 10
 */
#define C_TEST(m, n)  (((m).bits[((n)-1)/64] >> (((n)-1)%64)) & 1ULL)

/* ================= STATES ================= */
typedef enum {
    STATE_SB = 1,
    STATE_SR,
    STATE_LS,
    STATE_FS,
    STATE_OV,
    STATE_OS,
    STATE_TR,
    STATE_PT,
    STATE_RV,
    STATE_SH,
    STATE_NL,
    STATE_SF,
    STATE_IS,
    STATE_MAX
} State_t;

/* ================= INPUT ================= */
typedef struct {
    uint32_t raw_flags[4]; // 128 possible inputs // ? Total inputs = 4 � 32 = 128 signals
} Input_t;

/* ================= GLOBALS ================= */
extern volatile Input_t input_write;
extern Input_t input_read;

extern State_t g_current;
extern State_t g_previous;

extern volatile uint32_t input_seq;

/* ================= TRANSITION ================= */
typedef struct {
    uint8_t from;
    uint8_t to;
    uint8_t priority;
    cond_mask_t cond;
} Transition_t;

typedef struct {
    const Transition_t *list;
    uint8_t count;
} StateTable_t;

extern StateTable_t state_table[STATE_MAX];

/* ================= API ================= */
void input_swap(void);
cond_mask_t compute_conditions(void);
void fsm_init(void);
State_t fsm_step(State_t current, cond_mask_t mask);

#endif
