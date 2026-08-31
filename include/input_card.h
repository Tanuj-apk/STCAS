#ifndef INPUT_CARD_H
#define INPUT_CARD_H

#include <stdint.h>
#include "can_if.h"

#define INPUT_CARD1_CAN_ID 0x150U
#define INPUT_CARD2_CAN_ID 0x151U
#define INPUT_CARD3_CAN_ID 0x152U

#define INPUT_CARD_COUNT 3U
#define INPUT_CARD_CHANNELS 32U

#define INPUT_CARD_SEQ_TOTAL 1U
#define INPUT_CARD_SEQ_INDEX 0U

typedef struct 
{
  uint32_t inputs;
} input_card_data_t;

#define FIELD_INPUT_COUNT 93U

// * Numeric Key IDs
//!=================== Currently Hardcoded =========================
/* ============================================================
 * SIGNAL / ASPECT RELAYS : 1 - 43
 * ============================================================ */

#define KEY_D01_HECR 0x0001U
#define KEY_D01_DECR 0x0002U
#define KEY_D01_HHECR 0x0003U
#define KEY_ID01_HECR 0x0004U
#define KEY_ID01_DECR 0x0005U
#define KEY_ID01_HHECR 0x0006U
#define KEY_S01_RECR 0x0007U
#define KEY_S01_HECR 0x0008U
#define KEY_S01_DECR 0x0009U
#define KEY_ID02_HECR 0x000AU
#define KEY_ID02_DECR 0x000BU
#define KEY_ID02_HHECR 0x000CU
#define KEY_S02_DECR 0x000DU
#define KEY_S02_HECR 0x000EU
#define KEY_S02_RECR 0x000FU
#define KEY_S04_DECR 0x0010U
#define KEY_S04_HECR 0x0011U
#define KEY_S04_RECR 0x0012U
#define KEY_S06_HECR 0x0013U
#define KEY_S06_RECR 0x0014U
#define KEY_UP_IBS_RECR 0x0015U
#define KEY_UP_IBS_DECR 0x0016U
#define KEY_S05_RECR 0x0017U
#define KEY_S05_HECR 0x0018U
#define KEY_ID11_HECR 0x0019U
#define KEY_ID11_DECR 0x001AU
#define KEY_S11_RECR 0x001BU
#define KEY_S11_DECR 0x001CU
#define KEY_S08_DECR 0x001DU
#define KEY_S08_RECR 0x001EU
#define KEY_S07_RECR 0x001FU
#define KEY_S07_HECR 0x0020U
#define KEY_S09_D11_RECR 0x0021U
#define KEY_S09_D11_HECR 0x0022U
#define KEY_S09_D11_DECR 0x0023U
#define KEY_S09_D11_HHECR 0x0024U
#define KEY_S03_RECR 0x0025U
#define KEY_S03_HECR 0x0026U
#define KEY_S03_DECR 0x0027U
#define KEY_S03_HHECR 0x0028U
#define KEY_D02_HECR 0x0029U
#define KEY_D02_DECR 0x002AU
#define KEY_D02_HHECR 0x002BU

/* ============================================================
 * POINTS : 44 - 55
 * ============================================================ */

#define KEY_101A 0x002CU
#define KEY_104A 0x002DU
#define KEY_104B 0x002EU
#define KEY_102B 0x002FU
#define KEY_102A 0x0030U
#define KEY_106B 0x0031U
#define KEY_103B 0x0032U
#define KEY_103A 0x0033U
#define KEY_106A 0x0034U
#define KEY_101B 0x0035U
#define KEY_105B 0x0036U
#define KEY_105A 0x0037U

/* ============================================================
 * TRACK CIRCUITS : 56 - 82
 * ============================================================ */

#define KEY_C01T 0x0038U
#define KEY_04AT 0x0039U
#define KEY_04BT 0x003AU
#define KEY_104AT 0x003BU
#define KEY_H01T 0x003CU
#define KEY_101BT 0x003DU
#define KEY_09T 0x003EU
#define KEY_03AT 0x003FU
#define KEY_03BT 0x0040U
#define KEY_104BT 0x0041U
#define KEY_102BT 0x0042U
#define KEY_03_07T 0x0043U
#define KEY_08T 0x0044U
#define KEY_H02T 0x0045U
#define KEY_04_06T 0x0046U
#define KEY_102AT 0x0047U
#define KEY_106BT 0x0048U
#define KEY_02BT 0x0049U
#define KEY_02AT 0x004AU
#define KEY_103BT 0x004BU
#define KEY_103AT 0x004CU
#define KEY_01BT 0x004DU
#define KEY_01AT 0x004EU
#define KEY_105T 0x004FU
#define KEY_106AT 0x0050U
#define KEY_101AT 0x0051U
#define KEY_C02T 0x0052U

/* ============================================================
 * AXLE COUNTER : 83
 * ============================================================ */

#define KEY_11AC 0x0053U

/* ============================================================
 * SHUNT SIGNALS : 84 - 86
 * ============================================================ */

#define KEY_SH43 0x0054U
#define KEY_SH41 0x0055U
#define KEY_SH42 0x0056U

/* ============================================================
 * CALLING-ON : 87 - 88
 * ============================================================ */

#define KEY_C01 0x0057U
#define KEY_C02 0x0058U

/* ============================================================
 * ROUTE INDICATORS : 89 - 91
 * ============================================================ */

#define KEY_RS1 0x0059U
#define KEY_RS2 0x005AU
#define KEY_RS3 0x005BU

/* ============================================================
 * LC GATES : 92 - 93
 * ============================================================ */

#define KEY_LC8 0x005CU
#define KEY_LC9 0x005DU
//!=============================================================

typedef struct 
{
  uint16_t key;
  uint8_t value;
} field_input_t;

typedef struct 
{
  uint8_t card;
  uint8_t channel;
  uint16_t key;
} input_mapping_t;

void input_card_rx_handler(uint32_t can_id, uint8_t *data, can_source_t can_source);
uint8_t field_input_get_value(uint16_t key);

#endif /* INPUT_CARD_H */
