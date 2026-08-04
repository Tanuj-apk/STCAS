#include <stdio.h>
#include <stdio.h>
#include <stdint.h>

typedef struct
{

    // Frame 0 payload
    uint8_t  spd;           // buf[0]       — 8 bits, max 240
    uint8_t  lid;           // buf[1] 5:0   — 6 bits, max 63
    uint32_t abloc;         // buf[1] 7:6 + buf[2] + buf[3] + buf[4] 4:0 = 23 bits
    uint8_t  sl;            // buf[4] 7:5 + buf[5] 4:0 = 8 bits, max 240
    uint8_t  ss;            // buf[5] 7:5 + buf[6] 4:0 = 8 bits, max 240

    // Frame 1 payload
    uint16_t tl;            // buf[6] 7:5 + buf[7] 7:0 = 11 bits
    uint8_t  mode;          // buf[8] 3:0  — 4 bits, 13 modes
    uint32_t ma;            // buf[8] 7:4 + buf[9] + buf[10] 3:0 = 16 bits
    uint8_t  target;        // buf[10] 6:4 — 3 bits
    uint32_t dist;          // buf[10] 7 + buf[11] + buf[12] 6:0 = 16 bits

    // Frame 2 payload
    uint32_t sd;            // buf[12] 7 + buf[13] + buf[14] 5:0 = 15 bits
    uint32_t sm;            // buf[14] 7:6 + buf[15] + buf[16] 6:0 = 17 bitsds
    uint8_t  sa;            // buf[16] 7 + buf[17] 4:0 = 6 bits
    uint8_t  rssi;          // buf[17] 7:5 — 3 bits, 0-5

    // Frame 3 payload
    uint16_t tid;           // buf[18] + buf[19] 1:0 = 10 bits
    uint8_t  dir;           // buf[19] 3:2 — 2 bits
    uint16_t td;            // buf[19] 7:4 + buf[20] 6:0 = 11 bits
    uint8_t  tr;            // buf[20] 7 + buf[21] 0 = 2 bits
    uint8_t  um;            // buf[21] 7:1 — 7 bits, 70 messages
    uint8_t  flt;           // buf[22] 3:0 — 4 bits, 14 messages
    uint16_t dc;            // buf[22] 7:4 + buf[23] 5:0 = 10 bits
    uint8_t  brake;     // buf[23] 7:6 + buf[24] 0:0 = 3

    // Frame 4 playload
    uint8_t ns;         //
    uint8_t ri;




} DmiSharedData_t;

extern volatile DmiSharedData_t dmidata;
