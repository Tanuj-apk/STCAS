
#ifndef DMI_CAN_H
#define DMI_CAN_H

#include <stdint.h>

void DMI_init(void);
void DMI_update(void);
void dmi_rx_pilot_handle(uint32_t can_id, uint8_t *data);
extern volatile uint8_t isr_flag;

/* ---------------- Normal Tag ---------------- */
typedef enum
{
    NO_TRAIN=0,
    LIGHT_ENGINE,
    LIGHT_ENGINE_MULTI,
    GOODS_BCN,
    GOODS_BCNHL,
    GOODS_BOXNHL,
    GOODS_BTPN,
    GOODS_BTPGLN,
    GOODS_BLCS,
    GOODS_BRN,
    GOODS_BOBRN,
    GOODS_BCFCM,
    GOODS_BTAP,
    PASSENGER_ICF,
    PASSENGER_LHB,
    EMU,
    TRAIN_SET,
    PARCEL
}train_type_t;

typedef enum
{
    MODE_SB = 0,
    MODE_SR,
    MODE_REV,
    MODE_FS,
    MODE_LS,
    MODE_OV,
    MODE_SH,
    MODE_TR,
    MODE_PT,
    MODE_ISO,
    MODE_SF,
    MODE_OS,
    MODE_NL
} SystemMode_t;

typedef struct
{
    // Header signals
    uint8_t  pkt_type;      // byte 0, bits 3:0  â€�  4 bits
    uint8_t  seq_total_h;   // byte 0, bits 7:4  â€�  4 bits
    uint8_t  seq_total;     // byte 1, bits 1:0  â€�  2 bits
    uint8_t  seq_index;     // byte 1, bits 7:2  â€�  6 bits

    // Frame 0 payload
    uint8_t  spd;           // buf[0]       â€�  8 bits, max 240
    uint8_t  lid;           // buf[1] 5:0   â€�  6 bits, max 63
    uint32_t abloc;         // buf[1] 7:6 + buf[2] + buf[3] + buf[4] 4:0 = 23 bits
    uint8_t  sl;            // buf[4] 7:5 + buf[5] 4:0 = 8 bits, max 240
    uint8_t  ss;            // buf[5] 7:5 + buf[6] 4:0 = 8 bits, max 240

    // Frame 1 payload
    uint16_t tl;            // buf[6] 7:5 + buf[7] 7:0 = 11 bits
    uint8_t  mode;          // buf[8] 3:0  â€�  4 bits, 13 modes
    uint32_t ma;            // buf[8] 7:4 + buf[9] + buf[10] 3:0 = 16 bits
    uint8_t  target;        // buf[10] 6:4 â€�  3 bits
    uint32_t dist;          // buf[10] 7 + buf[11] + buf[12] 6:0 = 16 bits

    // Frame 2 payload
    uint32_t sd;            // buf[12] 7 + buf[13] + buf[14] 5:0 = 15 bits
    uint32_t sm;            // buf[14] 7:6 + buf[15] + buf[16] 6:0 = 17 bitsds
    uint8_t  sa;            // buf[16] 7 + buf[17] 4:0 = 6 bits
    uint8_t  rssi;          // buf[17] 7:5 â€�  3 bits, 0-5

    // Frame 3 payload
    uint16_t tid;           // buf[18] + buf[19] 1:0 = 10 bits
    uint8_t  dir;           // buf[19] 3:2 â€�  2 bits
    uint16_t td;            // buf[19] 7:4 + buf[20] 6:0 = 11 bits
    uint8_t  tr;            // buf[20] 7 + buf[21] 0 = 2 bits
    uint8_t  um;            // buf[21] 7:1 â€�  7 bits, 70 messages
    uint8_t  flt;           // buf[22] 3:0 â€�  4 bits, 14 messages
    uint16_t dc;            // buf[22] 7:4 + buf[23] 5:0 = 10 bits
    uint8_t  brake;     // buf[23] 7:6 + buf[24] 0:0 = 3

    // Frame 4 playload
    uint8_t ns;         //
    uint8_t ri;
} CANSignals_t;



//Transmit Data to CPU
typedef struct
{
    train_type_t train_type;
    uint8_t wagons;
    uint32_t length;
    uint32_t weight;
    uint8_t state;
    uint8_t brake;
} Train_data_t;

typedef struct
{
    SystemMode_t mode;

} mode_data_t;

extern mode_data_t mode_selected;

extern Train_data_t train_configuration;
extern CANSignals_t dmi_tags_signals;
extern float actual_length;
extern float actual_weight;

//! Added by Tanuj
extern uint8_t override_active;
extern uint32_t override_start_time;

#endif
