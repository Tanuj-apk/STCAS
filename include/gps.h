#ifndef GPS_H
#define GPS_H

#include "sys_common.h"
#include "sci.h"

#define GPS1_SCI scilinREG
#define GPS2_SCI sciREG

extern uint32_t radio_latitude;
extern uint32_t radio_longitude;
/* ---------- Types ---------- */

typedef enum {
    GPS_SEL_NONE = 0,
    GPS_SEL_GPS1,
    GPS_SEL_GPS2
} gps_sel_t;

typedef struct {
    gps_sel_t sel;
    uint8_t   nms_fault;
    uint8_t   faults;
} gps_decision_t;

#pragma pack(1)
typedef struct
{
    uint8_t  sync1;              // 0xA5
    uint8_t  sync2;              // 0x5A
    uint8_t  length;             // should be 24

    uint8_t  card_id;

    uint16_t gps_week;           // little-endian on wire
    uint32_t gps_tow_ms;         // little-endian
    uint32_t gps_system_seconds; // (week*604800 + tow_sec) from GPS-TIVA

    int32_t latitude;
    int32_t longitude;
    
    uint32_t tAcc_ns;

    uint8_t  numSV;
    uint8_t  gps_fix_ok;
    uint8_t  pps_ok;

    uint16_t status_flags;       // little-endian
    uint16_t crc16;              // assembled from CRC16_Modbus + wire LSB
} GPS_Frame_t;
#pragma pack()

/* ---------- Globals ---------- */

extern volatile uint32_t cpu_time_sec;
extern volatile uint8_t  cpu_time_valid;
extern volatile uint8_t  fallback_active;
extern volatile gps_sel_t current_sel;

extern volatile uint32_t seconds_uptime;

extern volatile uint32_t tod_sec;
extern volatile uint16_t tod_ms;
extern volatile uint32_t second_reference_tick;
uint32_t get_timer_tick(void);
uint32_t get_elapsed_ms(void);

//extern volatile uint8_t  rti_tick_flag;
extern volatile uint8_t  rti_1s_tick_flag;

volatile uint32_t rti_tick_flag;
volatile uint8_t rti_10ms_tick_flag;
volatile uint32_t seconds_uptime_10ms;

volatile uint8_t rti_1ms_tick_flag;
volatile uint32_t seconds_uptime_1ms;

volatile uint8_t rti_5ms_tick_flag;
volatile uint32_t seconds_uptime_5ms;

volatile uint8_t rti_100ms_tick_flag;
volatile uint32_t seconds_uptime_100ms;

static uint8_t gps_write_rtc = 1;
static uint8_t gps_write_rtc_count = 0;

volatile uint32_t rtc_abs_seconds;
volatile uint32_t gps_abs_seconds;

int32_t IncTest1;
int32_t IncTest2;
int32_t LastIncTest1;
int32_t LastIncTest2;

extern volatile bool incremental_test_failed;

extern volatile uint8_t gps_faults;

/* ---------- Functions ---------- */

gps_decision_t decide_gps_primary(volatile GPS_Frame_t *gps1,
                                  volatile GPS_Frame_t *gps2);
void gps_process(void);

/* ---------- RX helpers ---------- */
int gps1_rx_pop(void);
int gps2_rx_pop(void);

void gps1_parse_byte(uint8_t byte);
void gps2_parse_byte(uint8_t byte);

/* ---------- Frame data ---------- */
extern volatile GPS_Frame_t gps1_frame;
extern volatile GPS_Frame_t gps2_frame;

extern volatile uint8_t gps1_frame_valid;
extern volatile uint8_t gps2_frame_valid;

extern volatile uint8_t gps1_new_frame;
extern volatile uint8_t gps2_new_frame;

/* ---------- Timing ---------- */
extern volatile uint32_t gps1_last_ok_sec;
extern volatile uint32_t gps2_last_ok_sec;

extern volatile uint32_t seconds_in_fallback;
extern volatile uint8_t  time_initialized;
extern volatile uint8_t  current_nms_fault;

/* ---------- Fault macros ---------- */
#define GPSF_CPU_TIME_INV    (1u << 7)
#define FALLBACK_TIMEOUT_SEC 60u

#endif
