// CPU – TMS570LS1224 – GPS1 & GPS2 frame receive & parse (SCI1 + SCILIN) - Selection logic - Fallback CPU time + no-frame timeout
//- Boot up condition added + Faults + CAN
#include "sys_common.h"
#include "system.h"
#include "sci.h"
#include <stdio.h>
#include <string.h>
#include "rti.h"
#include "can.h"
#include "gps.h"
#include "i2c_UD.h"
#include "radio.h"

/* ============================================================
 *  CONFIG
 * ============================================================ */

#define GPS1_SCI        scilinREG
#define GPS2_SCI        sciREG

#define RX_BUF_SIZE     256

/* ----- Fallback Time Management ----- */
volatile uint8_t  fallback_active      = 0;
//volatile uint8_t  rti_tick_flag     = 0;
volatile uint32_t seconds_in_fallback  = 0;
volatile uint8_t  rti_1s_tick_flag       = 0;

/* ----- Global uptime (seconds) from RTI ----- */
volatile uint32_t seconds_uptime       = 0;

/* Last time (in seconds_uptime) when a valid frame was received */
volatile uint32_t gps1_last_ok_sec     = 0;
volatile uint32_t gps2_last_ok_sec     = 0;

volatile bool incremental_test_failed = 0;

/* No-frame timeout for each GPS (testing value; later 10–30 s) */
#define GPS_FRAME_TIMEOUT_SEC   3u

// Bytes on the wire from GPS-TIVA:
// A5 5A len card week[2] tow[4] syssec[4]
// lat[4] lon[4] tAcc[4]
// numSV fix pps status[2] crc16[2]
// Total = 33 bytes
#define GPS_FRAME_TOTAL 33

// status_flags bits in GPS frame (from GPS-TIVA)
#define GPS_STAT_FIX_OK      (1u << 0)  // gps_fix_ok
#define GPS_STAT_PPS_OK      (1u << 1)  // !pps_missed
#define GPS_STAT_TIME_VALID  (1u << 2)  // TIMEGPS.valid == 0x07
#define GPS_STAT_PPS_MISSED  (1u << 3)
#define GPS_STAT_TACC_BAD    (1u << 4)  // tAcc > limit

#define TIME_MISMATCH_THRESH_SEC   1u

//!================= LATITUDE AND LONGITUDE Implementation ====================
uint32_t radio_latitude = 0;
uint32_t radio_longitude = 0;

static uint32_t gps_encode_longitude(int32_t lon_1e7) {
  uint8_t sign = 0;

  if (lon_1e7 < 0) {
    sign = 1;
    lon_1e7 = -lon_1e7;
  }

  double lon_deg_f = ((double)lon_1e7) / 10000000.0;

  uint16_t deg = (uint16_t)lon_deg_f;

  double minutes_f = (lon_deg_f - deg) * 60.0;

  uint8_t min = (uint8_t)minutes_f;

  double seconds_f = (minutes_f - min) * 60.0;

  uint8_t sec = (uint8_t)(seconds_f + 0.5);

  int16_t signed_deg = sign ? -(int16_t)deg : (int16_t)deg;

  uint32_t encoded = 0;

  encoded |= ((signed_deg & 0x1FF) << 12);
  encoded |= ((min & 0x3F) << 6);
  encoded |= (sec & 0x3F);

  return encoded;
}

static uint32_t gps_encode_latitude(int32_t lat_1e7) {
  uint8_t sign = 0;

  if (lat_1e7 < 0) {
    sign = 1;
    lat_1e7 = -lat_1e7;
  }

  double lat_deg_f = ((double)lat_1e7) / 10000000.0;

  uint16_t deg = (uint16_t)lat_deg_f;

  double minutes_f = (lat_deg_f - deg) * 60.0;

  uint8_t min = (uint8_t)minutes_f;

  double seconds_f = (minutes_f - min) * 60.0;

  uint8_t sec = (uint8_t)(seconds_f + 0.5);

  int16_t signed_deg = sign ? -(int16_t)deg : (int16_t)deg;

  uint32_t encoded = 0;

  encoded |= ((signed_deg & 0xFF) << 12);
  encoded |= ((min & 0x3F) << 6);
  encoded |= (sec & 0x3F);

  return encoded;
}

/* ============================================================
 *  GPS FAULT BITMASK FOR NMS
 * ============================================================ */
/*
 * gps_faults bit meanings (0 = no fault):
 *
 *  bit0: GPS1 no FIX/TIME_VALID/TACC (gps_fix_bit(gps1) == 0)
 *  bit1: GPS1 PPS bad (gps_pps_bit(gps1) == 0)
 *  bit2: GPS2 no FIX/TIME_VALID/TACC (gps_fix_bit(gps2) == 0)
 *  bit3: GPS2 PPS bad (gps_pps_bit(gps2) == 0)
 *  bit4: Time mismatch between GPS1 & GPS2 (tm == 1)
 *  bit5: GPS1 frame timeout (no valid frame for GPS_FRAME_TIMEOUT_SEC)
 *  bit6: GPS2 frame timeout
 *  bit7: CPU time invalid (fallback expired)
 */

#define GPSF_G1_NO_FIX       (1u << 0)
#define GPSF_G1_NO_PPS       (1u << 1)
#define GPSF_G2_NO_FIX       (1u << 2)
#define GPSF_G2_NO_PPS       (1u << 3)
#define GPSF_TIME_MISMATCH   (1u << 4)
#define GPSF_G1_TIMEOUT      (1u << 5)
#define GPSF_G2_TIMEOUT      (1u << 6)
#define GPSF_CPU_TIME_INV    (1u << 7)

volatile uint8_t gps_faults = 0;   // 0 => no fault, otherwise bitwise faults

/* ============================================================
 *  FRAME STRUCT
 * ============================================================ */

volatile GPS_Frame_t gps1_frame;
volatile uint8_t     gps1_frame_valid = 0;
volatile uint8_t     gps1_new_frame   = 0;

volatile GPS_Frame_t gps2_frame;
volatile uint8_t     gps2_frame_valid = 0;
volatile uint8_t     gps2_new_frame   = 0;

/* ============================================================
 *  RX RING BUFFERS
 * ============================================================ */

/* GPS1 RX ring buffer */
volatile uint8_t  gps1_rx_buf[RX_BUF_SIZE];
volatile uint16_t gps1_rx_head = 0;
volatile uint16_t gps1_rx_tail = 0;

static inline void gps1_rx_push(uint8_t byte)
{
    uint16_t next = (gps1_rx_head + 1u) % RX_BUF_SIZE;
    if (next != gps1_rx_tail)
    {
        gps1_rx_buf[gps1_rx_head] = byte;
        gps1_rx_head = next;
    }
    // else overflow: byte dropped
}

int gps1_rx_pop(void)
{
    if (gps1_rx_head == gps1_rx_tail)
        return -1;

    uint8_t byte = gps1_rx_buf[gps1_rx_tail];
    gps1_rx_tail = (gps1_rx_tail + 1u) % RX_BUF_SIZE;
    return (int)byte;
}

/* GPS2 RX ring buffer */
volatile uint8_t  gps2_rx_buf[RX_BUF_SIZE];
volatile uint16_t gps2_rx_head = 0;
volatile uint16_t gps2_rx_tail = 0;

static inline void gps2_rx_push(uint8_t byte)
{
    uint16_t next = (gps2_rx_head + 1u) % RX_BUF_SIZE;
    if (next != gps2_rx_tail)
    {
        gps2_rx_buf[gps2_rx_head] = byte;
        gps2_rx_head = next;
    }
    // else overflow: byte dropped
}

int gps2_rx_pop(void)
{
    if (gps2_rx_head == gps2_rx_tail)
        return -1;

    uint8_t byte = gps2_rx_buf[gps2_rx_tail];
    gps2_rx_tail = (gps2_rx_tail + 1u) % RX_BUF_SIZE;
    return (int)byte;
}

/* ============================================================
 *  CRC16-MODBUS (same as GPS-TIVA)
 * ============================================================ */

static uint16_t CRC16_Modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}

/* ============================================================
 *  HALCoGen RX BYTE + ISR
 * ============================================================ */

/* Global RX byte used by sciReceive() and sciNotification() */
volatile uint8_t rx_byte;

/* sciNotification: called by HAL on RX interrupt */
void sciNotification(sciBASE_t *sci, uint32 flags)
{
    if (flags & SCI_RX_INT)
    {
        uint8_t byte = rx_byte;   // HAL already read the hardware

        if (sci == GPS1_SCI)
        {
            gps1_rx_push(byte);
        }
        if (sci == GPS2_SCI)
        {
            gps2_rx_push(byte);
        }

        // Re-arm reception of next byte on this SCI
        sciReceive(sci, 1U, (uint8 *)&rx_byte);
    }
}

/* ============================================================
 *  GPS1 FRAME PARSER
 * ============================================================ */

typedef enum {
    GPS_PARSE_SYNC1,
    GPS_PARSE_SYNC2,
    GPS_PARSE_LEN,
    GPS_PARSE_DATA
} gps_parse_state_t;

static gps_parse_state_t gps1_state = GPS_PARSE_SYNC1;

static uint8_t  gps1_frame_buf[GPS_FRAME_TOTAL];
static uint16_t gps1_frame_index = 0;
static uint8_t  gps1_expected_len = GPS_FRAME_TOTAL;

/* Decode GPS1 frame (little-endian -> host) */
static void gps1_parse_complete_frame(uint8_t *buf, uint8_t len)
{
    if (len < GPS_FRAME_TOTAL)
        return;  // too short, ignore

    uint8_t crc_rx_low = buf[31];
    uint8_t crc_rx_high = buf[32];

    uint16_t crc_calc = CRC16_Modbus(buf, 31);

    uint8_t crc_calc_low = crc_calc & 0xFF;
    uint8_t crc_calc_high = (crc_calc >> 8) & 0xFF;

    if ((crc_calc_low != crc_rx_low) || (crc_calc_high != crc_rx_high)) {
      return;
    }

    gps1_frame.sync1 = buf[0];
    gps1_frame.sync2 = buf[1];
    gps1_frame.length = buf[2];
    gps1_frame.card_id = buf[3];

    gps1_frame.gps_week = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    gps1_frame.gps_tow_ms = (uint32_t)buf[6] | ((uint32_t)buf[7] << 8) |
                            ((uint32_t)buf[8] << 16) | ((uint32_t)buf[9] << 24);

    gps1_frame.gps_system_seconds =
        (uint32_t)buf[10] | ((uint32_t)buf[11] << 8) |
        ((uint32_t)buf[12] << 16) | ((uint32_t)buf[13] << 24);

    gps1_frame.latitude = (int32_t)buf[14] | ((int32_t)buf[15] << 8) |
                          ((int32_t)buf[16] << 16) | ((int32_t)buf[17] << 24);

    gps1_frame.longitude = (int32_t)buf[18] | ((int32_t)buf[19] << 8) |
                           ((int32_t)buf[20] << 16) | ((int32_t)buf[21] << 24);

    gps1_frame.tAcc_ns = (uint32_t)buf[22] | ((uint32_t)buf[23] << 8) |
                         ((uint32_t)buf[24] << 16) | ((uint32_t)buf[25] << 24);

    gps1_frame.numSV = buf[26];
    gps1_frame.gps_fix_ok = buf[27];
    gps1_frame.pps_ok = buf[28];

    gps1_frame.status_flags = (uint16_t)buf[29] | ((uint16_t)buf[30] << 8);

    gps1_frame.crc16 = crc_calc;

    gps1_frame_valid = 1;   // we have at least one good frame
    gps1_new_frame   = 1;   // new data arrived
}

void gps1_parse_byte(uint8_t byte)
{
    switch (gps1_state)
    {
        case GPS_PARSE_SYNC1:
            if (byte == 0xA5)
            {
                gps1_frame_buf[0] = byte;
                gps1_frame_index  = 1;
                gps1_state        = GPS_PARSE_SYNC2;
            }
            break;

        case GPS_PARSE_SYNC2:
            if (byte == 0x5A)
            {
                gps1_frame_buf[1] = byte;
                gps1_frame_index  = 2;
                gps1_state        = GPS_PARSE_LEN;
            }
            else
            {
                gps1_state = GPS_PARSE_SYNC1;
            }
            break;

        case GPS_PARSE_LEN:
            gps1_frame_buf[2] = byte;
            gps1_frame_index  = 3;

            gps1_expected_len = byte;  // TIVA sends 33 byte frame
            if (gps1_expected_len != GPS_FRAME_TOTAL || gps1_expected_len == 0)
            {
                gps1_state       = GPS_PARSE_SYNC1;
                gps1_frame_index = 0;
            }
            else
            {
                gps1_state = GPS_PARSE_DATA;
            }
            break;

        case GPS_PARSE_DATA:
            if (gps1_frame_index < gps1_expected_len)
            {
                gps1_frame_buf[gps1_frame_index++] = byte;
            }

            if (gps1_frame_index >= gps1_expected_len)
            {
                if (gps1_frame_buf[0] == 0xA5 && gps1_frame_buf[1] == 0x5A)
                {
                    gps1_parse_complete_frame(gps1_frame_buf, gps1_expected_len);
                }

                gps1_state       = GPS_PARSE_SYNC1;
                gps1_frame_index = 0;
            }
            break;
    }
}

/* ============================================================
 *  GPS2 FRAME PARSER
 * ============================================================ */

typedef enum {
    GPS2_PARSE_SYNC1,
    GPS2_PARSE_SYNC2,
    GPS2_PARSE_LEN,
    GPS2_PARSE_DATA
} gps2_parse_state_t;

static gps2_parse_state_t gps2_state = GPS2_PARSE_SYNC1;

static uint8_t  gps2_frame_buf[GPS_FRAME_TOTAL];
static uint16_t gps2_frame_index = 0;
static uint8_t  gps2_expected_len = GPS_FRAME_TOTAL;

static void gps2_parse_complete_frame(uint8_t *buf, uint8_t len)
{
    if (len < GPS_FRAME_TOTAL)
        return;

    uint8_t crc_rx_low = buf[31];
    uint8_t crc_rx_high = buf[32];
    uint16_t crc_calc = CRC16_Modbus(buf, 31);
    uint8_t crc_calc_low = crc_calc & 0xFF;
    uint8_t crc_calc_high = (crc_calc >> 8) & 0xFF;

    if ((crc_calc_low != crc_rx_low) || (crc_calc_high != crc_rx_high)) {
      return; // CRC mismatch
    }

    gps2_frame.sync1   = buf[0];
    gps2_frame.sync2   = buf[1];
    gps2_frame.length  = buf[2];
    gps2_frame.card_id = buf[3];

    gps2_frame.gps_week = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

    gps2_frame.gps_tow_ms =
        (uint32_t)buf[6]        |
        ((uint32_t)buf[7] << 8) |
        ((uint32_t)buf[8] << 16)|
        ((uint32_t)buf[9] << 24);

    gps2_frame.gps_system_seconds =
        (uint32_t)buf[10]        |
        ((uint32_t)buf[11] << 8) |
        ((uint32_t)buf[12] << 16)|
        ((uint32_t)buf[13] << 24);

    gps2_frame.latitude = (int32_t)buf[14] | ((int32_t)buf[15] << 8) |
                          ((int32_t)buf[16] << 16) | ((int32_t)buf[17] << 24);

    gps2_frame.longitude = (int32_t)buf[18] | ((int32_t)buf[19] << 8) |
                           ((int32_t)buf[20] << 16) | ((int32_t)buf[21] << 24);

    gps2_frame.tAcc_ns = (uint32_t)buf[22] | ((uint32_t)buf[23] << 8) |
                         ((uint32_t)buf[24] << 16) | ((uint32_t)buf[25] << 24);

    gps2_frame.numSV = buf[26];
    gps2_frame.gps_fix_ok = buf[27];
    gps2_frame.pps_ok = buf[28];

    gps2_frame.status_flags = (uint16_t)buf[29] | ((uint16_t)buf[30] << 8);

    gps2_frame.crc16 = crc_calc;

    gps2_frame_valid = 1;
    gps2_new_frame   = 1;
}

void gps2_parse_byte(uint8_t byte)
{
    switch (gps2_state)
    {
        case GPS2_PARSE_SYNC1:
            if (byte == 0xA5)
            {
                gps2_frame_buf[0] = byte;
                gps2_frame_index  = 1;
                gps2_state        = GPS2_PARSE_SYNC2;
            }
            break;

        case GPS2_PARSE_SYNC2:
            if (byte == 0x5A)
            {
                gps2_frame_buf[1] = byte;
                gps2_frame_index  = 2;
                gps2_state        = GPS2_PARSE_LEN;
            }
            else
            {
                gps2_state = GPS2_PARSE_SYNC1;
            }
            break;

        case GPS2_PARSE_LEN:
            gps2_frame_buf[2] = byte;
            gps2_frame_index  = 3;
            gps2_expected_len = byte;

            if (gps2_expected_len != GPS_FRAME_TOTAL || gps2_expected_len == 0)
            {
                gps2_state       = GPS2_PARSE_SYNC1;
                gps2_frame_index = 0;
            }
            else
            {
                gps2_state = GPS2_PARSE_DATA;
            }
            break;

        case GPS2_PARSE_DATA:
            if (gps2_frame_index < gps2_expected_len)
            {
                gps2_frame_buf[gps2_frame_index++] = byte;
            }

            if (gps2_frame_index >= gps2_expected_len)
            {
                gps2_parse_complete_frame(gps2_frame_buf, gps2_expected_len);
                gps2_state       = GPS2_PARSE_SYNC1;
                gps2_frame_index = 0;
            }
            break;
    }
}

/* ============================================================
 *  SELECTION LOGIC
 * ============================================================ */

/* --- CPU TIME & OUTPUTS --- */
volatile gps_sel_t current_sel       = GPS_SEL_NONE;
volatile uint8_t  current_nms_fault  = 1;
volatile uint32_t cpu_time_sec       = 0;
volatile uint8_t  cpu_time_valid     = 0;

/* Has CPU time ever been initialised from a valid GPS? */
volatile uint8_t  time_initialized   = 0;   // 0 = no valid GPS yet, 1 = got time once

static inline uint8_t gps_fix_bit(volatile GPS_Frame_t *frame)
{
    uint16_t status = frame->status_flags;

    if (!(status & GPS_STAT_FIX_OK))     return 0;
    if (!(status & GPS_STAT_TIME_VALID)) return 0;
    if (status & GPS_STAT_TACC_BAD)      return 0;

    return 1;
}

static inline uint8_t gps_pps_bit(volatile GPS_Frame_t *frame)
{
    uint16_t status = frame->status_flags;

    if (!(status & GPS_STAT_PPS_OK))  return 0;
    if (status & GPS_STAT_PPS_MISSED) return 0;

    return 1;
}

static inline uint8_t gps_time_mismatch(volatile GPS_Frame_t *gps1,
                                        volatile GPS_Frame_t *gps2)
{
    if (!gps_fix_bit(gps1) || !gps_fix_bit(gps2))
        return 0;   // “x” in the table

    if (!gps_pps_bit(gps1) || !gps_pps_bit(gps2))
        return 0;   // “x” in the table

    uint32_t GPSTime1 = gps1->gps_system_seconds;
    uint32_t GPSTime2 = gps2->gps_system_seconds;

    uint32_t diff = (GPSTime1 > GPSTime2) ? (GPSTime1 - GPSTime2) : (GPSTime2 - GPSTime1);

    return (diff > TIME_MISMATCH_THRESH_SEC) ? 1 : 0;
}

gps_decision_t decide_gps_primary(volatile GPS_Frame_t *gps1,
                                  volatile GPS_Frame_t *gps2)
{
    gps_decision_t gps_select;
    gps_select.sel       = GPS_SEL_NONE;
    gps_select.nms_fault = 1;
    gps_select.faults    = 0;

    uint32_t GPSTime1, GPSTime2, RTCTime, diff;

    /* Raw status from frame status_flags (no timeout applied yet) */
    uint8_t fix1_raw = gps_fix_bit(gps1);
    uint8_t fix2_raw = gps_fix_bit(gps2);
    uint8_t pps1_raw = gps_pps_bit(gps1);
    uint8_t pps2_raw = gps_pps_bit(gps2);
    uint8_t tm       = gps_time_mismatch(gps1, gps2); // 1 = mismatch, 0 = match

    // RTC Check Start
    //Case 1: Both Working
    if ((fix1_raw && pps1_raw) && (fix2_raw && pps2_raw) && !tm)
    {
        GPSTime1 = gps1->gps_system_seconds;
        GPSTime2 = gps2->gps_system_seconds;
        RTCTime = rtc_abs_seconds;
        diff = (GPSTime1 > RTCTime) ? (GPSTime1 - RTCTime) : (RTCTime - GPSTime1);
        if(diff > TIME_MISMATCH_THRESH_SEC)
        {
            gps_write_rtc = 1;
        }
    }
    //Case 2: Both Working but mismatch
//    if ((fix1_raw && pps1_raw) && (fix2_raw && pps2_raw) && tm)
//    {
//        GPSTime1 = gps1->gps_system_seconds;
//        GPSTime2 = gps2->gps_system_seconds;
//        RTCTime = rtc_abs_seconds;
//        IncTest1 = (GPSTime1 > RTCTime) ? (GPSTime1 - RTCTime) : (RTCTime - GPSTime1);
//        IncTest2 = (GPSTime2 > RTCTime) ? (GPSTime2 - RTCTime) : (RTCTime - GPSTime2);
//        if(diff > TIME_MISMATCH_THRESH_SEC)
//        {
//            gps_write_rtc = 1;
//        }
//    }
    //Case 3: GPS1 Not Working
    else if ((fix2_raw && pps2_raw) && (!fix1_raw || !pps1_raw))
    {
        GPSTime2 = gps2->gps_system_seconds;
        RTCTime = rtc_abs_seconds;
        diff = (GPSTime2 > RTCTime) ? (GPSTime2 - RTCTime) : (RTCTime - GPSTime2);
        if(diff > TIME_MISMATCH_THRESH_SEC)
        {
            gps_write_rtc = 1;
        }
    }
    //Case 4: GPS2 Not Working
    else if ((fix1_raw && pps1_raw) && (!fix2_raw || !pps2_raw))
    {
        GPSTime1 = gps1->gps_system_seconds;
        RTCTime = rtc_abs_seconds;
        diff = (GPSTime1 > RTCTime) ? (GPSTime1 - RTCTime) : (RTCTime - GPSTime1);
        if(diff > TIME_MISMATCH_THRESH_SEC)
        {
            gps_write_rtc = 1;
        }
    }
    //RTC Check End

    //Incremental Test
    if(tm && !gps_write_rtc)
    {
        GPSTime1 = gps1->gps_system_seconds;
        GPSTime2 = gps2->gps_system_seconds;
        RTCTime = rtc_abs_seconds;
        if((IncTest1 > LastIncTest1) ? ((IncTest1 - LastIncTest1 > 0)) : (LastIncTest1 - IncTest1 > 0))
        {
            if((IncTest2 > LastIncTest2) ? ((IncTest2 - LastIncTest2 > 0)) : (LastIncTest2 - IncTest2 > 0))
            {
                fallback_active = 1;
                seconds_in_fallback = 0;
                incremental_test_failed = 1;
            }
            else
            {
                //Switch to GPS2?
                gps_select.sel = GPS_SEL_GPS2;
            }
        }
        else if((IncTest2 > LastIncTest2) ? ((IncTest2 - LastIncTest2 > 0)) : (LastIncTest2 - IncTest2 > 0))
        {
            //Switch to GPS1?
            gps_select.sel = GPS_SEL_GPS2;
        }
    }
    //

    /* Record raw faults */
    if (!fix1_raw) gps_select.faults |= GPSF_G1_NO_FIX;
    if (!pps1_raw) gps_select.faults |= GPSF_G1_NO_PPS;
    if (!fix2_raw) gps_select.faults |= GPSF_G2_NO_FIX;
    if (!pps2_raw) gps_select.faults |= GPSF_G2_NO_PPS;
    if (tm)        gps_select.faults |= GPSF_TIME_MISMATCH;

    /* Start with raw states; may be overridden by timeout logic */
    uint8_t fix1 = fix1_raw;
    uint8_t fix2 = fix2_raw;
    uint8_t pps1 = pps1_raw;
    uint8_t pps2 = pps2_raw;

    /* -------- No-frame timeout override -------- */
    uint32_t age1 = seconds_uptime - gps1_last_ok_sec;
    uint32_t age2 = seconds_uptime - gps2_last_ok_sec;

    if (gps1_last_ok_sec == 0u || age1 > GPS_FRAME_TIMEOUT_SEC)
    {
        /* Treat GPS1 as unusable because we stopped getting frames */
        fix1 = 0;
        pps1 = 0;
        gps_select.faults |= GPSF_G1_TIMEOUT;
    }

    if (gps2_last_ok_sec == 0u || age2 > GPS_FRAME_TIMEOUT_SEC)
    {
        /* Treat GPS2 as unusable because we stopped getting frames */
        fix2 = 0;
        pps2 = 0;
        gps_select.faults |= GPSF_G2_TIMEOUT;
    }
    /* ------------------------------------------- */

    /* -------- Selection logic (same truth table as before) -------- */
    if (!fix1 && !fix2) {
        gps_select.sel = GPS_SEL_NONE;
        /* faults already describe WHY: no fix / timeout / etc. */
    }
    else if (!fix1 && fix2) {
        if (pps2)
            gps_select.sel = GPS_SEL_GPS2;
        else
            gps_select.sel = GPS_SEL_NONE;
    }
    else if (fix1 && !fix2) {
        if (pps1)
            gps_select.sel = GPS_SEL_GPS1;
        else
            gps_select.sel = GPS_SEL_NONE;
    }
    else { // fix1 == 1 && fix2 == 1
        if (!pps1 && !pps2) {
            gps_select.sel = GPS_SEL_NONE;
        }
        else if (!pps1 && pps2) {
            gps_select.sel = GPS_SEL_GPS2;
        }
        else if (pps1 && !pps2) {
            gps_select.sel = GPS_SEL_GPS1;
        }
        else { // pps1 == 1 && pps2 == 1
            if (tm) {
                gps_select.sel = GPS_SEL_GPS1;  // or GPS2, per your policy
                /* TIME_MISMATCH already flagged in faults */
            } else {
                gps_select.sel = GPS_SEL_GPS1;  // normal healthy case
            }
        }
    }

    /* NMS_fault is 1 whenever any fault bit is set, else 0 */
    gps_select.nms_fault = (gps_select.faults == 0u) ? 0u : 1u;

    return gps_select;
}

/* ============================================================
 *  RTI NOTIFICATION
 * ============================================================ */

void rtiNotification(uint32 notification)
{
    // if (notification == rtiNOTIFICATION_COMPARE0)
    // {
        // rti_1s_tick_flag = 1;  // 1-second tick
        // seconds_uptime++;   // global uptime
    // }
    if (notification == rtiNOTIFICATION_COMPARE1)
    {
        rti_tick_flag++; //every 1 ms

        rti_1ms_tick_flag = 1;
        seconds_uptime_1ms++; // global uptime every 1ms

        if (seconds_uptime_1ms == 5U) 
        {
            rti_5ms_tick_flag = 1; // 5-milli second tick
            seconds_uptime_5ms++;  // global uptime every 5ms
            seconds_uptime_1ms = 0;
        }

        if (seconds_uptime_5ms == 2U)
        {
            rti_10ms_tick_flag = 1; // 10-milli second tick
            seconds_uptime_10ms++;  // global uptime every 10ms
            seconds_uptime_5ms = 0;
        }

        if (seconds_uptime_10ms == 10U)
        {
            rti_100ms_tick_flag = 1;  // 100-milli second tick
            seconds_uptime_100ms++;   // global uptime every 10ms
            seconds_uptime_10ms = 0;
        }
        
        if (seconds_uptime_100ms == 10U)
        {
          rti_1s_tick_flag = 1; // 1-second tick
          seconds_uptime++;     // global uptime
          seconds_uptime_100ms = 0;
        }
    }
}
/* Read free running RTI counter */
uint32_t get_timer_tick(void) 
{
    return rtiGetCurrentTick(rtiCOMPARE0); 
}
volatile uint32_t second_reference_tick = 0;

/* Return elapsed microseconds since latest second */
uint32_t get_elapsed_us(void) 
{
    uint32_t current_tick;

    current_tick = get_timer_tick();

    return (current_tick - second_reference_tick);
}

/* Return elapsed milliseconds since latest second */
uint32_t get_elapsed_ms(void) 
{
    return (get_elapsed_us() / 1000U); 
}

void gps_process(void)
{
    /* ---------- GPS1: drain ring buffer into parser ---------- */
    int ch1 = gps1_rx_pop();
    if (ch1 >= 0)
    {
        gps1_parse_byte((uint8_t)ch1);
    }

    /* ---------- GPS2: drain ring buffer into parser ---------- */
    int ch2 = gps2_rx_pop();
    if (ch2 >= 0)
    {
        gps2_parse_byte((uint8_t)ch2);
    }

    uint8_t gps1_new = gps1_new_frame;
    uint8_t gps2_new = gps2_new_frame;

    /* Update last_ok timestamps when NEW valid frame received */
    if (gps1_new)
    {
        gps1_last_ok_sec = seconds_uptime;
        gps1_new_frame = 0;   // clear flag

        // Optional per-frame debug:
        uint8_t dbg1[160];
        uint32 len1 = sprintf((char *)dbg1,
                              "GPS1: card=%u week=%u tow_ms=%lu syssec=%lu "
                              "SV=%u fix=%u pps=%u status=0x%04X crc16=0x%04X\r\n",
                              gps1_frame.card_id,
                              gps1_frame.gps_week,
                              (unsigned long)gps1_frame.gps_tow_ms,
                              (unsigned long)gps1_frame.gps_system_seconds,
                              gps1_frame.numSV,
                              gps1_frame.gps_fix_ok,
                              gps1_frame.pps_ok,
                              gps1_frame.status_flags,
                              gps1_frame.crc16);
        sciSend(GPS1_SCI, len1, dbg1);

        //Inc Test for GPS1
        LastIncTest1 = IncTest1;
        IncTest1 = gps1_frame.gps_system_seconds - rtc_abs_seconds;
    }

    if (gps2_new)
    {
        gps2_last_ok_sec = seconds_uptime;
        gps2_new_frame = 0;   // clear flag

        uint8_t dbg2[160];
        uint32 len2 = sprintf((char *)dbg2,
                              "GPS2: card=%u week=%u tow_ms=%lu syssec=%lu "
                              "SV=%u fix=%u pps=%u status=0x%04X crc16=0x%04X\r\n",
                              gps2_frame.card_id,
                              gps2_frame.gps_week,
                              (unsigned long)gps2_frame.gps_tow_ms,
                              (unsigned long)gps2_frame.gps_system_seconds,
                              gps2_frame.numSV,
                              gps2_frame.gps_fix_ok,
                              gps2_frame.pps_ok,
                              gps2_frame.status_flags,
                              gps2_frame.crc16);
        sciSend(GPS2_SCI, len2, dbg2);

        //Inc Test for GPS2
        LastIncTest2 = IncTest2;
        IncTest2 = gps2_frame.gps_system_seconds - rtc_abs_seconds;
    }

    /* ---------- Run selection when new data available ---------- */
    if ((gps1_frame_valid || gps2_frame_valid) &&
            (gps1_new || gps2_new))
    {
        gps_decision_t gps_select = decide_gps_primary(&gps1_frame, &gps2_frame);

        /* -------- GPS SELECTED -------- */
        if (gps_select.sel != GPS_SEL_NONE)
        {
            /* Exit the fallback only when a GPS is actually selected */
            fallback_active     = 0;
            seconds_in_fallback = 0;
            cpu_time_valid      = 1;
            incremental_test_failed = 0;

            if (gps_select.sel == GPS_SEL_GPS1)
            {
                cpu_time_sec = gps1_frame.gps_system_seconds;

                /* Close previous second block */
                // distance_db[current_sec_index].sec_span_ms = get_elapsed_ms();

                tlm_tod_sec = cpu_time_sec % 86400U;

                /* Save current timer tick */
                second_reference_tick = get_timer_tick();

                /* Move to next second block */
                current_sec_index++;

                if (current_sec_index >= TLM_HISTORY_SECONDS) {
                  current_sec_index = 0U;
                }

                /* Start new second block */
                distance_db[current_sec_index].tod_sec = tlm_tod_sec;

                /* Reset local RTI timing */
                // tlm_tod_ms = 0U;

                /* Reset sample index */
                current_sample_index = 0U;

                radio_update_frame_number();

                radio_longitude = gps_encode_longitude(gps1_frame.longitude);
                radio_latitude = gps_encode_latitude(gps1_frame.latitude);
            }
            else
            {
                cpu_time_sec = gps2_frame.gps_system_seconds;

                tlm_tod_sec = cpu_time_sec % 86400U;

                /* Save current timer tick */
                second_reference_tick = get_timer_tick();

                /* Close previous second block */
                // distance_db[current_sec_index].sec_span_ms = get_elapsed_ms();

                /* Move to next second block */
                current_sec_index++;

                if (current_sec_index >= TLM_HISTORY_SECONDS) {
                  current_sec_index = 0U;
                }

                /* Start new second block */
                distance_db[current_sec_index].tod_sec = tlm_tod_sec;

                /* Reset local RTI timing */
                // tlm_tod_ms = 0U;

                /* Reset sample index */
                current_sample_index = 0U;

                radio_update_frame_number();

                radio_longitude = gps_encode_longitude(gps2_frame.longitude);
                radio_latitude = gps_encode_latitude(gps2_frame.latitude);
            }

            time_initialized = 1;   // first-ever valid GPS time obtained

            if(gps_write_rtc)
            {
                //Probably Temp
                uint16_t year;
                uint8_t month, day, hour, min, sec;
                uint32_t unix_seconds;

                unix_seconds = cpu_time_sec;

                seconds_to_calendar(unix_seconds,
                                    &year, &month, &day,
                                    &hour, &min, &sec);

                rtc_set[0] = sec;
                rtc_set[1] = min;
                rtc_set[2] = hour;
                rtc_set[4] = day;
                rtc_set[5] = month;
                rtc_set[6] = (uint8_t)(year - 2000U); /* DS1307: 00 = 2000 */
                //
                start_rtc_write = 1;
                gps_write_rtc = 0;

                gps_write_rtc_count++;
                if(gps_write_rtc_count >= 6)
                {
                    gps_write_rtc_count = 0;
                    //Set Fault flag
                }
            }
        }
        /* -------- NO GPS SELECTED -------- */
        else
        {
            if (time_initialized)
            {
                /* Enter fallback ONLY ONCE */
                if (!fallback_active)
                {
                    fallback_active     = 1;
                    seconds_in_fallback = 0;
                    cpu_time_valid      = 1;   // fallback time is initially valid
                    gps_write_rtc = 1;
                }
                /* else: already in fallback - DO NOTHING */
            }
            else
            {
                /* Startup condition: never had GPS time */
                fallback_active = 0;
                cpu_time_valid  = 0;
                incremental_test_failed = 0;
            }
        }

        current_sel       = gps_select.sel;
        current_nms_fault = gps_select.nms_fault;
        gps_faults        = gps_select.faults;

        const char *selStr =
                (gps_select.sel == GPS_SEL_GPS1) ? "GPS1" :
                        (gps_select.sel == GPS_SEL_GPS2) ? "GPS2" :
                                "NONE";

        uint8_t dbgSel[120];
        uint32 lenSel = sprintf((char *)dbgSel,
                                "SELECT: %s  NMS_fault=%u  CPU_TIME_SEC=%lu  up=%lu init=%u fb=%u faults=0x%02X\r\n",
                                selStr,
                                current_nms_fault,
                                (unsigned long)cpu_time_sec,
                                (unsigned long)seconds_uptime,
                                (unsigned)time_initialized,
                                (unsigned)fallback_active,
                                (unsigned)gps_faults);

        sciSend(GPS1_SCI, lenSel, dbgSel);
    }
}
