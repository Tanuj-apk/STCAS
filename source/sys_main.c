/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "can_if.h"
#include "gps.h"
#include "kms.h"
#include "i2c_UD.h"
#include "radio.h"
//#include "rfid_rx.h"
#include "rti.h"
#include "sci.h"
#include "sys_common.h"
#include "system.h"
#include "het.h"
#include "input_card.h"
#include "SMOCIP.h"
#include "spi.h"
#include "hcms3902.h"
#include "NMS.h"
#include "datalogging.h"
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
extern volatile uint8_t rx_byte;
uint32_t calculated_firmware_checksum = 0U;
uint8_t g_device_id = CPU_DEVICE_ID_MIN;
void v_1msTasks(void);
void v_5msTasks(void);
void v_10msTasks(void);
void v_100msTasks(void);
void v_1sTasks(void);
void KavachInit(void);
static void Read_CPU_Device_ID(void);
static bool DataLogger_CanTx(
    uint32_t can_id,
    const uint8_t data[DATA_LOGGER_CAN_FRAME_SIZE]);
uint32_t calculate_firmware_crc(void);

#define NMS_TEST_ENABLE       1U
#define NMS_TEST_PERIOD_SEC   5U

static uint8_t nms_test_timer = 0U;

#define FIRMWARE_CRC_START 0x00000000UL
#define FIRMWARE_CRC_END 0x0002383FUL

/* ============================================================
 *  MAIN
 * ============================================================ */
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    KavachInit();
    calculated_firmware_checksum = calculate_firmware_crc();
    HCMS_DisplayString(" OK ");
    while (1)
    {
        /* Process received CAN frames outside interrupt context.  This must
         * run before startup polling so queued startup ACKs are consumed. */
        can_if_process_rx_pending();

        if(!can_manager_poll_startup())
            continue;

        gps_process();

        if (rti_1ms_tick_flag) 
        {
           v_1msTasks();
            rti_1ms_tick_flag = 0;
        }

        if (rti_5ms_tick_flag)
        {
            v_5msTasks();
            rti_5ms_tick_flag = 0;
        }

        if (rti_10ms_tick_flag)
        {
            // v_10msTasks();
            rti_10ms_tick_flag = 0;
        }

        if (rti_100ms_tick_flag)
        {
           v_100msTasks();
            rti_100ms_tick_flag = 0;
        }

        if (rti_1s_tick_flag)
        {
            v_1sTasks();
            rti_1s_tick_flag = 0;
        }
    }
    /* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* For Displaying CPU Checksum
Polynomial = 0xEDB88320
Initial    = 0xFFFFFFFF
Final XOR  = 0xFFFFFFFF
*/
uint32_t calculate_firmware_crc(void)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t address;
    uint8_t data;
    uint32_t bit;

    for (address = FIRMWARE_CRC_START; address <= FIRMWARE_CRC_END; address++)
    {
        data = *((volatile uint8_t *)address);

        crc ^= (uint32_t)data;

        for (bit = 0U; bit < 8U; bit++)
        {
            if (crc & 1U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            }
            else
            {
                crc = crc >> 1U;
            }
        }
    }

    return ~crc;
}

static void Read_CPU_Device_ID(void)
{
    uint8_t id_lsb;
    uint8_t id_msb;

    id_lsb = (uint8_t)gioGetBit(gioPORTA, 3U);
    id_msb = (uint8_t)gioGetBit(gioPORTA, 4U);

    id_lsb = 1;
    id_msb = 0;

    g_device_id = (uint8_t)(((id_msb << 1U) | id_lsb) +
                            CPU_DEVICE_ID_MIN);
}

static bool DataLogger_CanTx(
    uint32_t can_id,
    const uint8_t data[DATA_LOGGER_CAN_FRAME_SIZE])
{
    if (can_id != can_get_local_tx_id(DATA_LOGGER_TX_CAN_ID))
    {
        return false;
    }

    return (can_transmit_redundant(canMESSAGE_BOX16, data) != 0U);
}

void KavachInit(void)
{
    uint8_t msg[] = "CPU GPS1+GPS2 RX Ready\r\n";

    systemInit();
    gioInit();
    Read_CPU_Device_ID();
    gioSetBit(gioPORTA, 0, 0); // RS485 Receive Mode
    hetPORT1->DIR |= (1U << 24);
    gioSetBit(hetPORT1, 24, 0); // RS485 Receive Mode
    sciInit();
    i2cInit();
    _enable_IRQ();
    canInit();
    can_configure_device_ids();
    canEnableErrorNotification(canREG1);
    canEnableErrorNotification(canREG2);
    //LED STATUS INIT
    spiInit();
    HCMS_Init();

    /* Start RTI for fallback timer + uptime */
    rtiInit();  // ensure RTI started (if not auto from systemInit)
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    rtiEnableNotification(rtiNOTIFICATION_COMPARE1);
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    /* Enable RX interrupt for both GPS SCI modules */
    sciEnableNotification(GPS1_SCI, SCI_RX_INT);
    sciEnableNotification(GPS2_SCI, SCI_RX_INT);
    /* Send welcome (over GPS1 SCI for debug) */
    sciSend(GPS1_SCI, sizeof(msg) - 1U, msg);

    /* Start first RX on both (HAL u                         ses same rx_byte global) */
    sciReceive(GPS1_SCI, 1U, (uint8 *)&rx_byte);
    sciReceive(GPS2_SCI, 1U, (uint8 *)&rx_byte);

    can_manager_init();
    data_logger_init(DataLogger_CanTx);
    kms_init();

    start_rtc_write = 1;

}

void v_1msTasks(void)
{

}

void v_5msTasks(void)
{
    data_logger_process_ack();
    (void)data_logger_process_tx(1U);
    radio_ack_process();
    radio_tx_process();
    smocip_ack_process();
    smocip_tx_process();
    nms_ack_process();
    nms_tx_process();
}

void v_10msTasks(void)
{
    // distance_m += (speed_ms * 0.010f);
//    distance_m += (speed_ms_filtered * 0.010f);
//    distance_km = distance_m/ (1000.0f);

//    if(current_sample_index < TLM_MAX_SAMPLES_PER_SEC)
//    {
//        distance_db[current_sec_index].tod_ms[current_sample_index] = get_elapsed_ms();
//        distance_db[current_sec_index].distance_odo[current_sample_index] = distance_m;
//        current_sample_index++;
//
//        distance_db[current_sec_index].sec_span_ms = current_sample_index;
//    }

    /* Advance local RTI time */
    // tlm_tod_ms += 10U;

    if(start_rtc_read)
    {
        if(rtc_read_count != 3)
        {
            // Read one RTC register
            RTC_ReadByte(rtc_raw, rtc_read_count);
            // RTC register Count Increment
            rtc_read_count++;
            if(rtc_read_count >= RX_LEN)
            {
                rtc_read_count = 0;
                start_rtc_read = 0;
                // Mask control bits and convert BCD → binary
                // Skip Day-of-Week register
                seconds = BCD2Binary(rtc_raw[0] & 0x7F);
                minutes = BCD2Binary(rtc_raw[1]);
                hours   = BCD2Binary(rtc_raw[2] & 0x3F);
                date    = BCD2Binary(rtc_raw[4]);
                month   = BCD2Binary(rtc_raw[5]);
                year    = BCD2Binary(rtc_raw[6]);
                rtc_abs_seconds = calendar_to_seconds(year + 2000U, month, date, hours, minutes, seconds);
            }
        }
        else
        {
            rtc_read_count++;
        }
    }
    if(start_rtc_write)
    {
        if(rtc_write_count != 3)
        {
            //gps_abs_seconds = seconds_to_calendar(year + 2000U, month, date, hours, minutes, seconds);
            // Read one RTC register
            RTC_WriteByte(rtc_set, rtc_write_count);
            // RTC register Count Increment
            rtc_write_count++;
            if(rtc_write_count >= RX_LEN)
            {
                rtc_write_count = 0;
                start_rtc_write = 0;
            }
        }
        else
        {
            rtc_write_count++;
        }
    }

}

void v_100msTasks(void)
{
    // TODO: Update ref_odo and implement some way to find Normal tag from given array for next tags
//    if(((int32_t)distance_m - (int32_t)ref_odo) >= (reg_type1.TLI_Packet_reg_type1.dist_nxt_rfid[rfid_Count] + (LOCATION_ACCURACY_WINDOW/2)))  //Assuming Location accuracy window is 100 meter.
//    {
//        rfid_Miss_Count++;
//        uint8_t dbnum;
//
//        if(rfid_db_count != 0)
//            dbnum = (rfid_db_head + RFID_DB_SIZE - 1) % RFID_DB_SIZE;
//        else
//            dbnum = 0;
//
//        rfid_db[dbnum].location_check = 2;
//    }
//    if(rfid_Miss_Count >= 3)
//    {
//        rfid_Miss_Count = 3;
//        input_write.raw_flags[1] |= (1U << 21);
//    }
}

void v_1sTasks(void)
{
    #if NMS_TEST_ENABLE

    nms_test_timer++;

    if (nms_test_timer >= NMS_TEST_PERIOD_SEC)
    {
        nms_test_timer = 0U;

        send_loco_postion_info_to_nms(1);
    }

    #endif

    /* ---------- Fallback 1-second CPU time update + CAN send ---------- */
    start_rtc_read = 1;
    // seconds_uptime++;   // already in your rtiNotification (okay to keep here
    // too if not)
    if (fallback_active)
    {
        /* Always count how long we've been in fallback */
        seconds_in_fallback++;
        /* Advance CPU time only while still valid */
        if (cpu_time_valid)
        {
            cpu_time_sec = rtc_abs_seconds;

            tlm_tod_sec = cpu_time_sec % 86400U;

            /* Save current timer tick */
            second_reference_tick = get_timer_tick();

            /* Close previous second block */
            //            distance_db[current_sec_index].sec_span_ms = get_elapsed_ms();

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
            /* debug print every 5s while time is still valid in fallback */
            if ((seconds_in_fallback % 5U) == 0U)
            {
                uint8_t dbgFb[80];
                uint32 lenFb = sprintf((char *)dbgFb, "FALLBACK: cpu_time=%lu, sec_fb=%lu\r\n",
                                       (unsigned long)cpu_time_sec, (unsigned long)seconds_in_fallback);
                sciSend(GPS1_SCI, lenFb, dbgFb);
            }
            if (seconds_in_fallback == FALLBACK_TIMEOUT_SEC)
            {
                cpu_time_valid = 0;
                //! Switch to SF mode
                uint8_t dbgTime[96];
                uint32 lenT = sprintf((char *)dbgTime, "TIME INVALID: cpu=%lu fb_sec=%lu\r\n",
                                      (unsigned long)cpu_time_sec, (unsigned long)seconds_in_fallback);
                sciSend(GPS1_SCI, lenT, dbgTime);
            }
        }
    }
    /* Update CPU_TIME_INV fault bit */
    if (cpu_time_valid == 0u)
    {
        gps_faults |= GPSF_CPU_TIME_INV;
    }
    else
    {
        gps_faults &= (uint8_t)~GPSF_CPU_TIME_INV;
    }
    /* --- Transmit time on CAN each RTI tick (1 Hz) --- */
    can_scheduler_1s_tick();
    /* Optional debug print of CAN payload on SCI2 */
    //debug_print_can_payload();

    //    if(rfidDataMatchFlag)
    //    {
    //        rfidDataMatchFlag = 0;
    //        rfid_process_queues_1s();
    //        if(rfidMissCount1 >= 3)
    //        {
    //            rfid2_fault = 1;
    //            //Reader 2 NMS Fault
    //        }
    //        else if(rfidMissCount2 >= 3)
    //        {
    //            rfid1_fault = 1;
    //            //Reader 1 NMS Fault
    //        }
    //    }

}

/* USER CODE END */
