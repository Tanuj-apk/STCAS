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
//#include <StateMachine.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can.h"
#include "can_if.h"
#include "counter_card.h"
//#include "dmi_can.h"
#include "gps.h"
#include "gsm_rx.h"
#include "i2c_UD.h"
//#include "output_card.h"
//#include "pulse_generator.h"
#include "radio.h"
//#include "rfid_rx.h"
#include "rti.h"
#include "sci.h"
#include "sys_common.h"
#include "system.h"
#include "input_card.h"
//#include "BIUController.h"
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
int count = 0;
int frames_of_arp = 0;
bool some_flag = 0;
uint8_t flagSet;
uint8_t DataLogCheck = 0;
uint8_t DataLogCount = 0;
void v_1msTasks(void);
void v_5msTasks(void);
void v_10msTasks(void);
void v_100msTasks(void);
void v_1sTasks(void);
void KavachInit(void);
//void MainStateMachine(void);
//void MasterStateChange(State_t next, cond_mask_t mask);

#define REVERSE_TIMEOUT_SEC   600U   // 10 minutes
uint8_t reverse_timeout_flag = 0;
//Test Variables
//uint8_t BIU_Test = 1;

/* ============================================================
 *  MAIN
 * ============================================================ */
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    KavachInit();

    while (1)
    {
        //        if(!can_manager_poll_startup())
        //            continue;

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
            //            v_10msTasks();
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
//void MainStateMachine(void)
//{
//    // ? Take a clean, consistent snapshot of input_write
//    input_swap();                    // atomic snapshot
//
//    cond_mask_t mask = compute_conditions();
//    State_t next = fsm_step(g_current, mask);
//    MasterStateChange(next, mask);
//}

//void MasterStateChange(State_t next, cond_mask_t mask)
//{
//    if((next != STATE_SB) && (g_current == STATE_SB))
//    {
//        input_write.raw_flags[0] &= ~(1U << 11); //New Train Formation
//        input_write.raw_flags[0] &= ~(1U << 12); //No New Train Formation
//        input_write.raw_flags[0] &= ~(1U << 13); //Train Config available
//        input_write.raw_flags[0] &= ~(1U << 14); //Train Config not available
//    }
//    if((next != STATE_SR) && (g_current == STATE_SR))
//    {
//        input_write.raw_flags[1] &= ~(1U << 21); //Three Consecutive Normal Tags Missed
//    }
//    if((next != STATE_OS) && (g_current == STATE_OS))
//    {
//        input_write.raw_flags[1] &= ~(1U << 30); //
//    }
//    if((next != STATE_OV) && (g_current == STATE_OV))
//    {
//        input_write.raw_flags[1] &= ~(1U << 28); //
//    }
//    if((next != STATE_RV) && (g_current == STATE_RV))
//    {
//        reverse_timeout_flag = 0;
//        input_write.raw_flags[1] &= ~(1U << 8);
//        reverse_distance_flag = 0U;
//    }
//    if((next == STATE_RV) && (g_current != STATE_RV))
//    {
//        reverse_start_time = seconds_uptime;
//    }
//    if((next == STATE_SF) && (g_current != STATE_SF))
//    {
//        input_write.raw_flags[0] &= ~(1U << 4);
//        input_write.raw_flags[0] |= (1U << 5);
//    }
//
//    g_previous = g_current;
//    g_current = next;
//}

void KavachInit(void)
{
    uint8_t msg[] = "CPU GPS1+GPS2 RX Ready\r\n";

    systemInit();
    sciInit();
    i2cInit();
    _enable_IRQ();
    canInit();
    canEnableErrorNotification(canREG1);
//    DMI_init();
//    eqep_speed_init();

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

//    fsm_init();
    can_manager_init();

    start_rtc_write = 1;

    //BIU Check
//    BIU_Init();
}

void v_1msTasks(void)
{

}

void v_5msTasks(void)
{
    //    output_card_set_bit(OUT_EMERGENCY_BRAKE_1);
    //    output_card_set_bit(OUT_EMERGENCY_BRAKE_2);
    //    output_card_set_bit(OUT_HORN);
    //    output_card_send();
    if(radio_can_arp_transmit_flag)
    {
        radio_build_fragment(tx_buf, RADIO_PKT_TYPE_ARP, radio_ctx.seq_total, frames_of_arp);
        canTransmit(canREG1, tx_mb, tx_buf);
        frames_of_arp += 1;
        if(frames_of_arp >= 5)
        {
            radio_can_arp_transmit_flag = 0;
        }
    }
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

//    BIUStateMachine();
}

void v_100msTasks(void)
{
//    eqep_speed_update();
    //    DMI_update(); //! Commented because code gets stuck in while CAN Tx check

    if(!DataLogCheck)
    {
        if(DataLogCount < 3)
        {
            send_Data_Log(DataLogCount);
            DataLogCount++;
        }
    }
//    Target_Prune();
//    MainStateMachine();
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
    /* ---------- Fallback 1-second CPU time update + CAN send ---------- */
    start_rtc_read = 1;
    count++;
    check_for_transmit_arp();
    /* Only for testing     
    if(count >= 10)
    {
        gsm_start_request(GSM_1);
        radio_send_aap(RADIO_ID_1);
        radio_send_arp(RADIO_ID_1);
        radio_send_reg_type1(RADIO_ID_1);
        radio_send_reg_type2(RADIO_ID_1);
        count = 0;
    }
     */
    //seconds_uptime++;   // already in your rtiNotification (okay to keep here too if not)
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

    //    counter_card_set_bit(COUNTER_SOS);
    //    counter_card_set_bit(COUNTER_BRAKE);
    //    counter_card_send();

    send_Counter_Change_req(flagSet);
    flagSet++;
    if(flagSet >= 32)
    {
        flagSet = 0;
    }

    if(DataLogCheck)
    {
        DataLogCount = 0;
        DataLogCheck = 0;
    }
    else if(!DataLogCheck)
    {
        DataLogCheck = 1;
    }
    /* =========================================
     * OSMA EXPIRY (Condition 89)
     * ========================================= */
//    if(osma_active)
//    {
//        if((seconds_uptime - last_osma_rx_time) >= OSMA_HOLD_TIME)
//        {
//            osma_active = 0;   // expiry
//            input_write.raw_flags[1] |= (1U << 30);
//        }
//    }

    /* =========================================
     * OVERRIDE TIMEOUT CONDITION
     * ========================================= */
//    if(override_active)
//    {
//        if((seconds_uptime - override_start_time) >= OV_ACTIVE_TIME)
//        {
//            override_active = 0;
//            input_write.raw_flags[1] |= (1U << 28);  // Override timeout condition
//        }
//    }

    /* =========================================
     * REVERSE MODE TIMEOUT
     * ========================================= */
//    if(g_current == STATE_RV)
//    {
//        if((seconds_uptime - reverse_start_time) >= REVERSE_TIMEOUT_SEC)
//        {
//            reverse_active = 0;
//            reverse_timeout_flag = 1;  // Reverse timeout bit
//            input_write.raw_flags[1] |= (1U << 8);  // Reverse condition
//        }
//    }
    //BIU Test?
//    if(BIU_Test)
//    {
//        BIU_Test = 0;
//        Target_Set(0, TARGET_EOA, 12000U, 0U);
//        Target_Set(1, TARGET_PSR, 8000U, 60U);
//    }
//    BrakeSupervisor();
}

/* USER CODE END */
