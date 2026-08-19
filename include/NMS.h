#ifndef NMS_H
#define NMS_H

#include <stdint.h>

#define NMS_TX_MB               canMESSAGE_BOX20
#define NMS_MAX_FAULT_CODES    10U

#define NMS_PKT_TYPE_INFO                   0x11U
#define NMS_PKT_TYPE_POS_INFO               0x12U
#define NMS_PKT_TYPE_TSR_INFO               0x13U
#define NMS_PKT_TYPE_ADJ_INFO               0x14U
#define NMS_PKT_TYPE_FIELD_INPUT_STATUS     0x15U
#define NMS_PKT_TYPE_FIELD_INPUT_EVENT      0x16U
#define NMS_PKT_TYPE_HEALTH                 0x17U
#define NMS_PKT_TYPE_FAULT                  0x19U
#define NMS_PKT_TYPE_RSSI                   0x1BU

#define NMS_PKT_TYPE_ACK                    0x1FU

#define NMS_PAYLOAD_BYTES       6U
#define NMS_MAX_FRAGMENTS       64U
#define NMS_MAX_PAYLOAD_LEN     128U

void nms_send_health(void);
//typedef struct
//{
//    uint16_t SOF ;
//    uint8_t msg_type;
//    uint16_t msg_len;
//    uint16_t message_sequence;
//    uint32_t loco_kavach_id;
//    uint16_t nms_system_id;
//    uint8_t system_version;
//    uint8_t date_day;
//    uint8_t date_month;
//    uint8_t date_year;
//    uint8_t time_hour ;
//    uint8_t time_min  ;
//    uint8_t time_sec  ;
//    uint8_t event_count ;
//    uint16_t event_id ;
//    /* 1 - 16 */
//
//    uint8_t radio1_health;
//    uint8_t radio2_health;
//    uint8_t radio1_input_supply;
//    uint8_t radio2_input_supply;
//    int8_t radio1_temperature;
//    int8_t radio2_temperature;
//    uint8_t radio1_pa_temperature;
//    uint8_t radio2_pa_temperature;
//    uint8_t radio1_pa_voltage;
//    uint8_t radio2_pa_voltage;
//    uint8_t radio1_tx_pa_current;
//    uint8_t radio2_tx_pa_current;
//    uint8_t radio1_reverse_power;
//    uint8_t radio2_reverse_power;
//    uint8_t radio1_forward_power;
//    uint8_t radio2_forward_power;
//
//    /* 17 */
//    uint16_t stationary_pkt_time_offset;
//
//    /* 18 - 26 */
//    uint8_t active_gps_number;
//    uint8_t gps1_view_status;
//    uint8_t gps2_view_status;
//    uint8_t gps1_seconds;
//    uint8_t gps2_seconds;
//    uint8_t gps1_satellites;
//    uint8_t gps1_cno_max;
//    uint8_t gps2_satellites;
//    uint8_t gps2_cno_max;
//
//    /* 27 - 28 */
//    uint16_t gps1_link_status;
//    uint16_t gps2_link_status;
//
//    /* 29 - 32 */
//    uint8_t gsm1_rssi;
//    uint8_t gsm2_rssi;
//    uint8_t current_running_key;
//    uint8_t remaining_keys;
//
//    /* 33 */
//    uint16_t session_key_checksum;
//
//    /* 34 - 37 */
//    uint16_t dmi1_link_status;
//    uint16_t dmi2_link_status;
//    uint16_t rfid1_link_status;
//    uint16_t rfid2_link_status;
//
//    /* 38 */
//    uint16_t duplicate_missing_rfid_tag;
//
//    /* 39 */
//    uint32_t missing_linked_rfid_tag;
//
//    /* 40 */
//    uint32_t computed_tlm_status;
//
//    /* 41 - 45 */
//    uint8_t train_configuration_change;
//    uint8_t bootup_sequence_error;
//    uint8_t selected_train_formation;
//    uint8_t selected_cab;
//    uint8_t brake_application_reason;
//
//    /* 46 */
//    uint8_t station_general_sos[3];
//
//    /* 47 */
//    uint8_t station_loco_specific_sos[3];
//
//    /* 48 */
//    uint32_t collision_detection;
//
//    /* 49 - 54 */
//    uint8_t loco_self_sos;
//    uint8_t kavach_connection;
//    uint8_t biu_isolated;
//    uint8_t eb_bypassed;
//    uint8_t kavach_territory;
//    uint8_t brake_interface_error;
//
//    /* 55 */
//    uint16_t onboard_modules_health;
//
//    /* 56 */
//    uint16_t conflict_route_rfid;
//
//    /* 57 */
//    uint32_t train_configuration_checksum;
//} nms_health_event_t;

typedef struct
{
    /* =========================================================
     * MESSAGE HEADER
     * ========================================================= */

    uint16_t SOF;                    /* 2 bytes: 0xAAAA */
    uint8_t  msg_type;               /* 1 byte: 0x17 */
    uint16_t msg_len;                /* 2 bytes */

    uint16_t message_sequence;       /* 2 bytes */

    uint16_t stationary_kavach_id;   /* 2 bytes */
    uint16_t nms_system_id;           /* 2 bytes */

    uint8_t  system_version;          /* 1 byte */

    /* Date: DD/MM/YY */
    uint8_t  date_day;                /* 1 byte */
    uint8_t  date_month;              /* 1 byte */
    uint8_t  date_year;               /* 1 byte */

    /* Time: HH:MM:SS */
    uint8_t  time_hour;               /* 1 byte */
    uint8_t  time_min;                /* 1 byte */
    uint8_t  time_sec;                /* 1 byte */

    uint8_t  event_count;             /* 1 byte */
    uint16_t event_id;                /* 2 bytes */


    /* =========================================================
     * EVENT DATA
     * Stationary KAVACH Health Event Fields 1-45
     * ========================================================= */

    /* 1 */
    int8_t   system_temperature;

    /* 2 */
    uint8_t  active_radio_number;

    /* 3 - 4 */
    uint8_t  radio1_health;
    uint8_t  radio2_health;

    /* 5 - 6 */
    uint8_t  radio1_input_supply;
    uint8_t  radio2_input_supply;

    /* 7 - 8 */
    int8_t   radio1_temperature;
    int8_t   radio2_temperature;

    /* 9 - 10 */
    uint8_t  radio1_pa_temperature;
    uint8_t  radio2_pa_temperature;

    /* 11 - 12 */
    uint8_t  radio1_pa_supply_voltage;
    uint8_t  radio2_pa_supply_voltage;

    /* 13 - 14 */
    uint8_t  radio1_tx_pa_current;
    uint8_t  radio2_tx_pa_current;

    /* 15 - 16 */
    uint8_t  radio1_reverse_power;
    uint8_t  radio2_reverse_power;

    /* 17 - 18 */
    uint8_t  radio1_forward_power;
    uint8_t  radio2_forward_power;

    /* 19 - 20 */
    uint8_t  current_running_key;
    uint8_t  remaining_keys;

    /* 21 */
    uint16_t session_key_checksum;

    /* 22 */
    uint8_t  allocated_time_slot;

    /* 23 */
    uint16_t new_loco_regular_pkt_time_offset;

    /* 24 */
    uint8_t  loco_count;

    /* 25 - 26 */
    uint8_t  radio1_rx_packet_count;
    uint8_t  radio2_rx_packet_count;

    /* 27 */
    uint8_t  active_gps_number;

    /* 28 - 29 */
    uint8_t  gps1_view;
    uint8_t  gps2_view;

    /* 30 - 31 */
    uint8_t  gps1_seconds;
    uint8_t  gps2_seconds;

    /* 32 - 35 */
    uint8_t  gps1_satellites;
    uint8_t  gps1_cno_max;
    uint8_t  gps2_satellites;
    uint8_t  gps2_cno_max;

    /* 36 - 37 */
    uint8_t  gsm1_rssi;
    uint8_t  gsm2_rssi;

    /* 38 */
    uint16_t missing_rfid;

    /* 39 */
    uint16_t invalid_rfid;

    /* 40 */
    uint16_t conflict_route_rfid;

    /* 41 */
    uint16_t conflicting_tin;

    /* 42 */
    uint16_t missing_tin;

    /* 43 */
    uint32_t loco_specific_sos;

    /* 44 */
    uint32_t train_exit_mode;

    /* 45 */
    uint16_t station_modules_health;

} nms_health_event_t;

typedef struct
{
    /* =========================================================
     * STATIONARY KAVACH INFORMATION MESSAGE
     * ========================================================= */

    /* Field 1 */
    uint16_t SOF;                       /* 2 bytes: 0xAAAA */

    /* Field 2 */
    uint8_t  msg_type;                  /* 1 byte: 0x11 */

    /* Field 3 */
    uint16_t msg_len;                   /* 2 bytes */

    /* Field 4 */
    uint16_t message_sequence;          /* 2 bytes: 0-65535 */

    /* Field 5 */
    uint16_t stationary_kavach_id;      /* 2 bytes */

    /* Field 6 */
    uint16_t nms_system_id;             /* 2 bytes */

    /* Field 7 */
    uint8_t  system_version;            /* 1 byte
                                           0x00 = Version 3.2
                                           0x01 = Version 4.0 */

    /* Field 8: Date */
    uint8_t  date_day;                  /* 1 byte */
    uint8_t  date_month;                /* 1 byte */
    uint8_t  date_year;                 /* 1 byte */

    /* Field 9: Time */
    uint8_t  time_hour;                 /* 1 byte */
    uint8_t  time_min;                  /* 1 byte */
    uint8_t  time_sec;                  /* 1 byte */

    /* Field 10 */
    uint8_t  station_active_radio;      /* 1 byte
                                           0xF1 = Radio 1
                                           0xF2 = Radio 2
                                           0xE1 = Ethernet 1
                                           0xE2 = Ethernet 2
                                           Other = unknown */

    /* Field 11 */
    uint8_t  sof_tx_byte1;              /* 1 byte: 0xA5 */

    /* Field 12 */
    uint8_t  sof_tx_byte2;              /* 1 byte: 0xC3 */

    /*
     * Field 13 in the image is CRC, but immediately before it
     * comes the variable Station Regular / Access Authority /
     * Additional Emergency Packet / etc.
     *
     * The specification does not provide a fixed width for this
     * packet, so store its length separately.
     */
    uint16_t station_packet_len;

    /*
     * Station Regular / Access Authority /
     * Additional Emergency Packet / etc.
     *
     * Actual contents are according to the
     * KAVACH Radio Communication Protocol.
     */
    uint8_t station_packet[NMS_MAX_PAYLOAD_LEN];

} nms_kavach_info_t;

typedef struct
{
    uint16_t SOF;
    uint8_t msg_type;
    uint16_t msg_len;
    uint16_t message_sequence;
    uint32_t stationary_kavach_id;
    uint16_t nms_system_id;
    uint8_t system_version;
    uint8_t date_day;
    uint8_t date_month;
    uint8_t date_year;
    uint8_t time_hour;
    uint8_t time_min;
    uint8_t time_sec;
    uint8_t onboard_active_radio;
    uint8_t sof_tx_byte1;
    uint8_t sof_tx_byte2;
    uint8_t no_of_ma_section_count;
    uint8_t route_id;
} nms_kavach_postion_t;

//typedef struct
//{
//    uint16_t SOF;
//    uint8_t msg_type;
//    uint16_t msg_len;
//    uint16_t message_sequence;
//    uint32_t loco_kavach_id;
//    uint16_t nms_system_id;
//    uint8_t system_version;
//    uint8_t date_day;
//    uint8_t date_month;
//    uint8_t date_year;
//    uint8_t time_hour;
//    uint8_t time_min;
//    uint8_t time_sec;
//    uint16_t kavach_subsystem_type;
//    uint8_t staton_radio1_rssi_sample_count;
//    uint16_t ref_rfid_tag;
//    uint32_t abs_ref_rfid_tag;
//    uint8_t rssi_value;
//    uint8_t staton_radio2_rssi_sample_count;
//    uint16_t ref_rfid_tag2;
//    uint32_t abs_ref_rfid_tag2;
//    uint8_t rssi_value2;
//} nms_lkavach_rssi_t;

typedef struct
{
    /* =========================================================
     * STATIONARY KAVACH RSSI MESSAGE
     * Message Type = 0x21
     * ========================================================= */

    /* Field 1 */
    uint16_t SOF;                         /* 2 bytes: 0xAAAA */

    /* Field 2 */
    uint8_t  msg_type;                    /* 1 byte: 0x21 */

    /* Field 3 */
    uint16_t msg_len;                     /* 2 bytes */

    /* Field 4 */
    uint16_t message_sequence;            /* 2 bytes */

    /* Field 5 */
    uint16_t stationary_kavach_id;        /* 2 bytes */

    /* Field 6 */
    uint16_t nms_system_id;               /* 2 bytes */

    /* Field 7 */
    uint8_t  system_version;              /* 1 byte */

    /* Field 8 */
    uint8_t  date_day;                    /* 1 byte */
    uint8_t  date_month;                  /* 1 byte */
    uint8_t  date_year;                   /* 1 byte */

    /* Field 9 */
    uint8_t  time_hour;                   /* 1 byte */
    uint8_t  time_min;                    /* 1 byte */
    uint8_t  time_sec;                    /* 1 byte */

    /* Field 10 */
    uint32_t loco_kavach_id;              /* 3 bytes transmitted */

    /* Field 11 */
    uint8_t  onboard_radio1_rssi_sample_count;

    /* Field 12 */
    uint16_t ref_rfid_tag1;

    /* Field 13 */
    uint32_t abs_location1;               /* 3 bytes transmitted */

    /* Field 14 */
    int16_t  rssi_value1;                 /* 2 bytes, signed */

    /* Field 15 */
    uint8_t  onboard_radio2_rssi_sample_count;

    /* Field 16 */
    uint16_t ref_rfid_tag2;

    /* Field 17 */
    uint32_t abs_location2;               /* 3 bytes transmitted */

    /* Field 18 */
    int16_t  rssi_value2;                 /* 2 bytes, signed */

} nms_skavach_rssi_t;

//typedef struct
//{
//    uint16_t SOF;
//    uint8_t msg_type;
//    uint16_t msg_len;
//    uint16_t message_sequence;
//    uint32_t kavach_subsystem_id;
//    uint16_t nms_system_id;
//    uint8_t system_version;
//    uint8_t date_day;
//    uint8_t date_month;
//    uint8_t date_year;
//    uint8_t time_hour;
//    uint8_t time_min;
//    uint8_t time_sec;
//    uint16_t kavach_subsystem_type;
//    uint8_t total_fault_codes;
//    uint8_t module_id;
//    uint8_t fault_code_type;
//    uint8_t fault_code;
//} nms_kavach_fault_msg_t;

typedef struct
{
    uint8_t module_id;
    uint8_t fault_code_type;
    uint16_t fault_code;
} nms_fault_entry_t;


typedef struct
{
    /* =========================================================
     * KAVACH FAULT MESSAGE TO NMS
     * ========================================================= */

    /* Field 1 */
    uint16_t SOF;
    /* 0xAAAA = E1 / Network channel
     * 0xBBBB = GPRS channel
     */

    /* Field 2 */
    uint8_t msg_type;
    /* 0x19 */

    /* Field 3 */
    uint16_t msg_len;
    /* Message Type through CRC inclusive */

    /* Field 4 */
    uint16_t message_sequence;
    /* Last received KAVACH subsystem message sequence number */

    /* Field 5 */
    uint32_t kavach_subsystem_id;
    /* Only 24 bits are transmitted */

    /* Field 6 */
    uint16_t nms_system_id;

    /* Field 7 */
    uint8_t system_version;

    /* Field 8: Date */
    uint8_t date_day;
    uint8_t date_month;
    uint8_t date_year;

    /* Field 9: Time */
    uint8_t time_hour;
    uint8_t time_min;
    uint8_t time_sec;

    /* Field 10 */
    uint8_t kavach_subsystem_type;
    /*
     * 0x11 = Stationary KAVACH
     * 0x22 = Onboard KAVACH
     * 0x33 = TSRMS
     */

    /* Field 11 */
    uint8_t total_fault_codes;
    /* Maximum = 10 */

    /*
     * Fields 12-14
     *
     * These are repeated for each fault code.
     */
    nms_fault_entry_t fault[NMS_MAX_FAULT_CODES];

} nms_kavach_fault_msg_t;

typedef struct
{
    uint8_t payload[NMS_MAX_PAYLOAD_LEN];
    uint16_t payload_len;
    uint8_t seq_total;

    nms_health_event_t health;
    nms_kavach_info_t kavach_info;
    nms_kavach_postion_t kavach_postion;
    nms_skavach_rssi_t skavach_rssi;
    nms_kavach_fault_msg_t kavach_fault_msg;
} nms_tx_ctx_t;

extern nms_tx_ctx_t nms_ctx;

void send_skavach_info_msg_to_nms(uint8_t skavach_info_frame_num);
//uint16_t send_skavach_info_msg_to_nms(uint8_t *buf);

void send_skavach_health_msg_to_nms(uint8_t skavach_health_frame_num);
void send_loco_postion_info_to_nms(uint8_t stn_loco_postion_frame_num);
void send_skavach_fault_msg_to_nms(uint8_t skavach_fault_frame_num);
void send_skavach_rssi_msg_to_nms(uint8_t skavach_rssi_frame_num);

#endif
