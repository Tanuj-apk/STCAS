#ifndef NMS_H
#define NMS_H

#include <stdint.h>
#include "can.h"

#define NMS_TX_CAN_ID                    0x0220U
#define NMS_ACK_CAN_ID                   0x0221U
#define NMS_TX_MB                        canMESSAGE_BOX20

/* CAN envelope used between the TMS570 and the Ethernet gateway. */
#define NMS_CAN_ENVELOPE_TYPE            0x01U
#define NMS_PAYLOAD_BYTES                6U
#define NMS_MAX_FRAGMENTS                63U
#define NMS_MAX_PAYLOAD_LEN              (NMS_PAYLOAD_BYTES * NMS_MAX_FRAGMENTS)

#define NMS_PKT_TYPE_INFO                0x11U
#define NMS_PKT_TYPE_POS_INFO            0x12U
#define NMS_PKT_TYPE_TSR_INFO            0x13U
#define NMS_PKT_TYPE_ADJ_INFO            0x14U
#define NMS_PKT_TYPE_FIELD_INPUT_STATUS  0x15U
#define NMS_PKT_TYPE_FIELD_INPUT_EVENT   0x16U
#define NMS_PKT_TYPE_HEALTH              0x17U
#define NMS_PKT_TYPE_ONBOARD_HEALTH      0x18U
#define NMS_PKT_TYPE_FAULT               0x19U
#define NMS_PKT_TYPE_LOCO_RSSI           0x20U
#define NMS_PKT_TYPE_STATION_RSSI        0x21U
#define NMS_PKT_TYPE_RSSI                NMS_PKT_TYPE_STATION_RSSI
#define NMS_PKT_TYPE_ACK                 0x1FU

#define NMS_SOF_E1                       0xAAAAU
#define NMS_SOF_GPRS                     0xBBBBU
#define NMS_SYSTEM_VERSION_4             0x01U
#define NMS_MAX_FAULT_CODES              10U
#define NMS_MAX_EMBEDDED_PACKET_LEN      256U
#define NMS_MAX_MA_SECTIONS              32U
#define NMS_MAX_RELAY_STATUS_BYTES       128U
#define NMS_MAX_RELAY_EVENTS             64U

#define NMS_TRANSACTION_QUEUE_SIZE       8U
#define NMS_ACK_TIMEOUT_MS               100U
#define NMS_ACK_MAX_RETRIES              3U
#define NMS_ACK_REASSEMBLY_LEN           64U

typedef struct
{
    uint16_t message_sequence;
    uint16_t stationary_kavach_id;
    uint16_t nms_system_id;
    uint8_t system_version;
    uint8_t date_day;
    uint8_t date_month;
    uint8_t date_year;
    uint8_t time_hour;
    uint8_t time_min;
    uint8_t time_sec;
} nms_stationary_header_t;

typedef struct
{
    uint16_t message_sequence;
    uint32_t loco_kavach_id; /* Low 24 bits are transmitted. */
    uint16_t nms_system_id;
    uint8_t system_version;
    uint8_t date_day;
    uint8_t date_month;
    uint8_t date_year;
    uint8_t time_hour;
    uint8_t time_min;
    uint8_t time_sec;
} nms_onboard_header_t;

typedef struct
{
    nms_stationary_header_t header;
    uint8_t station_active_radio;
    uint16_t station_packet_len;
    uint8_t station_packet[NMS_MAX_EMBEDDED_PACKET_LEN];
} nms_kavach_info_t;

typedef struct
{
    nms_stationary_header_t header;
    uint8_t onboard_active_radio;
    uint16_t loco_packet_len;
    uint8_t loco_packet[NMS_MAX_EMBEDDED_PACKET_LEN];
    uint8_t ma_section_count;
    uint16_t route_id[NMS_MAX_MA_SECTIONS];
} nms_kavach_position_t;

/* Backwards-compatible spelling retained for existing code. */
typedef nms_kavach_position_t nms_kavach_postion_t;

typedef struct
{
    nms_stationary_header_t header;
    uint16_t packet_len;
    uint8_t packet[NMS_MAX_EMBEDDED_PACKET_LEN];
} nms_tsr_info_t;

typedef struct
{
    nms_stationary_header_t header;
    uint16_t packet_len;
    uint8_t packet[NMS_MAX_EMBEDDED_PACKET_LEN];
} nms_adjacent_info_t;

typedef struct
{
    nms_stationary_header_t header;
    uint16_t total_event_relays;
    uint8_t relay_status[NMS_MAX_RELAY_STATUS_BYTES];
} nms_field_input_status_t;

typedef struct
{
    uint16_t relay_address;
    uint8_t relay_status;
} nms_relay_event_t;

typedef struct
{
    nms_stationary_header_t header;
    uint8_t relay_event_count;
    nms_relay_event_t event[NMS_MAX_RELAY_EVENTS];
} nms_field_input_event_t;

typedef struct
{
    nms_stationary_header_t header;
    uint8_t event_count;
    uint16_t event_id;
    int8_t system_temperature;
    uint8_t active_radio_number;
    uint8_t radio1_health;
    uint8_t radio2_health;
    uint8_t radio1_input_supply;
    uint8_t radio2_input_supply;
    int8_t radio1_temperature;
    int8_t radio2_temperature;
    uint8_t radio1_pa_temperature;
    uint8_t radio2_pa_temperature;
    uint8_t radio1_pa_supply_voltage;
    uint8_t radio2_pa_supply_voltage;
    uint8_t radio1_tx_pa_current;
    uint8_t radio2_tx_pa_current;
    uint8_t radio1_reverse_power;
    uint8_t radio2_reverse_power;
    uint8_t radio1_forward_power;
    uint8_t radio2_forward_power;
    uint8_t current_running_key;
    uint8_t remaining_keys;
    uint16_t session_key_checksum;
    uint8_t allocated_time_slot;
    uint16_t new_loco_regular_pkt_time_offset;
    uint8_t loco_count;
    uint8_t radio1_rx_packet_count;
    uint8_t radio2_rx_packet_count;
    uint8_t active_gps_number;
    uint8_t gps1_view;
    uint8_t gps2_view;
    uint8_t gps1_seconds;
    uint8_t gps2_seconds;
    uint8_t gps1_satellites;
    uint8_t gps1_cno_max;
    uint8_t gps2_satellites;
    uint8_t gps2_cno_max;
    uint8_t gsm1_rssi;
    uint8_t gsm2_rssi;
    uint16_t missing_rfid;
    uint16_t invalid_rfid;
    uint16_t conflict_route_rfid;
    uint16_t conflicting_tin;
    uint16_t missing_tin;
    uint32_t loco_specific_sos;
    uint32_t train_exit_mode;
    uint16_t station_modules_health;
} nms_health_event_t;

typedef struct
{
    nms_onboard_header_t header;
    uint8_t event_count;
    uint16_t event_id;
    uint8_t radio1_health;
    uint8_t radio2_health;
    uint8_t radio1_input_supply;
    uint8_t radio2_input_supply;
    int8_t radio1_temperature;
    int8_t radio2_temperature;
    uint8_t radio1_pa_temperature;
    uint8_t radio2_pa_temperature;
    uint8_t radio1_pa_voltage;
    uint8_t radio2_pa_voltage;
    uint8_t radio1_tx_pa_current;
    uint8_t radio2_tx_pa_current;
    uint8_t radio1_reverse_power;
    uint8_t radio2_reverse_power;
    uint8_t radio1_forward_power;
    uint8_t radio2_forward_power;
    uint16_t stationary_pkt_time_offset;
    uint8_t active_gps_number;
    uint8_t gps1_view_status;
    uint8_t gps2_view_status;
    uint8_t gps1_seconds;
    uint8_t gps2_seconds;
    uint8_t gps1_satellites;
    uint8_t gps1_cno_max;
    uint8_t gps2_satellites;
    uint8_t gps2_cno_max;
    uint16_t gps1_link_status;
    uint16_t gps2_link_status;
    uint8_t gsm1_rssi;
    uint8_t gsm2_rssi;
    uint8_t current_running_key;
    uint8_t remaining_keys;
    uint16_t session_key_checksum;
    uint16_t dmi1_link_status;
    uint16_t dmi2_link_status;
    uint16_t rfid1_link_status;
    uint16_t rfid2_link_status;
    uint16_t duplicate_missing_rfid_tag;
    uint32_t missing_linked_rfid_tag;
    uint32_t computed_tlm_status;
    uint8_t train_configuration_change;
    uint8_t bootup_sequence_error;
    uint8_t selected_train_formation;
    uint8_t selected_cab;
    uint8_t brake_application_reason;
    uint8_t station_general_sos[3];
    uint8_t station_loco_specific_sos[3];
    uint32_t collision_detection;
    uint8_t loco_self_sos;
    uint8_t kavach_connection;
    uint8_t biu_isolated;
    uint8_t eb_bypassed;
    uint8_t kavach_territory;
    uint8_t brake_interface_error;
    uint16_t onboard_modules_health;
    uint16_t conflict_route_rfid;
    uint32_t train_configuration_checksum;
} nms_onboard_health_t;

typedef struct
{
    nms_onboard_header_t header;
    uint16_t stationary_kavach_id;
    uint8_t station_radio1_rssi_sample_count;
    uint16_t ref_rfid_tag1;
    uint32_t abs_ref_rfid_tag1;
    int16_t rssi_value1;
    uint8_t station_radio2_rssi_sample_count;
    uint16_t ref_rfid_tag2;
    uint32_t abs_ref_rfid_tag2;
    int16_t rssi_value2;
} nms_loco_rssi_t;

typedef struct
{
    nms_stationary_header_t header;
    uint32_t loco_kavach_id;
    uint8_t onboard_radio1_rssi_sample_count;
    uint16_t ref_rfid_tag1;
    uint32_t abs_location1;
    int16_t rssi_value1;
    uint8_t onboard_radio2_rssi_sample_count;
    uint16_t ref_rfid_tag2;
    uint32_t abs_location2;
    int16_t rssi_value2;
} nms_skavach_rssi_t;

typedef struct
{
    uint8_t module_id;
    uint8_t fault_code_type;
    uint16_t fault_code;
} nms_fault_entry_t;

typedef struct
{
    uint16_t sof;
    uint16_t message_sequence;
    uint32_t kavach_subsystem_id;
    uint16_t nms_system_id;
    uint8_t system_version;
    uint8_t date_day;
    uint8_t date_month;
    uint8_t date_year;
    uint8_t time_hour;
    uint8_t time_min;
    uint8_t time_sec;
    uint8_t kavach_subsystem_type;
    uint8_t total_fault_codes;
    nms_fault_entry_t fault[NMS_MAX_FAULT_CODES];
} nms_kavach_fault_msg_t;

typedef struct
{
    uint8_t message_type;
    uint16_t payload_len;
    uint8_t payload[NMS_MAX_PAYLOAD_LEN];
    uint8_t seq_total;
    uint8_t seq_index;
    uint8_t retry_count;
    uint8_t active;
    uint8_t waiting_for_ack;
    uint8_t ack_required;
    uint32_t start_time;
} nms_ack_transaction_t;

typedef struct
{
    uint8_t payload[NMS_MAX_PAYLOAD_LEN];
    uint16_t payload_len;
    nms_health_event_t health;
    nms_kavach_info_t kavach_info;
    nms_kavach_position_t kavach_postion;
    nms_tsr_info_t tsr_info;
    nms_adjacent_info_t adjacent_info;
    nms_field_input_status_t field_input_status;
    nms_field_input_event_t field_input_event;
    nms_onboard_health_t onboard_health;
    nms_loco_rssi_t loco_rssi;
    nms_skavach_rssi_t skavach_rssi;
    nms_kavach_fault_msg_t kavach_fault_msg;
} nms_tx_ctx_t;

extern nms_tx_ctx_t nms_ctx;

void nms_ack_transaction_start(uint8_t message_type,
                               const uint8_t *payload,
                               uint16_t payload_len);
void nms_ack_process(void);
void nms_tx_process(void);
void nms_ack_received(uint8_t action_type, uint8_t ack_status);
void nms_ack_rx_handle(uint32_t can_id, uint8_t *data);

void send_skavach_info_msg_to_nms(uint8_t unused_frame_num);
void send_loco_postion_info_to_nms(uint8_t unused_frame_num);
void send_tsr_info_msg_to_nms(uint8_t unused_frame_num);
void send_adjacent_info_msg_to_nms(uint8_t unused_frame_num);
void send_field_input_status_msg_to_nms(uint8_t unused_frame_num);
void send_field_input_event_msg_to_nms(uint8_t unused_frame_num);
void send_skavach_health_msg_to_nms(uint8_t unused_frame_num);
void send_onboard_health_msg_to_nms(uint8_t unused_frame_num);
void send_skavach_fault_msg_to_nms(uint8_t unused_frame_num);
void send_loco_rssi_msg_to_nms(uint8_t unused_frame_num);
void send_skavach_rssi_msg_to_nms(uint8_t unused_frame_num);

#endif
