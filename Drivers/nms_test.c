#include "nms_test.h"

#include "NMS.h"

#include <string.h>

#define NMS_TEST_MESSAGE_COUNT       11U
#define NMS_TEST_STATIONARY_ID       0x1234U
#define NMS_TEST_LOCO_ID             0x0056789AUL
#define NMS_TEST_NMS_SYSTEM_ID       0x4321U

volatile uint8_t nms_test_last_message_type = 0U;
volatile uint16_t nms_test_message_sequence = 1U;

static uint8_t nms_test_cycle_index = 0U;
#if NMS_TEST_ENABLE
static uint8_t nms_test_period_count = 0U;
#endif

static const uint8_t nms_test_message_types[NMS_TEST_MESSAGE_COUNT] =
{
    NMS_PKT_TYPE_INFO,
    NMS_PKT_TYPE_POS_INFO,
    NMS_PKT_TYPE_TSR_INFO,
    NMS_PKT_TYPE_ADJ_INFO,
    NMS_PKT_TYPE_FIELD_INPUT_STATUS,
    NMS_PKT_TYPE_FIELD_INPUT_EVENT,
    NMS_PKT_TYPE_HEALTH,
    NMS_PKT_TYPE_ONBOARD_HEALTH,
    NMS_PKT_TYPE_FAULT,
    NMS_PKT_TYPE_LOCO_RSSI,
    NMS_PKT_TYPE_STATION_RSSI
};

static void nms_test_fill_stationary_header(nms_stationary_header_t *header)
{
    memset(header, 0, sizeof(*header));
    header->message_sequence = nms_test_message_sequence;
    header->stationary_kavach_id = NMS_TEST_STATIONARY_ID;
    header->nms_system_id = NMS_TEST_NMS_SYSTEM_ID;
    header->system_version = NMS_SYSTEM_VERSION_4;
    header->date_day = 16U;
    header->date_month = 10U;
    header->date_year = 26U;
    header->time_hour = 12U;
    header->time_min = 34U;
    header->time_sec = (uint8_t)(nms_test_message_sequence % 60U);
}

static void nms_test_fill_onboard_header(nms_onboard_header_t *header)
{
    memset(header, 0, sizeof(*header));
    header->message_sequence = nms_test_message_sequence;
    header->loco_kavach_id = NMS_TEST_LOCO_ID;
    header->nms_system_id = NMS_TEST_NMS_SYSTEM_ID;
    header->system_version = NMS_SYSTEM_VERSION_4;
    header->date_day = 16U;
    header->date_month = 10U;
    header->date_year = 26U;
    header->time_hour = 12U;
    header->time_min = 34U;
    header->time_sec = (uint8_t)(nms_test_message_sequence % 60U);
}

static void nms_test_fill_pattern(uint8_t *data, uint16_t length,
                                  uint8_t seed)
{
    uint16_t index;

    for(index = 0U; index < length; index++)
    {
        data[index] = (uint8_t)(seed + index);
    }
}

static void nms_test_queue_info(void)
{
    nms_kavach_info_t *message = &nms_ctx.kavach_info;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->station_active_radio = 1U;
    message->station_packet_len = 12U;
    nms_test_fill_pattern(message->station_packet,
                          message->station_packet_len, 0x10U);
    send_skavach_info_msg_to_nms(0U);
}

static void nms_test_queue_position(void)
{
    nms_kavach_position_t *message = &nms_ctx.kavach_postion;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->onboard_active_radio = 2U;
    message->loco_packet_len = 12U;
    nms_test_fill_pattern(message->loco_packet,
                          message->loco_packet_len, 0x20U);
    message->ma_section_count = 3U;
    message->route_id[0] = 0x1101U;
    message->route_id[1] = 0x1102U;
    message->route_id[2] = 0x1103U;
    send_loco_postion_info_to_nms(0U);
}

static void nms_test_queue_tsr(void)
{
    nms_tsr_info_t *message = &nms_ctx.tsr_info;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->packet_len = 12U;
    nms_test_fill_pattern(message->packet, message->packet_len, 0x30U);
    send_tsr_info_msg_to_nms(0U);
}

static void nms_test_queue_adjacent(void)
{
    nms_adjacent_info_t *message = &nms_ctx.adjacent_info;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->packet_len = 12U;
    nms_test_fill_pattern(message->packet, message->packet_len, 0x40U);
    send_adjacent_info_msg_to_nms(0U);
}

static void nms_test_queue_field_status(void)
{
    nms_field_input_status_t *message = &nms_ctx.field_input_status;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->total_event_relays = 93U;
    nms_test_fill_pattern(message->relay_status, 12U, 0x50U);
    send_field_input_status_msg_to_nms(0U);
}

static void nms_test_queue_field_event(void)
{
    nms_field_input_event_t *message = &nms_ctx.field_input_event;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->relay_event_count = 3U;
    message->event[0].relay_address = 1U;
    message->event[0].relay_status = 1U;
    message->event[1].relay_address = 46U;
    message->event[1].relay_status = 0U;
    message->event[2].relay_address = 93U;
    message->event[2].relay_status = 1U;
    send_field_input_event_msg_to_nms(0U);
}

static void nms_test_queue_stationary_health(void)
{
    nms_health_event_t *message = &nms_ctx.health;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->event_count = 1U;
    message->event_id = 0x1701U;
    message->system_temperature = 35;
    message->active_radio_number = 1U;
    message->radio1_health = 1U;
    message->radio2_health = 1U;
    message->radio1_input_supply = 24U;
    message->radio2_input_supply = 24U;
    message->radio1_temperature = 31;
    message->radio2_temperature = 32;
    message->current_running_key = 4U;
    message->remaining_keys = 12U;
    message->session_key_checksum = 0x5A5AU;
    message->allocated_time_slot = 7U;
    message->loco_count = 2U;
    message->radio1_rx_packet_count = 21U;
    message->radio2_rx_packet_count = 22U;
    message->active_gps_number = 1U;
    message->gps1_view = 1U;
    message->gps2_view = 1U;
    message->gps1_satellites = 8U;
    message->gps2_satellites = 9U;
    message->gps1_cno_max = 42U;
    message->gps2_cno_max = 43U;
    message->gsm1_rssi = 25U;
    message->gsm2_rssi = 26U;
    message->station_modules_health = 0x00FFU;
    send_skavach_health_msg_to_nms(0U);
}

static void nms_test_queue_onboard_health(void)
{
    nms_onboard_health_t *message = &nms_ctx.onboard_health;

    memset(message, 0, sizeof(*message));
    nms_test_fill_onboard_header(&message->header);
    message->event_count = 1U;
    message->event_id = 0x1801U;
    message->radio1_health = 1U;
    message->radio2_health = 1U;
    message->active_gps_number = 2U;
    message->gps1_satellites = 8U;
    message->gps2_satellites = 9U;
    message->current_running_key = 5U;
    message->remaining_keys = 11U;
    message->session_key_checksum = 0xA55AU;
    message->selected_train_formation = 3U;
    message->selected_cab = 1U;
    message->kavach_connection = 1U;
    message->kavach_territory = 1U;
    message->onboard_modules_health = 0x0F0FU;
    message->train_configuration_checksum = 0x12345678UL;
    send_onboard_health_msg_to_nms(0U);
}

static void nms_test_queue_fault(void)
{
    nms_kavach_fault_msg_t *message = &nms_ctx.kavach_fault_msg;

    memset(message, 0, sizeof(*message));
    message->sof = NMS_SOF_E1;
    message->message_sequence = nms_test_message_sequence;
    message->kavach_subsystem_id = NMS_TEST_STATIONARY_ID;
    message->nms_system_id = NMS_TEST_NMS_SYSTEM_ID;
    message->system_version = NMS_SYSTEM_VERSION_4;
    message->date_day = 16U;
    message->date_month = 10U;
    message->date_year = 26U;
    message->time_hour = 12U;
    message->time_min = 34U;
    message->time_sec = (uint8_t)(nms_test_message_sequence % 60U);
    message->kavach_subsystem_type = 2U;
    message->total_fault_codes = 3U;
    message->fault[0].module_id = 1U;
    message->fault[0].fault_code_type = 1U;
    message->fault[0].fault_code = 0x0101U;
    message->fault[1].module_id = 2U;
    message->fault[1].fault_code_type = 1U;
    message->fault[1].fault_code = 0x0202U;
    message->fault[2].module_id = 3U;
    message->fault[2].fault_code_type = 2U;
    message->fault[2].fault_code = 0x0303U;
    send_skavach_fault_msg_to_nms(0U);
}

static void nms_test_queue_loco_rssi(void)
{
    nms_loco_rssi_t *message = &nms_ctx.loco_rssi;

    memset(message, 0, sizeof(*message));
    nms_test_fill_onboard_header(&message->header);
    message->stationary_kavach_id = NMS_TEST_STATIONARY_ID;
    message->station_radio1_rssi_sample_count = 4U;
    message->ref_rfid_tag1 = 0x1001U;
    message->abs_ref_rfid_tag1 = 0x00010203UL;
    message->rssi_value1 = -65;
    message->station_radio2_rssi_sample_count = 4U;
    message->ref_rfid_tag2 = 0x1002U;
    message->abs_ref_rfid_tag2 = 0x00040506UL;
    message->rssi_value2 = -72;
    send_loco_rssi_msg_to_nms(0U);
}

static void nms_test_queue_station_rssi(void)
{
    nms_skavach_rssi_t *message = &nms_ctx.skavach_rssi;

    memset(message, 0, sizeof(*message));
    nms_test_fill_stationary_header(&message->header);
    message->loco_kavach_id = NMS_TEST_LOCO_ID;
    message->onboard_radio1_rssi_sample_count = 4U;
    message->ref_rfid_tag1 = 0x2001U;
    message->abs_location1 = 0x00070809UL;
    message->rssi_value1 = -61;
    message->onboard_radio2_rssi_sample_count = 4U;
    message->ref_rfid_tag2 = 0x2002U;
    message->abs_location2 = 0x000A0B0CUL;
    message->rssi_value2 = -69;
    send_skavach_rssi_msg_to_nms(0U);
}

void NMS_Test_Init(void)
{
    nms_test_cycle_index = 0U;
#if NMS_TEST_ENABLE
    nms_test_period_count = 0U;
#endif
    nms_test_last_message_type = 0U;
    nms_test_message_sequence = 1U;
}

void NMS_Test_QueueMessage(uint8_t message_type)
{
    switch(message_type)
    {
        case NMS_PKT_TYPE_INFO:
            nms_test_queue_info();
            break;
        case NMS_PKT_TYPE_POS_INFO:
            nms_test_queue_position();
            break;
        case NMS_PKT_TYPE_TSR_INFO:
            nms_test_queue_tsr();
            break;
        case NMS_PKT_TYPE_ADJ_INFO:
            nms_test_queue_adjacent();
            break;
        case NMS_PKT_TYPE_FIELD_INPUT_STATUS:
            nms_test_queue_field_status();
            break;
        case NMS_PKT_TYPE_FIELD_INPUT_EVENT:
            nms_test_queue_field_event();
            break;
        case NMS_PKT_TYPE_HEALTH:
            nms_test_queue_stationary_health();
            break;
        case NMS_PKT_TYPE_ONBOARD_HEALTH:
            nms_test_queue_onboard_health();
            break;
        case NMS_PKT_TYPE_FAULT:
            nms_test_queue_fault();
            break;
        case NMS_PKT_TYPE_LOCO_RSSI:
            nms_test_queue_loco_rssi();
            break;
        case NMS_PKT_TYPE_STATION_RSSI:
            nms_test_queue_station_rssi();
            break;
        default:
            return;
    }

    nms_test_last_message_type = message_type;
    nms_test_message_sequence++;
}

void NMS_Test_QueueNextMessage(void)
{
    NMS_Test_QueueMessage(nms_test_message_types[nms_test_cycle_index]);
    nms_test_cycle_index++;
    if(nms_test_cycle_index >= NMS_TEST_MESSAGE_COUNT)
    {
        nms_test_cycle_index = 0U;
    }
}

void NMS_Test_1sTask(void)
{
#if NMS_TEST_ENABLE
    nms_test_period_count++;
    if(nms_test_period_count >= NMS_TEST_PERIOD_SEC)
    {
        nms_test_period_count = 0U;
        NMS_Test_QueueNextMessage();
    }
#endif
}
