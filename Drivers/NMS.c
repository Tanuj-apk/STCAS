#include "NMS.h"
#include "can_if.h"
#include "gps.h"

#include <stddef.h>
#include <string.h>

typedef struct
{
    uint8_t *data;
    uint16_t capacity;
    uint16_t length;
    uint8_t valid;
} nms_writer_t;

typedef struct
{
    uint8_t data[NMS_ACK_REASSEMBLY_LEN];
    uint64_t received_mask;
    uint8_t seq_total;
    uint8_t active;
} nms_ack_reassembly_t;

nms_tx_ctx_t nms_ctx;

volatile uint8_t nms_ack_received_flag = 0U;
volatile uint8_t nms_ack_action = 0U;
volatile uint8_t nms_ack_status = 0U;

static volatile uint8_t nms_ack_has_sequence = 0U;
static volatile uint16_t nms_ack_sequence = 0U;

static nms_ack_transaction_t nms_transaction_queue[NMS_TRANSACTION_QUEUE_SIZE];
static uint8_t nms_queue_head = 0U;
static uint8_t nms_queue_tail = 0U;
static uint8_t nms_queue_count = 0U;
static nms_ack_reassembly_t nms_ack_rx;

static void nms_pop_transaction(void);

static void nms_writer_init(nms_writer_t *writer, uint8_t *data, uint16_t capacity)
{
    writer->data = data;
    writer->capacity = capacity;
    writer->length = 0U;
    writer->valid = 1U;
    memset(data, 0, capacity);
}

static void nms_put_u8(nms_writer_t *writer, uint8_t value)
{
    if ((writer->valid == 0U) || (writer->length >= writer->capacity))
    {
        writer->valid = 0U;
        return;
    }

    writer->data[writer->length++] = value;
}

static void nms_put_u16(nms_writer_t *writer, uint16_t value)
{
    nms_put_u8(writer, (uint8_t)((value >> 8U) & 0xFFU));
    nms_put_u8(writer, (uint8_t)(value & 0xFFU));
}

static void nms_put_u24(nms_writer_t *writer, uint32_t value)
{
    nms_put_u8(writer, (uint8_t)((value >> 16U) & 0xFFU));
    nms_put_u8(writer, (uint8_t)((value >> 8U) & 0xFFU));
    nms_put_u8(writer, (uint8_t)(value & 0xFFU));
}

static void nms_put_u32(nms_writer_t *writer, uint32_t value)
{
    nms_put_u8(writer, (uint8_t)((value >> 24U) & 0xFFU));
    nms_put_u8(writer, (uint8_t)((value >> 16U) & 0xFFU));
    nms_put_u8(writer, (uint8_t)((value >> 8U) & 0xFFU));
    nms_put_u8(writer, (uint8_t)(value & 0xFFU));
}

static void nms_put_bytes(nms_writer_t *writer, const uint8_t *data, uint16_t length)
{
    if ((writer->valid == 0U) || (data == NULL) ||
        (length > (uint16_t)(writer->capacity - writer->length)))
    {
        writer->valid = 0U;
        return;
    }

    memcpy(&writer->data[writer->length], data, length);
    writer->length = (uint16_t)(writer->length + length);
}

static uint16_t nms_writer_result(const nms_writer_t *writer)
{
    return (writer->valid != 0U) ? writer->length : 0U;
}

static void nms_put_stationary_header(nms_writer_t *writer,
                                      uint8_t message_type,
                                      uint16_t message_length,
                                      const nms_stationary_header_t *header)
{
    nms_put_u16(writer, NMS_SOF_E1);
    nms_put_u8(writer, message_type);
    nms_put_u16(writer, message_length);
    nms_put_u16(writer, header->message_sequence);
    nms_put_u16(writer, header->stationary_kavach_id);
    nms_put_u16(writer, header->nms_system_id);
    nms_put_u8(writer, header->system_version);
    nms_put_u8(writer, header->date_day);
    nms_put_u8(writer, header->date_month);
    nms_put_u8(writer, header->date_year);
    nms_put_u8(writer, header->time_hour);
    nms_put_u8(writer, header->time_min);
    nms_put_u8(writer, header->time_sec);
}

static void nms_put_onboard_header(nms_writer_t *writer,
                                   uint16_t sof,
                                   uint8_t message_type,
                                   uint16_t message_length,
                                   const nms_onboard_header_t *header)
{
    nms_put_u16(writer, sof);
    nms_put_u8(writer, message_type);
    nms_put_u16(writer, message_length);
    nms_put_u16(writer, header->message_sequence);
    nms_put_u24(writer, header->loco_kavach_id);
    nms_put_u16(writer, header->nms_system_id);
    nms_put_u8(writer, header->system_version);
    nms_put_u8(writer, header->date_day);
    nms_put_u8(writer, header->date_month);
    nms_put_u8(writer, header->date_year);
    nms_put_u8(writer, header->time_hour);
    nms_put_u8(writer, header->time_min);
    nms_put_u8(writer, header->time_sec);
}

static uint16_t nms_build_info(uint8_t *buffer, const nms_kavach_info_t *message)
{
    nms_writer_t writer;
    uint16_t message_length;

    if (message->station_packet_len > NMS_MAX_EMBEDDED_PACKET_LEN)
    {
        return 0U;
    }

    message_length = (uint16_t)(23U + message->station_packet_len);
    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, NMS_PKT_TYPE_INFO, message_length, &message->header);
    nms_put_u8(&writer, message->station_active_radio);
    nms_put_u8(&writer, 0xA5U);
    nms_put_u8(&writer, 0xC3U);
    nms_put_bytes(&writer, message->station_packet, message->station_packet_len);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_position(uint8_t *buffer, const nms_kavach_position_t *message)
{
    nms_writer_t writer;
    uint16_t message_length;
    uint8_t i;

    if ((message->loco_packet_len > NMS_MAX_EMBEDDED_PACKET_LEN) ||
        (message->ma_section_count > NMS_MAX_MA_SECTIONS))
    {
        return 0U;
    }

    message_length = (uint16_t)(24U + message->loco_packet_len +
                                (2U * message->ma_section_count));
    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, NMS_PKT_TYPE_POS_INFO, message_length, &message->header);
    nms_put_u8(&writer, message->onboard_active_radio);
    nms_put_u8(&writer, 0xA5U);
    nms_put_u8(&writer, 0xC3U);
    nms_put_bytes(&writer, message->loco_packet, message->loco_packet_len);
    nms_put_u8(&writer, message->ma_section_count);

    for (i = 0U; i < message->ma_section_count; i++)
    {
        nms_put_u16(&writer, message->route_id[i]);
    }

    return nms_writer_result(&writer);
}

static uint16_t nms_build_raw_stationary(uint8_t *buffer,
                                         uint8_t message_type,
                                         const nms_stationary_header_t *header,
                                         const uint8_t *packet,
                                         uint16_t packet_len)
{
    nms_writer_t writer;

    if (packet_len > NMS_MAX_EMBEDDED_PACKET_LEN)
    {
        return 0U;
    }

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, message_type,
                              (uint16_t)(20U + packet_len), header);
    nms_put_bytes(&writer, packet, packet_len);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_field_input_status(uint8_t *buffer,
                                             const nms_field_input_status_t *message)
{
    nms_writer_t writer;
    uint16_t image_len;

    image_len = (uint16_t)((message->total_event_relays + 7U) / 8U);
    if (image_len > NMS_MAX_RELAY_STATUS_BYTES)
    {
        return 0U;
    }

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, NMS_PKT_TYPE_FIELD_INPUT_STATUS,
                              (uint16_t)(22U + image_len), &message->header);
    nms_put_u16(&writer, message->total_event_relays);
    nms_put_bytes(&writer, message->relay_status, image_len);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_field_input_event(uint8_t *buffer,
                                            const nms_field_input_event_t *message)
{
    nms_writer_t writer;
    uint8_t i;

    if (message->relay_event_count > NMS_MAX_RELAY_EVENTS)
    {
        return 0U;
    }

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, NMS_PKT_TYPE_FIELD_INPUT_EVENT,
                              (uint16_t)(21U + (3U * message->relay_event_count)),
                              &message->header);
    nms_put_u8(&writer, message->relay_event_count);

    for (i = 0U; i < message->relay_event_count; i++)
    {
        nms_put_u16(&writer, message->event[i].relay_address);
        nms_put_u8(&writer, message->event[i].relay_status);
    }

    return nms_writer_result(&writer);
}

static uint16_t nms_build_stationary_health(uint8_t *buffer,
                                            const nms_health_event_t *message)
{
    nms_writer_t writer;

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, NMS_PKT_TYPE_HEALTH, 82U, &message->header);
    nms_put_u8(&writer, message->event_count);
    nms_put_u16(&writer, message->event_id);
    nms_put_u8(&writer, (uint8_t)message->system_temperature);
    nms_put_u8(&writer, message->active_radio_number);
    nms_put_u8(&writer, message->radio1_health);
    nms_put_u8(&writer, message->radio2_health);
    nms_put_u8(&writer, message->radio1_input_supply);
    nms_put_u8(&writer, message->radio2_input_supply);
    nms_put_u8(&writer, (uint8_t)message->radio1_temperature);
    nms_put_u8(&writer, (uint8_t)message->radio2_temperature);
    nms_put_u8(&writer, message->radio1_pa_temperature);
    nms_put_u8(&writer, message->radio2_pa_temperature);
    nms_put_u8(&writer, message->radio1_pa_supply_voltage);
    nms_put_u8(&writer, message->radio2_pa_supply_voltage);
    nms_put_u8(&writer, message->radio1_tx_pa_current);
    nms_put_u8(&writer, message->radio2_tx_pa_current);
    nms_put_u8(&writer, message->radio1_reverse_power);
    nms_put_u8(&writer, message->radio2_reverse_power);
    nms_put_u8(&writer, message->radio1_forward_power);
    nms_put_u8(&writer, message->radio2_forward_power);
    nms_put_u8(&writer, message->current_running_key);
    nms_put_u8(&writer, message->remaining_keys);
    nms_put_u16(&writer, message->session_key_checksum);
    nms_put_u8(&writer, message->allocated_time_slot);
    nms_put_u16(&writer, message->new_loco_regular_pkt_time_offset);
    nms_put_u8(&writer, message->loco_count);
    nms_put_u8(&writer, message->radio1_rx_packet_count);
    nms_put_u8(&writer, message->radio2_rx_packet_count);
    nms_put_u8(&writer, message->active_gps_number);
    nms_put_u8(&writer, message->gps1_view);
    nms_put_u8(&writer, message->gps2_view);
    nms_put_u8(&writer, message->gps1_seconds);
    nms_put_u8(&writer, message->gps2_seconds);
    nms_put_u8(&writer, message->gps1_satellites);
    nms_put_u8(&writer, message->gps1_cno_max);
    nms_put_u8(&writer, message->gps2_satellites);
    nms_put_u8(&writer, message->gps2_cno_max);
    nms_put_u8(&writer, message->gsm1_rssi);
    nms_put_u8(&writer, message->gsm2_rssi);
    nms_put_u16(&writer, message->missing_rfid);
    nms_put_u16(&writer, message->invalid_rfid);
    nms_put_u16(&writer, message->conflict_route_rfid);
    nms_put_u16(&writer, message->conflicting_tin);
    nms_put_u16(&writer, message->missing_tin);
    nms_put_u32(&writer, message->loco_specific_sos);
    nms_put_u32(&writer, message->train_exit_mode);
    nms_put_u16(&writer, message->station_modules_health);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_onboard_health(uint8_t *buffer,
                                         const nms_onboard_health_t *message)
{
    nms_writer_t writer;

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_onboard_header(&writer, NMS_SOF_GPRS, NMS_PKT_TYPE_ONBOARD_HEALTH,
                           108U, &message->header);
    nms_put_u8(&writer, message->event_count);
    nms_put_u16(&writer, message->event_id);
    nms_put_u8(&writer, message->radio1_health);
    nms_put_u8(&writer, message->radio2_health);
    nms_put_u8(&writer, message->radio1_input_supply);
    nms_put_u8(&writer, message->radio2_input_supply);
    nms_put_u8(&writer, (uint8_t)message->radio1_temperature);
    nms_put_u8(&writer, (uint8_t)message->radio2_temperature);
    nms_put_u8(&writer, message->radio1_pa_temperature);
    nms_put_u8(&writer, message->radio2_pa_temperature);
    nms_put_u8(&writer, message->radio1_pa_voltage);
    nms_put_u8(&writer, message->radio2_pa_voltage);
    nms_put_u8(&writer, message->radio1_tx_pa_current);
    nms_put_u8(&writer, message->radio2_tx_pa_current);
    nms_put_u8(&writer, message->radio1_reverse_power);
    nms_put_u8(&writer, message->radio2_reverse_power);
    nms_put_u8(&writer, message->radio1_forward_power);
    nms_put_u8(&writer, message->radio2_forward_power);
    nms_put_u16(&writer, message->stationary_pkt_time_offset);
    nms_put_u8(&writer, message->active_gps_number);
    nms_put_u8(&writer, message->gps1_view_status);
    nms_put_u8(&writer, message->gps2_view_status);
    nms_put_u8(&writer, message->gps1_seconds);
    nms_put_u8(&writer, message->gps2_seconds);
    nms_put_u8(&writer, message->gps1_satellites);
    nms_put_u8(&writer, message->gps1_cno_max);
    nms_put_u8(&writer, message->gps2_satellites);
    nms_put_u8(&writer, message->gps2_cno_max);
    nms_put_u16(&writer, message->gps1_link_status);
    nms_put_u16(&writer, message->gps2_link_status);
    nms_put_u8(&writer, message->gsm1_rssi);
    nms_put_u8(&writer, message->gsm2_rssi);
    nms_put_u8(&writer, message->current_running_key);
    nms_put_u8(&writer, message->remaining_keys);
    nms_put_u16(&writer, message->session_key_checksum);
    nms_put_u16(&writer, message->dmi1_link_status);
    nms_put_u16(&writer, message->dmi2_link_status);
    nms_put_u16(&writer, message->rfid1_link_status);
    nms_put_u16(&writer, message->rfid2_link_status);
    nms_put_u16(&writer, message->duplicate_missing_rfid_tag);
    nms_put_u32(&writer, message->missing_linked_rfid_tag);
    nms_put_u32(&writer, message->computed_tlm_status);
    nms_put_u8(&writer, message->train_configuration_change);
    nms_put_u8(&writer, message->bootup_sequence_error);
    nms_put_u8(&writer, message->selected_train_formation);
    nms_put_u8(&writer, message->selected_cab);
    nms_put_u8(&writer, message->brake_application_reason);
    nms_put_bytes(&writer, message->station_general_sos, 3U);
    nms_put_bytes(&writer, message->station_loco_specific_sos, 3U);
    nms_put_u32(&writer, message->collision_detection);
    nms_put_u8(&writer, message->loco_self_sos);
    nms_put_u8(&writer, message->kavach_connection);
    nms_put_u8(&writer, message->biu_isolated);
    nms_put_u8(&writer, message->eb_bypassed);
    nms_put_u8(&writer, message->kavach_territory);
    nms_put_u8(&writer, message->brake_interface_error);
    nms_put_u16(&writer, message->onboard_modules_health);
    nms_put_u16(&writer, message->conflict_route_rfid);
    nms_put_u32(&writer, message->train_configuration_checksum);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_loco_rssi(uint8_t *buffer, const nms_loco_rssi_t *message)
{
    nms_writer_t writer;

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_onboard_header(&writer, NMS_SOF_E1, NMS_PKT_TYPE_LOCO_RSSI,
                           39U, &message->header);
    nms_put_u16(&writer, message->stationary_kavach_id);
    nms_put_u8(&writer, message->station_radio1_rssi_sample_count);
    nms_put_u16(&writer, message->ref_rfid_tag1);
    nms_put_u24(&writer, message->abs_ref_rfid_tag1);
    nms_put_u16(&writer, (uint16_t)message->rssi_value1);
    nms_put_u8(&writer, message->station_radio2_rssi_sample_count);
    nms_put_u16(&writer, message->ref_rfid_tag2);
    nms_put_u24(&writer, message->abs_ref_rfid_tag2);
    nms_put_u16(&writer, (uint16_t)message->rssi_value2);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_station_rssi(uint8_t *buffer,
                                       const nms_skavach_rssi_t *message)
{
    nms_writer_t writer;

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_stationary_header(&writer, NMS_PKT_TYPE_STATION_RSSI,
                              39U, &message->header);
    nms_put_u24(&writer, message->loco_kavach_id);
    nms_put_u8(&writer, message->onboard_radio1_rssi_sample_count);
    nms_put_u16(&writer, message->ref_rfid_tag1);
    nms_put_u24(&writer, message->abs_location1);
    nms_put_u16(&writer, (uint16_t)message->rssi_value1);
    nms_put_u8(&writer, message->onboard_radio2_rssi_sample_count);
    nms_put_u16(&writer, message->ref_rfid_tag2);
    nms_put_u24(&writer, message->abs_location2);
    nms_put_u16(&writer, (uint16_t)message->rssi_value2);
    return nms_writer_result(&writer);
}

static uint16_t nms_build_fault(uint8_t *buffer,
                                nms_kavach_fault_msg_t *message)
{
    nms_writer_t writer;
    uint8_t i;
    uint16_t sof;

    if (message->total_fault_codes > NMS_MAX_FAULT_CODES)
    {
        return 0U;
    }

    sof = message->sof;
    if ((sof != NMS_SOF_E1) && (sof != NMS_SOF_GPRS))
    {
        sof = NMS_SOF_E1;
    }

    nms_writer_init(&writer, buffer, NMS_MAX_PAYLOAD_LEN);
    nms_put_u16(&writer, sof);
    nms_put_u8(&writer, NMS_PKT_TYPE_FAULT);
    nms_put_u16(&writer, (uint16_t)(23U + (4U * message->total_fault_codes)));
    nms_put_u16(&writer, message->message_sequence);
    nms_put_u24(&writer, message->kavach_subsystem_id);
    nms_put_u16(&writer, message->nms_system_id);
    nms_put_u8(&writer, message->system_version);
    nms_put_u8(&writer, message->date_day);
    nms_put_u8(&writer, message->date_month);
    nms_put_u8(&writer, message->date_year);
    nms_put_u8(&writer, message->time_hour);
    nms_put_u8(&writer, message->time_min);
    nms_put_u8(&writer, message->time_sec);
    nms_put_u8(&writer, message->kavach_subsystem_type);
    nms_put_u8(&writer, message->total_fault_codes);

    for (i = 0U; i < message->total_fault_codes; i++)
    {
        nms_put_u8(&writer, message->fault[i].module_id);
        nms_put_u8(&writer, message->fault[i].fault_code_type);
        nms_put_u16(&writer, message->fault[i].fault_code);
    }

    return nms_writer_result(&writer);
}

static uint8_t nms_message_requires_ack(uint8_t message_type,
                                        const uint8_t *payload)
{
    if ((message_type == NMS_PKT_TYPE_INFO) ||
        (message_type == NMS_PKT_TYPE_POS_INFO) ||
        (message_type == NMS_PKT_TYPE_TSR_INFO) ||
        (message_type == NMS_PKT_TYPE_ADJ_INFO))
    {
        return 1U;
    }

    /* Annexure G.4.12 requires 0x19 acknowledgement on GPRS only. */
    if ((message_type == NMS_PKT_TYPE_FAULT) &&
        (payload[0] == 0xBBU) && (payload[1] == 0xBBU))
    {
        return 1U;
    }

    return 0U;
}

static nms_ack_transaction_t *nms_get_transaction(void)
{
    if (nms_queue_count == 0U)
    {
        return NULL;
    }

    return &nms_transaction_queue[nms_queue_head];
}

void nms_ack_transaction_start(uint8_t message_type,
                               const uint8_t *payload,
                               uint16_t payload_len)
{
    nms_ack_transaction_t *transaction;
    uint8_t seq_total;

    if ((payload == NULL) || (payload_len == 0U) ||
        (payload_len > NMS_MAX_PAYLOAD_LEN) ||
        (nms_queue_count >= NMS_TRANSACTION_QUEUE_SIZE))
    {
        return;
    }

    seq_total = (uint8_t)((payload_len + NMS_PAYLOAD_BYTES - 1U) /
                          NMS_PAYLOAD_BYTES);
    if ((seq_total == 0U) || (seq_total > NMS_MAX_FRAGMENTS))
    {
        return;
    }

    transaction = &nms_transaction_queue[nms_queue_tail];
    memset(transaction, 0, sizeof(*transaction));
    transaction->message_type = message_type;
    transaction->payload_len = payload_len;
    transaction->seq_total = seq_total;
    transaction->ack_required = nms_message_requires_ack(message_type, payload);
    memcpy(transaction->payload, payload, payload_len);

    nms_queue_tail++;
    if (nms_queue_tail >= NMS_TRANSACTION_QUEUE_SIZE)
    {
        nms_queue_tail = 0U;
    }

    nms_queue_count++;
    if (nms_queue_count == 1U)
    {
        transaction->active = 1U;
    }
}

static void nms_build_fragment(uint8_t *can_frame,
                               const nms_ack_transaction_t *transaction)
{
    uint8_t seq_total_lsb;
    uint8_t seq_total_msb;
    uint16_t payload_offset;
    uint8_t i;

    memset(can_frame, 0, 8U);
    seq_total_lsb = transaction->seq_total & 0x0FU;
    seq_total_msb = (transaction->seq_total >> 4U) & 0x03U;
    can_frame[0] = (uint8_t)((seq_total_lsb << 4U) | NMS_CAN_ENVELOPE_TYPE);
    can_frame[1] = (uint8_t)(((transaction->seq_index & 0x3FU) << 2U) |
                             seq_total_msb);
    payload_offset = (uint16_t)transaction->seq_index * NMS_PAYLOAD_BYTES;

    for (i = 0U; i < NMS_PAYLOAD_BYTES; i++)
    {
        if ((payload_offset + i) < transaction->payload_len)
        {
            can_frame[2U + i] = transaction->payload[payload_offset + i];
        }
    }
}

static void nms_pop_transaction(void)
{
    nms_ack_transaction_t *transaction = nms_get_transaction();

    if (transaction == NULL)
    {
        return;
    }

    memset(transaction, 0, sizeof(*transaction));
    nms_queue_head++;
    if (nms_queue_head >= NMS_TRANSACTION_QUEUE_SIZE)
    {
        nms_queue_head = 0U;
    }

    nms_queue_count--;
    transaction = nms_get_transaction();
    if (transaction != NULL)
    {
        transaction->active = 1U;
    }
}

static void nms_transaction_send_next_fragment(nms_ack_transaction_t *transaction)
{
    uint8_t can_frame[8];
    uint8_t tx_status;

    if ((transaction == NULL) ||
        (transaction->seq_index >= transaction->seq_total))
    {
        return;
    }

    nms_build_fragment(can_frame, transaction);
    tx_status = can_transmit_redundant(NMS_TX_MB, can_frame);

    if (tx_status == 0U)
    {
        return;
    }

    transaction->seq_index++;
    if (transaction->seq_index >= transaction->seq_total)
    {
        transaction->seq_index = 0U;
        if (transaction->ack_required != 0U)
        {
            transaction->waiting_for_ack = 1U;
            transaction->start_time = system_ms;
        }
        else
        {
            nms_pop_transaction();
        }
    }
}

static void nms_retry_or_finish(nms_ack_transaction_t *transaction)
{
    if (transaction->retry_count < NMS_ACK_MAX_RETRIES)
    {
        transaction->retry_count++;
        transaction->seq_index = 0U;
        transaction->waiting_for_ack = 0U;
        transaction->start_time = 0U;
    }
    else
    {
        nms_pop_transaction();
    }
}

static void nms_ack_received_internal(uint8_t ack_status,
                                      uint8_t has_sequence,
                                      uint16_t message_sequence)
{
    nms_ack_transaction_t *transaction = nms_get_transaction();
    uint16_t expected_sequence;

    if ((transaction == NULL) || (transaction->active == 0U) ||
        (transaction->waiting_for_ack == 0U))
    {
        return;
    }

    if ((has_sequence != 0U) && (transaction->payload_len >= 7U))
    {
        expected_sequence = (uint16_t)(((uint16_t)transaction->payload[5] << 8U) |
                                       transaction->payload[6]);
        if (expected_sequence != message_sequence)
        {
            return;
        }
    }

    if (ack_status == CPU_ACK_OK)
    {
        nms_pop_transaction();
    }
    else
    {
        nms_retry_or_finish(transaction);
    }
}

void nms_ack_received(uint8_t action_type, uint8_t ack_status)
{
    (void)action_type;
    nms_ack_received_internal(ack_status, 0U, 0U);
}

void nms_ack_process(void)
{
    nms_ack_transaction_t *transaction;
    uint8_t action;
    uint8_t status;
    uint8_t has_sequence;
    uint16_t sequence;

    if (nms_ack_received_flag != 0U)
    {
        action = nms_ack_action;
        status = nms_ack_status;
        has_sequence = nms_ack_has_sequence;
        sequence = nms_ack_sequence;
        nms_ack_received_flag = 0U;
        (void)action;
        nms_ack_received_internal(status, has_sequence, sequence);
    }

    transaction = nms_get_transaction();
    if ((transaction != NULL) && (transaction->waiting_for_ack != 0U) &&
        ((system_ms - transaction->start_time) >= NMS_ACK_TIMEOUT_MS))
    {
        nms_retry_or_finish(transaction);
    }
}

void nms_tx_process(void)
{
    nms_ack_transaction_t *transaction = nms_get_transaction();

    if ((transaction != NULL) && (transaction->active != 0U) &&
        (transaction->waiting_for_ack == 0U))
    {
        nms_transaction_send_next_fragment(transaction);
    }
}

static void nms_ack_reassembly_reset(void)
{
    memset(&nms_ack_rx, 0, sizeof(nms_ack_rx));
}

static void nms_accept_annexure_ack(void)
{
    uint16_t message_length;
    uint16_t available;
    uint16_t frame_without_crc;
    uint16_t sequence;

    available = (uint16_t)nms_ack_rx.seq_total * NMS_PAYLOAD_BYTES;
    if (available < 13U)
    {
        nms_ack_reassembly_reset();
        return;
    }

    if (!(((nms_ack_rx.data[0] == 0xAAU) && (nms_ack_rx.data[1] == 0xAAU)) ||
          ((nms_ack_rx.data[0] == 0xBBU) && (nms_ack_rx.data[1] == 0xBBU))) ||
        (nms_ack_rx.data[2] != NMS_PKT_TYPE_ACK))
    {
        nms_ack_reassembly_reset();
        return;
    }

    message_length = (uint16_t)(((uint16_t)nms_ack_rx.data[3] << 8U) |
                                nms_ack_rx.data[4]);
    if (message_length < 15U)
    {
        nms_ack_reassembly_reset();
        return;
    }

    frame_without_crc = (uint16_t)(2U + message_length - 4U);
    if (frame_without_crc > available)
    {
        nms_ack_reassembly_reset();
        return;
    }

    sequence = (uint16_t)(((uint16_t)nms_ack_rx.data[5] << 8U) |
                          nms_ack_rx.data[6]);
    nms_ack_action = NMS_PKT_TYPE_ACK;
    nms_ack_status = CPU_ACK_OK;
    nms_ack_sequence = sequence;
    nms_ack_has_sequence = 1U;
    nms_ack_received_flag = 1U;
    nms_ack_reassembly_reset();
}

void nms_ack_rx_handle(uint32_t can_id, uint8_t *data)
{
    uint16_t ack_can_id;
    uint8_t packet_type;
    uint8_t seq_total;
    uint8_t seq_index;
    uint16_t offset;
    uint8_t i;
    uint64_t complete_mask;

    if ((can_id != NMS_ACK_CAN_ID) || (data == NULL))
    {
        return;
    }

    /* Project-2-TMS570 compatible local ACK format. */
    ack_can_id = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
    if (ack_can_id == (uint16_t)can_get_local_tx_id(NMS_TX_CAN_ID))
    {
        nms_ack_action = data[2];
        nms_ack_status = data[3];
        nms_ack_has_sequence = 0U;
        nms_ack_received_flag = 1U;
        return;
    }

    /* Also accept the gateway's fragmented, CRC-validated Annexure-G ACK. */
    packet_type = data[0] & 0x0FU;
    seq_total = (uint8_t)(((data[0] >> 4U) & 0x0FU) |
                          ((data[1] & 0x03U) << 4U));
    seq_index = (data[1] >> 2U) & 0x3FU;

    if ((packet_type != NMS_CAN_ENVELOPE_TYPE) || (seq_total == 0U) ||
        (seq_index >= seq_total) ||
        (((uint16_t)seq_total * NMS_PAYLOAD_BYTES) > NMS_ACK_REASSEMBLY_LEN))
    {
        return;
    }

    if (seq_index == 0U)
    {
        nms_ack_reassembly_reset();
        nms_ack_rx.active = 1U;
        nms_ack_rx.seq_total = seq_total;
    }

    if ((nms_ack_rx.active == 0U) || (nms_ack_rx.seq_total != seq_total))
    {
        return;
    }

    offset = (uint16_t)seq_index * NMS_PAYLOAD_BYTES;
    for (i = 0U; i < NMS_PAYLOAD_BYTES; i++)
    {
        nms_ack_rx.data[offset + i] = data[2U + i];
    }

    nms_ack_rx.received_mask |= (1ULL << seq_index);
    complete_mask = (1ULL << seq_total) - 1ULL;
    if (nms_ack_rx.received_mask == complete_mask)
    {
        nms_accept_annexure_ack();
    }
}

static void nms_queue_built_message(uint8_t message_type, uint16_t payload_len)
{
    if ((payload_len != 0U) && (payload_len <= NMS_MAX_PAYLOAD_LEN))
    {
        nms_ctx.payload_len = payload_len;
        nms_ack_transaction_start(message_type, nms_ctx.payload, payload_len);
    }
}

void send_skavach_info_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(NMS_PKT_TYPE_INFO,
                            nms_build_info(nms_ctx.payload, &nms_ctx.kavach_info));
}

void send_loco_postion_info_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(NMS_PKT_TYPE_POS_INFO,
                            nms_build_position(nms_ctx.payload, &nms_ctx.kavach_postion));
}

void send_tsr_info_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_TSR_INFO,
        nms_build_raw_stationary(nms_ctx.payload, NMS_PKT_TYPE_TSR_INFO,
                                 &nms_ctx.tsr_info.header,
                                 nms_ctx.tsr_info.packet,
                                 nms_ctx.tsr_info.packet_len));
}

void send_adjacent_info_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_ADJ_INFO,
        nms_build_raw_stationary(nms_ctx.payload, NMS_PKT_TYPE_ADJ_INFO,
                                 &nms_ctx.adjacent_info.header,
                                 nms_ctx.adjacent_info.packet,
                                 nms_ctx.adjacent_info.packet_len));
}

void send_field_input_status_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_FIELD_INPUT_STATUS,
        nms_build_field_input_status(nms_ctx.payload, &nms_ctx.field_input_status));
}

void send_field_input_event_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_FIELD_INPUT_EVENT,
        nms_build_field_input_event(nms_ctx.payload, &nms_ctx.field_input_event));
}

void send_skavach_health_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_HEALTH,
        nms_build_stationary_health(nms_ctx.payload, &nms_ctx.health));
}

void send_onboard_health_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_ONBOARD_HEALTH,
        nms_build_onboard_health(nms_ctx.payload, &nms_ctx.onboard_health));
}

void send_skavach_fault_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_FAULT,
        nms_build_fault(nms_ctx.payload, &nms_ctx.kavach_fault_msg));
}

void send_loco_rssi_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_LOCO_RSSI,
        nms_build_loco_rssi(nms_ctx.payload, &nms_ctx.loco_rssi));
}

void send_skavach_rssi_msg_to_nms(uint8_t unused_frame_num)
{
    (void)unused_frame_num;
    nms_queue_built_message(
        NMS_PKT_TYPE_STATION_RSSI,
        nms_build_station_rssi(nms_ctx.payload, &nms_ctx.skavach_rssi));
}
