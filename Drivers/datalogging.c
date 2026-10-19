#include "datalogging.h"
#include "datalogging_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "can_if.h"
#include "gps.h"

volatile uint8_t data_logger_ack_received;
volatile uint8_t data_logger_ack_action;
volatile uint8_t data_logger_ack_status;
volatile uint32_t data_logger_queue_full_count;
volatile uint32_t data_logger_tx_failure_count;

/* ============================================================================
 * Internal context
 * ========================================================================== */

typedef struct
{
    EventLogRecord_t records[EVENT_LOG_QUEUE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} EventLogQueue_t;

typedef struct {
  EventLogQueue_t queue;
  EventLogCanTxCallback_t tx_callback;

  /* Current transaction */
  bool tx_active;
  bool tx_waiting_ack;

  uint8_t tx_total_fragments;
  uint8_t tx_fragment_index;
  uint8_t tx_retry_count;
  /* Fixed for the logical record and reused by every fragment/retry. */
  uint8_t tx_transaction_id;
  uint8_t next_transaction_id;

  uint16_t tx_record_length;
  uint8_t tx_record_buffer[EVENT_LOG_MAX_RECORD_SIZE];

  uint32_t tx_start_time;

} EventLogInternal_t;

static EventLogInternal_t g_event_log;

/* ============================================================================
 * Internal utility functions
 * ========================================================================== */

static uint16_t EventLog_NextIndex(uint16_t index)
{
    index++;
    if(index >= EVENT_LOG_QUEUE_SIZE)
    {
        index = 0U;
    }
    return index;
}

static void EventLog_PutU16(
    uint8_t  *buffer,
    uint16_t *index,
    uint16_t value
)
{
    buffer[*index] = (uint8_t)(value & 0xFFU);
    (*index)++;

    buffer[*index] = (uint8_t)((value >> 8U) & 0xFFU);
    (*index)++;
}

static void EventLog_PutU32(uint8_t  *buffer, uint16_t *index, uint32_t value)
{
    buffer[*index] = (uint8_t)(value & 0xFFU);
    (*index)++;

    buffer[*index] = (uint8_t)((value >> 8U) & 0xFFU);
    (*index)++;

    buffer[*index] = (uint8_t)((value >> 16U) & 0xFFU);
    (*index)++;

    buffer[*index] = (uint8_t)((value >> 24U) & 0xFFU);
    (*index)++;
}

static void EventLog_PutLocation(uint8_t *buffer, uint16_t *index,
                                 const EventLogLocation_t *location)
{
    EventLog_PutU32(buffer, index, location->reference_location_m);
    EventLog_PutU32(buffer, index, location->distance_from_rfid_m);
}

/* ============================================================================
 * Initialization
 * ========================================================================== */

void EventLog_Init(EventLogCanTxCallback_t tx_callback)
{
    memset(&g_event_log, 0, sizeof(g_event_log));
    g_event_log.tx_callback = tx_callback;
    data_logger_ack_received = 0U;
    data_logger_ack_action = 0U;
    data_logger_ack_status = 0U;
    data_logger_queue_full_count = 0U;
    data_logger_tx_failure_count = 0U;
}

void EventLog_AckReceived(uint8_t ack_status, uint8_t transaction_id)
{
    if(!g_event_log.tx_active)
    {
        return;
    }

    if(!g_event_log.tx_waiting_ack)
    {
        return;
    }

    if((transaction_id & 0x0FU) != g_event_log.tx_transaction_id)
    {
        return;
    }

    if(ack_status == EVENT_LOG_ACK_OK)
    {
        /* Transaction successfully completed. */
        g_event_log.tx_active = false;
        g_event_log.tx_waiting_ack = false;
        g_event_log.tx_retry_count = 0U;
        g_event_log.tx_start_time = 0U;
    }
}
void EventLog_ProcessAck(void)
{
    uint32_t elapsed_time;

    if(!g_event_log.tx_active)
    {
        return;
    }

    if(!g_event_log.tx_waiting_ack)
    {
        return;
    }

    elapsed_time = system_ms - g_event_log.tx_start_time;

    if(elapsed_time < EVENT_LOG_ACK_TIMEOUT_MS)
    {
        return;
    }

    /*
     * ACK timeout.
     */
    if(g_event_log.tx_retry_count < EVENT_LOG_ACK_MAX_RETRIES)
    {
        /*
         * Retry the complete event-log transaction.
         */
        g_event_log.tx_retry_count++;

        g_event_log.tx_fragment_index = 0U;
        g_event_log.tx_waiting_ack = false;
        g_event_log.tx_start_time = 0U;
    }
    else
    {
        /*
         * Maximum retries exhausted.
         *
         * Transaction is considered failed.
         */
        g_event_log.tx_active = false;
        g_event_log.tx_waiting_ack = false;
        g_event_log.tx_retry_count = 0U;
        g_event_log.tx_start_time = 0U;
        data_logger_tx_failure_count++;
    }
}

/* ============================================================================
 * Queue
 * ========================================================================== */

bool EventLog_QueuePush(const EventLogRecord_t *record)
{
    uint16_t next_head;
    if(record == NULL)
    {
        return false;
    }

    next_head = EventLog_NextIndex(g_event_log.queue.head);
    /** Queue full.
     * No overwrite is performed. For a safety-related logger, losing the
     * oldest event silently is undesirable. */
    if(next_head == g_event_log.queue.tail)
    {
        data_logger_queue_full_count++;
        return false;
    }

    memcpy(&g_event_log.queue.records[g_event_log.queue.head], record, sizeof(EventLogRecord_t));
    g_event_log.queue.head = next_head;
    return true;
}

bool EventLog_QueuePop(EventLogRecord_t *record)
{
    if(record == NULL)
    {
        return false;
    }

    if(g_event_log.queue.tail == g_event_log.queue.head)
    {
        return false;
    }

    memcpy(record, &g_event_log.queue.records[g_event_log.queue.tail], sizeof(EventLogRecord_t));
    g_event_log.queue.tail = EventLog_NextIndex(g_event_log.queue.tail);
    return true;
}

uint16_t EventLog_QueueCount(void)
{
    uint16_t head;
    uint16_t tail;

    head = g_event_log.queue.head;
    tail = g_event_log.queue.tail;

    if(head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(EVENT_LOG_QUEUE_SIZE - tail + head);
}

/* ============================================================================
 * Common record initialization
 * ========================================================================== */

void EventLog_RecordSetCommon(EventLogRecord_t *record, uint16_t event_id, EventLogFormat_t format_type, const EventLogContext_t *context)
{
    if(record == NULL)
    {
        return;
    }

    memset(record, 0, sizeof(EventLogRecord_t));
    record->event_id = event_id;
    record->format_type = format_type;

    if(context != NULL)
    {
        record->epoch_time = context->epoch_time;
    }
    record->source = EVENT_LOG_LOCAL_SOURCE;
}

/* ============================================================================
 * Simple event
 * Used for events whose FRS logging requirement is only:
 *     event + date + time
 * or
 *     event + date + time + location
 * ========================================================================== */

bool EventLog_AddSimple(uint16_t event_id, const EventLogContext_t *context)
{
    EventLogRecord_t record;
    EventLog_RecordSetCommon(&record, event_id, EVENT_LOG_FMT_NONE, context);
    return EventLog_QueuePush(&record);
}

bool EventLog_AddSimpleWithLocation(uint16_t event_id,
                                    const EventLogContext_t *context,
                                    const EventLogLocation_t *location)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, event_id, EVENT_LOG_FMT_LOCATION,
                             context);
    record.data.location = *location;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * SPAD - Onboard
 * ========================================================================== */
bool EventLog_AddSpadOnboard(const EventLogContext_t *context,
                             const EventLogLocation_t *location,
                             uint16_t stationary_kavach_id,
                             uint8_t signal_type)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, LOG_EVT_SPAD, EVENT_LOG_FMT_SPAD_ONBOARD, context);
    record.data.spad_onboard.location = *location;
    record.data.spad_onboard.stationary_kavach_id = stationary_kavach_id;
    record.data.spad_onboard.signal_type = signal_type;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * SPAD - Stationary
 * ========================================================================== */
bool EventLog_AddSpadStationary(const EventLogContext_t *context,
                                const EventLogLocation_t *location,
                                uint16_t onboard_kavach_id,
                                uint16_t signal_name)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, LOG_EVT_SPAD, EVENT_LOG_FMT_SPAD_STATIONARY, context);
    record.data.spad_stationary.location = *location;
    record.data.spad_stationary.onboard_kavach_id = onboard_kavach_id;
    record.data.spad_stationary.signal_name = signal_name;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * Head-on collision - Onboard
 * ========================================================================== */
bool EventLog_AddHeadOnOnboard(const EventLogContext_t *context,
                               const EventLogLocation_t *location,
                               uint16_t stationary_kavach_id)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, LOG_EVT_HEAD_ON_COLLISION, EVENT_LOG_FMT_HEAD_ON_ONBOARD, context);
    record.data.head_onboard.location = *location;
    record.data.head_onboard.stationary_kavach_id = stationary_kavach_id;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * Head-on collision - Stationary
 * ========================================================================== */
bool EventLog_AddHeadOnStationary(const EventLogContext_t *context,
                                  const EventLogLocation_t *location,
                                  uint16_t onboard_kavach_l1_id,
                                  uint16_t onboard_kavach_l2_id)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, LOG_EVT_HEAD_ON_COLLISION, EVENT_LOG_FMT_HEAD_ON_STATIONARY, context);
    record.data.head_stationary.location = *location;
    record.data.head_stationary.onboard_kavach_l1_id = onboard_kavach_l1_id;
    record.data.head_stationary.onboard_kavach_l2_id = onboard_kavach_l2_id;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * Rear-end collision - Onboard
 * ========================================================================== */
bool EventLog_AddRearEndOnboard(const EventLogContext_t *context,
                                const EventLogLocation_t *location,
                                uint16_t onboard_kavach_l2_id)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, LOG_EVT_REAR_END_COLLISION, EVENT_LOG_FMT_REAR_END_ONBOARD, context);
    record.data.rear_end_onboard.location = *location;
    record.data.rear_end_onboard.onboard_kavach_l2_id = onboard_kavach_l2_id;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * Rear-end collision - Stationary
 * ========================================================================== */
bool EventLog_AddRearEndStationary(const EventLogContext_t *context,
                                   const EventLogLocation_t *location,
                                   uint16_t onboard_kavach_l1_id,
                                   uint16_t onboard_kavach_l2_id,
                                   uint32_t distance_l1_l2_m)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, LOG_EVT_REAR_END_COLLISION, EVENT_LOG_FMT_REAR_END_STATIONARY, context);
    record.data.rear_end_stationary.location = *location;
    record.data.rear_end_stationary.onboard_kavach_l1_id = onboard_kavach_l1_id;
    record.data.rear_end_stationary.onboard_kavach_l2_id = onboard_kavach_l2_id;
    record.data.rear_end_stationary.distance_l1_l2_m = distance_l1_l2_m;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * Manual SOS / Loco SOS
 * ========================================================================== */
bool EventLog_AddLocoSos(uint16_t event_id,
                         const EventLogContext_t *context,
                         const EventLogLocation_t *location,
                         uint16_t source_onboard_kavach_id)
{
    EventLogRecord_t record;
    if(location == NULL)
    {
        return false;
    }
    EventLog_RecordSetCommon(&record, event_id, EVENT_LOG_FMT_LOCO_SOS, context);
    record.data.loco_sos.location = *location;
    record.data.loco_sos.source_onboard_kavach_id = source_onboard_kavach_id;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * Communication event
 * ========================================================================== */
bool EventLog_AddCommunication(uint16_t event_id, const EventLogContext_t *context, EventLogCommTarget_t target, EventLogCommChannel_t channel,
                               bool peer_id_valid, uint16_t peer_id,
                               const EventLogLocation_t *location)
{
    EventLogRecord_t record;
    EventLogCommunication_t *communication;

    if(location != NULL)
    {
        EventLog_RecordSetCommon(&record, event_id,
                                 EVENT_LOG_FMT_COMMUNICATION_WITH_LOCATION,
                                 context);
        record.data.communication_with_location.location = *location;
        communication = &record.data.communication_with_location.communication;
    }
    else
    {
        EventLog_RecordSetCommon(&record, event_id, EVENT_LOG_FMT_COMMUNICATION,
                                 context);
        communication = &record.data.communication;
    }
    communication->target = target;
    communication->channel = channel;
    communication->peer_id_valid = peer_id_valid ? 1U : 0U;
    communication->peer_id = peer_id;
    return EventLog_QueuePush(&record);
}

bool EventLog_AddFieldInput(const EventLogContext_t *context,
                            uint16_t relay_address,
                            uint8_t old_status,
                            uint8_t new_status)
{
    EventLogRecord_t record;
    EventLog_RecordSetCommon(&record, LOG_EVT_FIELD_INPUT_CHANGE,
                             EVENT_LOG_FMT_FIELD_INPUT, context);
    record.data.field_input.relay_address = relay_address;
    record.data.field_input.old_status = old_status;
    record.data.field_input.new_status = new_status;
    return EventLog_QueuePush(&record);
}

bool EventLog_AddPeripheralCommunication(uint16_t event_id,
                                         const EventLogContext_t *context,
                                         uint8_t device_index,
                                         bool mandatory)
{
    EventLogRecord_t record;
    EventLog_RecordSetCommon(&record, event_id, EVENT_LOG_FMT_PERIPHERAL,
                             context);
    record.data.peripheral.device_index = device_index;
    record.data.peripheral.mandatory = mandatory ? 1U : 0U;
    return EventLog_QueuePush(&record);
}

bool EventLog_AddStationarySnapshot(const EventLogContext_t *context,
                                    const uint8_t input_image[EVENT_LOG_INPUT_IMAGE_BYTES],
                                    bool station_sos_active,
                                    uint32_t system_fault_mask)
{
    EventLogRecord_t record;

    if(input_image == NULL)
    {
        return false;
    }

    EventLog_RecordSetCommon(&record, LOG_EVT_STATIONARY_SNAPSHOT,
                             EVENT_LOG_FMT_STATIONARY_SNAPSHOT, context);
    memcpy(record.data.stationary_snapshot.input_image, input_image,
           EVENT_LOG_INPUT_IMAGE_BYTES);
    record.data.stationary_snapshot.station_sos_active =
        station_sos_active ? 1U : 0U;
    record.data.stationary_snapshot.system_fault_mask = system_fault_mask;
    return EventLog_QueuePush(&record);
}

/* ============================================================================
 * STCAS application context and wrapper functions
 * ========================================================================== */
static EventLogContext_t EventLog_GetCurrentContext(void)
{
    EventLogContext_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.epoch_time = (cpu_time_valid != 0U) ? rtc_abs_seconds : 0U;
    return ctx;
}


/* ============================================================================
 * Simple event
 * ========================================================================== */
bool Log_Event_Simple(uint16_t event_id)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddSimple(event_id, &ctx);
}

bool Log_Event_SimpleWithLocation(uint16_t event_id,
                                  const EventLogLocation_t *location)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddSimpleWithLocation(event_id, &ctx, location);
}

/* ============================================================================
 * SPAD
 * ========================================================================== */
bool Log_Event_Spad(const EventLogLocation_t *location,
                    uint16_t onboard_kavach_id,
                    uint16_t signal_name)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddSpadStationary(&ctx, location, onboard_kavach_id,
                                      signal_name);
}

/* ============================================================================
 * Head-on collision
 * ========================================================================== */
bool Log_Event_HeadOn(uint16_t onboard_kavach_l1_id,
                      uint16_t onboard_kavach_l2_id,
                      const EventLogLocation_t *location)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddHeadOnStationary(&ctx, location, onboard_kavach_l1_id,
                                        onboard_kavach_l2_id);
}

/* ============================================================================
 * Rear-end collision
 * ========================================================================== */
bool Log_Event_RearEnd(uint16_t onboard_kavach_l1_id,
                       uint16_t onboard_kavach_l2_id,
                       uint32_t distance_l1_l2_m,
                       const EventLogLocation_t *location)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddRearEndStationary(&ctx, location,
                                         onboard_kavach_l1_id,
                                         onboard_kavach_l2_id,
                                         distance_l1_l2_m);
}

/* ============================================================================
 * Manual SOS
 * ========================================================================== */
bool Log_Event_ManualSos(uint16_t source_onboard_kavach_id,
                         const EventLogLocation_t *location)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddLocoSos(LOG_EVT_MANUAL_SOS, &ctx, location,
                               source_onboard_kavach_id);
}

/* ============================================================================
 * Communication event
 * ========================================================================== */
bool Log_Event_Communication(uint16_t event_id,
                             EventLogCommTarget_t target,
                             EventLogCommChannel_t channel,
                             bool peer_id_valid,
                             uint16_t peer_id,
                             const EventLogLocation_t *location)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddCommunication(event_id, &ctx, target, channel,
                                     peer_id_valid, peer_id, location);
}

bool Log_Event_FieldInput(uint16_t relay_address,
                          uint8_t old_status,
                          uint8_t new_status)
{
    EventLogContext_t ctx;
    ctx = EventLog_GetCurrentContext();
    return EventLog_AddFieldInput(&ctx, relay_address, old_status, new_status);
}

bool Log_Event_PeripheralCommunication(bool recovered,
                                       uint8_t device_index,
                                       bool mandatory)
{
    EventLogContext_t ctx;
    uint16_t event_id;

    ctx = EventLog_GetCurrentContext();
    event_id = recovered ? LOG_EVT_PERIPHERAL_COMM_RECOVERY :
                           LOG_EVT_PERIPHERAL_COMM_FAILURE;
    return EventLog_AddPeripheralCommunication(event_id, &ctx, device_index,
                                               mandatory);
}

bool Log_Event_StationarySnapshot(
                                  const uint8_t input_image[EVENT_LOG_INPUT_IMAGE_BYTES],
                                  bool station_sos_active,
                                  uint32_t system_fault_mask)
{
    EventLogContext_t ctx;

    ctx = EventLog_GetCurrentContext();
    return EventLog_AddStationarySnapshot(&ctx, input_image,
                                          station_sos_active,
                                          system_fault_mask);
}

/* ============================================================================
 * Event -> format lookup
 * DMI alerts have no alert-specific fields, but an onboard logger still adds
 * the event location as the payload.  Stationary-only events remain free of
 * location unless their explicit event format contains it.
 * ========================================================================== */
EventLogFormat_t EventLog_GetFormatForEvent(uint16_t event_id, EventLogSource_t logger_source)
{
    if((event_id >= LOG_EVT_DMI_SOS_SELF_MANUAL) &&
       (event_id <= LOG_EVT_DMI_BRAKE_SPAD_DETECTED))
    {
        return (logger_source == EVENT_LOG_SOURCE_ONBOARD) ?
               EVENT_LOG_FMT_LOCATION : EVENT_LOG_FMT_NONE;
    }

    switch(event_id)
    {
        case LOG_EVT_SPAD:
            if(logger_source == EVENT_LOG_SOURCE_ONBOARD)
            {
                return EVENT_LOG_FMT_SPAD_ONBOARD;
            }
            if(logger_source == EVENT_LOG_SOURCE_STATIONARY)
            {
                return EVENT_LOG_FMT_SPAD_STATIONARY;
            }
            return EVENT_LOG_FMT_NONE;

        case LOG_EVT_HEAD_ON_COLLISION:
            if(logger_source == EVENT_LOG_SOURCE_ONBOARD)
            {
                return EVENT_LOG_FMT_HEAD_ON_ONBOARD;
            }

            if(logger_source == EVENT_LOG_SOURCE_STATIONARY)
            {
                return EVENT_LOG_FMT_HEAD_ON_STATIONARY;
            }

            return EVENT_LOG_FMT_NONE;

        case LOG_EVT_REAR_END_COLLISION:
            if(logger_source == EVENT_LOG_SOURCE_ONBOARD)
            {
                return EVENT_LOG_FMT_REAR_END_ONBOARD;
            }

            if(logger_source == EVENT_LOG_SOURCE_STATIONARY)
            {
                return EVENT_LOG_FMT_REAR_END_STATIONARY;
            }

            return EVENT_LOG_FMT_NONE;

        case LOG_EVT_MANUAL_SOS:
            if(logger_source == EVENT_LOG_SOURCE_ONBOARD)
            {
                return EVENT_LOG_FMT_LOCO_SOS;
            }

            /* Stationary KAVACH logs date/time for this event. */
            return EVENT_LOG_FMT_NONE;

        case LOG_EVT_ADJACENT_KAVACH_COMM_FAILURE:
        case LOG_EVT_ADJACENT_KAVACH_COMM_RECOVERY:
        case LOG_EVT_TSRMS_COMM_FAILURE:
        case LOG_EVT_TSRMS_COMM_RECOVERY:
        case LOG_EVT_EI_COMM_FAILURE:
        case LOG_EVT_EI_COMM_RECOVERY:
        case LOG_EVT_COMM_FAILURE:
        case LOG_EVT_COMM_FAILURE_RECOVERY:
            return EVENT_LOG_FMT_COMMUNICATION;

        case LOG_EVT_PERIPHERAL_COMM_FAILURE:
        case LOG_EVT_PERIPHERAL_COMM_RECOVERY:
            return EVENT_LOG_FMT_PERIPHERAL;

        case LOG_EVT_FIELD_INPUT_CHANGE:
            return EVENT_LOG_FMT_FIELD_INPUT;

        case LOG_EVT_STATIONARY_SNAPSHOT:
            return EVENT_LOG_FMT_STATIONARY_SNAPSHOT;

        /* Onboard records carry location; stationary records do not. */
        case LOG_EVT_FOREIGN_RFID_SOS:
        case LOG_EVT_ODO_ERROR_SOS:
        case LOG_EVT_SHUNT_LIMIT_SOS:
        case LOG_EVT_TRAIN_TRIP:
        case LOG_EVT_UNUSUAL_STOP:
        case LOG_EVT_UNUSUAL_STOP_RELEASE:
        case LOG_EVT_MANUAL_SOS_CANCEL:
            return (logger_source == EVENT_LOG_SOURCE_ONBOARD) ?
                   EVENT_LOG_FMT_LOCATION : EVENT_LOG_FMT_NONE;

        case LOG_EVT_STATIONARY_SOS:
        case LOG_EVT_STATIONARY_SOS_CANCEL:
            return EVENT_LOG_FMT_NONE;

        default:
            return EVENT_LOG_FMT_NONE;
    }
}

/* ============================================================================
 * Serialize event record
 * Wire record:
 * Byte 0..1    EVENT_ID
 * Byte 2       SOURCE
 * Byte 3       FORMAT_TYPE
 * Byte 4..7    EPOCH_TIME       uint32, little-endian
 * Byte 8       PAYLOAD_LENGTH
 * Byte 9..     EVENT-SPECIFIC PAYLOAD
 * Location is present only in formats that require it.
 * ========================================================================== */

uint16_t EventLog_Serialize(const EventLogRecord_t *record, uint8_t out_buffer[EVENT_LOG_MAX_RECORD_SIZE])
{
    uint16_t index = 0U;
    uint16_t payload_start;
    uint16_t payload_length = 0U;

    if((record == NULL) || (out_buffer == NULL))
    {
        return 0U;
    }

    /** ---------------------------------------------------------------
     * Common header
     * ---------------------------------------------------------------*/

    EventLog_PutU16(out_buffer, &index, record->event_id);
    out_buffer[index++] = (uint8_t)record->source;
    out_buffer[index++] = (uint8_t)record->format_type;
    EventLog_PutU32(out_buffer, &index, record->epoch_time);
    /** Reserve byte 8 for payload length. */
    payload_start = index;
    index++;

    /* ---------------------------------------------------------------
     * Event-specific payload
     * --------------------------------------------------------------- */
    switch(record->format_type)
    {
        case EVENT_LOG_FMT_NONE:
        {
            payload_length = 0U;
        }
        break;

        case EVENT_LOG_FMT_LOCATION:
        {
            EventLog_PutLocation(out_buffer, &index, &record->data.location);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_SPAD_ONBOARD:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.spad_onboard.location);
            EventLog_PutU16(out_buffer, &index, record->data.spad_onboard.stationary_kavach_id);
            out_buffer[index++] = record->data.spad_onboard.signal_type;
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_SPAD_STATIONARY:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.spad_stationary.location);
            EventLog_PutU16(out_buffer, &index, record->data.spad_stationary.onboard_kavach_id);
            EventLog_PutU16(out_buffer, &index,
                            record->data.spad_stationary.signal_name);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_HEAD_ON_ONBOARD:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.head_onboard.location);
            EventLog_PutU16(out_buffer, &index, record->data.head_onboard.stationary_kavach_id);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_HEAD_ON_STATIONARY:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.head_stationary.location);
            EventLog_PutU16(out_buffer, &index, record->data.head_stationary.onboard_kavach_l1_id);
            EventLog_PutU16(out_buffer, &index, record->data.head_stationary.onboard_kavach_l2_id);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_REAR_END_ONBOARD:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.rear_end_onboard.location);
            EventLog_PutU16(out_buffer, &index, record->data.rear_end_onboard.onboard_kavach_l2_id);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_REAR_END_STATIONARY:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.rear_end_stationary.location);
            EventLog_PutU16(out_buffer, &index, record->data.rear_end_stationary.onboard_kavach_l1_id);
            EventLog_PutU16(out_buffer, &index, record->data.rear_end_stationary.onboard_kavach_l2_id);
            EventLog_PutU32(out_buffer, &index, record->data.rear_end_stationary.distance_l1_l2_m);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_LOCO_SOS:
        {
            EventLog_PutLocation(out_buffer, &index,
                                 &record->data.loco_sos.location);
            EventLog_PutU16(out_buffer, &index, record->data.loco_sos.source_onboard_kavach_id);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_COMMUNICATION_WITH_LOCATION:
        {
            const EventLogCommunication_t *communication =
                &record->data.communication_with_location.communication;
            EventLog_PutLocation(out_buffer, &index,
                &record->data.communication_with_location.location);
            out_buffer[index++] = (uint8_t)communication->target;
            out_buffer[index++] = (uint8_t)communication->channel;
            out_buffer[index++] = communication->peer_id_valid;
            if(communication->peer_id_valid != 0U)
            {
                EventLog_PutU16(out_buffer, &index, communication->peer_id);
            }
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_COMMUNICATION:
        {
            out_buffer[index++] = (uint8_t)record->data.communication.target;
            out_buffer[index++] = (uint8_t)record->data.communication.channel;
            out_buffer[index++] = record->data.communication.peer_id_valid;
            if(record->data.communication.peer_id_valid != 0U)
            {
                EventLog_PutU16(out_buffer, &index, record->data.communication.peer_id);
            }
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_FIELD_INPUT:
        {
            EventLog_PutU16(out_buffer, &index,
                            record->data.field_input.relay_address);
            out_buffer[index++] = record->data.field_input.old_status;
            out_buffer[index++] = record->data.field_input.new_status;
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_PERIPHERAL:
        {
            out_buffer[index++] = record->data.peripheral.device_index;
            out_buffer[index++] = record->data.peripheral.mandatory;
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        case EVENT_LOG_FMT_STATIONARY_SNAPSHOT:
        {
            memcpy(&out_buffer[index],
                   record->data.stationary_snapshot.input_image,
                   EVENT_LOG_INPUT_IMAGE_BYTES);
            index = (uint16_t)(index + EVENT_LOG_INPUT_IMAGE_BYTES);
            out_buffer[index++] =
                record->data.stationary_snapshot.station_sos_active;
            EventLog_PutU32(out_buffer, &index,
                record->data.stationary_snapshot.system_fault_mask);
            payload_length = (uint16_t)(index - payload_start - 1U);
        }
        break;

        default:
            return 0U;
    }

    if(index > EVENT_LOG_MAX_RECORD_SIZE)
    {
        return 0U;
    }

    if(payload_length > 255U)
    {
        return 0U;
    }

    out_buffer[payload_start] = (uint8_t)payload_length;
    return index;
}

/* ============================================================================
 * CAN header creation
 * ========================================================================== */

static uint8_t EventLog_MakeCanByte0(uint8_t seq_total, uint8_t transaction_id)
{
    /* PKT_TYPE is application data; the transport low nibble is the ID. */
    return (uint8_t)((((seq_total) & 0x0FU) << 4U) | (transaction_id & 0x0FU));
}

static uint8_t EventLog_MakeCanByte1(uint8_t seq_total, uint8_t seq_index)
{
    return (uint8_t)(((seq_index & 0x3FU) << 2U) | ((seq_total >> 4) & 0x03U));
}

/* ============================================================================
 * CAN transmission
 * Only one record is kept in the TX state at a time.
 * The record remains in the queue until it has been popped into the TX
 * buffer. If CAN transmission fails, the current fragment is retried.
 * ========================================================================== */

uint8_t EventLog_ProcessTx(uint8_t max_frames)
{
    uint8_t frames_sent = 0U;

    if(max_frames == 0U)
    {
        return 0U;
    }

    if(g_event_log.tx_callback == NULL)
    {
        return 0U;
    }

    /*
     * ---------------------------------------------------------------
     * Load next queued record.
     *
     * Only load a new record when there is no active transaction.
     * ---------------------------------------------------------------
     */
    if(!g_event_log.tx_active)
    {
        EventLogRecord_t record;

        if(!EventLog_QueuePop(&record))
        {
            return 0U;
        }

        g_event_log.tx_record_length = EventLog_Serialize(&record, g_event_log.tx_record_buffer);

        if(g_event_log.tx_record_length == 0U)
        {
            data_logger_tx_failure_count++;
            return 0U;
        }

        g_event_log.tx_total_fragments = (uint8_t)((g_event_log.tx_record_length + EVENT_LOG_CAN_PAYLOAD_SIZE - 1U) / EVENT_LOG_CAN_PAYLOAD_SIZE);

        if(g_event_log.tx_total_fragments == 0U)
        {
            data_logger_tx_failure_count++;
            return 0U;
        }

        if(g_event_log.tx_total_fragments >
           EVENT_LOG_MAX_FRAGMENTS)
        {
            data_logger_tx_failure_count++;
            return 0U;
        }

        g_event_log.tx_fragment_index = 0U;
        g_event_log.tx_retry_count = 0U;
        g_event_log.tx_transaction_id = g_event_log.next_transaction_id;
        g_event_log.next_transaction_id = (uint8_t)((g_event_log.next_transaction_id + 1U) & 0x0FU);
        g_event_log.tx_waiting_ack = false;
        g_event_log.tx_start_time = 0U;

        g_event_log.tx_active = true;
    }

    /*
     * ---------------------------------------------------------------
     * If all fragments have already been transmitted, wait for ACK.
     * Do not transmit anything else.
     * ---------------------------------------------------------------
     */
    if(g_event_log.tx_waiting_ack)
    {
        return 0U;
    }

    /*
     * ---------------------------------------------------------------
     * Send up to max_frames.
     * ---------------------------------------------------------------
     */
    while(g_event_log.tx_active && !g_event_log.tx_waiting_ack && (frames_sent < max_frames))
    {
        uint8_t can_data[EVENT_LOG_CAN_FRAME_SIZE];
        uint16_t offset;
        uint16_t bytes_remaining;
        uint8_t bytes_to_send;

        memset(can_data, 0, sizeof(can_data));

        /*
         * CAN transport header.
         */
        can_data[0] = EventLog_MakeCanByte0(g_event_log.tx_total_fragments,
                                             g_event_log.tx_transaction_id);

        can_data[1] = EventLog_MakeCanByte1(g_event_log.tx_total_fragments, g_event_log.tx_fragment_index);

        offset = (uint16_t)(g_event_log.tx_fragment_index * EVENT_LOG_CAN_PAYLOAD_SIZE);

        bytes_remaining = (uint16_t)(g_event_log.tx_record_length - offset);

        if(bytes_remaining > EVENT_LOG_CAN_PAYLOAD_SIZE)
        {
            bytes_to_send = EVENT_LOG_CAN_PAYLOAD_SIZE;
        }
        else
        {
            bytes_to_send = (uint8_t)bytes_remaining;
        }

        if(bytes_to_send > 0U)
        {
            memcpy(&can_data[2], &g_event_log.tx_record_buffer[offset], bytes_to_send);
        }

        /*
         * Try to transmit.
         */
        if(!g_event_log.tx_callback(EVENT_LOG_CAN_ID, can_data))
        {
            break;
        }

        frames_sent++;

        g_event_log.tx_fragment_index++;

        /*
         * -----------------------------------------------------------
         * Last fragment transmitted.
         *
         * Now stop sending and wait for ACK.
         * -----------------------------------------------------------
         */
        if(g_event_log.tx_fragment_index >= g_event_log.tx_total_fragments)
        {
            g_event_log.tx_waiting_ack = true;
            g_event_log.tx_start_time = system_ms;
        }
    }

    return frames_sent;
}

/* ============================================================
 *  DATALOGGER ACK RX
 * ============================================================ */
void data_logger_ack_rx_handle(uint32_t can_id, uint8_t *data)
{
    uint16_t ack_can_id;
    uint8_t action_type;
    uint8_t ack_status;
    uint8_t transaction_id;

    if(can_id != EVENT_LOG_ACK_CAN_ID)
    {
        return;
    }

    /*
     * Byte 0-1 : ACK_CAN_ID
     */
    ack_can_id = ((uint16_t)data[0] << 8) |
                 (uint16_t)data[1];

    /*
     * Byte 2 : ACTION_TYPE
     */
    action_type = data[2];

    /*
     * Byte 3 : ACK_STATUS
     */
    ack_status = data[3];
    transaction_id = (uint8_t)(data[4] & 0x0FU);

    /*
     * ACK must belong to Data Logger TX.
     */
    if(ack_can_id !=
       (uint16_t)can_get_local_tx_id(EVENT_LOG_CAN_ID))
    {
        return;
    }

    /*
     * Store ACK information if you still need these
     * variables for diagnostics/debugging.
     */
    data_logger_ack_received = 1U;
    data_logger_ack_action = action_type;
    data_logger_ack_status = ack_status;

    /*
     * Complete/reject the active Event Logger transaction.
     */
    EventLog_AckReceived(ack_status, transaction_id);
}
