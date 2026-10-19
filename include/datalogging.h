/*
 * datalogging.h
 *
 *  Created on: 18-Sep-2026
 *      Author: gamer
 */

#ifndef INCLUDE_DATALOGGING_H_
#define INCLUDE_DATALOGGING_H_

#include <stdint.h>
#include <stdbool.h>

/** ============================================================================
 * KAVACH EVENT LOGGER
 * ============================================================================
 * Logical event record: Common information + format_type + event-specific typed data
 * CAN transport:
 *   Byte 0:
 *       bits 3:0 = TRANSACTION_ID
 *       bits 7:4 = SEQ_TOTAL[3:0]
 *   Byte 1:
 *       bits 1:0 = SEQ_TOTAL[5:4]
 *       bits 7:2 = SEQ_INDEX[5:0]
 *   Bytes 2..7:
 *       6 bytes of serialized event record
 * A retry retains its TRANSACTION_ID; only a new logical record advances it.
 * No CRC is used here.
 * No magic value is used here.
 * ============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */
#define EVENT_LOG_CAN_ID                 0x210U
#define EVENT_LOG_ACK_CAN_ID             0x214U
#define EVENT_LOG_PKT_TYPE               0x0AU
#define EVENT_LOG_QUEUE_SIZE             16U
#define EVENT_LOG_MAX_RECORD_SIZE        64U
#define EVENT_LOG_CAN_FRAME_SIZE         8U
#define EVENT_LOG_CAN_PAYLOAD_SIZE       6U
#define EVENT_LOG_MAX_FRAGMENTS          63U
#define EVENT_LOG_STATIONARY_INPUT_COUNT 93U
#define EVENT_LOG_INPUT_IMAGE_BYTES      12U
#define EVENT_LOG_SNAPSHOT_PERIOD_SEC    2U

#define EVENT_LOG_ACK_TIMEOUT_MS         100U
#define EVENT_LOG_ACK_MAX_RETRIES        3U
#define EVENT_LOG_ACK_OK                 0x00U

extern volatile uint8_t data_logger_ack_received;
extern volatile uint8_t data_logger_ack_action;
extern volatile uint8_t data_logger_ack_status;
extern volatile uint32_t data_logger_queue_full_count;
extern volatile uint32_t data_logger_tx_failure_count;
/* --------------------------------------------------------------------------
 * Source of the event log
 * -------------------------------------------------------------------------- */
typedef enum
{
    EVENT_LOG_SOURCE_ONBOARD = 0U,
    EVENT_LOG_SOURCE_STATIONARY,
    EVENT_LOG_SOURCE_TSRMS,
    EVENT_LOG_SOURCE_EI
} EventLogSource_t;

/* --------------------------------------------------------------------------
 * Date/time
 * -------------------------------------------------------------------------- */
typedef uint32_t EventLogEpoch_t;

/* --------------------------------------------------------------------------
 * Location
 * The FRS specifies:
 *   Reference RFID
 *   Distance from the Reference RFID at zero speed
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint32_t reference_location_m;
    uint32_t distance_from_rfid_m;
} EventLogLocation_t;

/* --------------------------------------------------------------------------
 * Common context used when creating an event
 * -------------------------------------------------------------------------- */

typedef struct
{
    EventLogEpoch_t epoch_time;
} EventLogContext_t;

/* --------------------------------------------------------------------------
 * Event specific formats
 * -------------------------------------------------------------------------- */

/*
 * SPAD - Onboard KAVACH
 *
 * Required:
 *   Stationary KAVACH ID
 *   Signal Type
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t stationary_kavach_id;
    uint8_t  signal_type;
} EventLogSpadOnboard_t;

/*
 * SPAD - Stationary KAVACH
 *
 * Required:
 *   Onboard KAVACH ID
 *   Signal name
 *
 * Signal name is encoded as a two-byte station-configuration value.
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t onboard_kavach_id;
    uint16_t signal_name;
} EventLogSpadStationary_t;

/*
 * Head-on collision - Onboard
 *
 * Required:
 *   Stationary KAVACH ID
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t stationary_kavach_id;
} EventLogHeadOnOnboard_t;

/*
 * Head-on collision - Stationary
 *
 * Required:
 *   Onboard KAVACH L1 ID
 *   Onboard KAVACH L2 ID
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t onboard_kavach_l1_id;
    uint16_t onboard_kavach_l2_id;
} EventLogHeadOnStationary_t;

/*
 * Rear-end collision - Onboard
 *
 * Required:
 *   Onboard KAVACH L2 ID
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t onboard_kavach_l2_id;
} EventLogRearEndOnboard_t;

/*
 * Rear-end collision - Stationary
 *
 * Required:
 *   Onboard KAVACH L1 ID
 *   Onboard KAVACH L2 ID
 *   Distance between L1 and L2
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t onboard_kavach_l1_id;
    uint16_t onboard_kavach_l2_id;
    uint32_t distance_l1_l2_m;
} EventLogRearEndStationary_t;

/*
 * Manual SOS / SOS from loco
 *
 * Required:
 *   Source Onboard KAVACH ID
 */
typedef struct
{
    EventLogLocation_t location;
    uint16_t source_onboard_kavach_id;
} EventLogLocoSos_t;

/* --------------------------------------------------------------------------
 * Communication formats
 * -------------------------------------------------------------------------- */

typedef enum
{
    EVENT_LOG_COMM_PRIMARY = 0U,
    EVENT_LOG_COMM_SECONDARY,
    EVENT_LOG_COMM_BOTH
} EventLogCommChannel_t;

typedef enum
{
    EVENT_LOG_COMM_TARGET_ADJACENT_KAVACH = 0U,
    EVENT_LOG_COMM_TARGET_TSRMS,
    EVENT_LOG_COMM_TARGET_EI,
    EVENT_LOG_COMM_TARGET_ONBOARD_KAVACH
} EventLogCommTarget_t;

typedef struct
{
    EventLogCommTarget_t  target;
    EventLogCommChannel_t channel;

    /*
     * Some communication events identify the remote equipment.
     * For example:
     *   adjacent Stationary KAVACH ID
     *   Stationary KAVACH ID as seen by TSRMS/EI
     */
    uint8_t  peer_id_valid;
    uint16_t peer_id;
} EventLogCommunication_t;

typedef struct
{
    EventLogLocation_t location;
    EventLogCommunication_t communication;
} EventLogCommunicationWithLocation_t;

/* STCAS-only relay/input transition payload. */
typedef struct
{
    uint16_t relay_address;
    uint8_t old_status;
    uint8_t new_status;
} EventLogFieldInput_t;

/* Product-specific payloads are kept in the shared format definition. */
typedef struct
{
    uint8_t device_index;
    uint8_t mandatory;
} EventLogPeripheral_t;

typedef struct
{
    uint8_t input_image[EVENT_LOG_INPUT_IMAGE_BYTES];
    uint8_t station_sos_active;
    uint32_t system_fault_mask;
} EventLogStationarySnapshot_t;

/* --------------------------------------------------------------------------
 * Format type
 * -------------------------------------------------------------------------- */

typedef enum
{
    EVENT_LOG_FMT_NONE = 0U,
    EVENT_LOG_FMT_LOCATION,

    EVENT_LOG_FMT_SPAD_ONBOARD,
    EVENT_LOG_FMT_SPAD_STATIONARY,

    EVENT_LOG_FMT_HEAD_ON_ONBOARD,
    EVENT_LOG_FMT_HEAD_ON_STATIONARY,

    EVENT_LOG_FMT_REAR_END_ONBOARD,
    EVENT_LOG_FMT_REAR_END_STATIONARY,

    EVENT_LOG_FMT_LOCO_SOS,

    EVENT_LOG_FMT_COMMUNICATION,
    EVENT_LOG_FMT_COMMUNICATION_WITH_LOCATION,

    EVENT_LOG_FMT_FIELD_INPUT,
    EVENT_LOG_FMT_PERIPHERAL,
    EVENT_LOG_FMT_STATIONARY_SNAPSHOT

} EventLogFormat_t;

/* --------------------------------------------------------------------------
 * Event-specific union
 * -------------------------------------------------------------------------- */

typedef union
{
    uint8_t                       none;
    EventLogLocation_t            location;

    EventLogSpadOnboard_t         spad_onboard;
    EventLogSpadStationary_t      spad_stationary;

    EventLogHeadOnOnboard_t       head_onboard;
    EventLogHeadOnStationary_t    head_stationary;

    EventLogRearEndOnboard_t      rear_end_onboard;
    EventLogRearEndStationary_t   rear_end_stationary;

    EventLogLocoSos_t             loco_sos;

    EventLogCommunication_t       communication;
    EventLogCommunicationWithLocation_t communication_with_location;

    EventLogFieldInput_t          field_input;
    EventLogPeripheral_t          peripheral;
    EventLogStationarySnapshot_t  stationary_snapshot;

} EventLogSpecificData_t;

/* --------------------------------------------------------------------------
 * Complete logical event record
 * -------------------------------------------------------------------------- */

typedef struct
{
    uint16_t              event_id;
    EventLogSource_t      source;
    EventLogFormat_t      format_type;
    EventLogEpoch_t    epoch_time;
    EventLogSpecificData_t data;
} EventLogRecord_t;

/* --------------------------------------------------------------------------
 * Event IDs
 *
 * Values 0x0100 onward correspond to FRS/DMI event scenarios.
 * Values 0x0200 onward correspond to primary event logger scenarios.
 * Values 0x0300 onward correspond to communication scenarios.
 * -------------------------------------------------------------------------- */

/* ============================= DMI events =============================== */

/* FRS LP alert messages 1..78 */

#define LOG_EVT_DMI_SOS_SELF_MANUAL                 0x0101U
#define LOG_EVT_DMI_SOS_SELF_BLOCK_STOP             0x0102U
#define LOG_EVT_DMI_SOS_SELF_TRAIN_PARTED           0x0103U
#define LOG_EVT_DMI_SOS_FROM_LOCO_MANUAL            0x0104U
#define LOG_EVT_DMI_SOS_FROM_LOCO_BLOCK_STOP       0x0105U
#define LOG_EVT_DMI_SOS_FROM_LOCO_TRAIN_PARTED     0x0106U
#define LOG_EVT_DMI_SOS_FROM_STATION_ALL_LOCOS      0x0107U
#define LOG_EVT_DMI_SOS_FROM_STATION_THIS_LOCO     0x0108U
#define LOG_EVT_DMI_OVERSPEED                        0x0109U
#define LOG_EVT_DMI_BRAKE_SPEED_LIMIT_EXCEEDED      0x010AU
#define LOG_EVT_DMI_FSB_WILL_APPLIED                0x010BU
#define LOG_EVT_DMI_EB_WILL_APPLIED                 0x010CU
#define LOG_EVT_DMI_BIU_ISOLATED                    0x010DU
#define LOG_EVT_DMI_TRAIN_TYPE_SELECTED             0x010EU
#define LOG_EVT_DMI_SYSTEM_FAULT                    0x010FU

#define LOG_EVT_DMI_ACK_BLOCK_STOP_SOS              0x0110U
#define LOG_EVT_DMI_EB_BYPASSED                     0x0111U
#define LOG_EVT_DMI_TRAIN_TRIPPED                   0x0112U
#define LOG_EVT_DMI_DEAD_END                        0x0113U
#define LOG_EVT_DMI_STANDSTILL_PROTECTION           0x0114U
#define LOG_EVT_DMI_ROLLBACK_PROTECTION             0x0115U
#define LOG_EVT_DMI_REV_MOVEMENT_NOT_ALLOWED        0x0116U
#define LOG_EVT_DMI_STANDBY_CAB_INACTIVE            0x0117U
#define LOG_EVT_DMI_TERRITORY_EXIT                  0x0118U
#define LOG_EVT_DMI_STATION_RADIO_COMM_FAIL_SR      0x0119U
#define LOG_EVT_DMI_NO_TRACK_PROFILE                0x011AU
#define LOG_EVT_DMI_TAGS_MISSING                    0x011BU
#define LOG_EVT_DMI_DIRECTION_UNKNOWN               0x011CU
#define LOG_EVT_DMI_GPS_FAIL                        0x011DU
#define LOG_EVT_DMI_STATION_RADIO_COMM_FAIL_LS      0x011EU
#define LOG_EVT_DMI_TSR_LINK_FAIL                   0x011FU

#define LOG_EVT_DMI_HEAD_ON_COLLISION_ALERT         0x0120U
#define LOG_EVT_DMI_REAR_ON_COLLISION_ALERT         0x0121U
#define LOG_EVT_DMI_OVERRIDE_SELECTED               0x0122U
#define LOG_EVT_DMI_REV_MODE_EXPIRES                0x0123U
#define LOG_EVT_DMI_MANNED_LC_GATE                  0x0124U
#define LOG_EVT_DMI_UNMANNED_LC_GATE                0x0125U
#define LOG_EVT_DMI_WAITING_STATION_RADIO_LS       0x0126U
#define LOG_EVT_DMI_LEADING_NONLEADING_BOTH        0x0127U
#define LOG_EVT_DMI_TRAIN_LENGTH_PROGRESS           0x0128U
#define LOG_EVT_DMI_TRAIN_LENGTH_SUCCESS            0x0129U
#define LOG_EVT_DMI_TRAIN_LENGTH_FAIL               0x012AU
#define LOG_EVT_DMI_TRAIN_LENGTH_ABORTED           0x012BU
#define LOG_EVT_DMI_TURNOUT                         0x012CU
#define LOG_EVT_DMI_TSR                             0x012DU
#define LOG_EVT_DMI_PSR                             0x012EU
#define LOG_EVT_DMI_END_OF_AUTHORITY                0x012FU

#define LOG_EVT_DMI_KAVACH_TERRITORY_ENTRY         0x0130U
#define LOG_EVT_DMI_SELF_TEST_PROGRESS              0x0131U
#define LOG_EVT_DMI_SELF_TEST_SUCCESS               0x0132U
#define LOG_EVT_DMI_SELF_TEST_FAIL                  0x0133U
#define LOG_EVT_DMI_BRAKE_TEST_WAIT_MR              0x0134U
#define LOG_EVT_DMI_BRAKE_TEST_WAIT_BP              0x0135U
#define LOG_EVT_DMI_BRAKE_TEST_NSB_BP               0x0136U
#define LOG_EVT_DMI_BRAKE_TEST_FSB_BP               0x0137U
#define LOG_EVT_DMI_BRAKE_TEST_EB_BP                0x0138U
#define LOG_EVT_DMI_BRAKE_TEST_LEB_BC               0x0139U
#define LOG_EVT_DMI_BRAKE_TEST_SUCCESS_1            0x013AU
#define LOG_EVT_DMI_BRAKE_TEST_SUCCESS_2            0x013BU
#define LOG_EVT_DMI_BRAKE_TEST_FAIL                 0x013CU
#define LOG_EVT_DMI_BRAKE_TEST_FAIL_ACK_RETEST      0x013DU
#define LOG_EVT_DMI_TRAIN_CONFIG_SELECTED           0x013EU
#define LOG_EVT_DMI_SHUNT_MODE_SELECTED             0x013FU
#define LOG_EVT_DMI_RADIO_HOLE                      0x0140U
#define LOG_EVT_DMI_ACK_OS_MODE                     0x0141U
#define LOG_EVT_DMI_WAITING_TRACTION_COMMAND        0x0142U
#define LOG_EVT_DMI_TRACTION_CUTOFF_FAIL            0x0143U
#define LOG_EVT_DMI_FOULING_MARK_ENTRY              0x0144U
#define LOG_EVT_DMI_NEUTRAL_SECTION                 0x0145U
#define LOG_EVT_DMI_BRAKING_SYSTEM_MALFUNCTION     0x0146U
#define LOG_EVT_DMI_NO_FORWARD_DIRECTION_REV       0x0147U
#define LOG_EVT_DMI_ACK_SR_AUTH_RECEIVED            0x0148U
#define LOG_EVT_DMI_ACK_SR_SLIP_SKID                0x0149U
#define LOG_EVT_DMI_ACK_SR_FOREIGN_TAG              0x014AU
#define LOG_EVT_DMI_ACK_SR_ODO_ERROR                0x014BU
#define LOG_EVT_DMI_BRAKE_SHUNT_LIMIT_EXCEEDED      0x014CU
#define LOG_EVT_DMI_BRAKE_STATION_GENERAL_SOS       0x014DU
#define LOG_EVT_DMI_BRAKE_SPAD_DETECTED             0x014EU

/* =========================== Core event IDs ============================== */

#define LOG_EVT_SPAD                                0x0201U
#define LOG_EVT_HEAD_ON_COLLISION                   0x0202U
#define LOG_EVT_REAR_END_COLLISION                  0x0203U

#define LOG_EVT_MANUAL_SOS                          0x0204U
#define LOG_EVT_MANUAL_SOS_CANCEL                   0x0205U

#define LOG_EVT_STATIONARY_SOS                      0x0206U
#define LOG_EVT_STATIONARY_SOS_CANCEL               0x0207U

#define LOG_EVT_FOREIGN_RFID_SOS                    0x0208U
#define LOG_EVT_ODO_ERROR_SOS                       0x0209U
#define LOG_EVT_SHUNT_LIMIT_SOS                     0x020AU

#define LOG_EVT_TRAIN_TRIP                           0x020BU
#define LOG_EVT_UNUSUAL_STOP                        0x020CU
#define LOG_EVT_UNUSUAL_STOP_RELEASE                0x020DU

/* ========================= Communication IDs ============================ */

#define LOG_EVT_COMM_FAILURE                        0x0301U
#define LOG_EVT_COMM_FAILURE_RECOVERY               0x0302U

#define LOG_EVT_ADJACENT_KAVACH_COMM_FAILURE        0x0303U
#define LOG_EVT_ADJACENT_KAVACH_COMM_RECOVERY       0x0304U

#define LOG_EVT_TSRMS_COMM_FAILURE                  0x0305U
#define LOG_EVT_TSRMS_COMM_RECOVERY                 0x0306U

#define LOG_EVT_EI_COMM_FAILURE                     0x0307U
#define LOG_EVT_EI_COMM_RECOVERY                    0x0308U

#define LOG_EVT_PERIPHERAL_COMM_FAILURE             0x0309U
#define LOG_EVT_PERIPHERAL_COMM_RECOVERY            0x030AU

/* STCAS-specific field-input event. */
#define LOG_EVT_FIELD_INPUT_CHANGE                  0x0401U
#define LOG_EVT_STATIONARY_SNAPSHOT                 0x0402U

/* --------------------------------------------------------------------------
 * CAN transmit callback
 *
 * The application supplies this function.
 *
 * Return:
 *   true  -> CAN frame accepted
 *   false -> frame not transmitted; EventLog will retry later
 * -------------------------------------------------------------------------- */

typedef bool (*EventLogCanTxCallback_t)(
    uint32_t can_id,
    const uint8_t data[EVENT_LOG_CAN_FRAME_SIZE]
);

/* --------------------------------------------------------------------------
 * Initialization
 * -------------------------------------------------------------------------- */

void EventLog_Init(EventLogCanTxCallback_t tx_callback);

/* --------------------------------------------------------------------------
 * Queue functions
 * -------------------------------------------------------------------------- */

bool EventLog_QueuePush(const EventLogRecord_t *record);

bool EventLog_QueuePop(EventLogRecord_t *record);

uint16_t EventLog_QueueCount(void);

/* --------------------------------------------------------------------------
 * Record construction
 * -------------------------------------------------------------------------- */
void EventLog_RecordSetCommon(EventLogRecord_t *record, uint16_t event_id, EventLogFormat_t format_type, const EventLogContext_t *context);
bool EventLog_AddSimple(uint16_t event_id, const EventLogContext_t *context);
bool EventLog_AddSimpleWithLocation(uint16_t event_id, const EventLogContext_t *context, const EventLogLocation_t *location);
bool EventLog_AddSpadOnboard(const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t stationary_kavach_id, uint8_t signal_type);
bool EventLog_AddSpadStationary(const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t onboard_kavach_id, uint16_t signal_name);
bool EventLog_AddHeadOnOnboard(const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t stationary_kavach_id);
bool EventLog_AddHeadOnStationary(const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t onboard_kavach_l1_id, uint16_t onboard_kavach_l2_id);
bool EventLog_AddRearEndOnboard(const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t onboard_kavach_l2_id);
bool EventLog_AddRearEndStationary(const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t onboard_kavach_l1_id, uint16_t onboard_kavach_l2_id, uint32_t distance_l1_l2_m);
bool EventLog_AddLocoSos(uint16_t event_id, const EventLogContext_t *context, const EventLogLocation_t *location, uint16_t source_onboard_kavach_id);
bool EventLog_AddCommunication(uint16_t event_id, const EventLogContext_t *context, EventLogCommTarget_t target, EventLogCommChannel_t channel,
                               bool peer_id_valid, uint16_t peer_id, const EventLogLocation_t *location);
bool EventLog_AddFieldInput(const EventLogContext_t *context,
                            uint16_t relay_address,
                            uint8_t old_status,
                            uint8_t new_status);
bool EventLog_AddPeripheralCommunication(uint16_t event_id,
                                         const EventLogContext_t *context,
                                         uint8_t device_index,
                                         bool mandatory);
bool EventLog_AddStationarySnapshot(const EventLogContext_t *context,
                                    const uint8_t input_image[EVENT_LOG_INPUT_IMAGE_BYTES],
                                    bool station_sos_active,
                                    uint32_t system_fault_mask);

/* --------------------------------------------------------------------------
 * Serialization
 * Returns number of serialized bytes.
 * Returns 0 on error.
 * -------------------------------------------------------------------------- */
uint16_t EventLog_Serialize(const EventLogRecord_t *record, uint8_t out_buffer[EVENT_LOG_MAX_RECORD_SIZE]);

/* --------------------------------------------------------------------------
 * CAN transmission
 * Process at most max_frames CAN frames in one call.
 * Recommended in a periodic task:
 *     EventLog_ProcessTx(1U);
 * This avoids blocking the main loop with a large event.
 * -------------------------------------------------------------------------- */
uint8_t EventLog_ProcessTx(uint8_t max_frames);

void EventLog_AckReceived(uint8_t ack_status, uint8_t transaction_id);
void EventLog_ProcessAck(void);
void data_logger_ack_rx_handle(uint32_t can_id, uint8_t *data);
/* --------------------------------------------------------------------------
 * Utility
 * -------------------------------------------------------------------------- */
EventLogFormat_t EventLog_GetFormatForEvent(uint16_t event_id, EventLogSource_t logger_source);

bool Log_Event_Simple(uint16_t event_id);
bool Log_Event_SimpleWithLocation(uint16_t event_id, const EventLogLocation_t *location);
bool Log_Event_Spad(const EventLogLocation_t *location, uint16_t onboard_kavach_id, uint16_t signal_name);
bool Log_Event_HeadOn(uint16_t onboard_kavach_l1_id,
                      uint16_t onboard_kavach_l2_id,
                      const EventLogLocation_t *location);
bool Log_Event_RearEnd(uint16_t onboard_kavach_l1_id,
                       uint16_t onboard_kavach_l2_id,
                       uint32_t distance_l1_l2_m,
                       const EventLogLocation_t *location);
bool Log_Event_ManualSos(uint16_t source_onboard_kavach_id,
                         const EventLogLocation_t *location);
bool Log_Event_Communication(uint16_t event_id, EventLogCommTarget_t target, EventLogCommChannel_t channel, bool peer_id_valid, uint16_t peer_id, const EventLogLocation_t *location);
bool Log_Event_FieldInput(uint16_t relay_address,
                          uint8_t old_status,
                          uint8_t new_status);
bool Log_Event_PeripheralCommunication(bool recovered,
                                       uint8_t device_index,
                                       bool mandatory);
bool Log_Event_StationarySnapshot(
                                  const uint8_t input_image[EVENT_LOG_INPUT_IMAGE_BYTES],
                                  bool station_sos_active,
                                  uint32_t system_fault_mask);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_DATALOGGING_H_ */
