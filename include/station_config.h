#ifndef STATION_CONFIG_H
#define STATION_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Generic Stationary KAVACH configuration container.
 *
 * Station data is supplied as a versioned, big-endian binary blob generated
 * by an external commissioning tool.  No station-specific table is compiled
 * into this module.
 *
 * The caller owns the blob memory and must keep it unchanged while the
 * configuration is active.
 */

#define STATION_CONFIG_MAGIC                    0x53544346UL /* "STCF" */
#define STATION_CONFIG_FORMAT_MAJOR             1U
#define STATION_CONFIG_FORMAT_MINOR             1U

#define STATION_CONFIG_FIXED_HEADER_SIZE        32U
#define STATION_CONFIG_SECTION_DESCRIPTOR_SIZE  16U
#define STATION_CONFIG_FIELD_INPUT_ENTRY_SIZE   32U
#define STATION_CONFIG_FIELD_NAME_SIZE          24U
#define STATION_CONFIG_FIELD_INPUT_SECTION_VERSION 2U

#define STATION_CONFIG_SIGNAL_ENTRY_SIZE        32U
#define STATION_CONFIG_SIGNAL_NAME_SIZE         20U
#define STATION_CONFIG_SIGNAL_SECTION_VERSION   1U
#define STATION_CONFIG_POINT_ENTRY_SIZE         24U
#define STATION_CONFIG_POINT_NAME_SIZE          16U
#define STATION_CONFIG_POINT_SECTION_VERSION    1U
#define STATION_CONFIG_TRACK_ENTRY_SIZE         24U
#define STATION_CONFIG_TRACK_NAME_SIZE          16U
#define STATION_CONFIG_TRACK_SECTION_VERSION    1U

#define STATION_CONFIG_MAX_BLOB_SIZE            131072UL
#define STATION_CONFIG_MAX_SECTIONS             16U
#define STATION_CONFIG_MAX_FIELD_INPUTS         512U
#define STATION_CONFIG_MAX_INPUT_CARDS          16U
#define STATION_CONFIG_INPUT_CHANNELS_PER_CARD  32U
#define STATION_CONFIG_MAX_SIGNALS              256U
#define STATION_CONFIG_MAX_POINTS               256U
#define STATION_CONFIG_MAX_TRACKS               512U

typedef enum
{
    STATION_CONFIG_SECTION_FIELD_INPUTS = 1U,
    STATION_CONFIG_SECTION_SIGNALS = 2U,
    STATION_CONFIG_SECTION_POINTS = 3U,
    STATION_CONFIG_SECTION_TRACKS = 4U,
    STATION_CONFIG_SECTION_ROUTES = 5U,
    STATION_CONFIG_SECTION_ROUTE_CONFLICTS = 6U,
    STATION_CONFIG_SECTION_TOPOLOGY = 7U,
    STATION_CONFIG_SECTION_PROFILES = 8U,
    STATION_CONFIG_SECTION_ADJACENT_STATIONS = 9U,
    STATION_CONFIG_SECTION_RADIO_PARAMETERS = 10U
} station_config_section_type_t;

typedef enum
{
    STATION_INPUT_TYPE_SIGNAL_RELAY = 1U,
    STATION_INPUT_TYPE_POINT_POSITION = 2U,
    STATION_INPUT_TYPE_TRACK_CIRCUIT = 3U,
    STATION_INPUT_TYPE_AXLE_COUNTER = 4U,
    STATION_INPUT_TYPE_SHUNT_SIGNAL = 5U,
    STATION_INPUT_TYPE_CALLING_ON_SIGNAL = 6U,
    STATION_INPUT_TYPE_ROUTE_INDICATOR = 7U,
    STATION_INPUT_TYPE_LC_GATE = 8U,
    STATION_INPUT_TYPE_SPARE = 9U
} station_input_type_t;

/*
 * Describes only the meaning of the raw bit at the CPU boundary.  Conversion
 * to qualified signal/point/track states belongs to the later application
 * layer, not to the CAN driver or configuration loader.
 */
typedef enum
{
    STATION_INPUT_ENCODING_ONE_ASSERTED = 1U,
    STATION_INPUT_ENCODING_ZERO_ASSERTED = 2U,
    STATION_INPUT_ENCODING_ONE_NORMAL = 3U,
    STATION_INPUT_ENCODING_ONE_CLEAR = 4U,
    STATION_INPUT_ENCODING_ONE_OFF = 5U
} station_input_encoding_t;

typedef enum
{
    STATION_CONFIG_STATE_EMPTY = 0U,
    STATION_CONFIG_STATE_VALID = 1U,
    STATION_CONFIG_STATE_INVALID = 2U
} station_config_state_t;

typedef enum
{
    STATION_CONFIG_OK = 0U,
    STATION_CONFIG_ERR_NULL,
    STATION_CONFIG_ERR_SIZE,
    STATION_CONFIG_ERR_MAGIC,
    STATION_CONFIG_ERR_VERSION,
    STATION_CONFIG_ERR_HEADER,
    STATION_CONFIG_ERR_STATION_ID,
    STATION_CONFIG_ERR_REVISION,
    STATION_CONFIG_ERR_CRC,
    STATION_CONFIG_ERR_SECTION_COUNT,
    STATION_CONFIG_ERR_SECTION_BOUNDS,
    STATION_CONFIG_ERR_SECTION_OVERLAP,
    STATION_CONFIG_ERR_DUPLICATE_SECTION,
    STATION_CONFIG_ERR_REQUIRED_SECTION,
    STATION_CONFIG_ERR_ENTRY_SIZE,
    STATION_CONFIG_ERR_ENTRY_COUNT,
    STATION_CONFIG_ERR_INPUT_ID,
    STATION_CONFIG_ERR_DUPLICATE_INPUT_ID,
    STATION_CONFIG_ERR_INPUT_LOCATION,
    STATION_CONFIG_ERR_DUPLICATE_INPUT_LOCATION,
    STATION_CONFIG_ERR_INPUT_TYPE,
    STATION_CONFIG_ERR_INPUT_ENCODING,
    STATION_CONFIG_ERR_INPUT_NAME,
    STATION_CONFIG_ERR_INPUT_FLAGS,
    STATION_CONFIG_ERR_OBJECT_ID,
    STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID,
    STATION_CONFIG_ERR_OBJECT_REFERENCE,
    STATION_CONFIG_ERR_OBJECT_TYPE,
    STATION_CONFIG_ERR_OBJECT_NAME,
    STATION_CONFIG_ERR_OBJECT_FLAGS,
    STATION_CONFIG_ERR_NOT_ACTIVE,
    STATION_CONFIG_ERR_NOT_FOUND
} station_config_result_t;

typedef struct
{
    station_config_result_t result;
    uint16_t section_type;
    uint16_t entry_index;
} station_config_validation_report_t;

typedef struct
{
    uint32_t revision;
    uint16_t station_id;
    uint16_t section_count;
    uint32_t flags;
    uint32_t total_size;
} station_config_header_t;

typedef struct
{
    uint16_t type;
    uint16_t version;
    uint32_t offset;
    uint32_t length;
    uint16_t entry_count;
    uint16_t entry_size;
} station_config_section_t;

typedef struct
{
    uint16_t logical_id;
    uint8_t card_index;
    uint8_t channel_index;
    station_input_type_t type;
    station_input_encoding_t encoding;
    uint16_t flags;
    char name[STATION_CONFIG_FIELD_NAME_SIZE + 1U];
} station_config_field_input_t;

/*
 * Signal relay bindings use logical field-input IDs.  A zero binding means
 * that the corresponding relay is not provided for that signal.
 */
typedef struct
{
    uint16_t signal_id;
    uint16_t recr_input_id;
    uint16_t hecr_input_id;
    uint16_t decr_input_id;
    uint16_t hhecr_input_id;
    uint16_t flags;
    char name[STATION_CONFIG_SIGNAL_NAME_SIZE + 1U];
} station_config_signal_t;

typedef struct
{
    uint16_t point_id;
    uint16_t end_a_input_id;
    uint16_t end_b_input_id;
    uint16_t flags;
    char name[STATION_CONFIG_POINT_NAME_SIZE + 1U];
} station_config_point_t;

typedef enum
{
    STATION_TRACK_KIND_TRACK_CIRCUIT = 1U,
    STATION_TRACK_KIND_AXLE_COUNTER = 2U
} station_track_kind_t;

typedef struct
{
    uint16_t track_id;
    uint16_t input_id;
    station_track_kind_t kind;
    uint16_t flags;
    char name[STATION_CONFIG_TRACK_NAME_SIZE + 1U];
} station_config_track_t;

void station_config_init(void);

station_config_result_t station_config_validate(
    const uint8_t *blob,
    uint32_t blob_size,
    station_config_validation_report_t *report);

station_config_result_t station_config_activate(
    const uint8_t *blob,
    uint32_t blob_size,
    station_config_validation_report_t *report);

void station_config_deactivate(void);
station_config_state_t station_config_get_state(void);
station_config_result_t station_config_get_last_error(void);

station_config_result_t station_config_get_header(
    station_config_header_t *header);

station_config_result_t station_config_get_section(
    uint16_t section_type,
    station_config_section_t *section);

station_config_result_t station_config_get_field_input_count(
    uint16_t *count);

station_config_result_t station_config_get_field_input_by_index(
    uint16_t index,
    station_config_field_input_t *input);

station_config_result_t station_config_find_field_input_by_id(
    uint16_t logical_id,
    station_config_field_input_t *input);

station_config_result_t station_config_find_field_input_by_location(
    uint8_t card_index,
    uint8_t channel_index,
    station_config_field_input_t *input);

station_config_result_t station_config_get_signal_count(uint16_t *count);
station_config_result_t station_config_get_signal_by_index(
    uint16_t index,
    station_config_signal_t *signal);
station_config_result_t station_config_find_signal_by_id(
    uint16_t signal_id,
    station_config_signal_t *signal);

station_config_result_t station_config_get_point_count(uint16_t *count);
station_config_result_t station_config_get_point_by_index(
    uint16_t index,
    station_config_point_t *point);
station_config_result_t station_config_find_point_by_id(
    uint16_t point_id,
    station_config_point_t *point);

station_config_result_t station_config_get_track_count(uint16_t *count);
station_config_result_t station_config_get_track_by_index(
    uint16_t index,
    station_config_track_t *track);
station_config_result_t station_config_find_track_by_id(
    uint16_t track_id,
    station_config_track_t *track);

uint32_t station_config_calculate_crc32(
    const uint8_t *blob,
    uint32_t blob_size);

#endif /* STATION_CONFIG_H */
