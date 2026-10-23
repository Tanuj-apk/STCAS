#include "station_config.h"

#include <stddef.h>
#include <string.h>

#define STATION_CONFIG_CRC_OFFSET              20U
#define STATION_CONFIG_CRC_SIZE                4U
#define STATION_CONFIG_SECTION_DIRECTORY_OFFSET 32U

typedef struct
{
    const uint8_t *blob;
    uint32_t blob_size;
    station_config_state_t state;
    station_config_result_t last_error;
} station_config_context_t;

static station_config_context_t station_config_context;

static uint16_t station_config_read_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) |
                      (uint16_t)data[1]);
}

static uint32_t station_config_read_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           (uint32_t)data[3];
}

static void station_config_report(
    station_config_validation_report_t *report,
    station_config_result_t result,
    uint16_t section_type,
    uint16_t entry_index)
{
    if (report != NULL)
    {
        report->result = result;
        report->section_type = section_type;
        report->entry_index = entry_index;
    }
}

static void station_config_decode_header(
    const uint8_t *blob,
    station_config_header_t *header)
{
    header->total_size = station_config_read_u32(&blob[8]);
    header->revision = station_config_read_u32(&blob[12]);
    header->station_id = station_config_read_u16(&blob[16]);
    header->section_count = station_config_read_u16(&blob[18]);
    header->flags = station_config_read_u32(&blob[24]);
}

static void station_config_decode_section(
    const uint8_t *data,
    station_config_section_t *section)
{
    section->type = station_config_read_u16(&data[0]);
    section->version = station_config_read_u16(&data[2]);
    section->offset = station_config_read_u32(&data[4]);
    section->length = station_config_read_u32(&data[8]);
    section->entry_count = station_config_read_u16(&data[12]);
    section->entry_size = station_config_read_u16(&data[14]);
}

static void station_config_decode_field_input(
    const uint8_t *data,
    station_config_field_input_t *input)
{
    uint8_t index;

    input->logical_id = station_config_read_u16(&data[0]);
    input->card_index = data[2];
    input->channel_index = data[3];
    input->type = (station_input_type_t)data[4];
    input->encoding = (station_input_encoding_t)data[5];
    input->flags = station_config_read_u16(&data[6]);

    for (index = 0U; index < STATION_CONFIG_FIELD_NAME_SIZE; index++)
    {
        input->name[index] = (char)data[8U + index];
    }
    input->name[STATION_CONFIG_FIELD_NAME_SIZE] = '\0';
}

static void station_config_copy_name(
    const uint8_t *source,
    uint8_t size,
    char *destination)
{
    uint8_t index;

    for (index = 0U; index < size; index++)
    {
        destination[index] = (char)source[index];
    }
    destination[size] = '\0';
}

static void station_config_decode_signal(
    const uint8_t *data,
    station_config_signal_t *signal)
{
    signal->signal_id = station_config_read_u16(&data[0]);
    signal->recr_input_id = station_config_read_u16(&data[2]);
    signal->hecr_input_id = station_config_read_u16(&data[4]);
    signal->decr_input_id = station_config_read_u16(&data[6]);
    signal->hhecr_input_id = station_config_read_u16(&data[8]);
    signal->flags = station_config_read_u16(&data[10]);
    station_config_copy_name(&data[12], STATION_CONFIG_SIGNAL_NAME_SIZE,
                             signal->name);
}

static void station_config_decode_point(
    const uint8_t *data,
    station_config_point_t *point)
{
    point->point_id = station_config_read_u16(&data[0]);
    point->end_a_input_id = station_config_read_u16(&data[2]);
    point->end_b_input_id = station_config_read_u16(&data[4]);
    point->flags = station_config_read_u16(&data[6]);
    station_config_copy_name(&data[8], STATION_CONFIG_POINT_NAME_SIZE,
                             point->name);
}

static void station_config_decode_track(
    const uint8_t *data,
    station_config_track_t *track)
{
    track->track_id = station_config_read_u16(&data[0]);
    track->input_id = station_config_read_u16(&data[2]);
    track->kind = (station_track_kind_t)data[4];
    track->flags = station_config_read_u16(&data[6]);
    station_config_copy_name(&data[8], STATION_CONFIG_TRACK_NAME_SIZE,
                             track->name);
}

static uint8_t station_config_name_is_valid(
    const uint8_t *name,
    uint8_t size)
{
    uint8_t index;

    if (name[0] == 0U)
    {
        return 0U;
    }

    for (index = 0U; index < size; index++)
    {
        if (name[index] == 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t station_config_input_type_is_valid(uint8_t type)
{
    return (uint8_t)((type >= (uint8_t)STATION_INPUT_TYPE_SIGNAL_RELAY) &&
                     (type <= (uint8_t)STATION_INPUT_TYPE_SPARE));
}

static uint8_t station_config_input_encoding_is_valid(uint8_t encoding)
{
    return (uint8_t)((encoding >=
                      (uint8_t)STATION_INPUT_ENCODING_ONE_ASSERTED) &&
                     (encoding <=
                      (uint8_t)STATION_INPUT_ENCODING_ONE_OFF));
}

uint32_t station_config_calculate_crc32(
    const uint8_t *blob,
    uint32_t blob_size)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t byte_index;
    uint8_t bit_index;
    uint8_t value;
    uint32_t mask;

    if ((blob == NULL) || (blob_size < STATION_CONFIG_FIXED_HEADER_SIZE))
    {
        return 0U;
    }

    for (byte_index = 0U; byte_index < blob_size; byte_index++)
    {
        if ((byte_index >= STATION_CONFIG_CRC_OFFSET) &&
            (byte_index <
             (STATION_CONFIG_CRC_OFFSET + STATION_CONFIG_CRC_SIZE)))
        {
            value = 0U;
        }
        else
        {
            value = blob[byte_index];
        }

        crc ^= (uint32_t)value;
        for (bit_index = 0U; bit_index < 8U; bit_index++)
        {
            mask = (uint32_t)(0U - (crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

static station_config_result_t station_config_validate_sections(
    const uint8_t *blob,
    const station_config_header_t *header,
    uint16_t header_size,
    station_config_validation_report_t *report)
{
    uint16_t section_index;
    uint16_t compare_index;
    const uint8_t *section_data;
    const uint8_t *compare_data;
    station_config_section_t section;
    station_config_section_t compare;
    uint32_t section_end;
    uint32_t compare_end;

    for (section_index = 0U;
         section_index < header->section_count;
         section_index++)
    {
        section_data = &blob[STATION_CONFIG_SECTION_DIRECTORY_OFFSET +
                             ((uint32_t)section_index *
                              STATION_CONFIG_SECTION_DESCRIPTOR_SIZE)];
        station_config_decode_section(section_data, &section);

        if ((section.type == 0U) || (section.version == 0U))
        {
            station_config_report(report, STATION_CONFIG_ERR_HEADER,
                                  section.type, section_index);
            return STATION_CONFIG_ERR_HEADER;
        }

        if ((section.offset < (uint32_t)header_size) ||
            (section.offset > header->total_size) ||
            (section.length > (header->total_size - section.offset)))
        {
            station_config_report(report,
                                  STATION_CONFIG_ERR_SECTION_BOUNDS,
                                  section.type, section_index);
            return STATION_CONFIG_ERR_SECTION_BOUNDS;
        }

        if ((section.entry_count == 0U) || (section.entry_size == 0U) ||
            ((uint32_t)section.entry_count * (uint32_t)section.entry_size !=
             section.length))
        {
            station_config_report(report, STATION_CONFIG_ERR_ENTRY_SIZE,
                                  section.type, section_index);
            return STATION_CONFIG_ERR_ENTRY_SIZE;
        }

        section_end = section.offset + section.length;

        for (compare_index = 0U;
             compare_index < section_index;
             compare_index++)
        {
            compare_data = &blob[STATION_CONFIG_SECTION_DIRECTORY_OFFSET +
                                 ((uint32_t)compare_index *
                                  STATION_CONFIG_SECTION_DESCRIPTOR_SIZE)];
            station_config_decode_section(compare_data, &compare);

            if (compare.type == section.type)
            {
                station_config_report(report,
                                      STATION_CONFIG_ERR_DUPLICATE_SECTION,
                                      section.type, section_index);
                return STATION_CONFIG_ERR_DUPLICATE_SECTION;
            }

            compare_end = compare.offset + compare.length;
            if ((section.offset < compare_end) &&
                (compare.offset < section_end))
            {
                station_config_report(report,
                                      STATION_CONFIG_ERR_SECTION_OVERLAP,
                                      section.type, section_index);
                return STATION_CONFIG_ERR_SECTION_OVERLAP;
            }
        }
    }

    return STATION_CONFIG_OK;
}

static station_config_result_t station_config_validate_field_inputs(
    const uint8_t *blob,
    const station_config_header_t *header,
    station_config_validation_report_t *report)
{
    uint16_t section_index;
    uint16_t entry_index;
    uint16_t compare_index;
    const uint8_t *section_data;
    const uint8_t *entry_data;
    const uint8_t *compare_data;
    station_config_section_t section;
    station_config_field_input_t input;
    station_config_field_input_t compare;
    uint8_t section_found = 0U;

    for (section_index = 0U;
         section_index < header->section_count;
         section_index++)
    {
        section_data = &blob[STATION_CONFIG_SECTION_DIRECTORY_OFFSET +
                             ((uint32_t)section_index *
                              STATION_CONFIG_SECTION_DESCRIPTOR_SIZE)];
        station_config_decode_section(section_data, &section);

        if (section.type != (uint16_t)STATION_CONFIG_SECTION_FIELD_INPUTS)
        {
            continue;
        }

        section_found = 1U;

        if (section.version != STATION_CONFIG_FIELD_INPUT_SECTION_VERSION)
        {
            station_config_report(report, STATION_CONFIG_ERR_VERSION,
                                  section.type, 0U);
            return STATION_CONFIG_ERR_VERSION;
        }

        if (section.entry_size != STATION_CONFIG_FIELD_INPUT_ENTRY_SIZE)
        {
            station_config_report(report, STATION_CONFIG_ERR_ENTRY_SIZE,
                                  section.type, 0U);
            return STATION_CONFIG_ERR_ENTRY_SIZE;
        }

        if (section.entry_count > STATION_CONFIG_MAX_FIELD_INPUTS)
        {
            station_config_report(report, STATION_CONFIG_ERR_ENTRY_COUNT,
                                  section.type, 0U);
            return STATION_CONFIG_ERR_ENTRY_COUNT;
        }

        for (entry_index = 0U;
             entry_index < section.entry_count;
             entry_index++)
        {
            entry_data = &blob[section.offset +
                               ((uint32_t)entry_index * section.entry_size)];
            station_config_decode_field_input(entry_data, &input);

            if (input.logical_id == 0U)
            {
                station_config_report(report, STATION_CONFIG_ERR_INPUT_ID,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_INPUT_ID;
            }

            if ((input.card_index == 0U) ||
                (input.card_index > STATION_CONFIG_MAX_INPUT_CARDS) ||
                (input.channel_index >=
                 STATION_CONFIG_INPUT_CHANNELS_PER_CARD))
            {
                station_config_report(report,
                                      STATION_CONFIG_ERR_INPUT_LOCATION,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_INPUT_LOCATION;
            }

            if (station_config_input_type_is_valid((uint8_t)input.type) == 0U)
            {
                station_config_report(report, STATION_CONFIG_ERR_INPUT_TYPE,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_INPUT_TYPE;
            }

            if (station_config_input_encoding_is_valid(
                    (uint8_t)input.encoding) == 0U)
            {
                station_config_report(report,
                                      STATION_CONFIG_ERR_INPUT_ENCODING,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_INPUT_ENCODING;
            }

            if (station_config_name_is_valid(
                    &entry_data[8], STATION_CONFIG_FIELD_NAME_SIZE) == 0U)
            {
                station_config_report(report, STATION_CONFIG_ERR_INPUT_NAME,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_INPUT_NAME;
            }

            if (input.flags != 0U)
            {
                station_config_report(report, STATION_CONFIG_ERR_INPUT_FLAGS,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_INPUT_FLAGS;
            }

            for (compare_index = 0U;
                 compare_index < entry_index;
                 compare_index++)
            {
                compare_data = &blob[section.offset +
                                     ((uint32_t)compare_index *
                                      section.entry_size)];
                station_config_decode_field_input(compare_data, &compare);

                if (compare.logical_id == input.logical_id)
                {
                    station_config_report(
                        report, STATION_CONFIG_ERR_DUPLICATE_INPUT_ID,
                        section.type, entry_index);
                    return STATION_CONFIG_ERR_DUPLICATE_INPUT_ID;
                }

                if ((compare.card_index == input.card_index) &&
                    (compare.channel_index == input.channel_index))
                {
                    station_config_report(
                        report, STATION_CONFIG_ERR_DUPLICATE_INPUT_LOCATION,
                        section.type, entry_index);
                    return STATION_CONFIG_ERR_DUPLICATE_INPUT_LOCATION;
                }
            }
        }

        return STATION_CONFIG_OK;
    }

    if (section_found == 0U)
    {
        station_config_report(report, STATION_CONFIG_ERR_REQUIRED_SECTION,
                              (uint16_t)STATION_CONFIG_SECTION_FIELD_INPUTS,
                              0U);
        return STATION_CONFIG_ERR_REQUIRED_SECTION;
    }

    return STATION_CONFIG_OK;
}

static uint8_t station_config_find_section_in_blob(
    const uint8_t *blob,
    const station_config_header_t *header,
    uint16_t section_type,
    station_config_section_t *section)
{
    uint16_t section_index;
    const uint8_t *section_data;

    for (section_index = 0U;
         section_index < header->section_count;
         section_index++)
    {
        section_data = &blob[STATION_CONFIG_SECTION_DIRECTORY_OFFSET +
                             ((uint32_t)section_index *
                              STATION_CONFIG_SECTION_DESCRIPTOR_SIZE)];
        station_config_decode_section(section_data, section);
        if (section->type == section_type)
        {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t station_config_find_input_in_blob(
    const uint8_t *blob,
    const station_config_header_t *header,
    uint16_t logical_id,
    station_config_field_input_t *input)
{
    station_config_section_t section;
    uint16_t index;
    const uint8_t *entry_data;

    if (station_config_find_section_in_blob(
            blob, header,
            (uint16_t)STATION_CONFIG_SECTION_FIELD_INPUTS,
            &section) == 0U)
    {
        return 0U;
    }

    for (index = 0U; index < section.entry_count; index++)
    {
        entry_data = &blob[section.offset +
                           ((uint32_t)index * section.entry_size)];
        station_config_decode_field_input(entry_data, input);
        if (input->logical_id == logical_id)
        {
            return 1U;
        }
    }

    return 0U;
}

static station_config_result_t station_config_validate_signals(
    const uint8_t *blob,
    const station_config_header_t *header,
    station_config_validation_report_t *report)
{
    station_config_section_t section;
    station_config_signal_t signal;
    station_config_signal_t compare;
    station_config_field_input_t input;
    const uint8_t *entry_data;
    const uint8_t *compare_data;
    uint16_t input_ids[4];
    uint16_t entry_index;
    uint16_t compare_index;
    uint8_t input_index;

    if (station_config_find_section_in_blob(
            blob, header, (uint16_t)STATION_CONFIG_SECTION_SIGNALS,
            &section) == 0U)
    {
        return STATION_CONFIG_OK;
    }

    if (section.version != STATION_CONFIG_SIGNAL_SECTION_VERSION)
    {
        station_config_report(report, STATION_CONFIG_ERR_VERSION,
                              section.type, 0U);
        return STATION_CONFIG_ERR_VERSION;
    }
    if ((section.entry_size != STATION_CONFIG_SIGNAL_ENTRY_SIZE) ||
        (section.entry_count > STATION_CONFIG_MAX_SIGNALS))
    {
        station_config_report(report, STATION_CONFIG_ERR_ENTRY_SIZE,
                              section.type, 0U);
        return STATION_CONFIG_ERR_ENTRY_SIZE;
    }

    for (entry_index = 0U; entry_index < section.entry_count; entry_index++)
    {
        entry_data = &blob[section.offset +
                           ((uint32_t)entry_index * section.entry_size)];
        station_config_decode_signal(entry_data, &signal);

        if (signal.signal_id == 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_ID,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_ID;
        }
        if (signal.flags != 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_FLAGS,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_FLAGS;
        }
        if (station_config_name_is_valid(
                &entry_data[12], STATION_CONFIG_SIGNAL_NAME_SIZE) == 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_NAME,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_NAME;
        }

        input_ids[0] = signal.recr_input_id;
        input_ids[1] = signal.hecr_input_id;
        input_ids[2] = signal.decr_input_id;
        input_ids[3] = signal.hhecr_input_id;
        if ((input_ids[0] == 0U) && (input_ids[1] == 0U) &&
            (input_ids[2] == 0U) && (input_ids[3] == 0U))
        {
            station_config_report(report,
                                  STATION_CONFIG_ERR_OBJECT_REFERENCE,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_REFERENCE;
        }

        for (input_index = 0U; input_index < 4U; input_index++)
        {
            if (input_ids[input_index] == 0U)
            {
                continue;
            }
            if ((station_config_find_input_in_blob(
                     blob, header, input_ids[input_index], &input) == 0U) ||
                (input.type != STATION_INPUT_TYPE_SIGNAL_RELAY))
            {
                station_config_report(report,
                                      STATION_CONFIG_ERR_OBJECT_REFERENCE,
                                      section.type, entry_index);
                return STATION_CONFIG_ERR_OBJECT_REFERENCE;
            }
        }

        for (compare_index = 0U;
             compare_index < entry_index;
             compare_index++)
        {
            compare_data = &blob[section.offset +
                                  ((uint32_t)compare_index *
                                   section.entry_size)];
            station_config_decode_signal(compare_data, &compare);
            if (compare.signal_id == signal.signal_id)
            {
                station_config_report(
                    report, STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID,
                    section.type, entry_index);
                return STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID;
            }
        }
    }

    return STATION_CONFIG_OK;
}

static station_config_result_t station_config_validate_points(
    const uint8_t *blob,
    const station_config_header_t *header,
    station_config_validation_report_t *report)
{
    station_config_section_t section;
    station_config_point_t point;
    station_config_point_t compare;
    station_config_field_input_t input;
    const uint8_t *entry_data;
    const uint8_t *compare_data;
    uint16_t entry_index;
    uint16_t compare_index;

    if (station_config_find_section_in_blob(
            blob, header, (uint16_t)STATION_CONFIG_SECTION_POINTS,
            &section) == 0U)
    {
        return STATION_CONFIG_OK;
    }

    if (section.version != STATION_CONFIG_POINT_SECTION_VERSION)
    {
        station_config_report(report, STATION_CONFIG_ERR_VERSION,
                              section.type, 0U);
        return STATION_CONFIG_ERR_VERSION;
    }
    if ((section.entry_size != STATION_CONFIG_POINT_ENTRY_SIZE) ||
        (section.entry_count > STATION_CONFIG_MAX_POINTS))
    {
        station_config_report(report, STATION_CONFIG_ERR_ENTRY_SIZE,
                              section.type, 0U);
        return STATION_CONFIG_ERR_ENTRY_SIZE;
    }

    for (entry_index = 0U; entry_index < section.entry_count; entry_index++)
    {
        entry_data = &blob[section.offset +
                           ((uint32_t)entry_index * section.entry_size)];
        station_config_decode_point(entry_data, &point);

        if (point.point_id == 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_ID,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_ID;
        }
        if ((point.end_a_input_id == 0U) ||
            (point.end_b_input_id == 0U) ||
            (point.end_a_input_id == point.end_b_input_id))
        {
            station_config_report(report,
                                  STATION_CONFIG_ERR_OBJECT_REFERENCE,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_REFERENCE;
        }
        if (point.flags != 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_FLAGS,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_FLAGS;
        }
        if (station_config_name_is_valid(
                &entry_data[8], STATION_CONFIG_POINT_NAME_SIZE) == 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_NAME,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_NAME;
        }
        if ((station_config_find_input_in_blob(
                 blob, header, point.end_a_input_id, &input) == 0U) ||
            (input.type != STATION_INPUT_TYPE_POINT_POSITION) ||
            (station_config_find_input_in_blob(
                 blob, header, point.end_b_input_id, &input) == 0U) ||
            (input.type != STATION_INPUT_TYPE_POINT_POSITION))
        {
            station_config_report(report,
                                  STATION_CONFIG_ERR_OBJECT_REFERENCE,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_REFERENCE;
        }

        for (compare_index = 0U;
             compare_index < entry_index;
             compare_index++)
        {
            compare_data = &blob[section.offset +
                                  ((uint32_t)compare_index *
                                   section.entry_size)];
            station_config_decode_point(compare_data, &compare);
            if (compare.point_id == point.point_id)
            {
                station_config_report(
                    report, STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID,
                    section.type, entry_index);
                return STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID;
            }
        }
    }

    return STATION_CONFIG_OK;
}

static station_config_result_t station_config_validate_tracks(
    const uint8_t *blob,
    const station_config_header_t *header,
    station_config_validation_report_t *report)
{
    station_config_section_t section;
    station_config_track_t track;
    station_config_track_t compare;
    station_config_field_input_t input;
    const uint8_t *entry_data;
    const uint8_t *compare_data;
    uint16_t entry_index;
    uint16_t compare_index;

    if (station_config_find_section_in_blob(
            blob, header, (uint16_t)STATION_CONFIG_SECTION_TRACKS,
            &section) == 0U)
    {
        return STATION_CONFIG_OK;
    }

    if (section.version != STATION_CONFIG_TRACK_SECTION_VERSION)
    {
        station_config_report(report, STATION_CONFIG_ERR_VERSION,
                              section.type, 0U);
        return STATION_CONFIG_ERR_VERSION;
    }
    if ((section.entry_size != STATION_CONFIG_TRACK_ENTRY_SIZE) ||
        (section.entry_count > STATION_CONFIG_MAX_TRACKS))
    {
        station_config_report(report, STATION_CONFIG_ERR_ENTRY_SIZE,
                              section.type, 0U);
        return STATION_CONFIG_ERR_ENTRY_SIZE;
    }

    for (entry_index = 0U; entry_index < section.entry_count; entry_index++)
    {
        entry_data = &blob[section.offset +
                           ((uint32_t)entry_index * section.entry_size)];
        station_config_decode_track(entry_data, &track);

        if (track.track_id == 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_ID,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_ID;
        }
        if ((track.input_id == 0U) ||
            (station_config_find_input_in_blob(
                 blob, header, track.input_id, &input) == 0U))
        {
            station_config_report(report,
                                  STATION_CONFIG_ERR_OBJECT_REFERENCE,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_REFERENCE;
        }
        if (((track.kind == STATION_TRACK_KIND_TRACK_CIRCUIT) &&
             (input.type != STATION_INPUT_TYPE_TRACK_CIRCUIT)) ||
            ((track.kind == STATION_TRACK_KIND_AXLE_COUNTER) &&
             (input.type != STATION_INPUT_TYPE_AXLE_COUNTER)) ||
            ((track.kind != STATION_TRACK_KIND_TRACK_CIRCUIT) &&
             (track.kind != STATION_TRACK_KIND_AXLE_COUNTER)))
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_TYPE,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_TYPE;
        }
        if (track.flags != 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_FLAGS,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_FLAGS;
        }
        if (station_config_name_is_valid(
                &entry_data[8], STATION_CONFIG_TRACK_NAME_SIZE) == 0U)
        {
            station_config_report(report, STATION_CONFIG_ERR_OBJECT_NAME,
                                  section.type, entry_index);
            return STATION_CONFIG_ERR_OBJECT_NAME;
        }

        for (compare_index = 0U;
             compare_index < entry_index;
             compare_index++)
        {
            compare_data = &blob[section.offset +
                                  ((uint32_t)compare_index *
                                   section.entry_size)];
            station_config_decode_track(compare_data, &compare);
            if (compare.track_id == track.track_id)
            {
                station_config_report(
                    report, STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID,
                    section.type, entry_index);
                return STATION_CONFIG_ERR_DUPLICATE_OBJECT_ID;
            }
        }
    }

    return STATION_CONFIG_OK;
}

void station_config_init(void)
{
    station_config_context.blob = NULL;
    station_config_context.blob_size = 0U;
    station_config_context.state = STATION_CONFIG_STATE_EMPTY;
    station_config_context.last_error = STATION_CONFIG_OK;
}

station_config_result_t station_config_validate(
    const uint8_t *blob,
    uint32_t blob_size,
    station_config_validation_report_t *report)
{
    station_config_header_t header;
    station_config_result_t result;
    uint16_t encoded_header_size;
    uint32_t expected_header_size;
    uint32_t stored_crc;
    uint32_t calculated_crc;

    station_config_report(report, STATION_CONFIG_OK, 0U, 0U);

    if (blob == NULL)
    {
        station_config_report(report, STATION_CONFIG_ERR_NULL, 0U, 0U);
        return STATION_CONFIG_ERR_NULL;
    }

    if ((blob_size < STATION_CONFIG_FIXED_HEADER_SIZE) ||
        (blob_size > STATION_CONFIG_MAX_BLOB_SIZE))
    {
        station_config_report(report, STATION_CONFIG_ERR_SIZE, 0U, 0U);
        return STATION_CONFIG_ERR_SIZE;
    }

    if (station_config_read_u32(&blob[0]) != STATION_CONFIG_MAGIC)
    {
        station_config_report(report, STATION_CONFIG_ERR_MAGIC, 0U, 0U);
        return STATION_CONFIG_ERR_MAGIC;
    }

    if ((blob[4] != STATION_CONFIG_FORMAT_MAJOR) ||
        (blob[5] > STATION_CONFIG_FORMAT_MINOR))
    {
        station_config_report(report, STATION_CONFIG_ERR_VERSION, 0U, 0U);
        return STATION_CONFIG_ERR_VERSION;
    }

    station_config_decode_header(blob, &header);
    encoded_header_size = station_config_read_u16(&blob[6]);

    if ((header.section_count > STATION_CONFIG_MAX_SECTIONS) ||
        (header.section_count == 0U))
    {
        station_config_report(report, STATION_CONFIG_ERR_SECTION_COUNT,
                              0U, 0U);
        return STATION_CONFIG_ERR_SECTION_COUNT;
    }

    expected_header_size = STATION_CONFIG_FIXED_HEADER_SIZE +
        ((uint32_t)header.section_count *
         STATION_CONFIG_SECTION_DESCRIPTOR_SIZE);

    if ((encoded_header_size != expected_header_size) ||
        (header.total_size != blob_size) ||
        (header.total_size < expected_header_size))
    {
        station_config_report(report, STATION_CONFIG_ERR_HEADER, 0U, 0U);
        return STATION_CONFIG_ERR_HEADER;
    }

    if (header.station_id == 0U)
    {
        station_config_report(report, STATION_CONFIG_ERR_STATION_ID,
                              0U, 0U);
        return STATION_CONFIG_ERR_STATION_ID;
    }

    if (header.revision == 0U)
    {
        station_config_report(report, STATION_CONFIG_ERR_REVISION, 0U, 0U);
        return STATION_CONFIG_ERR_REVISION;
    }

    if ((header.flags != 0U) ||
        (station_config_read_u32(&blob[28]) != 0U))
    {
        station_config_report(report, STATION_CONFIG_ERR_HEADER, 0U, 0U);
        return STATION_CONFIG_ERR_HEADER;
    }

    stored_crc = station_config_read_u32(&blob[STATION_CONFIG_CRC_OFFSET]);
    calculated_crc = station_config_calculate_crc32(blob, blob_size);
    if (stored_crc != calculated_crc)
    {
        station_config_report(report, STATION_CONFIG_ERR_CRC, 0U, 0U);
        return STATION_CONFIG_ERR_CRC;
    }

    result = station_config_validate_sections(blob, &header,
                                              encoded_header_size, report);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    result = station_config_validate_field_inputs(blob, &header, report);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    result = station_config_validate_signals(blob, &header, report);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    result = station_config_validate_points(blob, &header, report);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    result = station_config_validate_tracks(blob, &header, report);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    station_config_report(report, STATION_CONFIG_OK, 0U, 0U);
    return STATION_CONFIG_OK;
}

station_config_result_t station_config_activate(
    const uint8_t *blob,
    uint32_t blob_size,
    station_config_validation_report_t *report)
{
    station_config_result_t result;

    result = station_config_validate(blob, blob_size, report);
    if (result != STATION_CONFIG_OK)
    {
        station_config_context.blob = NULL;
        station_config_context.blob_size = 0U;
        station_config_context.state = STATION_CONFIG_STATE_INVALID;
        station_config_context.last_error = result;
        return result;
    }

    station_config_context.blob = blob;
    station_config_context.blob_size = blob_size;
    station_config_context.state = STATION_CONFIG_STATE_VALID;
    station_config_context.last_error = STATION_CONFIG_OK;
    return STATION_CONFIG_OK;
}

void station_config_deactivate(void)
{
    station_config_context.blob = NULL;
    station_config_context.blob_size = 0U;
    station_config_context.state = STATION_CONFIG_STATE_EMPTY;
    station_config_context.last_error = STATION_CONFIG_OK;
}

station_config_state_t station_config_get_state(void)
{
    return station_config_context.state;
}

station_config_result_t station_config_get_last_error(void)
{
    return station_config_context.last_error;
}

station_config_result_t station_config_get_header(
    station_config_header_t *header)
{
    if (header == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    if (station_config_context.state != STATION_CONFIG_STATE_VALID)
    {
        return STATION_CONFIG_ERR_NOT_ACTIVE;
    }

    station_config_decode_header(station_config_context.blob, header);
    return STATION_CONFIG_OK;
}

station_config_result_t station_config_get_section(
    uint16_t section_type,
    station_config_section_t *section)
{
    station_config_header_t header;
    uint16_t section_index;
    const uint8_t *section_data;

    if (section == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    if (station_config_context.state != STATION_CONFIG_STATE_VALID)
    {
        return STATION_CONFIG_ERR_NOT_ACTIVE;
    }

    station_config_decode_header(station_config_context.blob, &header);

    for (section_index = 0U;
         section_index < header.section_count;
         section_index++)
    {
        section_data =
            &station_config_context.blob[
                STATION_CONFIG_SECTION_DIRECTORY_OFFSET +
                ((uint32_t)section_index *
                 STATION_CONFIG_SECTION_DESCRIPTOR_SIZE)];
        station_config_decode_section(section_data, section);
        if (section->type == section_type)
        {
            return STATION_CONFIG_OK;
        }
    }

    return STATION_CONFIG_ERR_NOT_FOUND;
}

station_config_result_t station_config_get_field_input_count(
    uint16_t *count)
{
    station_config_section_t section;
    station_config_result_t result;

    if (count == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    result = station_config_get_section(
        (uint16_t)STATION_CONFIG_SECTION_FIELD_INPUTS, &section);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    *count = section.entry_count;
    return STATION_CONFIG_OK;
}

station_config_result_t station_config_get_field_input_by_index(
    uint16_t index,
    station_config_field_input_t *input)
{
    station_config_section_t section;
    station_config_result_t result;
    const uint8_t *entry_data;

    if (input == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    result = station_config_get_section(
        (uint16_t)STATION_CONFIG_SECTION_FIELD_INPUTS, &section);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    if (index >= section.entry_count)
    {
        return STATION_CONFIG_ERR_NOT_FOUND;
    }

    entry_data = &station_config_context.blob[
        section.offset + ((uint32_t)index * section.entry_size)];
    station_config_decode_field_input(entry_data, input);
    return STATION_CONFIG_OK;
}

station_config_result_t station_config_find_field_input_by_id(
    uint16_t logical_id,
    station_config_field_input_t *input)
{
    uint16_t count;
    uint16_t index;
    station_config_result_t result;

    if (input == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    result = station_config_get_field_input_count(&count);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    for (index = 0U; index < count; index++)
    {
        result = station_config_get_field_input_by_index(index, input);
        if (result != STATION_CONFIG_OK)
        {
            return result;
        }

        if (input->logical_id == logical_id)
        {
            return STATION_CONFIG_OK;
        }
    }

    return STATION_CONFIG_ERR_NOT_FOUND;
}

station_config_result_t station_config_find_field_input_by_location(
    uint8_t card_index,
    uint8_t channel_index,
    station_config_field_input_t *input)
{
    uint16_t count;
    uint16_t index;
    station_config_result_t result;

    if (input == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    result = station_config_get_field_input_count(&count);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    for (index = 0U; index < count; index++)
    {
        result = station_config_get_field_input_by_index(index, input);
        if (result != STATION_CONFIG_OK)
        {
            return result;
        }

        if ((input->card_index == card_index) &&
            (input->channel_index == channel_index))
        {
            return STATION_CONFIG_OK;
        }
    }

    return STATION_CONFIG_ERR_NOT_FOUND;
}

static station_config_result_t station_config_get_object_count(
    uint16_t section_type,
    uint16_t *count)
{
    station_config_section_t section;
    station_config_result_t result;

    if (count == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    result = station_config_get_section(section_type, &section);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }

    *count = section.entry_count;
    return STATION_CONFIG_OK;
}

static station_config_result_t station_config_get_object_entry(
    uint16_t section_type,
    uint16_t index,
    const uint8_t **entry_data)
{
    station_config_section_t section;
    station_config_result_t result;

    if (entry_data == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }

    result = station_config_get_section(section_type, &section);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }
    if (index >= section.entry_count)
    {
        return STATION_CONFIG_ERR_NOT_FOUND;
    }

    *entry_data = &station_config_context.blob[
        section.offset + ((uint32_t)index * section.entry_size)];
    return STATION_CONFIG_OK;
}

station_config_result_t station_config_get_signal_count(uint16_t *count)
{
    return station_config_get_object_count(
        (uint16_t)STATION_CONFIG_SECTION_SIGNALS, count);
}

station_config_result_t station_config_get_signal_by_index(
    uint16_t index,
    station_config_signal_t *signal)
{
    const uint8_t *entry_data;
    station_config_result_t result;

    if (signal == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }
    result = station_config_get_object_entry(
        (uint16_t)STATION_CONFIG_SECTION_SIGNALS, index, &entry_data);
    if (result == STATION_CONFIG_OK)
    {
        station_config_decode_signal(entry_data, signal);
    }
    return result;
}

station_config_result_t station_config_find_signal_by_id(
    uint16_t signal_id,
    station_config_signal_t *signal)
{
    uint16_t count;
    uint16_t index;
    station_config_result_t result;

    if (signal == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }
    result = station_config_get_signal_count(&count);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }
    for (index = 0U; index < count; index++)
    {
        result = station_config_get_signal_by_index(index, signal);
        if ((result != STATION_CONFIG_OK) ||
            (signal->signal_id == signal_id))
        {
            return result;
        }
    }
    return STATION_CONFIG_ERR_NOT_FOUND;
}

station_config_result_t station_config_get_point_count(uint16_t *count)
{
    return station_config_get_object_count(
        (uint16_t)STATION_CONFIG_SECTION_POINTS, count);
}

station_config_result_t station_config_get_point_by_index(
    uint16_t index,
    station_config_point_t *point)
{
    const uint8_t *entry_data;
    station_config_result_t result;

    if (point == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }
    result = station_config_get_object_entry(
        (uint16_t)STATION_CONFIG_SECTION_POINTS, index, &entry_data);
    if (result == STATION_CONFIG_OK)
    {
        station_config_decode_point(entry_data, point);
    }
    return result;
}

station_config_result_t station_config_find_point_by_id(
    uint16_t point_id,
    station_config_point_t *point)
{
    uint16_t count;
    uint16_t index;
    station_config_result_t result;

    if (point == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }
    result = station_config_get_point_count(&count);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }
    for (index = 0U; index < count; index++)
    {
        result = station_config_get_point_by_index(index, point);
        if ((result != STATION_CONFIG_OK) || (point->point_id == point_id))
        {
            return result;
        }
    }
    return STATION_CONFIG_ERR_NOT_FOUND;
}

station_config_result_t station_config_get_track_count(uint16_t *count)
{
    return station_config_get_object_count(
        (uint16_t)STATION_CONFIG_SECTION_TRACKS, count);
}

station_config_result_t station_config_get_track_by_index(
    uint16_t index,
    station_config_track_t *track)
{
    const uint8_t *entry_data;
    station_config_result_t result;

    if (track == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }
    result = station_config_get_object_entry(
        (uint16_t)STATION_CONFIG_SECTION_TRACKS, index, &entry_data);
    if (result == STATION_CONFIG_OK)
    {
        station_config_decode_track(entry_data, track);
    }
    return result;
}

station_config_result_t station_config_find_track_by_id(
    uint16_t track_id,
    station_config_track_t *track)
{
    uint16_t count;
    uint16_t index;
    station_config_result_t result;

    if (track == NULL)
    {
        return STATION_CONFIG_ERR_NULL;
    }
    result = station_config_get_track_count(&count);
    if (result != STATION_CONFIG_OK)
    {
        return result;
    }
    for (index = 0U; index < count; index++)
    {
        result = station_config_get_track_by_index(index, track);
        if ((result != STATION_CONFIG_OK) || (track->track_id == track_id))
        {
            return result;
        }
    }
    return STATION_CONFIG_ERR_NOT_FOUND;
}
