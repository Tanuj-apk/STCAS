#include "station_config_test.h"

#include <stddef.h>
#include <string.h>

#define NAWANDGI_TEST_STATION_ID       0xFFFFU
#define NAWANDGI_TEST_REVISION         1UL
#define NAWANDGI_TEST_SECTION_COUNT    4U
#define NAWANDGI_TEST_INPUT_COUNT      93U
#define NAWANDGI_TEST_SIGNAL_COUNT     16U
#define NAWANDGI_TEST_POINT_COUNT      6U
#define NAWANDGI_TEST_TRACK_COUNT      28U
#define NAWANDGI_TEST_HEADER_SIZE      \
    (STATION_CONFIG_FIXED_HEADER_SIZE + \
     (NAWANDGI_TEST_SECTION_COUNT * \
      STATION_CONFIG_SECTION_DESCRIPTOR_SIZE))
#define NAWANDGI_TEST_INPUT_OFFSET     NAWANDGI_TEST_HEADER_SIZE
#define NAWANDGI_TEST_SIGNAL_OFFSET    \
    (NAWANDGI_TEST_INPUT_OFFSET + \
     (NAWANDGI_TEST_INPUT_COUNT * STATION_CONFIG_FIELD_INPUT_ENTRY_SIZE))
#define NAWANDGI_TEST_POINT_OFFSET     \
    (NAWANDGI_TEST_SIGNAL_OFFSET + \
     (NAWANDGI_TEST_SIGNAL_COUNT * STATION_CONFIG_SIGNAL_ENTRY_SIZE))
#define NAWANDGI_TEST_TRACK_OFFSET     \
    (NAWANDGI_TEST_POINT_OFFSET + \
     (NAWANDGI_TEST_POINT_COUNT * STATION_CONFIG_POINT_ENTRY_SIZE))
#define NAWANDGI_TEST_BLOB_SIZE        \
    (NAWANDGI_TEST_TRACK_OFFSET + \
     (NAWANDGI_TEST_TRACK_COUNT * STATION_CONFIG_TRACK_ENTRY_SIZE))

typedef struct
{
    uint16_t logical_id;
    uint8_t card;
    uint8_t channel;
    station_input_type_t type;
    station_input_encoding_t encoding;
    const char *name;
} nawandgi_test_input_t;

typedef struct
{
    uint16_t signal_id;
    uint16_t recr_input_id;
    uint16_t hecr_input_id;
    uint16_t decr_input_id;
    uint16_t hhecr_input_id;
    const char *name;
} nawandgi_test_signal_t;

typedef struct
{
    uint16_t point_id;
    uint16_t end_a_input_id;
    uint16_t end_b_input_id;
    const char *name;
} nawandgi_test_point_t;

typedef struct
{
    uint16_t track_id;
    uint16_t input_id;
    station_track_kind_t kind;
    const char *name;
} nawandgi_test_track_t;

/*
 * TEST ONLY: transcribed from
 * station_configs/examples/nawandgi/field_inputs.csv.
 * This is not approved commissioning data.
 */
static const nawandgi_test_input_t nawandgi_test_inputs[] =
{
    {1U, 1U, 0U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "D01_HECR"},
    {2U, 1U, 1U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "D01_DECR"},
    {3U, 1U, 2U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "D01_HHECR"},
    {4U, 1U, 3U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID01_HECR"},
    {5U, 1U, 4U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID01_DECR"},
    {6U, 1U, 5U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID01_HHECR"},
    {7U, 1U, 6U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S01_RECR"},
    {8U, 1U, 7U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S01_HECR"},
    {9U, 1U, 8U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S01_DECR"},
    {10U, 1U, 9U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID02_HECR"},
    {11U, 1U, 10U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID02_DECR"},
    {12U, 1U, 11U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID02_HHECR"},
    {13U, 1U, 12U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S02_DECR"},
    {14U, 1U, 13U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S02_HECR"},
    {15U, 1U, 14U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S02_RECR"},
    {16U, 1U, 15U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S04_DECR"},
    {17U, 1U, 16U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S04_HECR"},
    {18U, 1U, 17U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S04_RECR"},
    {19U, 1U, 18U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S06_HECR"},
    {20U, 1U, 19U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S06_RECR"},
    {21U, 1U, 20U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "UP_IBS_RECR"},
    {22U, 1U, 21U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "UP_IBS_DECR"},
    {23U, 1U, 22U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S05_RECR"},
    {24U, 1U, 23U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S05_HECR"},
    {25U, 1U, 24U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID11_HECR"},
    {26U, 1U, 25U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "ID11_DECR"},
    {27U, 1U, 26U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S11_RECR"},
    {28U, 1U, 27U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S11_DECR"},
    {29U, 1U, 28U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S08_DECR"},
    {30U, 1U, 29U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S08_RECR"},
    {31U, 1U, 30U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S07_RECR"},
    {32U, 1U, 31U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S07_HECR"},
    {33U, 2U, 0U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S09_D11_RECR"},
    {34U, 2U, 1U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S09_D11_HECR"},
    {35U, 2U, 2U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S09_D11_DECR"},
    {36U, 2U, 3U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S09_D11_HHECR"},
    {37U, 2U, 4U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S03_RECR"},
    {38U, 2U, 5U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S03_HECR"},
    {39U, 2U, 6U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S03_DECR"},
    {40U, 2U, 7U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "S03_HHECR"},
    {41U, 2U, 8U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "D02_HECR"},
    {42U, 2U, 9U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "D02_DECR"},
    {43U, 2U, 10U, STATION_INPUT_TYPE_SIGNAL_RELAY,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "D02_HHECR"},
    {44U, 2U, 11U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "101A"},
    {45U, 2U, 12U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "104A"},
    {46U, 2U, 13U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "104B"},
    {47U, 2U, 14U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "102B"},
    {48U, 2U, 15U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "102A"},
    {49U, 2U, 16U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "106B"},
    {50U, 2U, 17U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "103B"},
    {51U, 2U, 18U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "103A"},
    {52U, 2U, 19U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "106A"},
    {53U, 2U, 20U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "101B"},
    {54U, 2U, 21U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "105B"},
    {55U, 2U, 22U, STATION_INPUT_TYPE_POINT_POSITION,
     STATION_INPUT_ENCODING_ONE_NORMAL, "105A"},
    {56U, 2U, 23U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "C01T"},
    {57U, 2U, 24U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "04AT"},
    {58U, 2U, 25U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "04BT"},
    {59U, 2U, 26U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "104AT"},
    {60U, 2U, 27U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "H01T"},
    {61U, 2U, 28U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "101BT"},
    {62U, 2U, 29U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "09T"},
    {63U, 2U, 30U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "03AT"},
    {64U, 2U, 31U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "03BT"},
    {65U, 3U, 0U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "104BT"},
    {66U, 3U, 1U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "102BT"},
    {67U, 3U, 2U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "03_07T"},
    {68U, 3U, 3U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "08T"},
    {69U, 3U, 4U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "H02T"},
    {70U, 3U, 5U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "04_06T"},
    {71U, 3U, 6U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "102AT"},
    {72U, 3U, 7U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "106BT"},
    {73U, 3U, 8U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "02BT"},
    {74U, 3U, 9U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "02AT"},
    {75U, 3U, 10U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "103BT"},
    {76U, 3U, 11U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "103AT"},
    {77U, 3U, 12U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "01BT"},
    {78U, 3U, 13U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "01AT"},
    {79U, 3U, 14U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "105T"},
    {80U, 3U, 15U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "106AT"},
    {81U, 3U, 16U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "101AT"},
    {82U, 3U, 17U, STATION_INPUT_TYPE_TRACK_CIRCUIT,
     STATION_INPUT_ENCODING_ONE_CLEAR, "C02T"},
    {83U, 3U, 18U, STATION_INPUT_TYPE_AXLE_COUNTER,
     STATION_INPUT_ENCODING_ONE_CLEAR, "11AC"},
    {84U, 3U, 19U, STATION_INPUT_TYPE_SHUNT_SIGNAL,
     STATION_INPUT_ENCODING_ONE_OFF, "SH43"},
    {85U, 3U, 20U, STATION_INPUT_TYPE_SHUNT_SIGNAL,
     STATION_INPUT_ENCODING_ONE_OFF, "SH41"},
    {86U, 3U, 21U, STATION_INPUT_TYPE_SHUNT_SIGNAL,
     STATION_INPUT_ENCODING_ONE_OFF, "SH42"},
    {87U, 3U, 22U, STATION_INPUT_TYPE_CALLING_ON_SIGNAL,
     STATION_INPUT_ENCODING_ONE_OFF, "C01"},
    {88U, 3U, 23U, STATION_INPUT_TYPE_CALLING_ON_SIGNAL,
     STATION_INPUT_ENCODING_ONE_OFF, "C02"},
    {89U, 3U, 24U, STATION_INPUT_TYPE_ROUTE_INDICATOR,
     STATION_INPUT_ENCODING_ONE_OFF, "RS1"},
    {90U, 3U, 25U, STATION_INPUT_TYPE_ROUTE_INDICATOR,
     STATION_INPUT_ENCODING_ONE_OFF, "RS2"},
    {91U, 3U, 26U, STATION_INPUT_TYPE_ROUTE_INDICATOR,
     STATION_INPUT_ENCODING_ONE_OFF, "RS3"},
    {92U, 3U, 27U, STATION_INPUT_TYPE_LC_GATE,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "LC8"},
    {93U, 3U, 28U, STATION_INPUT_TYPE_LC_GATE,
     STATION_INPUT_ENCODING_ONE_ASSERTED, "LC9"}
};

/* Relay groups only; aspect deduction is a later application step. */
static const nawandgi_test_signal_t nawandgi_test_signals[] =
{
    {1U, 0U, 1U, 2U, 3U, "D01"},
    {2U, 0U, 4U, 5U, 6U, "ID01"},
    {3U, 7U, 8U, 9U, 0U, "S01"},
    {4U, 0U, 10U, 11U, 12U, "ID02"},
    {5U, 15U, 14U, 13U, 0U, "S02"},
    {6U, 18U, 17U, 16U, 0U, "S04"},
    {7U, 20U, 19U, 0U, 0U, "S06"},
    {8U, 21U, 0U, 22U, 0U, "UP_IBS"},
    {9U, 23U, 24U, 0U, 0U, "S05"},
    {10U, 0U, 25U, 26U, 0U, "ID11"},
    {11U, 27U, 0U, 28U, 0U, "S11"},
    {12U, 30U, 0U, 29U, 0U, "S08"},
    {13U, 31U, 32U, 0U, 0U, "S07"},
    {14U, 33U, 34U, 35U, 36U, "S09_D11"},
    {15U, 37U, 38U, 39U, 40U, "S03"},
    {16U, 0U, 41U, 42U, 43U, "D02"}
};

static const nawandgi_test_point_t nawandgi_test_points[] =
{
    {101U, 44U, 53U, "101"},
    {102U, 48U, 47U, "102"},
    {103U, 51U, 50U, "103"},
    {104U, 45U, 46U, "104"},
    {105U, 55U, 54U, "105"},
    {106U, 52U, 49U, "106"}
};

static const nawandgi_test_track_t nawandgi_test_tracks[] =
{
    {1U, 56U, STATION_TRACK_KIND_TRACK_CIRCUIT, "C01T"},
    {2U, 57U, STATION_TRACK_KIND_TRACK_CIRCUIT, "04AT"},
    {3U, 58U, STATION_TRACK_KIND_TRACK_CIRCUIT, "04BT"},
    {4U, 59U, STATION_TRACK_KIND_TRACK_CIRCUIT, "104AT"},
    {5U, 60U, STATION_TRACK_KIND_TRACK_CIRCUIT, "H01T"},
    {6U, 61U, STATION_TRACK_KIND_TRACK_CIRCUIT, "101BT"},
    {7U, 62U, STATION_TRACK_KIND_TRACK_CIRCUIT, "09T"},
    {8U, 63U, STATION_TRACK_KIND_TRACK_CIRCUIT, "03AT"},
    {9U, 64U, STATION_TRACK_KIND_TRACK_CIRCUIT, "03BT"},
    {10U, 65U, STATION_TRACK_KIND_TRACK_CIRCUIT, "104BT"},
    {11U, 66U, STATION_TRACK_KIND_TRACK_CIRCUIT, "102BT"},
    {12U, 67U, STATION_TRACK_KIND_TRACK_CIRCUIT, "03_07T"},
    {13U, 68U, STATION_TRACK_KIND_TRACK_CIRCUIT, "08T"},
    {14U, 69U, STATION_TRACK_KIND_TRACK_CIRCUIT, "H02T"},
    {15U, 70U, STATION_TRACK_KIND_TRACK_CIRCUIT, "04_06T"},
    {16U, 71U, STATION_TRACK_KIND_TRACK_CIRCUIT, "102AT"},
    {17U, 72U, STATION_TRACK_KIND_TRACK_CIRCUIT, "106BT"},
    {18U, 73U, STATION_TRACK_KIND_TRACK_CIRCUIT, "02BT"},
    {19U, 74U, STATION_TRACK_KIND_TRACK_CIRCUIT, "02AT"},
    {20U, 75U, STATION_TRACK_KIND_TRACK_CIRCUIT, "103BT"},
    {21U, 76U, STATION_TRACK_KIND_TRACK_CIRCUIT, "103AT"},
    {22U, 77U, STATION_TRACK_KIND_TRACK_CIRCUIT, "01BT"},
    {23U, 78U, STATION_TRACK_KIND_TRACK_CIRCUIT, "01AT"},
    {24U, 79U, STATION_TRACK_KIND_TRACK_CIRCUIT, "105T"},
    {25U, 80U, STATION_TRACK_KIND_TRACK_CIRCUIT, "106AT"},
    {26U, 81U, STATION_TRACK_KIND_TRACK_CIRCUIT, "101AT"},
    {27U, 82U, STATION_TRACK_KIND_TRACK_CIRCUIT, "C02T"},
    {28U, 83U, STATION_TRACK_KIND_AXLE_COUNTER, "11AC"}
};

static uint8_t nawandgi_test_blob[NAWANDGI_TEST_BLOB_SIZE];

typedef char nawandgi_test_input_count_must_match[
    ((sizeof(nawandgi_test_inputs) / sizeof(nawandgi_test_inputs[0])) ==
     NAWANDGI_TEST_INPUT_COUNT) ? 1 : -1];
typedef char nawandgi_test_signal_count_must_match[
    ((sizeof(nawandgi_test_signals) / sizeof(nawandgi_test_signals[0])) ==
     NAWANDGI_TEST_SIGNAL_COUNT) ? 1 : -1];
typedef char nawandgi_test_point_count_must_match[
    ((sizeof(nawandgi_test_points) / sizeof(nawandgi_test_points[0])) ==
     NAWANDGI_TEST_POINT_COUNT) ? 1 : -1];
typedef char nawandgi_test_track_count_must_match[
    ((sizeof(nawandgi_test_tracks) / sizeof(nawandgi_test_tracks[0])) ==
     NAWANDGI_TEST_TRACK_COUNT) ? 1 : -1];

static void test_write_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value >> 8);
    destination[1] = (uint8_t)value;
}

static void test_write_u32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value >> 24);
    destination[1] = (uint8_t)(value >> 16);
    destination[2] = (uint8_t)(value >> 8);
    destination[3] = (uint8_t)value;
}

static void test_write_name(
    uint8_t *destination,
    const char *name,
    uint8_t maximum_size)
{
    uint8_t name_index;

    for (name_index = 0U;
         (name_index < maximum_size) && (name[name_index] != '\0');
         name_index++)
    {
        destination[name_index] = (uint8_t)name[name_index];
    }
}

static void nawandgi_test_build_blob(void)
{
    uint16_t input_index;
    uint16_t signal_index;
    uint16_t point_index;
    uint16_t track_index;
    uint8_t *entry;
    uint32_t crc;
    const nawandgi_test_input_t *input;
    const nawandgi_test_signal_t *signal;
    const nawandgi_test_point_t *point;
    const nawandgi_test_track_t *track;

    memset(nawandgi_test_blob, 0, sizeof(nawandgi_test_blob));

    test_write_u32(&nawandgi_test_blob[0], STATION_CONFIG_MAGIC);
    nawandgi_test_blob[4] = STATION_CONFIG_FORMAT_MAJOR;
    nawandgi_test_blob[5] = STATION_CONFIG_FORMAT_MINOR;
    test_write_u16(&nawandgi_test_blob[6], NAWANDGI_TEST_HEADER_SIZE);
    test_write_u32(&nawandgi_test_blob[8], NAWANDGI_TEST_BLOB_SIZE);
    test_write_u32(&nawandgi_test_blob[12], NAWANDGI_TEST_REVISION);
    test_write_u16(&nawandgi_test_blob[16], NAWANDGI_TEST_STATION_ID);
    test_write_u16(&nawandgi_test_blob[18], NAWANDGI_TEST_SECTION_COUNT);

    test_write_u16(&nawandgi_test_blob[32],
                   (uint16_t)STATION_CONFIG_SECTION_FIELD_INPUTS);
    test_write_u16(&nawandgi_test_blob[34],
                   STATION_CONFIG_FIELD_INPUT_SECTION_VERSION);
    test_write_u32(&nawandgi_test_blob[36], NAWANDGI_TEST_INPUT_OFFSET);
    test_write_u32(&nawandgi_test_blob[40],
                   NAWANDGI_TEST_INPUT_COUNT *
                   STATION_CONFIG_FIELD_INPUT_ENTRY_SIZE);
    test_write_u16(&nawandgi_test_blob[44], NAWANDGI_TEST_INPUT_COUNT);
    test_write_u16(&nawandgi_test_blob[46],
                   STATION_CONFIG_FIELD_INPUT_ENTRY_SIZE);

    test_write_u16(&nawandgi_test_blob[48],
                   (uint16_t)STATION_CONFIG_SECTION_SIGNALS);
    test_write_u16(&nawandgi_test_blob[50],
                   STATION_CONFIG_SIGNAL_SECTION_VERSION);
    test_write_u32(&nawandgi_test_blob[52], NAWANDGI_TEST_SIGNAL_OFFSET);
    test_write_u32(&nawandgi_test_blob[56],
                   NAWANDGI_TEST_SIGNAL_COUNT *
                   STATION_CONFIG_SIGNAL_ENTRY_SIZE);
    test_write_u16(&nawandgi_test_blob[60], NAWANDGI_TEST_SIGNAL_COUNT);
    test_write_u16(&nawandgi_test_blob[62],
                   STATION_CONFIG_SIGNAL_ENTRY_SIZE);

    test_write_u16(&nawandgi_test_blob[64],
                   (uint16_t)STATION_CONFIG_SECTION_POINTS);
    test_write_u16(&nawandgi_test_blob[66],
                   STATION_CONFIG_POINT_SECTION_VERSION);
    test_write_u32(&nawandgi_test_blob[68], NAWANDGI_TEST_POINT_OFFSET);
    test_write_u32(&nawandgi_test_blob[72],
                   NAWANDGI_TEST_POINT_COUNT *
                   STATION_CONFIG_POINT_ENTRY_SIZE);
    test_write_u16(&nawandgi_test_blob[76], NAWANDGI_TEST_POINT_COUNT);
    test_write_u16(&nawandgi_test_blob[78],
                   STATION_CONFIG_POINT_ENTRY_SIZE);

    test_write_u16(&nawandgi_test_blob[80],
                   (uint16_t)STATION_CONFIG_SECTION_TRACKS);
    test_write_u16(&nawandgi_test_blob[82],
                   STATION_CONFIG_TRACK_SECTION_VERSION);
    test_write_u32(&nawandgi_test_blob[84], NAWANDGI_TEST_TRACK_OFFSET);
    test_write_u32(&nawandgi_test_blob[88],
                   NAWANDGI_TEST_TRACK_COUNT *
                   STATION_CONFIG_TRACK_ENTRY_SIZE);
    test_write_u16(&nawandgi_test_blob[92], NAWANDGI_TEST_TRACK_COUNT);
    test_write_u16(&nawandgi_test_blob[94],
                   STATION_CONFIG_TRACK_ENTRY_SIZE);

    for (input_index = 0U;
         input_index < NAWANDGI_TEST_INPUT_COUNT;
         input_index++)
    {
        input = &nawandgi_test_inputs[input_index];
        entry = &nawandgi_test_blob[
            NAWANDGI_TEST_INPUT_OFFSET +
            ((uint32_t)input_index *
             STATION_CONFIG_FIELD_INPUT_ENTRY_SIZE)];

        test_write_u16(&entry[0], input->logical_id);
        entry[2] = input->card;
        entry[3] = input->channel;
        entry[4] = (uint8_t)input->type;
        entry[5] = (uint8_t)input->encoding;

        test_write_name(&entry[8], input->name,
                        STATION_CONFIG_FIELD_NAME_SIZE);
    }

    for (signal_index = 0U;
         signal_index < NAWANDGI_TEST_SIGNAL_COUNT;
         signal_index++)
    {
        signal = &nawandgi_test_signals[signal_index];
        entry = &nawandgi_test_blob[
            NAWANDGI_TEST_SIGNAL_OFFSET +
            ((uint32_t)signal_index * STATION_CONFIG_SIGNAL_ENTRY_SIZE)];
        test_write_u16(&entry[0], signal->signal_id);
        test_write_u16(&entry[2], signal->recr_input_id);
        test_write_u16(&entry[4], signal->hecr_input_id);
        test_write_u16(&entry[6], signal->decr_input_id);
        test_write_u16(&entry[8], signal->hhecr_input_id);
        test_write_name(&entry[12], signal->name,
                        STATION_CONFIG_SIGNAL_NAME_SIZE);
    }

    for (point_index = 0U;
         point_index < NAWANDGI_TEST_POINT_COUNT;
         point_index++)
    {
        point = &nawandgi_test_points[point_index];
        entry = &nawandgi_test_blob[
            NAWANDGI_TEST_POINT_OFFSET +
            ((uint32_t)point_index * STATION_CONFIG_POINT_ENTRY_SIZE)];
        test_write_u16(&entry[0], point->point_id);
        test_write_u16(&entry[2], point->end_a_input_id);
        test_write_u16(&entry[4], point->end_b_input_id);
        test_write_name(&entry[8], point->name,
                        STATION_CONFIG_POINT_NAME_SIZE);
    }

    for (track_index = 0U;
         track_index < NAWANDGI_TEST_TRACK_COUNT;
         track_index++)
    {
        track = &nawandgi_test_tracks[track_index];
        entry = &nawandgi_test_blob[
            NAWANDGI_TEST_TRACK_OFFSET +
            ((uint32_t)track_index * STATION_CONFIG_TRACK_ENTRY_SIZE)];
        test_write_u16(&entry[0], track->track_id);
        test_write_u16(&entry[2], track->input_id);
        entry[4] = (uint8_t)track->kind;
        test_write_name(&entry[8], track->name,
                        STATION_CONFIG_TRACK_NAME_SIZE);
    }

    crc = station_config_calculate_crc32(nawandgi_test_blob,
                                         sizeof(nawandgi_test_blob));
    test_write_u32(&nawandgi_test_blob[20], crc);
}

station_config_result_t station_config_test_activate_nawandgi(
    station_config_validation_report_t *report)
{
    nawandgi_test_build_blob();
    return station_config_activate(nawandgi_test_blob,
                                   sizeof(nawandgi_test_blob),
                                   report);
}
