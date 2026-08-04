#ifndef RFID_RX_H
#define RFID_RX_H

#include "stdbool.h"
#include <stdint.h>
#include <stdlib.h>


/* ---------------- Tag Types ---------------- */
typedef enum
{
    RFID_TAG_NORMAL      = 0x09,
    RFID_TAG_LC          = 0x02,
    RFID_TAG_ADJ_LINE    = 0x03,
    RFID_TAG_JUNCTION    = 0x04
} rfid_tag_type_t;

typedef enum
{
    RFID_SUB_NONE        = 0x00,
    RFID_SUB_SIGNAL_FOOT = 0x01,
    RFID_SUB_ENTRY       = 0x02,
    RFID_SUB_EXIT        = 0x03
} rfid_subtype_t;

/* ---------------- Normal Tag ---------------- */
typedef struct
{
    uint8_t  version;

    uint16_t tag_uid;          // Unique ID of RFID tag set

    uint32_t abs_loc_m;        // Absolute location in meters (23 bits)

    uint16_t  tin_nominal;
    uint16_t  tin_reverse;

    uint16_t station_nominal;
    uint16_t station_reverse;

    uint8_t  section_nominal;
    uint8_t  section_reverse;

    uint8_t  tag_placement;

    uint8_t  is_duplicate;
    uint8_t  comm_nominal_req;
    uint8_t  comm_reverse_req;
} rfid_normal_tag_t;

/* ---------------- LC Tag ---------------- */
typedef struct
{
    uint8_t  version;
    uint16_t tag_set_id;

    uint32_t abs_loc_m;

    uint16_t  tin_nominal;
    uint16_t  tin_reverse;

    uint8_t  section_nominal;
    uint8_t  section_reverse;

    uint8_t  tag_placement;

    uint8_t  lc_territory;
    uint8_t  applicable_dir;

    uint16_t gate_id;
    uint8_t  gate_alpha;
    uint8_t  gate_type;

    uint16_t distance_to_gate;

    uint8_t  auto_whistle;
    uint8_t  whistle_type;

    uint8_t  is_duplicate;
    uint8_t  comm_nominal;
    uint8_t  comm_reverse;

} rfid_lc_tag_t;

/* ---------------- Adjacent Line Tag ---------------- */
typedef struct
{
    uint8_t  version;
    uint16_t tag_set_id;

    uint32_t abs_loc_m;

    uint16_t  tin_nominal;
    uint16_t  tin_reverse;

    uint8_t  adj_tin[5];   /* Adjacent Line 1..5 */

    uint8_t  is_duplicate;

} rfid_adj_line_tag_t;

/* ----------------  Adjustment/Junction Tag ---------------- */
typedef struct
{
    uint8_t  version;
    uint16_t tag_set_id;

    uint32_t abs_loc_1;
    uint16_t  tin_1;
    uint16_t  tin_2;
    uint32_t abs_loc_2;

    uint8_t  dir_corr_1;
    uint8_t  dir_corr_2;
    uint8_t  loc_corr_type;

    uint8_t  section_type_1;
    uint8_t  section_type_2;

    uint8_t  tag_duplicate;
    uint8_t  comm_nominal;
    uint8_t  comm_reverse;
} rfid_junction_tag_t;

/* Called from CAN RX ISR context */
void rfid_rx_handle(uint32_t can_id, uint8_t *data);

/* Called from 1s scheduler (optional for now) */
void rfid_rx_poll(void);

//void rfid_process_queues_1s(void);

#define RFID_MAX_FRAGMENTS     2
#define RFID_MAX_DATA_BYTES   16   /* enough for 98 bits */

//!================ Database Implementation Start =====================//
#define RFID_DB_SIZE 11
#define LOCATION_ACCURACY_WINDOW     100

typedef struct
{
    uint8_t tag_type;

    union
    {
        rfid_normal_tag_t     normal;
        rfid_lc_tag_t         lc;
        rfid_adj_line_tag_t   adj;
        rfid_junction_tag_t   junction;
    } data;

    uint32_t odo_distance_rfid_ref;
    uint8_t valid;
    uint8_t location_check;

} rfid_db_entry_t;

extern uint8_t rfid_db_head;
extern uint8_t rfid_db_count;
extern uint8_t trainDir;
extern uint8_t rfid_Count;
extern uint8_t current_Tag_Type;
extern uint8_t rfid_Miss_Count;
extern uint8_t match_index;
extern uint8_t rfidDataMatchFlag;
extern uint8_t rfidMissCount1;
extern uint8_t rfidMissCount2;
extern uint8_t rfid1_fault;   // Reader at CAN ID 0x120
extern uint8_t rfid2_fault;   // Reader at CAN ID 0x121

extern uint32_t odo_distance_radio_rfid_ref;
extern uint32_t prev_abs_loc_m;
//extern rfid_junction_tag_t juntag;
extern rfid_db_entry_t rfid_db[RFID_DB_SIZE];
extern bool comm_mandatory_area;
#endif
