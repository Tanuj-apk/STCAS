

#include "rfid_rx.h"
#include "StateMachine.h"
//#include "dmi_can.h"
#include "gps.h"
//#include "pulse_generator.h"
#include "radio.h"
#include "stdbool.h"
#include <stdlib.h>
#include <string.h>

uint8_t tag_type;
// uint8_t seq_info;
// uint8_t tag_instance;
uint8_t seq_total;
uint8_t seq_index;

//Added by UDAY
uint8_t rfidDataMatchFlag;
uint8_t rfidMissCount1;
uint8_t rfidMissCount2;
uint8_t msg_type;

uint8_t rfid1_fault = 0;   // Reader at CAN ID 0x120
uint8_t rfid2_fault = 0;   // Reader at CAN ID 0x121

rfid_normal_tag_t tag;
rfid_lc_tag_t LCtag;
rfid_adj_line_tag_t AdjLinetag;
rfid_junction_tag_t juntag;

//uint16_t prev_tag_id = 0xFFFF;
//uint8_t current_tag_dup_type;

bool comm_mandatory_area;

    uint32_t odo_distance_radio_rfid_ref;

typedef struct
{
    uint8_t  active;
    uint8_t  tag_type;
    // uint8_t  tag_instance;

    uint8_t  seq_total;
    uint8_t  received_mask;


    uint8_t  data_len;
    uint8_t  data[RFID_MAX_DATA_BYTES];

    uint32_t start_time;
} rfid_rx_ctx_t;

rfid_rx_ctx_t rfid_ctx;

//Added by UDAY
#define RFID_QUEUE_DEPTH 5
uint8_t rfid_Count;

typedef struct
{
    rfid_rx_ctx_t buf[RFID_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} rfid_ctx_queue_t;

rfid_ctx_queue_t rfid_q_120 = {0};
rfid_ctx_queue_t rfid_q_121 = {0};

//!================ Database Implementation Start =====================//
rfid_db_entry_t rfid_db[RFID_DB_SIZE];
uint8_t rfid_db_head = 0;
uint8_t rfid_db_count = 0;

uint8_t current_Tag_Type;
uint8_t rfid_Miss_Count;

/* ---------------- Build Entry ---------------- */
void rfid_db_build_entry(rfid_db_entry_t *e)
{
    memset(e, 0, sizeof(*e));

    e->tag_type = tag_type;
    e->valid = 1;
    e->odo_distance_rfid_ref = ref_odo;
    e->location_check = 0;

    switch(tag_type)
    {
        case RFID_TAG_NORMAL:
            e->data.normal = tag;
            break;

        case RFID_TAG_LC:
            e->data.lc = LCtag;
            break;

        case RFID_TAG_ADJ_LINE:
            e->data.adj = AdjLinetag;
            break;

        case RFID_TAG_JUNCTION:
            e->data.junction = juntag;
            break;
    }
}

/* ---------------- Entry Match Logic ---------------- */
uint8_t rfid_entries_match(rfid_db_entry_t *a, rfid_db_entry_t *b)
{
    /* -------- SAME TYPE -------- */
    if (a->tag_type == b->tag_type)
    {
        switch(a->tag_type)
        {
        case RFID_TAG_NORMAL:
            if((a->data.normal.tag_uid == b->data.normal.tag_uid) &&
                    (a->data.normal.abs_loc_m == b->data.normal.abs_loc_m) &&
                    (a->data.normal.is_duplicate != b->data.normal.is_duplicate))
            {
                a->location_check = 1;
                b->location_check = 1;
            }
            return (a->data.normal.tag_uid == b->data.normal.tag_uid) &&
                    (a->data.normal.abs_loc_m == b->data.normal.abs_loc_m);

        case RFID_TAG_LC:
            if((a->data.lc.tag_set_id == b->data.lc.tag_set_id) &&
                    (a->data.lc.abs_loc_m == b->data.lc.abs_loc_m) &&
                    (a->data.lc.is_duplicate != b->data.lc.is_duplicate))
            {
                a->location_check = 1;
                b->location_check = 1;
            }
            return (a->data.lc.tag_set_id == b->data.lc.tag_set_id) &&
                    (a->data.lc.abs_loc_m == b->data.lc.abs_loc_m);

            case RFID_TAG_ADJ_LINE:
                if((a->data.adj.tag_set_id == b->data.adj.tag_set_id) &&
                        (a->data.adj.abs_loc_m == b->data.adj.abs_loc_m) &&
                        (a->data.adj.is_duplicate != b->data.adj.is_duplicate))
                {
                    a->location_check = 1;
                    b->location_check = 1;
                }
                return (a->data.adj.tag_set_id == b->data.adj.tag_set_id) &&
                       (a->data.adj.abs_loc_m == b->data.adj.abs_loc_m);
                       /* optional: && adj_tin[0] */

            case RFID_TAG_JUNCTION:
                if((a->data.junction.tag_set_id == b->data.junction.tag_set_id) &&
                        ((a->data.junction.abs_loc_1 == b->data.junction.abs_loc_1) ||
                                (a->data.junction.abs_loc_2 == b->data.junction.abs_loc_2)) &&
                                (a->data.junction.tag_duplicate != b->data.junction.tag_duplicate))
                {
                    a->location_check = 1;
                    b->location_check = 1;
                }
                return (a->data.junction.tag_set_id == b->data.junction.tag_set_id) &&
                       ((a->data.junction.abs_loc_1 == b->data.junction.abs_loc_1) ||
                        (a->data.junction.abs_loc_2 == b->data.junction.abs_loc_2));
        }
    }

    /* -------- DIFFERENT TYPE (main vs duplicate) -------- */
    else
    {
        return 0;
    }

    return 0;
}

/* ---------------- Duplicate Check ---------------- */
uint8_t rfid_db_is_duplicate(rfid_db_entry_t *new_entry)
{
    for(uint8_t i = 0; i < rfid_db_count; i++)
    {
        uint8_t idx = (rfid_db_head + RFID_DB_SIZE - 1 - i) % RFID_DB_SIZE;
        rfid_db_entry_t *old = &rfid_db[idx];

        if(!old->valid)
            continue;

        if (rfid_entries_match(old, new_entry))
        {
            return 1;
        }
    }

    return 0;
}

/* ---------------- Insert ---------------- */
void rfid_db_insert(rfid_db_entry_t *entry)
{
    if (rfid_db_is_duplicate(entry))
    {
        return;
    }

    rfid_db[rfid_db_head] = *entry;

    rfid_db_head = (rfid_db_head + 1) % RFID_DB_SIZE;

    if (rfid_db_count < RFID_DB_SIZE)
        rfid_db_count++;
}
//!================ Database Implementation end =====================//

//#define ABS(x) ((x) < 0 ? -(x) : (x))

//static inline void rfid_queue_push(rfid_ctx_queue_t *q,
//                                   const rfid_rx_ctx_t *ctx)
//{
//    if (q->count >= RFID_QUEUE_DEPTH)
//    {
//        /* Queue full - drop newest or oldest */
//        return;
//    }
//
//    q->buf[q->head] = *ctx;   // STRUCT COPY
//    q->head = (q->head + 1U) % RFID_QUEUE_DEPTH;
//    q->count++;
//}

//static void rfid_queue_drop_n(rfid_ctx_queue_t *q, uint8_t n)
//{
//    if (n > q->count)
//        n = q->count;
//
//    q->tail = (q->tail + n) % RFID_QUEUE_DEPTH;
//    q->count -= n;
//}
//
//static uint8_t rfid_queue_peek(const rfid_ctx_queue_t *q,
//                               uint8_t index,
//                               rfid_rx_ctx_t *out)
//{
//    if (index >= q->count)
//        return 0U;
//
//    uint8_t pos = (q->tail + index) % RFID_QUEUE_DEPTH;
//    *out = q->buf[pos];
//    return 1U;
//}
//

void rfid_ctx_reset(void)
{
    rfid_ctx.active = 0;
    rfid_ctx.received_mask = 0;
    rfid_ctx.data_len = 0;
}

/* ================= TAG DECODE STUBS ================= */
uint32_t get_bits_le(const uint8_t *buf, uint16_t bit, uint8_t len)
{
    uint32_t val = 0;

    uint8_t i;

    for ( i = 0; i < len; i++)
    {
        uint16_t b = bit + i;
        uint8_t byte = b >> 3;
        uint8_t shift = b & 7;

        if (buf[byte] & (1U << shift))
            val |= (1U << i);
    }
    return val;
}
//Direction check
uint32_t prev_abs_loc_m = 0xFFFFFFFF;
uint8_t trainDir = 0xFF;

/* ---------------- Normal Tag ---------------- */
void rfid_decode_normal_tag(const uint8_t *raw)
{
    /* Header / Common */
    tag.version        = get_bits_le(raw, 4, 2);
    tag.tag_uid        = get_bits_le(raw, 6, 10);
    tag.abs_loc_m      = get_bits_le(raw, 16, 23);

    if(prev_abs_loc_m == 0xFFFFFFFF && tag.abs_loc_m != 0xFFFFFFFF)
    {
        prev_abs_loc_m = tag.abs_loc_m;
    }

    tag.tin_nominal    = get_bits_le(raw, 39, 8);
    tag.tin_reverse    = get_bits_le(raw, 47, 8);

    /* Station IDs (cross X/Y boundary) */
    tag.station_nominal =
            get_bits_le(raw, 55, 9) |          /* X63-X55 */
            (get_bits_le(raw, 64, 7) << 9);    /* Y6-Y0 */

    tag.station_reverse =
            get_bits_le(raw, 71, 16);          /* Y22-Y7 */

    if (trainDir == 0) {
      approaching_station_id = tag.station_nominal;
    } else if (trainDir == 1) {
      approaching_station_id = tag.station_reverse;
    }

    /* Section types */
    tag.section_nominal = get_bits_le(raw, 87, 2);   /* Y24-Y23 */
    tag.section_reverse = get_bits_le(raw, 89, 2);   /* Y26-Y25 */

    /* Placement & flags */
    tag.tag_placement      = get_bits_le(raw, 91, 4);  /* Y30-Y27 */
    tag.is_duplicate       = get_bits_le(raw, 95, 1);  /* Y31 */
    tag.comm_nominal_req   = get_bits_le(raw, 96, 1);  /* Y32 */
    tag.comm_reverse_req   = get_bits_le(raw, 97, 1);  /* Y33 */

    /* Optional validation hooks */
    /*
    if (tag.abs_loc_m == 0x7FFFFF) { }
    if (tag.tag_uid == 0) { }
     */

//    if(prev_abs_loc_m > tag.abs_loc_m)
//    {
//        trainDir = 1;
//        prev_abs_loc_m = tag.abs_loc_m;
//        input_write.raw_flags[0] |= (1U << 24);
//        input_write.raw_flags[0] &= ~(1U << 25);
//    }
//    else if(prev_abs_loc_m < tag.abs_loc_m)
//    {
//        trainDir = 0;
//        prev_abs_loc_m = tag.abs_loc_m;
//        input_write.raw_flags[0] |= (1U << 24);
//        input_write.raw_flags[0] &= ~(1U << 25);
//    }
//    else
//    {
//        trainDir = 0xFF;
//        input_write.raw_flags[0] &= ~(1U << 24);
//        input_write.raw_flags[0] |= (1U << 25);
//    }
//
//    //Check State Machine Changes:
//    /*nominal direction*/
//    if(trainDir == 0)
//    {
//        if((tag.tag_placement == 4) || (tag.tag_placement == 7))
//        {
//            if(tag.tin_nominal)
//            {
//                input_write.raw_flags[0] |= (1U << 15);
//                input_write.raw_flags[0] &= ~(1U << 16);
//            }
//            else
//            {
//                input_write.raw_flags[0] &= ~(1U << 15);
//                input_write.raw_flags[0] |= (1U << 16);
//            }
//        }
//
//        if((tag.tag_placement == 1) || (tag.tag_placement == 6))
//        {
//            input_write.raw_flags[1] |= (1U << 19);
//        }
//        else
//        {
//            input_write.raw_flags[1] &= ~(1U << 19);
//        }
//
//        if(!tag.section_nominal)
//        {
//            input_write.raw_flags[0] |= (1U << 17);
//            input_write.raw_flags[0] &= ~(1U << 18);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 17);
//            input_write.raw_flags[0] |= (1U << 18);
//        }
//
//        if(!tag.comm_nominal_req)
//        {
//            comm_mandatory_area = 1;
//            input_write.raw_flags[0] |= (1U << 19);
//            input_write.raw_flags[0] &= ~(1U << 20);
//        }
//        else
//        {
//            comm_mandatory_area = 0;
//            input_write.raw_flags[0] &= ~(1U << 19);
//            input_write.raw_flags[0] |= (1U << 20);
//        }
//    }
//    /*reverse direction*/
//    else if(trainDir == 1)
//    {
//        if((tag.tag_placement == 5) || (tag.tag_placement == 7))
//        {
//            if(tag.tin_reverse)
//            {
//                input_write.raw_flags[0] |= (1U << 15);
//                input_write.raw_flags[0] &= ~(1U << 16);
//            }
//            else
//            {
//                input_write.raw_flags[0] &= ~(1U << 15);
//                input_write.raw_flags[0] |= (1U << 16);
//            }
//        }
//
//        if((tag.tag_placement == 2) || (tag.tag_placement == 6))
//        {
//            input_write.raw_flags[1] |= (1U << 19);
//        }
//        else
//        {
//            input_write.raw_flags[1] &= ~(1U << 19);
//        }
//
//        if(!tag.section_reverse)
//        {
//            input_write.raw_flags[0] |= (1U << 17);
//            input_write.raw_flags[0] &= ~(1U << 18);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 17);
//            input_write.raw_flags[0] |= (1U << 18);
//        }
//
//        if(!tag.comm_reverse_req)
//        {
//          comm_mandatory_area = 1;
//          input_write.raw_flags[0] |= (1U << 19);
//          input_write.raw_flags[0] &= ~(1U << 20);
//        }
//        else
//        {
//          comm_mandatory_area = 0;
//          input_write.raw_flags[0] &= ~(1U << 19);
//          input_write.raw_flags[0] |= (1U << 20);
//        }
//    }
}

/* ---------------- LC Tag ---------------- */
void rfid_decode_lc_tag(const uint8_t *raw)
{
    //    rfid_lc_tag_t LCtag;

    LCtag.version          = get_bits_le(raw, 4, 2);
    LCtag.tag_set_id       = get_bits_le(raw, 6, 10);

    LCtag.abs_loc_m        = get_bits_le(raw, 16, 23);

    if(prev_abs_loc_m == 0xFFFFFFFF && LCtag.abs_loc_m != 0xFFFFFFFF)
    {
        prev_abs_loc_m = LCtag.abs_loc_m;
    }

    LCtag.tin_nominal      = get_bits_le(raw, 39, 8);
    LCtag.tin_reverse      = get_bits_le(raw, 47, 8);

    LCtag.section_nominal  = get_bits_le(raw, 55, 2);
    LCtag.section_reverse  = get_bits_le(raw, 57, 2);

    LCtag.tag_placement    = get_bits_le(raw, 59, 4);

    LCtag.lc_territory     = get_bits_le(raw, 63, 2);
    LCtag.applicable_dir   = get_bits_le(raw, 65, 1);

    LCtag.gate_id          = get_bits_le(raw, 66, 10);
    LCtag.gate_alpha       = get_bits_le(raw, 76, 3);
    LCtag.gate_type        = get_bits_le(raw, 79, 1);

    LCtag.distance_to_gate = get_bits_le(raw, 80, 10);

    LCtag.auto_whistle     = get_bits_le(raw, 90, 1);
    LCtag.whistle_type     = get_bits_le(raw, 91, 1);

    LCtag.is_duplicate     = get_bits_le(raw, 95, 1);
    LCtag.comm_nominal     = get_bits_le(raw, 96, 1);
    LCtag.comm_reverse     = get_bits_le(raw, 97, 1);

    /* ---- NEXT STEP ----
       Push this into your navigation / braking / DMI logic
     */

//    if(prev_abs_loc_m > LCtag.abs_loc_m)
//    {
//        trainDir = 1;
//        prev_abs_loc_m = LCtag.abs_loc_m;
//        input_write.raw_flags[0] |= (1U << 24);
//        input_write.raw_flags[0] &= ~(1U << 25);
//    }
//    else if(prev_abs_loc_m < LCtag.abs_loc_m)
//    {
//        trainDir = 0;
//        prev_abs_loc_m = LCtag.abs_loc_m;
//        input_write.raw_flags[0] |= (1U << 24);
//        input_write.raw_flags[0] &= ~(1U << 25);
//    }
//    else
//    {
//        trainDir = 0xFF;
//        input_write.raw_flags[0] &= ~(1U << 24);
//        input_write.raw_flags[0] |= (1U << 25);
//    }
//
//    //Check State Machine Changes:
//    /*nominal direction*/
//    if(trainDir == 0)
//    {
//        if((LCtag.tag_placement == 4) || (LCtag.tag_placement == 7))
//        {
//            if(LCtag.tin_nominal)
//            {
//                input_write.raw_flags[0] |= (1U << 15);
//                input_write.raw_flags[0] &= ~(1U << 16);
//            }
//            else
//            {
//                input_write.raw_flags[0] &= ~(1U << 15);
//                input_write.raw_flags[0] |= (1U << 16);
//            }
//        }
//
//        if(!LCtag.section_nominal)
//        {
//            input_write.raw_flags[0] |= (1U << 17);
//            input_write.raw_flags[0] &= ~(1U << 18);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 17);
//            input_write.raw_flags[0] |= (1U << 18);
//        }
//
//        if(!LCtag.comm_nominal)
//        {
//          comm_mandatory_area = 1;
//          input_write.raw_flags[0] |= (1U << 19);
//          input_write.raw_flags[0] &= ~(1U << 20);
//        }
//        else
//        {
//          comm_mandatory_area = 0;
//          input_write.raw_flags[0] &= ~(1U << 19);
//          input_write.raw_flags[0] |= (1U << 20);
//        }
//    }
//    /*reverse direction*/
//    else if(trainDir == 1)
//    {
//        if((LCtag.tag_placement == 5) || (LCtag.tag_placement == 7))
//        {
//            if(LCtag.tin_reverse)
//            {
//                input_write.raw_flags[0] |= (1U << 15);
//                input_write.raw_flags[0] &= ~(1U << 16);
//            }
//            else
//            {
//                input_write.raw_flags[0] &= ~(1U << 15);
//                input_write.raw_flags[0] |= (1U << 16);
//            }
//        }
//
//        if(!LCtag.section_reverse)
//        {
//            input_write.raw_flags[0] |= (1U << 17);
//            input_write.raw_flags[0] &= ~(1U << 18);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 17);
//            input_write.raw_flags[0] |= (1U << 18);
//        }
//
//        if(!LCtag.comm_reverse)
//        {
//          comm_mandatory_area = 1;
//          input_write.raw_flags[0] |= (1U << 19);
//          input_write.raw_flags[0] &= ~(1U << 20);
//        }
//        else
//        {
//          comm_mandatory_area = 0;
//          input_write.raw_flags[0] &= ~(1U << 19);
//          input_write.raw_flags[0] |= (1U << 20);
//        }
//    }
}

/* ---------------- Adjacent Line Tag ---------------- */
void rfid_decode_adj_line_tag(const uint8_t *raw)
{
    //    rfid_adj_line_tag_t AdjLinetag;

    AdjLinetag.version      = get_bits_le(raw, 4, 2);
    AdjLinetag.tag_set_id   = get_bits_le(raw, 6, 10);

    AdjLinetag.abs_loc_m    = get_bits_le(raw, 16, 23);

    if(prev_abs_loc_m == 0xFFFFFFFF && AdjLinetag.abs_loc_m != 0xFFFFFFFF)
    {
        prev_abs_loc_m = AdjLinetag.abs_loc_m;
    }

    AdjLinetag.tin_nominal  = get_bits_le(raw, 39, 8);
    AdjLinetag.tin_reverse  = get_bits_le(raw, 47, 8);

    AdjLinetag.adj_tin[0]   = get_bits_le(raw, 55, 8);
    AdjLinetag.adj_tin[1]   = get_bits_le(raw, 63, 8);
    AdjLinetag.adj_tin[2]   = get_bits_le(raw, 71, 8);
    AdjLinetag.adj_tin[3]   = get_bits_le(raw, 79, 8);
    AdjLinetag.adj_tin[4]   = get_bits_le(raw, 87, 8);

    AdjLinetag.is_duplicate = get_bits_le(raw, 95, 1);

//    if(prev_abs_loc_m > AdjLinetag.abs_loc_m)
//    {
//        trainDir = 1;
//        prev_abs_loc_m = AdjLinetag.abs_loc_m;
//        input_write.raw_flags[0] |= (1U << 24);
//        input_write.raw_flags[0] &= ~(1U << 25);
//    }
//    else if(prev_abs_loc_m < AdjLinetag.abs_loc_m)
//    {
//        trainDir = 0;
//        prev_abs_loc_m = AdjLinetag.abs_loc_m;
//        input_write.raw_flags[0] |= (1U << 24);
//        input_write.raw_flags[0] &= ~(1U << 25);
//    }
//    else
//    {
//        trainDir = 0xFF;
//        input_write.raw_flags[0] &= ~(1U << 24);
//        input_write.raw_flags[0] |= (1U << 25);
//    }
//
//    //Check State Machine Changes:
//    /*nominal direction*/
//    if(trainDir == 0)
//    {
//        if(AdjLinetag.tin_nominal)
//        {
//            input_write.raw_flags[0] |= (1U << 15);
//            input_write.raw_flags[0] &= ~(1U << 16);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 15);
//            input_write.raw_flags[0] |= (1U << 16);
//        }
//    }
//    /*reverse direction*/
//    else if(trainDir == 1)
//    {
//        if(AdjLinetag.tin_reverse)
//        {
//            input_write.raw_flags[0] |= (1U << 15);
//            input_write.raw_flags[0] &= ~(1U << 16);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 15);
//            input_write.raw_flags[0] |= (1U << 16);
//        }
//    }
}

/* ----------------  Adjustment/Junction Tag  ---------------- */
void rfid_decode_junction_tag(const uint8_t *d)
{
    //    rfid_junction_tag_t juntag;

    /* Header */
    juntag.version        = get_bits_le(d, 4, 2);
    juntag.tag_set_id     = get_bits_le(d, 6, 10);

    /* Absolute Location 1 */
    juntag.abs_loc_1      = get_bits_le(d, 16, 23);
    juntag.tin_1          = get_bits_le(d, 39, 8);
    juntag.tin_2          = get_bits_le(d, 47, 8);

    /* Absolute Location 2 (crosses X/Y boundary) */
    juntag.abs_loc_2 =
            get_bits_le(d, 55, 9) |          // X63-X55
            (get_bits_le(d, 64, 14) << 9);   // Y13-Y0

    if(prev_abs_loc_m == 0xFFFFFFFF && (juntag.abs_loc_1 != 0xFFFFFFFF || juntag.abs_loc_2 != 0xFFFFFFFF))
    {
        if(juntag.abs_loc_1 != 0xFFFFFFFF)
        {
            prev_abs_loc_m = juntag.abs_loc_1;
        }
        else
        {
            prev_abs_loc_m = juntag.abs_loc_2;
        }
    }

    /* Direction Corrections */
    juntag.dir_corr_1     = get_bits_le(d, 78, 3);  // Y16-Y14
    juntag.dir_corr_2     = get_bits_le(d, 81, 3);  // Y19-Y17
    juntag.loc_corr_type  = get_bits_le(d, 84, 1);  // Y20

    /* Section Types */
    juntag.section_type_1 = get_bits_le(d, 87, 2);  // Y24-Y23
    juntag.section_type_2 = get_bits_le(d, 89, 2);  // Y26-Y25

    /* Control Flags */
    juntag.tag_duplicate  = get_bits_le(d, 93, 1);  // Y31
    juntag.comm_nominal   = get_bits_le(d, 94, 1);  // Y32
    juntag.comm_reverse   = get_bits_le(d, 95, 1);  // Y33

//    if(abs(prev_abs_loc_m - juntag.abs_loc_1) < abs(prev_abs_loc_m - juntag.abs_loc_2))
//    {
//        if(juntag.dir_corr_1 != 0)
//        {
//            /* Now pass tag to location / route logic */
//            if(prev_abs_loc_m > juntag.abs_loc_1)
//            {
//                trainDir = 1;
//                prev_abs_loc_m = juntag.abs_loc_1;
//                input_write.raw_flags[0] |= (1U << 24);
//                input_write.raw_flags[0] &= ~(1U << 25);
//            }
//            else if(prev_abs_loc_m < juntag.abs_loc_1)
//            {
//                trainDir = 0;
//                prev_abs_loc_m = juntag.abs_loc_1;
//                input_write.raw_flags[0] |= (1U << 24);
//                input_write.raw_flags[0] &= ~(1U << 25);
//            }
//            else
//            {
//                trainDir = 0xFF;
//                input_write.raw_flags[0] &= ~(1U << 24);
//                input_write.raw_flags[0] |= (1U << 25);
//            }
//        }
//
//    }
//    else
//    {
//        if(juntag.dir_corr_2 != 0)
//        {
//            /* Now pass tag to location / route logic */
//            if(prev_abs_loc_m > juntag.abs_loc_2)
//            {
//                trainDir = 1;
//                prev_abs_loc_m = juntag.abs_loc_2;
//                input_write.raw_flags[0] |= (1U << 24);
//                input_write.raw_flags[0] &= ~(1U << 25);
//            }
//            else if(prev_abs_loc_m < juntag.abs_loc_2)
//            {
//                trainDir = 0;
//                prev_abs_loc_m = juntag.abs_loc_2;
//                input_write.raw_flags[0] |= (1U << 24);
//                input_write.raw_flags[0] &= ~(1U << 25);
//            }
//            else
//            {
//                trainDir = 0xFF;
//                input_write.raw_flags[0] &= ~(1U << 24);
//                input_write.raw_flags[0] |= (1U << 25);
//            }
//        }
//    }

    //Check State Machine Changes:
    /*nominal direction*/
//    if(trainDir == 0)
//    {
//        if((juntag.tag_placement == 4) && (juntag.tag_placement == 7))
//        {
//            if(juntag.tin_nominal)
//            {
//                input_write.raw_flags[0] |= (1U << 15);
//                input_write.raw_flags[0] &= ~(1U << 16);
//            }
//            else
//            {
//                input_write.raw_flags[0] &= ~(1U << 15);
//                input_write.raw_flags[0] |= (1U << 16);
//            }
//        }
//
//        if(!juntag.section_nominal)
//        {
//            input_write.raw_flags[0] |= (1U << 17);
//            input_write.raw_flags[0] &= ~(1U << 18);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 17);
//            input_write.raw_flags[0] |= (1U << 18);
//        }
//
//        if(!juntag.comm_nominal)
//        {
//            input_write.raw_flags[0] |= (1U << 19);
//            input_write.raw_flags[0] &= ~(1U << 20);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 19);
//            input_write.raw_flags[0] |= (1U << 20);
//        }
//    }
    /*reverse direction*/
//    else if(trainDir == 1)
//    {
//        if((juntag.tag_placement == 5) && (juntag.tag_placement == 7))
//        {
//            if(juntag.tin_reverse)
//            {
//                input_write.raw_flags[0] |= (1U << 15);
//                input_write.raw_flags[0] &= ~(1U << 16);
//            }
//            else
//            {
//                input_write.raw_flags[0] &= ~(1U << 15);
//                input_write.raw_flags[0] |= (1U << 16);
//            }
//        }
//
//        if(!juntag.section_reverse)
//        {
//            input_write.raw_flags[0] |= (1U << 17);
//            input_write.raw_flags[0] &= ~(1U << 18);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 17);
//            input_write.raw_flags[0] |= (1U << 18);
//        }
//
//        if(!juntag.comm_reverse)
//        {
//            input_write.raw_flags[0] |= (1U << 19);
//            input_write.raw_flags[0] &= ~(1U << 20);
//        }
//        else
//        {
//            input_write.raw_flags[0] &= ~(1U << 19);
//            input_write.raw_flags[0] |= (1U << 20);
//        }
//    }
}

uint8_t match_index = 0xFF;
void rfid_process_complete_tag(void)
{
    uint16_t current_tag_id = 0xFFFF;
    switch ((rfid_tag_type_t)rfid_ctx.tag_type)
    {
    case RFID_TAG_NORMAL:
        rfid_decode_normal_tag(rfid_ctx.data);
        current_tag_id = tag.tag_uid;
        break;

    case RFID_TAG_LC:
        rfid_decode_lc_tag(rfid_ctx.data);
        current_tag_id = LCtag.tag_set_id;
        break;

    case RFID_TAG_ADJ_LINE:
        rfid_decode_adj_line_tag(rfid_ctx.data);
        current_tag_id = AdjLinetag.tag_set_id;
        break;

    case RFID_TAG_JUNCTION:
        rfid_decode_junction_tag(rfid_ctx.data);
        current_tag_id = juntag.tag_set_id;
        break;

    default:
        /* Unknown tag type */
        return;
    }

//    ref_odo = distance_m;

//    if(prev_tag_id != current_tag_id)
//    {
//        current_tag_dup_type = 0;
//    }

    rfid_ctx.active = 0;
    rfid_db_entry_t entry;
    rfid_db_build_entry(&entry);
    rfid_db_insert(&entry);

//    if (rfid_ctx.tag_type == RFID_TAG_NORMAL)
//    {
//        current_tag_dup_type |= (1 << tag.is_duplicate);
//    }
//    else if (rfid_ctx.tag_type == RFID_TAG_LC)
//    {
//        current_tag_dup_type |= (1 << LCtag.is_duplicate);
//    }
//    else if (rfid_ctx.tag_type == RFID_TAG_ADJ_LINE)
//    {
//        current_tag_dup_type |= (1 << AdjLinetag.is_duplicate);
//    }
//    else if (rfid_ctx.tag_type == RFID_TAG_JUNCTION)
//    {
//        current_tag_dup_type |= (1 << juntag.tag_duplicate);
//    }

    uint8_t match_found = 0xFF;
    match_index = 0xFF;

    for(uint8_t i = 0; i < reg_type1.TLI_Packet_reg_type1.route_rfid_cnt; i++)
    {
        if(reg_type1.TLI_Packet_reg_type1.nxt_rfid_tag_id[i] == current_tag_id)
        {
            match_found = 1;
            match_index = i;
            break;
        }
        else 
        {
            match_found = 0;
        }
    }

//    if(match_found == 0)
//    {
//        input_write.raw_flags[1] |= (1U << 17);
//    }
//    else if(match_index == rfid_Count)
//    {
//
//        // CONDITION 2:
//        // Match with FIRST entry
//    }
//    else if((match_found != 0xFF) && (match_index != 0xFF))
//    {
//        // CONDITION 3:
//        // Match but NOT first entry
//        input_write.raw_flags[1] |= (1U << 24);
//    }

//    prev_tag_id = current_tag_id;
    rfid_Count++;
    rfid_Miss_Count = 0;
}

void rfid_rx_handle(uint32_t can_id, uint8_t *data)
{
    /* ---------------- New 1-byte header ----------------
    *
    * Byte0:
    * Bits 3:0 = MSG_TYPE
    * Bits 5:4 = SEQ_TOTAL
    * Bits 7:6 = SEQ_INDEX
    */

    uint8_t header = data[0];

    msg_type = (header) & 0x0F;

    if (msg_type == 0x00)
    {
//        dmi_tags_signals.tr  = 1;
    }

    seq_total = (header >> 4) & 0x03;
    seq_index = (header >> 6) & 0x03;

    /* Check MSG_TYPE */
    // if (msg_type != 0x02U) //TODO: Add MSG_TYPE Checking
        // return;

    /* Sanity checks */
    if (seq_total == 0 || seq_total > RFID_MAX_FRAGMENTS)
        return;

    if (seq_index >= seq_total)
        return;

    if (!rfid_ctx.active)
    {
        rfid_ctx_reset();

        rfid_ctx.active       = 1;
        // rfid_ctx.tag_type     = tag_type;
        // rfid_ctx.tag_instance = tag_instance;
        rfid_ctx.seq_total    = seq_total;
        rfid_ctx.start_time   = seconds_uptime;
    }

    /* Ignore duplicate fragment */
    if (rfid_ctx.received_mask & (1U << seq_index))
        return;

    /* Copy payload (bytes 1-7) */
    uint8_t offset = seq_index * 7;

    if ((offset + 7U) <= RFID_MAX_DATA_BYTES)
    {
        // rfid_ctx.data[offset + 0] = data[4];
        // rfid_ctx.data[offset + 1] = data[5];
        // rfid_ctx.data[offset + 2] = data[6];
        // rfid_ctx.data[offset + 3] = data[7];
        memcpy(&rfid_ctx.data[offset], &data[1], 7U);
    }

    rfid_ctx.received_mask |= (1U << seq_index);
    rfid_ctx.data_len = offset + 7U;

    /* Check completion */
    uint8_t expected_mask = (1U << seq_total) - 1U;

    if (rfid_ctx.received_mask == expected_mask)
    {
        rfid_ctx.tag_type = rfid_ctx.data[0] & 0x0F;
        tag_type = rfid_ctx.data[0] & 0x0F;
        rfid_process_complete_tag();
        rfidDataMatchFlag = 1;
    }
}

void rfid_rx_poll(void)
{
    if (rfid_ctx.active)
    {
        if ((seconds_uptime - rfid_ctx.start_time) > 1U)
        {
            /* Timeout waiting for fragments */
            rfid_ctx_reset();
        }
    }
}

//Added by UDAY Functions:
//static uint8_t rfid_ctx_data_match(const rfid_rx_ctx_t *ctx1,
//                                   const rfid_rx_ctx_t *ctx2)
//{
//    uint8_t i;
//
//    if (ctx1->data_len != ctx2->data_len)
//        return 0U;
//
//    for (i = 0U; i < ctx1->data_len; i++)
//    {
//        if (ctx1->data[i] != ctx2->data[i])
//            return 0U;
//    }
//
//    return 1U;
//}

//void rfid_process_queues_1s(void)
//{
//    uint8_t i, j;
//    rfid_rx_ctx_t ctx120;
//    rfid_rx_ctx_t ctx121;
//
//    if ((rfid_q_120.count == 0U) || (rfid_q_121.count == 0U))
//        return;
//
//    for (i = 0U; i < rfid_q_120.count; i++)
//    {
//        rfid_queue_peek(&rfid_q_120, i, &ctx120);
//        for (j = 0U; j < rfid_q_121.count; j++)
//        {
//            rfid_queue_peek(&rfid_q_121, j, &ctx121);
//            if (rfid_ctx_data_match(&ctx120, &ctx121))
//            {
//                /* DROP everything before + including the match */
//                rfid_queue_drop_n(&rfid_q_120, i + 1U);
//                rfid_queue_drop_n(&rfid_q_121, j + 1U);
//                rfid1_fault = 0;   // Reader at CAN ID 0x120
//                rfid2_fault = 0;   // Reader at CAN ID 0x121
//                return;
//            }
//        }
//    }
//}
