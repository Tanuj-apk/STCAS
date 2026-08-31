#include "radio.h"
#include "StateMachine.h"
#include "can.h"
#include "can_if.h"
//#include "dmi_can.h"
#include "gps.h"
//#include "pulse_generator.h"
#include "rfid_rx.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

//! Hardcoded loco ID and prev_frame for testing Condition 24 in StateMachine.c
uint32_t g_my_stn_id = 0x12345; // hardcoded for testing
uint16_t approaching_station_id = 0; //Updated in rfid.c
//!==========================================================================

#define KAVACH_VERSION_3_2   1
#define KAVACH_VERSION_4_0   2

dist_array_t distance_db[TLM_HISTORY_SECONDS];
uint8_t current_sec_index = 0U;
uint16_t current_sample_index = 0U;
volatile uint32_t tlm_tod_sec = 0U;
volatile uint32_t tlm_tod_ms = 0U;

//static uint32_t tlm_start_distance = 0U;
//static uint32_t tlm_end_distance = 0U;

uint32_t calculated_train_length = 0U;

//static uint8_t tlm_start_valid = 0U;

//static void radio_process_tlm_type1(void);
//static void radio_process_tlm_type2(void);

//static float radio_get_tlm_distance(uint32_t event_sec, uint32_t event_ms);

static uint8_t tlm_active = 0;

//!=========== Info Ack implementation =================
#define INFO_ACK_QUEUE_SIZE 16
#define INFO_ACK_REPEAT_COUNT 5

typedef struct
{
  uint8_t ack_code;
  uint8_t repeat_count;
  uint8_t persistent;
} info_ack_event_t;

static info_ack_event_t info_ack_queue[INFO_ACK_QUEUE_SIZE];

static uint8_t info_ack_head = 0;
//static uint8_t info_ack_tail = 0;
static uint8_t info_ack_count = 0;

typedef enum
{
  INFO_ACK_NONE = 0,
  INFO_ACK_LOCO_SOS = 1,
  INFO_ACK_FS_TO_LS = 2,
  INFO_ACK_LS_TO_SR = 3,
  INFO_ACK_FS_TO_SR = 4,
  INFO_ACK_OS_TO_SR = 5,
  INFO_ACK_OV_TO_SR = 6,
  INFO_ACK_TRIP = 7,
  INFO_ACK_PTRIP_TO_SR = 8,
  INFO_ACK_AUTO_HORN = 9,
  INFO_ACK_TLM_START = 10,
  INFO_ACK_TLM_END = 11,
  INFO_ACK_UNUSUAL_STOPPAGE = 12,
  INFO_ACK_MANUAL_SOS = 13
} InfoAck_t;

/* ================= INTERNAL PROTOTYPES ================= */
static void radio_process_complete_packet(void);
static void radio_rx_reset(void);

volatile uint32_t radio_rx_isr_count = 0;     /* total RX frames */
volatile uint32_t radio_rx_pkt_count = 0;     /* completed packets */
volatile uint32_t radio_rx_frag_count = 0;    /* fragments stored */

bool check_for_sending_arp_using_aap =1;
bool radio_can_arp_transmit_flag=0;

uint16_t prev_stn_id = 0;


/* ================= STATIC STATE ================= */

radio_rx_ctx_t radio_rx_ctx;
uint32_t frame_num = 1;
uint32_t ref_odo = 4200;   // meters (example)
#define EOA_MARGIN 30
/* Distance travelled */
int32_t distance_travelled;
//static int32_t prev_ma_type1 = 0;
//static uint8_t prev_ma_valid_type1 = 0;
//static int32_t prev_ma_type2 = 0;
//static uint8_t prev_ma_valid_type2 = 0;

uint32_t last_osma_rx_time = 0;
uint8_t osma_active = 0;

uint8_t last_ref_profile_num = 0;

typedef struct
{
    uint8_t  pkt_type;
    uint16_t pkt_len;
    uint32_t frame_num;
    uint32_t station_id;
    uint8_t  version;
    uint32_t location;
    uint32_t dest_loco_id;
    uint16_t uplink_freq;
    uint16_t downlink_freq;
    uint8_t  tdma_slot;
    uint32_t station_random;
    uint8_t station_tdma;
    uint8_t  mac[4];
} radio_aap_t;

//static radio_aap_t g_aap;

typedef struct
{
    uint8_t  pkt_type;
    uint16_t pkt_len;
    uint32_t frame_num;
    uint32_t station_id;
    uint8_t  version;
    uint32_t location;
    uint8_t  gen_sos_call;
    uint32_t crc;
} radio_aep_t;

//static radio_aep_t g_aep;

typedef struct
{
    uint8_t  pkt_type;
    uint16_t pkt_len;
    uint32_t frame_num;
    uint32_t source_loco_id;
    uint32_t source_loco_version;
    uint32_t abs_loco_loc;
    uint16_t l_doubtover;
    uint16_t l_doubtunder;
    uint8_t  train_int;
    uint16_t train_length;
    uint16_t train_speed;
    uint8_t  movement_dir;
    uint8_t  emergency_status;
    uint8_t  loco_mode;
    uint16_t last_rfid_tag;
    uint8_t  tag_dup;
    uint8_t  tag_link_info;
    uint16_t tin;
    uint8_t  brake_applied;
    uint8_t  new_ma_reply;
    uint8_t  last_ref_profile_num;
    uint8_t sig_ov;
    uint8_t info_ack;
    uint8_t spare;
    uint8_t loco_health_status;
    uint32_t mac_code;
    uint32_t pkt_crc;
} radio_orp_t;

static radio_orp_t g_orp;

typedef struct
{
    uint8_t  pkt_type;
    uint16_t pkt_len;
    uint32_t frame_num;
    uint32_t source_loco_id;
    uint32_t source_loco_version;
    uint32_t abs_loco_loc;
    uint16_t l_doubtover;
    uint16_t l_doubtunder;
    uint8_t  train_int;
    uint16_t train_length;
    uint16_t train_speed;
    uint8_t  movement_dir;
    uint8_t  emergency_status;
    uint8_t  loco_mode;
    uint16_t approaching_station_id;
    uint16_t last_rfid_tag;
    uint16_t tin;
    uint32_t radio_longitude;
    uint32_t radio_latitude;
    uint16_t loco_rnd_num_rl;
    uint8_t padding_bits;
    uint32_t pkt_crc;
} radio_arp_t;

static radio_arp_t g_arp;

radio_reg_type1_t reg_type1;
radio_reg_type2_t reg_type2;

uint16_t index_var;

/* ================= INTERNAL HELPERS ================= */
uint32_t get_bits(const uint8_t *buf, uint16_t bit, uint8_t len)
{

    uint32_t val = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        uint16_t b = bit + i;
        uint8_t byte = b >> 3;           // Which byte: b / 8
        uint8_t shift = 7 - (b & 7);    // MSB: 7 - (b % 8)

        if (buf[byte] & (1U << shift))
            val |= (1U << (len - 1 - i));  // MSB-first bit ordering in result
    }
    index_var += len;
    return val;
}

//static void radio_info_ack_push(uint8_t ack_code)
//{
//    if (info_ack_count >= INFO_ACK_QUEUE_SIZE)
//    {
//        return; // Queue full
//    }
//
//    info_ack_queue[info_ack_tail].ack_code = ack_code;
//    info_ack_queue[info_ack_tail].repeat_count = INFO_ACK_REPEAT_COUNT;
//    info_ack_queue[info_ack_tail].persistent = 0;
//
//    if (ack_code == INFO_ACK_TLM_START)
//    {
//      info_ack_queue[info_ack_tail].persistent = 1;
//    }
//
//    info_ack_tail = (info_ack_tail + 1) % INFO_ACK_QUEUE_SIZE;
//    info_ack_count++;
//}

//static uint8_t radio_info_ack_get_current(void)
//{
//    if (info_ack_count == 0)
//    {
//        return INFO_ACK_NONE;
//    }
//
//    return info_ack_queue[info_ack_head].ack_code;
//}

static void radio_info_ack_tx_done(void)
{
    if (info_ack_count == 0)
    {
        return;
    }

    if (info_ack_queue[info_ack_head].repeat_count > 0)
    {
        info_ack_queue[info_ack_head].repeat_count--;
    }

    if (info_ack_queue[info_ack_head].repeat_count == 0)
    {
        uint8_t remove_event = 1;
        if(info_ack_queue[info_ack_head].persistent)
        {
            if(tlm_active)
            {
                remove_event = 0;
            }
        }

        if(remove_event)
        {
            info_ack_head = (info_ack_head + 1) % INFO_ACK_QUEUE_SIZE;
            info_ack_count--;
        }
    }
}
//!=========== Info Ack implementation End =================

/* ================= TX: ARP ================= */
//? Builds ARP payload
//? [Frame0][Frame1][Frame2][Frame3][Frame4]

static uint32_t radio_get_latest_abs_loc(void)
{
    if(rfid_db_count == 0)
        return 0;

    uint8_t dbnum = (rfid_db_head + RFID_DB_SIZE - 1) % RFID_DB_SIZE;

    switch(rfid_db[dbnum].tag_type)
    {
    case RFID_TAG_NORMAL:
        return rfid_db[dbnum].data.normal.abs_loc_m;

    case RFID_TAG_LC:
        return rfid_db[dbnum].data.lc.abs_loc_m;

    case RFID_TAG_ADJ_LINE:
        return rfid_db[dbnum].data.adj.abs_loc_m;

    case RFID_TAG_JUNCTION:
    {
        if(abs(prev_abs_loc_m - rfid_db[dbnum].data.junction.abs_loc_1) < abs(prev_abs_loc_m - rfid_db[dbnum].data.junction.abs_loc_2))
        {
            return rfid_db[dbnum].data.junction.abs_loc_1;
        }
        else
        {
            return rfid_db[dbnum].data.junction.abs_loc_2;
        }
    }

    default:
        return 0;
    }
}

//===================== HELPER FUNCTION FOR ARP AND ORP ===========================
static void set_bits(uint8_t *buf, uint16_t bit, uint8_t len, uint32_t value)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uint16_t b = bit + i;
        uint8_t byte = b >> 3;
        uint8_t shift = b & 7;

        if (value & (1UL << i))
        {
            buf[byte] |= (1U << shift);
        }
        else
        {
            buf[byte] &= ~(1U << shift);
        }
    }
}

void radio_update_frame_number(void)
{
    uint32_t seconds_of_day;
    seconds_of_day = cpu_time_sec % 86400U;
    frame_num = (seconds_of_day & (~1U)) + 1U;
}

static uint8_t radio_build_arp_payload(uint8_t *payload)
{
    uint16_t bit_index = 0;
    memset(payload, 0, RADIO_MAX_PAYLOAD_LEN);

    /* =========================================================
     * FRAME 0  ? payload[0–5]
     * ========================================================= */
    uint32_t source_loco_id /*= g_my_loco_id*/;

    /* PKT_TYPE : 4 bits */
    set_bits(payload, bit_index, 4, RADIO_PKT_TYPE_ARP);
    bit_index += 4;

    /* PKT_LEN : 7 bits (filled later) */
    uint16_t pkt_len_bit_pos = bit_index;
    set_bits(payload, bit_index, 7, 0);
    bit_index += 7;

    /* FRAME_NUM : 17 bits */
    set_bits(payload, bit_index, 17, frame_num);
    bit_index += 17;

    /* SOURCE_LOCO_ID : 20 bits */
    set_bits(payload, bit_index, 20, source_loco_id);
    bit_index += 20;

    /* =========================================================
     * FRAME 1
     * ========================================================= */

    uint8_t source_loco_version = 2;
    uint32_t abs_loc = radio_get_latest_abs_loc();
    // uint16_t train_length = 0; // TODO: Trail length measurement implementation
    uint16_t train_length = (uint16_t)calculated_train_length;
    uint16_t train_speed /*= (uint16_t)speed_kmh*/;
    uint8_t movement_dir = 0;

    if (trainDir == 0)
    {
        movement_dir = 1; // nominal
    }
    else if (trainDir == 1)
    {
        movement_dir = 2; // reverse
    }
    else
    {
        movement_dir = 0; // unknown
    }

    /* SOURCE_LOCO_VERSION : 3 bits */
    set_bits(payload, bit_index, 3, source_loco_version);
    bit_index += 3;

    /* ABS_LOCO_LOC : 23 bits */
    set_bits(payload, bit_index, 23, abs_loc);
    bit_index += 23;

    /* TRAIN_LENGTH : 11 bits */
    set_bits(payload, bit_index, 11, train_length);
    bit_index += 11;

    /* TRAIN_SPEED : 9 bits */
    set_bits(payload, bit_index, 9, train_speed);
    bit_index += 9;

    /* MOVEMENT_DIR : 2 bits */
    set_bits(payload, bit_index, 2, movement_dir);
    bit_index += 2;

    /* =========================================================
     * FRAME 2  ? payload[12–17]
     * ========================================================= */
    uint8_t loco_mode /*= (uint8_t)g_current*/;
    uint8_t emg_status = 0; //TODO: Add emergency state
    uint8_t dbnum;

    if(rfid_db_count != 0)
        dbnum = (rfid_db_head + RFID_DB_SIZE - 1) % RFID_DB_SIZE;
    else
        dbnum = 0;

    odo_distance_radio_rfid_ref = rfid_db[dbnum].odo_distance_rfid_ref;

    uint16_t tag_uid = 0;
    uint16_t tin = 10;

    if(rfid_db[dbnum].tag_type == 1)
    {
        tag_uid = rfid_db[dbnum].data.normal.tag_uid;
        if(trainDir == 0)
        {
            tin = rfid_db[dbnum].data.normal.tin_nominal;
        }
        else if(trainDir == 1)
        {
            tin = rfid_db[dbnum].data.normal.tin_reverse;
        }
    }
    else if(rfid_db[dbnum].tag_type == 2)
    {
        tag_uid = rfid_db[dbnum].data.lc.tag_set_id;
        if(trainDir == 0)
        {
            tin = rfid_db[dbnum].data.lc.tin_nominal;
        }
        else if(trainDir == 1)
        {
            tin = rfid_db[dbnum].data.lc.tin_reverse;
        }
    }
    else if(rfid_db[dbnum].tag_type == 3)
    {
        tag_uid = rfid_db[dbnum].data.adj.tag_set_id;
        if(trainDir == 0)
        {
            tin = rfid_db[dbnum].data.adj.tin_nominal;
        }
        else if(trainDir == 1)
        {
            tin = rfid_db[dbnum].data.adj.tin_reverse;
        }
    }
    else if(rfid_db[dbnum].tag_type == 4)
    {
        tag_uid = rfid_db[dbnum].data.junction.tag_set_id;
        if (abs(prev_abs_loc_m - rfid_db[dbnum].data.junction.abs_loc_1) < abs(prev_abs_loc_m - rfid_db[dbnum].data.junction.abs_loc_2))
        {
            tin = rfid_db[dbnum].data.junction.tin_1;
        }
        else
        {
            tin = rfid_db[dbnum].data.junction.tin_2;
        }
    }

    /* Byte2
     * EMG_STATUS : 3 bits
     */
    set_bits(payload, bit_index, 3, emg_status);
    bit_index += 3;

    /* LOCO_MODE : 4 bits */
    set_bits(payload, bit_index, 4, loco_mode);
    bit_index += 4;

    /* APPROACHING_STATION_ID : 16 bits */
    set_bits(payload, bit_index, 16, approaching_station_id);
    bit_index += 16;

    /* LAST_RFID_TAG : 10 bits */
    set_bits(payload, bit_index, 10, tag_uid);
    bit_index += 10;

    /* TIN : 9 bits */
    set_bits(payload, bit_index, 9, tin);
    bit_index += 9;

    /* LONGITUDE : 21 bits */
    set_bits(payload, bit_index, 21, radio_longitude);
    bit_index += 21;

    /* =========================================================
     * FRAME 3
     * ========================================================= */

    uint16_t loco_random_number = 354; // TODO

    /* LATITUDE : 20 bits */
    set_bits(payload, bit_index, 20, radio_latitude);
    bit_index += 20;

    /* LOCO_RND_NUM_RL : 16 bits */
    set_bits(payload, bit_index, 16, loco_random_number);
    bit_index += 16;

    /* =========================================================
     * INSERT PKT_LEN
     * ========================================================= */

    uint16_t payload_len = (bit_index + 7) / 8;

    /* Insert PKT_LEN back into header */
    set_bits(payload, pkt_len_bit_pos, 7, payload_len);
    return payload_len;
}

radio_arp_t arp = {0};
static uint8_t radio_parse_arp(const uint8_t *p, uint16_t len)
{
    if (!p || len < 18)   // Minimum length check
        return 0;

    index_var = 0;

    /* -------- HEADER -------- */
    arp.pkt_type = get_bits(p, index_var, 4);                   //4 bits
    arp.pkt_len = get_bits(p, index_var, 7);                    //7 bits
    arp.frame_num = get_bits(p, index_var, 17);                 //17 bits
    arp.source_loco_id = get_bits(p, index_var, 20);            //20 bits

    /* =========================================================
     * FRAME 1
     * ========================================================= */
    arp.source_loco_version = get_bits(p, index_var, 3);        //3 bits
    arp.abs_loco_loc = get_bits(p, index_var, 23);              //23 bits
    arp.train_length = get_bits(p, index_var, 11);              //11 bits
    arp.train_speed = get_bits(p, index_var, 9);                //9 bits
    arp.movement_dir = get_bits(p, index_var, 2);               //2 bits

    /* =========================================================
     * FRAME 2
     * ========================================================= */
    arp.emergency_status = get_bits(p, index_var, 3);           //3 bits
    arp.loco_mode = get_bits(p, index_var, 4);                  //4 bits
    arp.approaching_station_id = get_bits(p, index_var, 16);    //16 bits
    arp.last_rfid_tag = get_bits(p, index_var, 10);             //10 bits
    arp.tin = get_bits(p, index_var, 9);                        //9 bits
    arp.radio_longitude = get_bits(p, index_var, 21);           //21 bits

    /* =========================================================
     * FRAME 3
     * ========================================================= */
    arp.radio_latitude = get_bits(p, index_var, 20);            //20 bits
    arp.loco_rnd_num_rl = get_bits(p, index_var, 16);           //16 bits

    /* =========================================================
     * INSERT PKT_LEN
     * ========================================================= */
    arp.padding_bits = get_bits(p, index_var, 5);               //5 bits
    arp.pkt_crc = get_bits(p, index_var, 32);                   //32 bits

    g_arp = arp;
    return 1;
}

void radio_build_fragment(uint8_t *can_frame, uint8_t pkt_type, uint8_t seq_total, uint8_t seq_index)
{
    /* =========================================================
     * CAN FRAME BYTE 0
     * ---------------------------------------------------------
     * [7:4] ? Total number of fragments (seq_total)
     * [3:0] ? Packet type (ORP / ARP / etc.)
     * ========================================================= */
    can_frame[0] = ((seq_total & 0x0F) << 4) | (pkt_type & 0x0F);

    /* =========================================================
     * CAN FRAME BYTE 1
     * ---------------------------------------------------------
     * [7:4] ? Current fragment index (seq_index)
     * [3:0] ? Total number of fragments (seq_total)
     * ========================================================= */
    can_frame[1] = ((seq_index & 0x3f) << 2) | ((seq_total >>4) & 0x03);

    /* =========================================================
     * PAYLOAD SELECTION
     * ---------------------------------------------------------
     * Each CAN frame carries 6 bytes of ORP payload
     * offset = which ORP frame we are sending
     * ========================================================= */
    uint8_t payload_offset = seq_index * RADIO_PAYLOAD_BYTES;

    /* =========================================================
     * COPY PAYLOAD INTO CAN FRAME (BYTE 2 ? BYTE 7)
     * ---------------------------------------------------------
     * can_frame[2–7] = 6 bytes of ORP payload
     * ========================================================= */
    for (uint8_t i = 0; i < RADIO_PAYLOAD_BYTES; i++)
    {
        if ((payload_offset + i) < radio_ctx.payload_len)
        {
            can_frame[2 + i] = radio_ctx.payload[payload_offset + i];
        }
        else
        {
            can_frame[2 + i] = 0x00;   // padding if needed
        }
    }
}

uint8_t tx_buf[8];
uint32_t tx_mb;
void radio_send_arp(radio_id_t radio_id)
{
    tx_mb = (radio_id == RADIO_ID_1) ? RADIO1_TX_MB : RADIO2_TX_MB;
    radio_ctx.payload_len = radio_build_arp_payload(radio_ctx.payload);
    radio_ctx.seq_total = (radio_ctx.payload_len + RADIO_PAYLOAD_BYTES - 1U) /RADIO_PAYLOAD_BYTES;

    if (radio_ctx.seq_total > RADIO_MAX_FRAGMENTS)
        return;

    radio_can_arp_transmit_flag = 1;
}

/* ================= RX: ONBOARD REGULAR PACKET ================= */

radio_orp_t orp = {0};
static uint8_t radio_parse_orp(const uint8_t *p, uint16_t len)
{
    if (!p || len < 18)   // Minimum length check
        return 0;

    index_var = 0;

    /* -------- HEADER -------- */
    orp.pkt_type = get_bits(p, index_var, 4);               //4 bits
    orp.pkt_len = get_bits(p, index_var, 7);                //7 bits
    orp.frame_num = get_bits(p, index_var, 17);             //17 bits
    orp.source_loco_id = get_bits(p, index_var, 20);        //20 bits

    /* =========================================================
     * FRAME 1  ? payload[6–11]
     * ========================================================= */
    orp.source_loco_version = get_bits(p, index_var, 3);    //3 bits
    orp.abs_loco_loc = get_bits(p, index_var, 23);          //23 bits
    orp.l_doubtover = get_bits(p, index_var, 9);            //9 bits
    orp.l_doubtunder = get_bits(p, index_var, 9);           //9 bits
    orp.train_int = get_bits(p, index_var, 2);              //2 bits
    orp.train_length = get_bits(p, index_var, 11);          //11 bits

    /* =========================================================
     * FRAME 2  ? payload[12–17]
     * ========================================================= */
    orp.train_speed = get_bits(p, index_var, 9);            //9 bits
    orp.movement_dir = get_bits(p, index_var, 2);           //2 bits
    orp.emergency_status = get_bits(p, index_var, 3);       //3 bits
    orp.loco_mode = get_bits(p, index_var, 4);              //4 bits
    orp.last_rfid_tag = get_bits(p, index_var, 10);         //10 bits
    orp.tag_dup = get_bits(p, index_var, 1);                //1 bits
    orp.tag_link_info = get_bits(p, index_var, 3);          //3 bits
    orp.tin = get_bits(p, index_var, 9);                    //9 bits

    /* =========================================================
     * FRAME 3  ? payload[18–23]
     * ========================================================= */
    orp.brake_applied = get_bits(p, index_var, 3);          //3 bits
    orp.new_ma_reply = get_bits(p, index_var, 2);           //2 bits
    orp.last_ref_profile_num = get_bits(p, index_var, 4);   //4 bits
    orp.sig_ov = get_bits(p, index_var, 1);                 //1 bits
    orp.info_ack = get_bits(p, index_var, 4);               //4 bits
    orp.spare = get_bits(p, index_var, 2);                  //2 bits
    orp.loco_health_status = get_bits(p, index_var, 6);     //6 bits

    /* =========================================================
     * INSERT PKT_LEN
     * ========================================================= */
    orp.mac_code = get_bits(p, index_var, 32);              //32 bits
    orp.pkt_crc = get_bits(p, index_var, 32);               //32 bits

    g_orp = orp;
    return 1;
}

/* ================= RX ================= */

void radio_rx_handle(uint32_t can_id, uint8_t *data)
{
    (void)can_id;

    /* ===== DEBUG COUNTER ===== */
    radio_rx_isr_count++;   /* increments on every CAN RX */

    uint8_t pkt_type  = (data[0]) & 0x0F;
    uint8_t total_frags = ((data[1] & 0x03) << 4) | ((data[0] & 0xf0) >> 4);
    uint8_t seq_index = (data[1] & 0xfc) >>2;

    if (total_frags == 0 || total_frags > RADIO_MAX_FRAGMENTS)
        return;

    if (seq_index >= total_frags)
        return;

    if (!radio_rx_ctx.active)
    {
        if(seq_index != 0)
            return;

        radio_rx_reset();
        radio_rx_ctx.active     = 1;
        radio_rx_ctx.pkt_type   = pkt_type;
        radio_rx_ctx.seq_total  = total_frags;
        radio_rx_ctx.start_time = seconds_uptime;
    }

    if (radio_rx_ctx.pkt_type != pkt_type)
    {
        radio_rx_reset();
        return;
    }

    if(radio_rx_ctx.received_mask & (1U << seq_index))
        return;

    /* ===== FRAGMENT STORED COUNTER ===== */
    radio_rx_frag_count++;

    uint8_t offset = seq_index * RADIO_PAYLOAD_BYTES;
    //  memset(radio_rx_ctx.payload, 0, sizeof(radio_rx_ctx.payload));

    uint8_t i;
    for(i = 0; i < RADIO_PAYLOAD_BYTES; i++)
    {
        if ((offset + i) < RADIO_MAX_PAYLOAD_LEN)
            radio_rx_ctx.payload[offset + i] = data[2 + i];
    }

    if (seq_index >= 64)
        return;

    radio_rx_ctx.received_mask |= (1U << seq_index);

    uint16_t new_len = offset + RADIO_PAYLOAD_BYTES;
    if (new_len > radio_rx_ctx.payload_len)
    {
        radio_rx_ctx.payload_len = new_len;
    }

    if (radio_rx_ctx.received_mask == ((1U << total_frags) - 1U))
    {
        /* ===== FULL PACKET RECEIVED ===== */
        radio_rx_pkt_count++;
        radio_process_complete_packet();
        radio_rx_ctx.active = 0;
    }
}

// static uint8_t radio_validate_crc(const uint8_t *p,
//                                   uint16_t len,
//                                   uint32_t expected_crc)
// {
//     if(p == NULL)
//         return 0;

//     if(len < RADIO_CRC_SIZE)
//         return 0;

//     uint32_t calc_crc =
//         radio_crc32(p, len - RADIO_CRC_SIZE);

//     return (calc_crc == expected_crc);
// }

//! ===========================================================================
//static uint32_t prev_frame = 0;
//! ===========================================================================
/*
 * ------------------------------------------------------------------------
 * Field Name                Bits     Bit Range
 * ------------------------------------------------------------------------
 * PKT_TYPE                  4        [0   - 3]
 * PKT_LEN                   7        [4   - 10]
 * FRAME_NUM                 17       [11  - 27]
 * SOURCE_STN_ID             16       [28  - 43]
 * VERSION                   3        [44  - 46]
 * STN_ILC_IBS_LOC           23       [47  - 69]
 * DEST_LOCO_ID              20       [70  - 89]
 * ALLOTTED_UPLINK_FREQ      12       [90  - 101]
 * ALLOTTED_DOWNLINK_FREQ    12       [102 - 113]
 * ALLOTTED_TDMA_TIMESLOT    7        [114 - 120]
 * STN_RANDOM_NUMBER         16       [121 - 136]
 * STN_TDMA                  7        [137 - 143]
 * MAC_CODE                  32       [144 - 175]
 * PKT_CRC                   32       [176 - 207]
 * TOTAL                     208 bits = 26 bytes
 * ------------------------------------------------------------------------
 * */
static uint8_t radio_build_aap_payload(uint8_t *buf)
{
    memset(buf, 0, RADIO_MAX_PAYLOAD_LEN);
    uint16_t bit_index = 0;

    /* =========================================================
     * FRAME 0  ? payload[0–5]  ? (Spec Byte 2–7)
     * ========================================================= */

    uint32_t source_stn_id = g_my_stn_id;

    /* PKT_TYPE : 4 bits */
    set_bits(buf, bit_index, 4, RADIO_PKT_TYPE_AAP);
    bit_index += 4;

    /* PKT_LEN : 7 bits (filled later) */
    uint16_t pkt_len_bit_pos = bit_index;
    set_bits(buf, bit_index, 7, 0);
    bit_index += 7;

    /* FRAME_NUM : 17 bits */
    set_bits(buf, bit_index, 17, frame_num);
    bit_index += 17;

    /* SOURCE_STN_ID : 16 bits */
    set_bits(buf, bit_index, 16, source_stn_id);
    bit_index += 16;

    /* =========================================================
     * FRAME 1  ? payload[6–11]
     * ========================================================= */
    uint8_t source_stn_version = KAVACH_VERSION_4_0;
    /* SOURCE_LOCO_VERSION : 3 bits */
    set_bits(buf, bit_index, 3, source_stn_version);
    bit_index += 3;

    uint32_t stn_abs_loc = 0/*radio_get_latest_abs_loc()*/;  //Temp set to 0 TO DO: add variable that will be hardcoded to a certain value
    /* ABS_LOCO_LOC : 23 bits */
    set_bits(buf, bit_index, 23, stn_abs_loc);
    bit_index += 23;

    uint32_t dest_loco_id = 0 /*= g_my_loco_id*/;
    /* DEST_LOCO_ID : 20 bits */
    set_bits(payload, bit_index, 20, dest_loco_id);
    bit_index += 20;

    uint16_t alloted_uplink_freq = 0;  // Set the freq
    /* ALLOTED_UPLINK_FREQ : 12 bits */
    set_bits(buf, bit_index, 12, alloted_uplink_freq);
    bit_index += 12;

    uint16_t alloted_downlink_freq = 0;  // Set the freq
    /* ALLOTED_DOWNLINK_FREQ : 12 bits */
    set_bits(buf, bit_index, 12, alloted_downlink_freq);
    bit_index += 12;

    uint8_t alloted_TDMA_timeslot = 0;  // Set the timeslot
    /* ALLOTED_TDMA_TIMESLOT : 7 bits */
    set_bits(buf, bit_index, 7, alloted_TDMA_timeslot);
    bit_index += 7;

    uint16_t stn_rnd_num_rs = 0;  // Set the num
    /* STATION_RND_NUM_RS : 16 bits */
    set_bits(buf, bit_index, 16, stn_rnd_num_rs);
    bit_index += 16;

    uint8_t stn_TDMA = 0;  // Set the timeslot
    /* STATION_TDMA : 7 bits */
    set_bits(buf, bit_index, 7, stn_TDMA);
    bit_index += 7;

    uint16_t payload_len = (bit_index + 7) / 8;
    /* Insert PKT_LEN back into header */
    set_bits(payload, pkt_len_bit_pos, 7, payload_len);
    return payload_len;
}

void radio_send_aap(radio_id_t radio_id)
{
    uint8_t can_frame[8];
    radio_ctx.payload_len = radio_build_aap_payload(radio_ctx.payload);
    radio_ctx.seq_total = (radio_ctx.payload_len + RADIO_PAYLOAD_BYTES - 1U) / RADIO_PAYLOAD_BYTES;
    if (radio_ctx.seq_total > RADIO_MAX_FRAGMENTS)
        return;

    uint8_t i; // Loop runs 5 times (for 5 frames)
    for ( i = 0; i < radio_ctx.seq_total; i++)
    {
        radio_build_fragment(can_frame,RADIO_PKT_TYPE_AAP,radio_ctx.seq_total,i);
        canTransmit(canREG1, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
        canTransmit(canREG2, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
    }
    radio_info_ack_tx_done();
}

/*
 * ------------------------------------------------------------------------
 * Field Name                Bits     Bit Range
 * ------------------------------------------------------------------------
 * PKT_TYPE                  4        [0   - 3]
 * PKT_LEN                   7        [4   - 10]
 * FRAME_NUM                 17       [11  - 27]
 * SOURCE_STN_ID             16       [28  - 43]
 * VERSION                   3        [44  - 46]
 * STN_ILC_IBS_LOC           23       [47  - 69]
 * GEN_SOS_CALL              1        [70]
 * PADDING                   1        [71]
 * PKT_CRC                   32       [72  - 103]
 * ------------------------------------------------------------------------
 * TOTAL                     104 bits = 13 bytes
 * ------------------------------------------------------------------------
 */

static uint8_t radio_build_aep_payload(uint8_t *buf)
{
    memset(buf, 0, RADIO_MAX_PAYLOAD_LEN);
    uint16_t bit_index = 0;

    /* =========================================================
     * FRAME 0  ? payload[0–5]  ? (Spec Byte 2–7)
     * ========================================================= */

    uint32_t source_stn_id = g_my_stn_id;

    /* PKT_TYPE : 4 bits */
    set_bits(buf, bit_index, 4, RADIO_PKT_TYPE_AAP);
    bit_index += 4;

    /* PKT_LEN : 7 bits (filled later) */
    uint16_t pkt_len_bit_pos = bit_index;
    set_bits(buf, bit_index, 7, 0);
    bit_index += 7;

    /* FRAME_NUM : 17 bits */
    set_bits(buf, bit_index, 17, frame_num);
    bit_index += 17;

    /* SOURCE_STN_ID : 16 bits */
    set_bits(buf, bit_index, 16, source_stn_id);
    bit_index += 16;

    /* =========================================================
     * FRAME 1  ? payload[6–11]
     * ========================================================= */
    uint8_t source_stn_version = KAVACH_VERSION_4_0;

    /* SOURCE_LOCO_VERSION : 3 bits */
    set_bits(buf, bit_index, 3, source_stn_version);
    bit_index += 3;

    uint32_t stn_abs_loc = 0/*radio_get_latest_abs_loc()*/;  //Temp set to 0 TO DO: add variable that will be hardcoded to a certain value

    /* ABS_LOCO_LOC : 23 bits */
    set_bits(buf, bit_index, 23, stn_abs_loc);
    bit_index += 23;

    uint32_t gen_sos_call /*= g_my_loco_id*/;

    /* GENERAL_SOS_CALL : 1 bit */
    set_bits(payload, bit_index, 1, gen_sos_call);
    bit_index += 1;

    uint16_t padding_bits = 1;  // Set the bits
    /* PADDING_BITS : X bits */
    set_bits(buf, bit_index, padding_bits, 0);
    bit_index += padding_bits;

    uint16_t payload_len = (bit_index + 7) / 8;
    /* Insert PKT_LEN back into header */
    set_bits(payload, pkt_len_bit_pos, 7, payload_len);
    return payload_len;
}

void radio_send_aep(radio_id_t radio_id)
{
    uint8_t can_frame[8];
    radio_ctx.payload_len = radio_build_aep_payload(radio_ctx.payload);
    radio_ctx.seq_total = (radio_ctx.payload_len + RADIO_PAYLOAD_BYTES - 1U) / RADIO_PAYLOAD_BYTES;
    if (radio_ctx.seq_total > RADIO_MAX_FRAGMENTS)
        return;

    uint8_t i; // Loop runs 5 times (for 5 frames)
    for ( i = 0; i < radio_ctx.seq_total; i++)
    {
        radio_build_fragment(can_frame,RADIO_PKT_TYPE_AEP,radio_ctx.seq_total,i);
        canTransmit(canREG1, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
        canTransmit(canREG2, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
    }
    radio_info_ack_tx_done();
}

//static uint8_t radio_is_track_profile_valid(void)
//{
//    /* ===== MA ===== */
//    if(reg_type1.MA_Packet_reg_type1.sub_pkt_type == 0 ||
//            reg_type1.MA_Packet_reg_type1.sub_pkt_len  == 0)
//        return 0;
//
//    /* ===== SSP ===== */
//    if(reg_type1.SSP_Packet_reg_type1.sub_pkt_type_ssp == 0 ||
//            reg_type1.SSP_Packet_reg_type1.sub_pkt_len_ssp  == 0)
//        return 0;
//
//    /* ===== GP ===== */
//    if(reg_type1.GP_Packet_reg_type1.sub_pkt_type_grad == 0 ||
//            reg_type1.GP_Packet_reg_type1.sub_pkt_len_grad  == 0)
//        return 0;
//
//    /* ===== LCGP ===== */
//    if(reg_type1.LCGP_Packet_reg_type1.sub_pkt_type_lc == 0 ||
//            reg_type1.LCGP_Packet_reg_type1.sub_pkt_len_lc  == 0)
//        return 0;
//
//    /* ===== TSP ===== */
//    if(reg_type1.TSP_Packet_reg_type1.sub_pkt_type_tsp == 0 ||
//            reg_type1.TSP_Packet_reg_type1.sub_pkt_len_tsp  == 0)
//        return 0;
//
//    /* ===== TLI ===== */
//    if(reg_type1.TLI_Packet_reg_type1.sub_pkt_type_tli == 0 ||
//            reg_type1.TLI_Packet_reg_type1.sub_pkt_len_tli  == 0)
//        return 0;
//
//    /* ===== TCD ===== */
//    if(reg_type1.TCD_Packet_reg_type1.sub_pkt_type_tc == 0 ||
//            reg_type1.TCD_Packet_reg_type1.sub_pkt_len_tc  == 0)
//        return 0;
//
//    /* ===== TSR ===== */
//    if(reg_type1.TSR_Packet_reg_type1.sub_pkt_type_tsr == 0 ||
//            reg_type1.TSR_Packet_reg_type1.sub_pkt_len_tsr  == 0)
//        return 0;
//
//    return 1;
//}

//static float radio_get_tlm_distance(uint32_t event_sec, uint32_t event_ms)
//{
//    dist_array_t *match = NULL;
//
//    uint32_t normalized_ms;
//
//    uint16_t lower_index = 0U;
//    uint16_t upper_index = 0U;
//
//    float d1;
//    float d2;
//
//    uint32_t t1;
//    uint32_t t2;
//
//    float interpolated_distance;
//
//    uint8_t sec_index;
//
//    for (uint8_t i = 0U; i < TLM_HISTORY_SECONDS; i++)
//    {
//        if (distance_db[i].tod_sec == event_sec)
//        {
//            match = &distance_db[i];
//            sec_index = i;
//            break;
//        }
//    }
//
//    if (match == NULL)
//        return 0U;
//
//    normalized_ms = (event_ms * (match->tod_ms[match->sec_span_ms])) / 1000U;
//
//    if(match->tod_ms[0] > normalized_ms)
//    {
//        d1 = distance_db[sec_index - 1].distance_odo[(distance_db[sec_index - 1].sec_span_ms) - 1];
//        d2 = distance_db[sec_index].distance_odo[0];
//
//        t1 = distance_db[sec_index - 1].tod_ms[(distance_db[sec_index - 1].sec_span_ms) - 1];
//        t2 = distance_db[sec_index].tod_ms[0];
//    }
//    else if(normalized_ms > match->tod_ms[(match->sec_span_ms) - 1])
//    {
//        d1 = distance_db[sec_index].distance_odo[(distance_db[sec_index].sec_span_ms) - 1];
//        d2 = distance_db[sec_index + 1].distance_odo[0];
//
//        t1 = distance_db[sec_index].tod_ms[(distance_db[sec_index].sec_span_ms) - 1];
//        t2 = distance_db[sec_index + 1].tod_ms[0];
//    }
//    else
//    {
//        for(uint16_t i = 0U; i < ( match->sec_span_ms - 1U); i++)
//        {
//            //Valid for event_ms range 10ms and 990ms - Not valid for event_ms ~= 0ms and event_ms ~= 1000ms
//            if(match->tod_ms[i] == normalized_ms)
//            {
//                lower_index = i;
//                upper_index = i;
//                break;
//            }
//            else if(match->tod_ms[i + 1U] == normalized_ms)
//            {
//                lower_index = i + 1U;
//                upper_index = i + 1U;
//                break;
//            }
//            else if ((match->tod_ms[i] < normalized_ms) && (normalized_ms < match->tod_ms[i + 1U]))
//            {
//                lower_index = i;
//                upper_index = i + 1U;
//                break;
//            }
//        }
//
//        d1 = match->distance_odo[lower_index];
//        d2 = match->distance_odo[upper_index];
//
//        t1 = match->tod_ms[lower_index];
//        t2 = match->tod_ms[upper_index];
//    }
//
//    if (t2 == t1)
//        return d1;
//
//    interpolated_distance = d1 + (((normalized_ms - t1) * (d2 - d1)) / (t2 - t1));
//
//    return interpolated_distance;
//}

//static void radio_process_tlm_type1(void)
//{
//    uint32_t event_sec;
//    uint32_t event_ms;
//
//    float interpolated_distance;
//
//    if (reg_type1.MA_Packet_reg_type1.trn_len_info_sts != 1U)
//    {
//        return;
//    }
//
//    event_sec = (reg_type1.MA_Packet_reg_type1.ref_frame_num_tlm - 1U);
//
//    event_ms = reg_type1.MA_Packet_reg_type1.ref_offset_int_tlm * 10U;
//
//    interpolated_distance = radio_get_tlm_distance(event_sec, event_ms);
//
//    /* START */
//    if (reg_type1.MA_Packet_reg_type1.trn_len_info_type == 0U)
//    {
//        tlm_start_distance = interpolated_distance;
//
//        tlm_start_valid = 1U;
//
//        tlm_active = 1;
//
//        radio_info_ack_push(INFO_ACK_TLM_START);
//    }
//    else
//    {
//        /* END */
//
//        if (tlm_start_valid == 1U)
//        {
//            tlm_end_distance = interpolated_distance;
//
//            if (tlm_end_distance > tlm_start_distance)
//            {
//                calculated_train_length = tlm_end_distance - tlm_start_distance;
//            }
//
//            tlm_start_valid = 0U;
//
//            tlm_active = 0;
//
//            radio_info_ack_push(INFO_ACK_TLM_END);
//        }
//    }
//}

//static void radio_process_tlm_type2(void)
//{
//    uint32_t event_sec;
//    uint32_t event_ms;
//    uint32_t interpolated_distance;
//
//    if (reg_type2.MA_Packet_reg_type2.trn_len_info_sts != 1U)
//    {
//        return;
//    }
//
//    event_sec = (reg_type2.MA_Packet_reg_type2.ref_frame_num_tlm - 1U) / 2U;
//    event_ms = reg_type2.MA_Packet_reg_type2.ref_offset_int_tlm * 10U;
//    interpolated_distance = radio_get_tlm_distance(event_sec, event_ms);
//    if (reg_type2.MA_Packet_reg_type2.trn_len_info_type == 0U)
//    {
//        tlm_start_distance = interpolated_distance;
//
//        tlm_start_valid = 1U;
//
//        tlm_active = 1;
//
//        radio_info_ack_push(INFO_ACK_TLM_START);
//    }
//    else
//    {
//        if (tlm_start_valid == 1U)
//        {
//            tlm_end_distance = interpolated_distance;
//            if (tlm_end_distance > tlm_start_distance)
//            {
//                calculated_train_length = tlm_end_distance - tlm_start_distance;
//            }
//            tlm_start_valid = 0U;
//            tlm_active = 0;
//            radio_info_ack_push(INFO_ACK_TLM_END);
//        }
//    }
//}

uint8_t result;
static void radio_process_complete_packet(void)
{
//    distance_travelled = (int32_t)distance_m - (int32_t)odo_distance_radio_rfid_ref;

    switch (radio_rx_ctx.pkt_type)
    {
    case RADIO_PKT_TYPE_ORP:
        result = radio_parse_orp(radio_rx_ctx.payload, radio_rx_ctx.payload_len);
        break;

    case RADIO_PKT_TYPE_ARP:
        result = radio_parse_arp(radio_rx_ctx.payload, radio_rx_ctx.payload_len);
        break;

    default:
        break;
    }
}

static void radio_rx_reset(void)
{
    radio_rx_ctx.active = 0;
    radio_rx_ctx.received_mask = 0;
    radio_rx_ctx.payload_len = 0;
}

/* ================= TIMEOUT ================= */

void radio_rx_poll_1s(void)
{
    if (radio_rx_ctx.active && (seconds_uptime - radio_rx_ctx.start_time) > 1U)
    {
        radio_rx_reset();
    }
}

static uint8_t radio_build_reg_type1_payload(uint8_t *buf)
{
    memset(buf, 0, RADIO_MAX_PAYLOAD_LEN);
    uint16_t bit_index = 0;

    /* =========================================================
     * FRAME 0  ? payload[0–5]  ? (Spec Byte 2–7)
     * ========================================================= */

    uint32_t source_stn_id = g_my_stn_id;

    /* PKT_TYPE : 4 bits */
    set_bits(buf, bit_index, 4, RADIO_PKT_TYPE_AAP);
    bit_index += 4;

    /* PKT_LEN : 7 bits (filled later) */
    uint16_t pkt_len_bit_pos = bit_index;
    set_bits(buf, bit_index, 7, 0);
    bit_index += 7;

    /* FRAME_NUM : 17 bits */
    set_bits(buf, bit_index, 17, frame_num);
    bit_index += 17;

    /* SOURCE_STN_ID : 16 bits */
    set_bits(buf, bit_index, 16, source_stn_id);
    bit_index += 16;

    /* =========================================================
     * FRAME 1  ? payload[6–11]
     * ========================================================= */
    uint8_t source_stn_version = KAVACH_VERSION_4_0;
    /* SOURCE_LOCO_VERSION : 3 bits */
    set_bits(buf, bit_index, 3, source_stn_version);
    bit_index += 3;

    uint32_t dest_loco_id = 0 /*= g_my_loco_id*/;
    /* DEST_LOCO_ID : 20 bits */
    set_bits(payload, bit_index, 20, dest_loco_id);
    bit_index += 20;

    uint8_t ref_prof_id = 0 /*= g_my_loco_id*/;
    /* REF_PROF_ID (4 bits) */
    set_bits(payload, bit_index, 4, ref_prof_id);
    bit_index += 4;

    uint16_t last_ref_rfid = 0 /*= g_my_loco_id*/;
    /* LAST_REF_RFID (10 bits) */
    set_bits(payload, bit_index, 10, last_ref_rfid);
    bit_index += 10;

    uint16_t dist_pkt_start = 0 /*= g_my_loco_id*/;
    /* DIST_PKT_START (15 bits) */
    set_bits(payload, bit_index, 15, dist_pkt_start);
    bit_index += 15;

    uint8_t pkt_dir = 0 /*= g_my_loco_id*/;
    /* PKT_DIR (2 bits) */
    set_bits(payload, bit_index, 2, pkt_dir);
    bit_index += 2;

    uint8_t padding_bits = 3;  // Set the bits
    /* PADDING_BITS : X bits */
    set_bits(buf, bit_index, padding_bits, 0);
    bit_index += padding_bits;

    uint16_t MA_pkt_start = bit_index;
    /*MA Packet Parsing*/
    uint8_t sub_pkt_type /*= g_my_loco_id*/;
    /* SUB_PKT_TYPE (4 bits) */
    set_bits(payload, bit_index, 4, sub_pkt_type);
    bit_index += 4;

    uint8_t sub_pkt_len = 0 /*= g_my_loco_id*/;
    /* SUB_PKT_LEN (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len);
    bit_index += 7;

    uint8_t frame_offset = 0 /*= g_my_loco_id*/;
    /* FRAME_OFFSET (4 bits) */
    set_bits(payload, bit_index, 4, frame_offset);
    bit_index += 4;

    uint8_t dest_loco_sos = 0 /*= g_my_loco_id*/;
    /* DEST_LOCO_SOS (1 + 3 = 4 bits) */
    set_bits(payload, bit_index, 4, dest_loco_sos);
    bit_index += 4;

    uint8_t train_section_type = 0 /*= g_my_loco_id*/;
    /* TRAIN_SECTION_TYPE (2 bits) */
    set_bits(payload, bit_index, 2, train_section_type);
    bit_index += 2;

    uint32_t cur_sig_info = 0 /*= g_my_loco_id*/;
    /* ================= CUR_SIG_INFO (17 bits) ================= */
    set_bits(payload, bit_index, 17, cur_sig_info);
    bit_index += 17;

    uint8_t cur_sig_aspect = 0 /*= g_my_loco_id*/;
    /* CUR_SIG_ASPECT (complete 2 + 4 = 6 bits) */
    set_bits(payload, bit_index, 6, cur_sig_aspect);
    bit_index += 6;

    uint8_t next_sig_aspect = 0 /*= g_my_loco_id*/;
    /* NEXT_SIG_ASPECT (4 + 2 = 6 bits) */
    set_bits(payload, bit_index, 6, next_sig_aspect);
    bit_index += 6;

    uint16_t appr_sig_dist = 0 /*= g_my_loco_id*/;
    /* APPR_SIG_DIST (6 + 8 + 1 = 15 bits) */
    set_bits(payload, bit_index, 15, appr_sig_dist);
    bit_index += 15;

    uint8_t authority_type = 0 /*= g_my_loco_id*/;
    /* AUTHORITY_TYPE (2 bits) */
    set_bits(payload, bit_index, 2, authority_type);
    bit_index += 2;

    /* AUTHORIZED_SPEED (6 bits) */
    if(authority_type == 1)
    {
        uint8_t authorized_speed = 0 /*= g_my_loco_id*/;
        set_bits(payload, bit_index, 6, authorized_speed);
        bit_index += 6;
    }

    uint16_t ma_wrt_sig = 0 /*= g_my_loco_id*/;
    /* MA_W_R_T_SIG (16 bits) */
    set_bits(payload, bit_index, 16, ma_wrt_sig);
    bit_index += 16;

    uint8_t req_shorten_ma = 0 /*= g_my_loco_id*/;
    /* REQ_SHORTEN_MA (1 bit) */
    set_bits(payload, bit_index, 1, req_shorten_ma);
    bit_index += 1;

    /* NEW_MA (16 bits) */
    if(req_shorten_ma == 1)
    {
        uint16_t new_ma = 0 /*= g_my_loco_id*/;
        set_bits(payload, bit_index, 16, new_ma);
        bit_index += 16;
    }

    uint8_t trn_len_info_sts = 0 /*= g_my_loco_id*/;
    /* TRAIN_LENGTH_INFO_STS (1 bit)*/
    set_bits(payload, bit_index, 1, trn_len_info_sts);
    bit_index += 1;

    if(trn_len_info_sts == 1)
    {
        uint8_t trn_len_info_type = 0 /*= g_my_loco_id*/;
        /*TRAIN_LENGTH_INFO_TYPE (1 bit)*/
        set_bits(payload, bit_index, 1, trn_len_info_type);
        bit_index += 1;

        uint32_t ref_frame_num_tlm = 0 /*= g_my_loco_id*/;
        /* REF_FRAME_NUM_TLM (17 bits) */
        set_bits(payload, bit_index, 17, ref_frame_num_tlm);
        bit_index += 17;

        uint8_t ref_offset_int_tlm = 0 /*= g_my_loco_id*/;
        /* REF_OFFSET_INT_TLM (8 bits) */
        set_bits(payload, bit_index, 8, ref_offset_int_tlm);
        bit_index += 8;
    }

    uint8_t next_stn_comm = 0 /*= g_my_loco_id*/;
    /* NEXT_STN_COMM (1 bit) */
    set_bits(payload, bit_index, 1, next_stn_comm);
    bit_index += 1;


    /* APPR_STN_ID (16 bits) */
    if(next_stn_comm == 1)
    {
        uint16_t appr_stn_id = 0 /*= g_my_loco_id*/;
        set_bits(payload, bit_index, 16, appr_stn_id);
        bit_index += 16;
    }

    if((bit_index - MA_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - MA_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*SSP Packet Parsing*/
    uint16_t SSP_pkt_start = bit_index;

    uint8_t sub_pkt_type_ssp = 0;
    /* SUB_PKT_TYPE (SSP) (4 bits) */
    set_bits(payload, bit_index, 4, sub_pkt_type_ssp);
    bit_index += 4;

    uint8_t sub_pkt_len_ssp = 0;
    /* SUB_PKT_LEN (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_ssp);
    bit_index += 7;

    uint8_t lm_speed_info_cnt = 0;
    /* LM_Speed_Info_CNT (5 bits) */
    set_bits(payload, bit_index, 5, lm_speed_info_cnt);
    bit_index += 5;

    for(uint8_t i = 0; i < lm_speed_info_cnt; i++)
    {
        uint16_t lm_static_speed_dist = 0;
        /* LM_Static_Speed_Distance (15 bits) */
        set_bits(payload, bit_index, 15, lm_static_speed_dist);
        bit_index += 15;

        uint8_t lm_static_speed_class = 0;
        /* LM_Static_Speed_Class (1 bit) */
        set_bits(payload, bit_index, 1, lm_static_speed_class);
        bit_index += 1;

        /* ================= SPEED VALUES ================= */
        if(lm_static_speed_class == 0)
        {
            uint8_t lm_speed_universal = 0;
            /* Universal Speed (6 bits) */
            set_bits(payload, bit_index, 6, lm_speed_universal);
            bit_index += 6;
        }
        else if(lm_static_speed_class == 1)
        {
            uint8_t lm_speed_class_a = 0;
            /* Class A (6 bits) */
            set_bits(payload, bit_index, 6, lm_speed_class_a);
            bit_index += 6;

            uint8_t lm_speed_class_b = 0;
            /* Class B (6 bits) */
            set_bits(payload, bit_index, 6, lm_speed_class_b);
            bit_index += 6;

            uint8_t lm_speed_class_c = 0;
            /* Class C (6 bits) */
            set_bits(payload, bit_index, 6, lm_speed_class_c);
            bit_index += 6;
        }
    }

    if((bit_index - SSP_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - SSP_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*GP Packet Parsing*/
    uint16_t GP_pkt_start = bit_index;

    uint8_t sub_pkt_type_grad = 0;
    /* SUB_PKT_TYPE (GRAD) (4 bits)*/
    set_bits(payload, bit_index, 4, sub_pkt_type_grad);
    bit_index += 4;

    uint8_t sub_pkt_len_grad = 0;
    /* SUB_PKT_LEN_GRAD (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_grad);
    bit_index += 7;

    uint8_t lm_grad_info_cnt = 0;
    /* LM_Grad_Info_CNT (5 bits) */
    set_bits(payload, bit_index, 5, lm_grad_info_cnt);
    bit_index += 5;

    for(uint8_t i = 0; i < lm_grad_info_cnt; i++)
    {
        uint16_t lm_gradient_distance = 0;
        /* LM_Gradient_Distance (15 bits) */
        set_bits(payload, bit_index, 15, lm_gradient_distance);
        bit_index += 15;

        uint8_t lm_gdir = 0;
        /* LM_GDIR (1 bit) */
        set_bits(payload, bit_index, 1, lm_gdir);
        bit_index += 1;

        uint8_t lm_gradient_value = 0;
        /* LM_GRADIENT_VALUE (5 bits) */
        set_bits(payload, bit_index, 5, lm_gradient_value);
        bit_index += 5;
    }

    /* Padding Bits (X bits) */
    if((bit_index - GP_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - GP_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*LCGP Packet Parsing*/
    uint16_t LCGP_pkt_start = bit_index;

    uint8_t sub_pkt_type_lc = 0;
    /* SUB_PKT_TYPE (LC) (4 bits)*/
    set_bits(payload, bit_index, 4, sub_pkt_type_lc);
    bit_index += 4;

    uint8_t sub_pkt_len_lc = 0;
    /* SUB_PKT_LEN_LC (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_lc);
    bit_index += 7;

    uint8_t lm_lc_info_cnt = 0;
    /* LM_LC_Info_CNT (5 bits) */
    set_bits(payload, bit_index, 5, lm_lc_info_cnt);
    bit_index += 5;

    for(uint8_t i = 0; i < lm_lc_info_cnt; i++)
    {
        uint16_t lm_lc_distance = 0;
        /* LM_LC_Distance (15 bits) */
        set_bits(payload, bit_index, 15, lm_lc_distance);
        bit_index += 15;

        uint16_t lm_lc_id_numeric = 0;
        /* LM_LC_ID_Numeric (10 bits) */
        set_bits(payload, bit_index, 10, lm_lc_id_numeric);
        bit_index += 10;

        uint8_t lm_lc_id_alpha_suffix = 0;
        /* Alpha suffix (3 bits) */
        set_bits(payload, bit_index, 3, lm_lc_id_alpha_suffix);
        bit_index += 3;

        uint8_t lm_lc_manning_type = 0;
        /* Manning type (1 bit)*/
        set_bits(payload, bit_index, 1, lm_lc_manning_type);
        bit_index += 1;

        uint8_t lm_lc_class = 0;
        /* LC class (3 bits)*/
        set_bits(payload, bit_index, 3, lm_lc_class);
        bit_index += 3;

        uint8_t lm_lc_auto_whistle_en = 0;
        /* LM LC Auto whistle Enabled (1 bit)*/
        set_bits(payload, bit_index, 1, lm_lc_auto_whistle_en);
        bit_index += 1;

        if(lm_lc_auto_whistle_en == 1)
        {
            uint8_t lm_lc_auto_whistle_type = 0;
            /* LM LC Auto whistle Type(2 bit)*/
            set_bits(payload, bit_index, 2, lm_lc_auto_whistle_type);
            bit_index += 2;
        }
    }

    /* Padding Bits (X bits) */
    if((bit_index - LCGP_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - LCGP_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*TSP Packet Parsing*/
    uint16_t TSP_pkt_start = bit_index;

    uint8_t sub_pkt_type_tsp = 0;
    /* SUB_PKT_TYPE (TSP) (4 bits)*/
    set_bits(payload, bit_index, 4, sub_pkt_type_tsp);
    bit_index += 4;

    uint8_t sub_pkt_len_tsp = 0;
    /* SUB_PKT_LEN_TSP (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_tsp);
    bit_index += 7;

    uint8_t to_cnt = 0;
    /* TO_CNT (2 bits) */
    set_bits(payload, bit_index, 2, to_cnt);
    bit_index += 2;

    for(uint8_t i = 0; i < to_cnt; i++)
    {
        uint8_t to_speed = 0;
        /* TO_SPEED (5 bits) */
        set_bits(payload, bit_index, 5, to_speed);
        bit_index += 5;

        uint16_t diff_dist_to = 0;
        /* DIFF_DIST_TO (15 bits) */
        set_bits(payload, bit_index, 15, diff_dist_to);
        bit_index += 15;

        uint16_t to_speed_rel_dist = 0;
        /* TO_SPEED_REL_DIST (12 bits) */
        set_bits(payload, bit_index, 12, to_speed_rel_dist);
        bit_index += 12;
    }

    /* Padding Bits (X bits) */
    if((bit_index - TSP_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - TSP_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*TLI Packet Parsing*/
    uint16_t TLI_pkt_start = bit_index;

    uint8_t sub_pkt_type_tli = 0;
    /* SUB_PKT_TYPE (TLI) (4 bits)*/
    set_bits(payload, bit_index, 4, sub_pkt_type_tli);
    bit_index += 4;

    uint8_t sub_pkt_len_tli = 0;
    /* SUB_PKT_LEN_TLI (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_tli);
    bit_index += 7;

    uint8_t dist_dup_tag = 0;
    /* DIST_DUP_TAG (4 bits) */
    set_bits(payload, bit_index, 4, dist_dup_tag);
    bit_index += 4;

    uint8_t route_rfid_cnt = 0;
    /* ROUTE_RFID_CNT (6 bits) */
    set_bits(payload, bit_index, 6, route_rfid_cnt);
    bit_index += 6;

//    if(route_rfid_cnt > 0)
//        rfid_Count = 0; // used in rfid_rx.c file

    for(uint8_t i = 0; i < route_rfid_cnt; i++)
    {
        uint16_t dist_nxt_rfid;
        /* DIST_NXT_RFID (11 bits) */
        set_bits(payload, bit_index, 11, dist_nxt_rfid);
        bit_index += 11;

        uint16_t nxt_rfid_tag_id;
        /* NXT_RFID_TAG_ID (10 bits) */
        set_bits(payload, bit_index, 10, nxt_rfid_tag_id);
        bit_index += 10;

        uint8_t dup_tag_dir;
        /* DUP_TAG_DIR (1 bit)*/
        set_bits(payload, bit_index, 1, dup_tag_dir);
        bit_index += 1;
    }

    uint8_t abs_loc_reset = 0;
    /* ABS_LOC_RESET (1 bit)*/
    set_bits(payload, bit_index, 1, abs_loc_reset);
    bit_index += 1;

    if(abs_loc_reset == 1)
    {
        uint16_t start_dist_loc_reset = 0;
        /* START_DIST_TO_LOC_RESET (15 bits) */
        set_bits(payload, bit_index, 15, start_dist_loc_reset);
        bit_index += 15;

        uint8_t adj_loco_dir = 0;
        /* ADJ_LOCO_DIR (2 bits) */
        set_bits(payload, bit_index, 2, adj_loco_dir);
        bit_index += 2;

        uint32_t abs_loc_correction = 0;
        /* ABS_LOC_CORRECTION (23 bits) */
        set_bits(payload, bit_index, 23, abs_loc_correction);
        bit_index += 23;
    }

    uint8_t adj_line_cnt = 0;
    /* ADJ_LINE_CNT (3 bits) */
    set_bits(payload, bit_index, 3, adj_line_cnt);
    bit_index += 3;

    for(uint8_t i = 0; i <= adj_line_cnt; i++)
    {
        uint16_t line_tin;
        /* LINE_TIN (9 bits) */
        set_bits(payload, bit_index, 9, line_tin);
        bit_index += 9;
    }

    /* Padding Bits (X bits) */
    if((bit_index - TLI_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - TLI_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*TCD Packet Parsing*/
    uint16_t TCD_pkt_start = bit_index;

    uint8_t sub_pkt_type_tc;
    /* SUB_PKT_TYPE (TC) (4 bits)*/
    set_bits(payload, bit_index, 4, sub_pkt_type_tc);
    bit_index += 4;

    uint8_t sub_pkt_len_tc;
    /* SUB_PKT_LEN_TC (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_tc);
    bit_index += 7;

    uint8_t trackcond_cnt;
    /* TRACKCOND_CNT (4 bits) */
    set_bits(payload, bit_index, 4, trackcond_cnt);
    bit_index += 4;

    for(uint8_t i = 0; i < trackcond_cnt; i++)
    {
        uint8_t trackcond_type;
        /* TRACKCOND_TYPE (4 bits) */
        set_bits(payload, bit_index, 4, trackcond_type);
        bit_index += 4;

        uint16_t start_dist_trackcond;
        /* START_DIST_TRACKCOND (15 bits) */
        set_bits(payload, bit_index, 15, start_dist_trackcond);
        bit_index += 15;

        uint16_t length_trackcond;
        /* LENGTH_TRACKCOND (15 bits) */
        set_bits(payload, bit_index, 15, length_trackcond);
        bit_index += 15;
    }

    /* Padding Bits (X bits) */
    if((bit_index - TCD_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - TCD_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    /*TSR Packet Parsing*/
    uint16_t TSR_pkt_start = bit_index;

    uint8_t sub_pkt_type_tsr;
    /* SUB_PKT_TYPE (TSR) (4 bits)*/
    set_bits(payload, bit_index, 4, sub_pkt_type_tsr);
    bit_index += 4;

    uint8_t sub_pkt_len_tsr;
    /* SUB_PKT_LEN_TSR (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len_tsr);
    bit_index += 7;

    uint8_t tsr_status;
    /* TSR_STATUS (2 bits) */
    set_bits(payload, bit_index, 2, tsr_status);
    bit_index += 2;

    uint8_t tsr_info_cnt;
    /* TSR_Info_CNT (5 bits) */
    set_bits(payload, bit_index, 5, tsr_info_cnt);
    bit_index += 5;

    for(uint8_t i = 0; i < tsr_info_cnt; i++)
    {
        uint8_t tsr_id;
        /* TSR_ID (8 bits) */
        set_bits(payload, bit_index, 8, tsr_id);
        bit_index += 8;

        uint16_t tsr_distance;
        /* TSR_DISTANCE (15 bits) */
        set_bits(payload, bit_index, 15, tsr_distance);
        bit_index += 15;

        uint16_t tsr_length;
        /* TSR_LENGTH (15 bits) */
        set_bits(payload, bit_index, 15, tsr_length);
        bit_index += 15;

        uint8_t tsr_class;
        /* TSR_CLASS (1 bit)*/
        set_bits(payload, bit_index, 1, tsr_class);
        bit_index += 1;

        /* SPEEDS (6 bits each)*/
        if(tsr_class == 0)
        {
            uint8_t tsr_universal_speed;
            set_bits(payload, bit_index, 6, tsr_universal_speed);
            bit_index += 6;
        }
        else if(tsr_class == 1)
        {
            uint8_t tsr_class_a_speed;
            set_bits(payload, bit_index, 6, tsr_class_a_speed);
            bit_index += 6;

            uint8_t tsr_class_b_speed;
            set_bits(payload, bit_index, 6, tsr_class_b_speed);
            bit_index += 6;

            uint8_t tsr_class_c_speed;
            set_bits(payload, bit_index, 6, tsr_class_c_speed);
            bit_index += 6;
        }

        uint8_t tsr_info_cnt;
        /* TSR_WHISTLE (2 bits) */
        set_bits(payload, bit_index, 2, tsr_info_cnt);
        bit_index += 2;
    }

    /* Padding Bits (X bits) */
    if((bit_index - TSR_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - TSR_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    uint16_t payload_len = (bit_index + 7) / 8;
    /* Insert PKT_LEN back into header */
    set_bits(payload, pkt_len_bit_pos, 7, payload_len);
    return payload_len;
}

void radio_send_reg_type1(radio_id_t radio_id)
{
    uint8_t can_frame[8];
    radio_ctx.payload_len = radio_build_reg_type1_payload(radio_ctx.payload);
    radio_ctx.seq_total = (radio_ctx.payload_len + RADIO_PAYLOAD_BYTES - 1U) / RADIO_PAYLOAD_BYTES;
    if (radio_ctx.seq_total > RADIO_MAX_FRAGMENTS)
        return;

    uint8_t i; // Loop runs 5 times (for 5 frames)
    for ( i = 0; i < radio_ctx.seq_total; i++)
    {
        radio_build_fragment(can_frame,RADIO_PKT_TYPE_REG_TYPE1,radio_ctx.seq_total,i);
        canTransmit(canREG1, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
        canTransmit(canREG2, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
    }
    radio_info_ack_tx_done();
}

static uint8_t radio_build_reg_type2_payload(uint8_t *buf)
{
    memset(buf, 0, RADIO_MAX_PAYLOAD_LEN);
    uint16_t bit_index = 0;

    /* =========================================================
     * FRAME 0  ? payload[0–5]  ? (Spec Byte 2–7)
     * ========================================================= */

    uint32_t source_stn_id = g_my_stn_id;

    /* PKT_TYPE : 4 bits */
    set_bits(buf, bit_index, 4, RADIO_PKT_TYPE_AAP);
    bit_index += 4;

    /* PKT_LEN : 7 bits (filled later) */
    uint16_t pkt_len_bit_pos = bit_index;
    set_bits(buf, bit_index, 7, 0);
    bit_index += 7;

    /* FRAME_NUM : 17 bits */
    set_bits(buf, bit_index, 17, frame_num);
    bit_index += 17;

    /* SOURCE_STN_ID : 16 bits */
    set_bits(buf, bit_index, 16, source_stn_id);
    bit_index += 16;

    /* =========================================================
     * FRAME 1  ? payload[6–11]
     * ========================================================= */
    uint8_t source_stn_version = KAVACH_VERSION_4_0;

    /* SOURCE_LOCO_VERSION : 3 bits */
    set_bits(buf, bit_index, 3, source_stn_version);
    bit_index += 3;

    uint32_t dest_loco_id /*= g_my_loco_id*/;
    /* DEST_LOCO_ID : 20 bits */
    set_bits(payload, bit_index, 20, dest_loco_id);
    bit_index += 20;

    uint8_t ref_prof_id /*= g_my_loco_id*/;
    /* REF_PROF_ID (4 bits) */
    set_bits(payload, bit_index, 4, ref_prof_id);
    bit_index += 4;

    uint16_t last_ref_rfid /*= g_my_loco_id*/;
    /* LAST_REF_RFID (10 bits) */
    set_bits(payload, bit_index, 10, last_ref_rfid);
    bit_index += 10;

    uint16_t dist_pkt_start /*= g_my_loco_id*/;
    /* DIST_PKT_START (15 bits) */
    set_bits(payload, bit_index, 15, dist_pkt_start);
    bit_index += 15;

    uint8_t pkt_dir /*= g_my_loco_id*/;
    /* PKT_DIR (2 bits) */
    set_bits(payload, bit_index, 2, pkt_dir);
    bit_index += 2;

    uint8_t padding_bits = 3;  // Set the bits
    /* PADDING_BITS : X bits */
    set_bits(buf, bit_index, padding_bits, 0);
    bit_index += padding_bits;

    uint16_t MA_pkt_start = bit_index;
    /*MA Packet Parsing*/
    uint8_t sub_pkt_type /*= g_my_loco_id*/;
    /* SUB_PKT_TYPE (4 bits) */
    set_bits(payload, bit_index, 4, sub_pkt_type);
    bit_index += 4;

    uint8_t sub_pkt_len = 0 /*= g_my_loco_id*/;
    /* SUB_PKT_LEN (7 bits) */
    set_bits(payload, bit_index, 7, sub_pkt_len);
    bit_index += 7;

    uint8_t frame_offset = 0 /*= g_my_loco_id*/;
    /* FRAME_OFFSET (4 bits) */
    set_bits(payload, bit_index, 4, frame_offset);
    bit_index += 4;

    uint8_t dest_loco_sos = 0 /*= g_my_loco_id*/;
    /* DEST_LOCO_SOS (1 + 3 = 4 bits) */
    set_bits(payload, bit_index, 4, dest_loco_sos);
    bit_index += 4;

    uint8_t train_section_type = 0 /*= g_my_loco_id*/;
    /* TRAIN_SECTION_TYPE (2 bits) */
    set_bits(payload, bit_index, 2, train_section_type);
    bit_index += 2;

    uint32_t cur_sig_info = 0 /*= g_my_loco_id*/;
    /* ================= CUR_SIG_INFO (17 bits) ================= */
    set_bits(payload, bit_index, 17, cur_sig_info);
    bit_index += 17;

    uint8_t cur_sig_aspect = 0 /*= g_my_loco_id*/;
    /* CUR_SIG_ASPECT (complete 2 + 4 = 6 bits) */
    set_bits(payload, bit_index, 6, cur_sig_aspect);
    bit_index += 6;

    uint8_t next_sig_aspect = 0 /*= g_my_loco_id*/;
    /* NEXT_SIG_ASPECT (4 + 2 = 6 bits) */
    set_bits(payload, bit_index, 6, next_sig_aspect);
    bit_index += 6;

    uint16_t appr_sig_dist = 0 /*= g_my_loco_id*/;
    /* APPR_SIG_DIST (6 + 8 + 1 = 15 bits) */
    set_bits(payload, bit_index, 15, appr_sig_dist);
    bit_index += 15;

    uint8_t authority_type = 0 /*= g_my_loco_id*/;
    /* AUTHORITY_TYPE (2 bits) */
    set_bits(payload, bit_index, 2, authority_type);
    bit_index += 2;

    /* AUTHORIZED_SPEED (6 bits) */
    if(authority_type == 1)
    {
        uint8_t authorized_speed = 0 /*= g_my_loco_id*/;
        set_bits(payload, bit_index, 6, authorized_speed);
        bit_index += 6;
    }

    uint16_t ma_wrt_sig = 0 /*= g_my_loco_id*/;
    /* MA_W_R_T_SIG (16 bits) */
    set_bits(payload, bit_index, 16, ma_wrt_sig);
    bit_index += 16;

    uint8_t req_shorten_ma = 0 /*= g_my_loco_id*/;
    /* REQ_SHORTEN_MA (1 bit) */
    set_bits(payload, bit_index, 1, req_shorten_ma);
    bit_index += 1;

    /* NEW_MA (16 bits) */
    if(req_shorten_ma == 1)
    {
        uint16_t new_ma = 0 /*= g_my_loco_id*/;
        set_bits(payload, bit_index, 16, new_ma);
        bit_index += 16;
    }

    uint8_t trn_len_info_sts = 0 /*= g_my_loco_id*/;
    /* TRAIN_LENGTH_INFO_STS (1 bit)*/
    set_bits(payload, bit_index, 1, trn_len_info_sts);
    bit_index += 1;

    if(trn_len_info_sts == 1)
    {
        uint8_t trn_len_info_type = 0 /*= g_my_loco_id*/;
        /*TRAIN_LENGTH_INFO_TYPE (1 bit)*/
        set_bits(payload, bit_index, 1, trn_len_info_type);
        bit_index += 1;

        uint32_t ref_frame_num_tlm = 0 /*= g_my_loco_id*/;
        /* REF_FRAME_NUM_TLM (17 bits) */
        set_bits(payload, bit_index, 17, ref_frame_num_tlm);
        bit_index += 17;

        uint8_t ref_offset_int_tlm = 0 /*= g_my_loco_id*/;
        /* REF_OFFSET_INT_TLM (8 bits) */
        set_bits(payload, bit_index, 8, ref_offset_int_tlm);
        bit_index += 8;
    }

    uint8_t next_stn_comm = 0 /*= g_my_loco_id*/;
    /* NEXT_STN_COMM (1 bit) */
    set_bits(payload, bit_index, 1, next_stn_comm);
    bit_index += 1;


    /* APPR_STN_ID (16 bits) */
    if(next_stn_comm == 1)
    {
        uint16_t appr_stn_id = 0 /*= g_my_loco_id*/;
        set_bits(payload, bit_index, 16, appr_stn_id);
        bit_index += 16;
    }

    if((bit_index - MA_pkt_start) % 8)
    {
        uint8_t padding_bits = 8 - ((bit_index - MA_pkt_start) % 8);  // Set the bits
        /* PADDING_BITS : X bits */
        set_bits(buf, bit_index, padding_bits, 0);
        bit_index += padding_bits;
    }

    uint16_t payload_len = (bit_index + 7) / 8;
    /* Insert PKT_LEN back into header */
    set_bits(payload, pkt_len_bit_pos, 7, payload_len);
    return payload_len;
}

void radio_send_reg_type2(radio_id_t radio_id)
{
    uint8_t can_frame[8];
    radio_ctx.payload_len = radio_build_reg_type2_payload(radio_ctx.payload);
    radio_ctx.seq_total = (radio_ctx.payload_len + RADIO_PAYLOAD_BYTES - 1U) / RADIO_PAYLOAD_BYTES;
    if (radio_ctx.seq_total > RADIO_MAX_FRAGMENTS)
        return;

    uint8_t i; // Loop runs 5 times (for 5 frames)
    for ( i = 0; i < radio_ctx.seq_total; i++)
    {
        radio_build_fragment(can_frame,RADIO_PKT_TYPE_REG_TYPE2,radio_ctx.seq_total,i);
        canTransmit(canREG1, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
        canTransmit(canREG2, (radio_id == RADIO_ID_1) ? canMESSAGE_BOX12 : canMESSAGE_BOX13, can_frame);
    }
    radio_info_ack_tx_done();
}

void check_for_transmit_arp(void)
{
    if(comm_mandatory_area && (approaching_station_id != prev_stn_id))
    {
        radio_send_arp(RADIO_ID_1);
        radio_send_arp(RADIO_ID_2);
    }
}
