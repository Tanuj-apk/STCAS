#ifndef RADIO_H
#define RADIO_H
#include "stdlib.h"
#include <stdint.h>
#include "stdbool.h"

/* ================= RADIO IDS ================= */

typedef enum
{
    RADIO_ID_1 = 0,
    RADIO_ID_2 = 1
} radio_id_t;

#define RADIO1_TX_MB   canMESSAGE_BOX12
#define RADIO2_TX_MB   canMESSAGE_BOX13

#define RADIO_RX_MAX_PAYLOAD   256U
uint8_t payload[RADIO_RX_MAX_PAYLOAD];

#define OSMA_HOLD_TIME 240
#define OV_ACTIVE_TIME 240
extern uint32_t last_osma_rx_time;
extern uint8_t osma_active;

/* ================= API ================= */

/* Send Access Request Packet (ARP) */
void radio_send_arp(radio_id_t radio_id);

/* Optional polling (future retry / timeout) */
void radio_poll_1s(void);
/* ================= CAN DEFINES ================= */

#define RADIO_AAP_RX_BASE_ID   0x0142U
#define RADIO_AAP_RX_MASK      0x000007FEU   /* accepts 0x142 & 0x143 */

/* ================= PROTOCOL LIMITS ================= */

#define RADIO_MAX_FRAGMENTS    64U
#define RADIO_PAYLOAD_BYTES    6U
#define RADIO_MAX_PAYLOAD_LEN  128 * 11   //1 byte Sequence Total with 5.5 bytes payload per sequence
#define RADIO_CRC_SIZE 4U

/* ================= RX CONTEXT ================= */

typedef struct
{
    uint8_t  active;
    uint8_t  pkt_type;
    uint8_t  seq_total;
    uint64_t  received_mask;
    uint8_t  payload_len;
    uint8_t  payload[RADIO_MAX_PAYLOAD_LEN];
    uint32_t start_time;
} radio_rx_ctx_t;
/* ================= RADIO PACKET TYPES ================= */

#define RADIO_PKT_TYPE_ARP   0x0DU
#define RADIO_PKT_TYPE_AAP   0x02U
#define RADIO_PKT_TYPE_ORP   0x0AU   /* Onboard Regular Packet */
#define RADIO_PKT_TYPE_REG_TYPE1  0x09U
#define RADIO_PKT_TYPE_REG_TYPE2   0x05U
#define RADIO_PKT_TYPE_AEP   0x06U

/* ============================================================
 *  RADIO UNIVERSAL ACK ACTION TYPES
 * ============================================================ */
#define ACK_ACTION_RADIO_ARP        RADIO_PKT_TYPE_ARP
#define ACK_ACTION_RADIO_ORP        RADIO_PKT_TYPE_ORP

//#define RADIO_PKT_TYPE_REG_TYPE  0x09U

typedef struct
{
    uint8_t  pkt_type;          // 4 bits
    uint16_t pkt_length;        // 10 bits
    uint16_t frame_num;         // 17 bits
    uint16_t source_stn_id;     // 16 bits
    uint8_t  source_version;    // 3 bits
    uint32_t dest_loco_id;      // 20 bits
    uint8_t  ref_prof_id;       // 4 bits
    uint16_t last_ref_rfid;     // 10 bits
    uint16_t dist_pkt_start;    // 15 bits
    uint8_t  pkt_dir;           // 2 bits
    uint8_t  padding_bits;      // 3 bits
}radio_header;

typedef struct
{
    uint8_t  sub_pkt_type;
    uint8_t  sub_pkt_len;
    uint8_t  frame_offset;
    uint8_t  dest_loco_sos;
    uint8_t  train_section_type;
    uint32_t cur_sig_info;       //17 bits
    uint8_t  cur_sig_aspect;     //6 bits
    uint8_t  next_sig_aspect;    //6 bits
    uint16_t appr_sig_dist;      //15 bits
    uint8_t  authority_type;
    uint8_t  authorized_speed;
    uint32_t ma_wrt_sig;         //16 bits
    uint8_t  req_shorten_ma;
    uint16_t new_ma;             //16 bits
    uint8_t  trn_len_info_sts;
    uint8_t  trn_len_info_type;
    uint32_t ref_frame_num_tlm;  //17 bits
    uint8_t  ref_offset_int_tlm; // continues to next frame
    uint8_t  next_stn_comm;
    uint16_t appr_stn_id;        //16 bits
    uint8_t  padding_bits;       //X bits
}MA_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_ssp;
    uint8_t  sub_pkt_len_ssp;
    uint8_t  lm_speed_info_cnt;
    uint16_t lm_static_speed_dist[31];   // 8 + 7 = 15 bits
    uint8_t  lm_static_speed_class[31];
    uint16_t lm_speed_universal[31];     // 6 bits
    uint16_t lm_speed_class_a[31];       // 2 + 4 = 6 bits
    uint16_t lm_speed_class_b[31];       // 4 + 2 = 6 bits
    uint16_t lm_speed_class_c[31];       // 6 bits
    uint8_t  padding_bits;      // X bits
}SSP_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_grad;
    uint8_t  sub_pkt_len_grad;
    uint8_t  lm_grad_info_cnt;
    uint16_t lm_gradient_distance[31];   // 8 + 7 = 15 bits
    uint8_t  lm_gdir[31];
    uint8_t  lm_gradient_value[31];
    uint8_t  padding_bits;      // X bits
}GP_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_lc;
    uint8_t  sub_pkt_len_lc;
    uint8_t  lm_lc_info_cnt;
    uint16_t lm_lc_distance[31];        // 8 + 7 = 15 bits
    uint16_t lm_lc_id_numeric[31];      // 1 + 8 + 1 = 10 bits
    uint8_t  lm_lc_id_alpha_suffix[31]; // 3 bits
    uint8_t  lm_lc_manning_type[31];    // 1 bit
    uint8_t  lm_lc_class[31];           // 3 bits
    uint8_t  lm_lc_auto_whistle_en[31]; // 1 bit
    uint8_t  lm_lc_auto_whistle_type[31]; // 2 bits
    uint8_t  padding_bits;      // X bits
}LCGP_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_tsp;
    uint8_t  sub_pkt_len_tsp;
    uint8_t  to_cnt;
    uint8_t  to_speed[3];
    uint16_t diff_dist_to[3];          // 6 + 8 + 1 = 15 bits
    uint16_t to_speed_rel_dist[3];     // 7 + 5 = 12 bits
    uint8_t  padding_bits;      // X bits
}TSP_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_tli;
    uint8_t  sub_pkt_len_tli;
    uint8_t  dist_dup_tag;          // 4 bits
    uint8_t  route_rfid_cnt;        // 1 + 5 = 6 bits
    uint16_t dist_nxt_rfid[62];         // 3 + 8 = 11 bits
    uint16_t nxt_rfid_tag_id[62];       // 8 + 2 = 10 bits
    uint8_t  dup_tag_dir[62];
    uint8_t  abs_loc_reset;
    uint16_t start_dist_loc_reset;  // 4 + 8 + 3 = 15 bits
    uint8_t  adj_loco_dir;
    uint32_t abs_loc_correction;    // 3 + 8 + 8 + 4 = 23 bits
    uint8_t  adj_line_cnt;
    uint16_t line_tin[6];              // 1 + 8 = 9 bits
    uint8_t  padding_bits;      // X bits
}TLI_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_tc;
    uint8_t  sub_pkt_len_tc;
    uint8_t  trackcond_cnt;
    uint8_t  trackcond_type[15];          // 1 + 3 = 4 bits
    uint16_t start_dist_trackcond[15];    // 5 + 8 + 2 = 15 bits
    uint16_t length_trackcond[15];        // 6 + 8 + 1 = 15 bits
    uint8_t  padding_bits;      // X bits
}TCD_Packet;

typedef struct
{
    uint8_t  sub_pkt_type_tsr;
    uint8_t  sub_pkt_len_tsr;
    uint8_t  tsr_status;
    uint8_t  tsr_info_cnt;
    uint16_t tsr_id[31];                  // 6 + 2 = 8 bits
    uint16_t tsr_distance[31];            // 6 + 8 + next frame MSB
    uint16_t tsr_length[31];          // 7 + 8 = 15 bits
    uint8_t  tsr_class[31];
    uint8_t  tsr_universal_speed[31];
    uint8_t  tsr_class_a_speed[31];
    uint8_t  tsr_class_b_speed[31];
    uint8_t  tsr_class_c_speed[31];
    uint8_t  tsr_whistle[31];
    uint8_t  padding_bits;      // X bits
}TSR_Packet;

typedef struct
{
    uint32_t loco_mac_code;       // 32 bits
    uint32_t pkt_crc;             // 32 bits
}END_Packet;

typedef struct
{
    radio_header radio_header_reg_type1;
    MA_Packet MA_Packet_reg_type1;
    SSP_Packet SSP_Packet_reg_type1;
    GP_Packet GP_Packet_reg_type1;
    LCGP_Packet LCGP_Packet_reg_type1;
    TSP_Packet TSP_Packet_reg_type1;
    TLI_Packet TLI_Packet_reg_type1;
    TCD_Packet TCD_Packet_reg_type1;
    TSR_Packet TSR_Packet_reg_type1;
    END_Packet END_Packet_reg_type1;
} radio_reg_type1_t;

typedef struct
{
    radio_header radio_header_reg_type2;
    MA_Packet MA_Packet_reg_type2;
    END_Packet END_Packet_reg_type2;
} radio_reg_type2_t;


typedef struct
{
    uint8_t  active;
    uint8_t  seq_total;
    uint8_t  seq_index;
    uint8_t  payload_len;
    uint8_t  payload[RADIO_MAX_PAYLOAD_LEN];
} radio_tx_ctx_t;

radio_tx_ctx_t radio_ctx;

extern radio_reg_type1_t reg_type1;
extern radio_reg_type2_t reg_type2;

extern int32_t distance_travelled;
extern uint32_t ref_odo;

extern uint16_t approaching_station_id;
extern bool check_for_sending_arp_using_aap;
extern bool radio_can_arp_transmit_flag;
extern uint16_t prev_stn_id;
extern radio_tx_ctx_t radio_ctx;
extern uint8_t tx_buf[];
extern uint32_t tx_mb;

#define TLM_MAX_SAMPLES_PER_SEC 150U
#define TLM_HISTORY_SECONDS 20U

typedef struct
{
  uint32_t tod_sec;
  uint32_t tod_ms[TLM_MAX_SAMPLES_PER_SEC];
  uint16_t sec_span_ms;
  float distance_odo[TLM_MAX_SAMPLES_PER_SEC];
} dist_array_t;

extern dist_array_t distance_db[TLM_HISTORY_SECONDS];
extern uint8_t current_sec_index;
extern uint16_t current_sample_index;
extern volatile uint32_t tlm_tod_sec;
extern volatile uint32_t tlm_tod_ms;

extern uint32_t calculated_train_length;

extern uint8_t last_ref_profile_num;

/* Called from CAN RX ISR */
void radio_rx_handle(uint32_t can_id, uint8_t *data);

/* Called from 1s scheduler */
void radio_rx_poll_1s(void);
extern void check_for_transmit_arp(void);

/* Query APIs */
uint8_t radio_aap_available(void);
const uint8_t *radio_get_aap_payload(uint8_t *len);
void radio_clear_aap(void);
/* Existing */
void radio_send_arp(radio_id_t radio_id);
void radio_update_frame_number(void);
void radio_build_fragment(uint8_t *can_frame, uint8_t pkt_type, uint8_t seq_total, uint8_t seq_index);

/* New */
void radio_send_orp(radio_id_t radio_id);

uint8_t radio_parse_reg_type1(const uint8_t *p, uint16_t len, radio_reg_type1_t *out);
uint8_t radio_parse_reg_type2(const uint8_t *p, uint16_t len, radio_reg_type2_t *out);

#endif /* RADIO_H */
