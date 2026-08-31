#ifndef CAN_IF_H
#define CAN_IF_H

#include <stdint.h>

/* ============================================================
 *  CAN MESSAGE IDENTIFIERS (EVENT REQUESTS)
 * ============================================================ */
typedef enum
{
    CAN_MSG_CPU_STARTUP = 0,
    CAN_MSG_CPU_TIME,
    CAN_MSG_COUNT
} can_msg_id_t;

typedef enum
{
    CAN_SOURCE_1 = 0,
    CAN_SOURCE_2 = 1
} can_source_t;

/* ============================================================
 *  STARTUP ACK RANGE
 * ============================================================ */
#define PERIPH_ACK_BASE_ID        0x081U
#define PERIPH_ACK_MAX_ID         0x0B1U
#define PERIPH_ACK_ID_MASK        0x07CU

#define MSG_TYPE_CPU_STARTUP      0x01U
#define MSG_TYPE_PERIPH_ACK       0x81U

/* ============================================================
 *  HEARTBEAT CAN
 * ============================================================ */
#define CPU_HEARTBEAT_CAN_ID      0x110U

#define PERIPH_HB_ACK_BASE_ID     0x0C1U
#define PERIPH_HB_ACK_MAX_ID      0x0F1U
#define PERIPH_HB_ACK_ID_MASK     0x07CU

#define MSG_TYPE_CPU_HEARTBEAT    ((uint16_t)0x0110)
#define MSG_TYPE_HB_ACK           ((uint16_t)0x0111)

#define HEARTBEAT_PERIOD_SEC     5U
#define HEARTBEAT_ACK_TIMEOUT    2U

/* ============================================================
 *  GSM CAN
 * ============================================================ */
#define GSM_START_REQ_CAN_ID      0x130U

/* RX filter: accepts 0x131 and 0x132 */
#define GSM_AUTH_KEY_RX_ID        0x130U
#define GSM_AUTH_KEY_RX_MASK      0x7FCU

#define MSG_TYPE_GSM_START_REQ    0x30U
#define MSG_TYPE_GSM_AUTH_KEY     0x31U

#define GSM_ACTION_START          0x01U
#define GSM_ACTION_STOP           0x00U

/* ============================================================
 *  INPUT CARD CAN
 * ============================================================ */

/* RX filter: accepts 0x150 � 0x153 */
#define INPUT_CARD_RX_ID     0x150U
#define INPUT_CARD_RX_MASK   0x7FCU

/* ============================================================
 *  CPU UNIVERSAL ACK
 * ============================================================ */

#define CPU_UNIVERSAL_ACK_CAN_ID   0x160U
#define CPU_UNIVERSAL_ACK_MASK     0x7E0U   /* accepts 0x160�0x17F */

#define MSG_TYPE_CPU_UNIVERSAL_ACK 0x40U

typedef enum
{
    PERIPH_RADIO      = 0x01,
    PERIPH_GSM        = 0x02,
    PERIPH_RFID       = 0x03,
    PERIPH_INPUT_CARD = 0x04
} peripheral_id_t;

typedef enum
{
    ACK_ACTION_ACCESS_AUTH   = 0x01,
    ACK_ACTION_REGULAR_MSG_1 = 0x02,
    ACK_ACTION_REGULAR_MSG_2 = 0x03,
    ACK_ACTION_CONFIG_CTRL   = 0x04,
    ACK_ACTION_DIAGNOSTIC    = 0x05
} cpu_ack_action_t;

typedef enum
{
    CPU_ACK_OK       = 0x00,
    CPU_ACK_REJECTED = 0x01,
    CPU_ACK_INVALID  = 0x02,
    CPU_ACK_BUSY     = 0x03
} cpu_ack_status_t;

void send_cpu_universal_ack(uint8_t peripheral_id,
                            uint8_t action_type,
                            uint8_t ack_status);

typedef enum
{
    GSM_1 = 0,
    GSM_2 = 1
} gsm_id_t;
/* ================= RADIO CAN IDs ================= */

#define RADIO1_CAN_ID        0x0140U
#define RADIO2_CAN_ID        0x0141U

/* ============================================================
 *  CAN IF APIs
 * ============================================================ */

/* TX */
void send_cpu_startup_can(void);
void send_cpu_time_can(void);
void send_cpu_heartbeat_can(void);
void send_gsm_start_req(uint8_t gsm_id, uint8_t action);
void send_Counter_Change_req(uint8_t flagSet);
void send_Data_Log(uint8_t count);

/* RX dispatch */
void can_if_process_rx(uint32_t can_id, uint8_t *data, can_source_t can_source);
void input_card_rx_handler(uint32_t can_id, uint8_t *data, can_source_t can_source);

/* CAN manager */
void can_manager_init(void);
void can_scheduler_1s_tick(void);
int  can_manager_poll_startup(void);
uint8_t can_startup_in_progress(void);

/* ACK handlers */
void can_manager_handle_ack(uint32_t can_id, uint8_t *data);
void can_manager_handle_hb_ack(uint32_t can_id);

/* Optional debug */
void debug_print_can_payload(void);

/* Optional event-driven TX (future use) */
void can_request_event(can_msg_id_t msg);

/* ============================================================
 *  TX BUFFERS
 * ============================================================ */
extern uint8_t tx_data_log[18];

#endif /* CAN_IF_H */
