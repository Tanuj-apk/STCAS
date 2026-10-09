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
 *  CPU IDENTITY AND CPU-OWNED CAN IDS
 * ============================================================ */
#define CPU_DEVICE_ID_MIN         1U
#define CPU_DEVICE_ID_MAX         4U

#define CPU_STARTUP_CAN_ID        0x080U
#define CPU_TIME_CAN_ID           0x100U

extern uint8_t g_device_id;

/* Bit mask returned by can_transmit_redundant(). */
#define CAN_TX_ACCEPTED_CAN1      0x01U
#define CAN_TX_ACCEPTED_CAN2      0x02U
#define CAN_TX_ACCEPTED_BOTH      (CAN_TX_ACCEPTED_CAN1 | CAN_TX_ACCEPTED_CAN2)

extern volatile uint32_t can1_tx_reject_count;
extern volatile uint32_t can2_tx_reject_count;
extern volatile uint32_t can_dual_tx_reject_count;

/* ============================================================
 * CPU UNIVERSAL ACK STATUS
 * ============================================================ */
#define CPU_ACK_OK 0x00U
#define CPU_ACK_INVALID 0x01U

/* ============================================================
 *  STARTUP ACK RANGE
 * ============================================================ */
#define PERIPH_ACK_BASE_ID        0x084U
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
 *  DATALOGGER ACK
 * ============================================================ */
#define DATA_LOGGER_TX_CAN_ID 0x0210U
#define DATA_LOGGER_ACK_CAN_ID 0x0214U


/* ============================================================
 *  KMS CAN
 * ============================================================ */
#define KMS_QUERY_TX_CAN_ID       0x130U
#define KMS_RX_CAN_ID_1           0x134U
#define KMS_RX_CAN_ID_2           0x135U
#define KMS_RX_CAN_ID_BASE        KMS_RX_CAN_ID_1
#define KMS_RX_CAN_ID_MASK        0x7FEU

/* ============================================================
 *  INPUT CARD CAN
 * ============================================================ */

/* RX filter: accepts 0x150 � 0x153 */
#define INPUT_CARD_RX_ID     0x150U
#define INPUT_CARD_RX_MASK   0x7F0U

/* ============================================================
 *  CPU UNIVERSAL ACK
 * ============================================================ */

#define CPU_UNIVERSAL_ACK_CAN_ID   0x160U
#define CPU_UNIVERSAL_ACK_MASK     0x7E0U   /* accepts 0x160�0x17F */

#define MSG_TYPE_CPU_UNIVERSAL_ACK 0x40U

typedef enum
{
    PERIPH_RADIO      = 0x01,
    PERIPH_KMS        = 0x02,
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

// typedef enum
// {
//     CPU_ACK_OK       = 0x00,
//     CPU_ACK_REJECTED = 0x01,
//     CPU_ACK_INVALID  = 0x02,
//     CPU_ACK_BUSY     = 0x03
// } cpu_ack_status_t;

void send_cpu_universal_ack(uint16_t peripheral_can_id, uint8_t action_type, uint8_t ack_status);

/* ================= RADIO CAN IDs ================= */

#define RADIO1_CAN_ID        0x0140U
#define RADIO2_CAN_ID        0x0144U

/* ============================================================
 *  CAN RX SOFTWARE QUEUE
 * ============================================================ */
#define CAN_RX_PROCESS_LIMIT  8U
#define CAN_RX_QUEUE_SIZE    64U

typedef struct
{
    uint32_t id;
    can_source_t can_source;
    uint8_t data[8];
} can_rx_queue_entry_t;

extern volatile can_rx_queue_entry_t can_rx_queue[CAN_RX_QUEUE_SIZE];
extern volatile uint8_t can_rx_head;
extern volatile uint8_t can_rx_tail;
extern volatile uint32_t can_rx_queue_overflow;
extern volatile uint32_t can_rx_message_lost;
extern volatile uint32_t can_rx_read_failures;

/* ============================================================
 *  CAN IF APIs
 * ============================================================ */

/* TX */
uint8_t can_transmit_redundant(uint32_t message_box, const uint8_t data[8]);
uint32_t can_get_local_tx_id(uint32_t base_can_id);
void can_configure_device_ids(void);
void send_cpu_startup_can(void);
void send_cpu_time_can(void);
void send_cpu_heartbeat_can(void);

/* RX dispatch */
void can_if_process_rx(uint32_t can_id, uint8_t *data, can_source_t can_source);
void can_if_process_rx_pending(void);
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
#endif /* CAN_IF_H */
