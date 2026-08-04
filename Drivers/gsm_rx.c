#include "gsm_rx.h"
#include "gps.h"
#include "can_if.h"

/* ============================================================
 * CONTEXT STRUCTURES
 * ============================================================ */

typedef struct
{
    uint8_t  active;
    uint8_t  key_instance;
    uint8_t  seq_total;
    uint8_t  received_mask;
    uint8_t  key[GSM_AUTH_KEY_BYTES];
    uint32_t start_time;
} gsm_rx_ctx_t;

typedef struct
{
    uint8_t received;
    uint8_t gsm_id;
    uint8_t status;
    uint8_t error_code;
} gsm_start_ack_ctx_t;

typedef struct
{
    gsm_start_state_t state;
    uint8_t  gsm_id;
    uint8_t  retry_count;
    uint32_t start_time;
} gsm_start_ctx_t;

/* ============================================================
 * MODULE STATE
 * ============================================================ */

static gsm_rx_ctx_t        gsm_ctx;
static gsm_start_ack_ctx_t gsm_start_ack;
static gsm_start_ctx_t     gsm_start_ctx = { .state = GSM_FSM_IDLE };

static uint8_t auth_key_ready = 0;
uint8_t gsm_fault_flags = GSM_FAULT_NONE;

/* ============================================================
 * INTERNAL HELPERS
 * ============================================================ */

static void gsm_ctx_reset(void)
{
    gsm_ctx.active = 0;
    gsm_ctx.received_mask = 0;
    auth_key_ready = 0;
}

static void gsm_start_ack_reset(void)
{
    gsm_start_ack.received   = 0;
    gsm_start_ack.gsm_id     = 0;
    gsm_start_ack.status     = 0;
    gsm_start_ack.error_code = 0;
}

/* ============================================================
 * RX HANDLER (CALLED FROM CAN IF)
 * ============================================================ */

void gsm_rx_handle(uint32_t can_id, uint8_t *data)
{
    (void)can_id;

    /* ---------- START ACK ---------- */
    if (data[0] == MSG_TYPE_GSM_START_ACK)
    {
        if (gsm_start_ctx.state != GSM_FSM_WAIT_ACK)
            return;

        gsm_start_ack.received   = 1;
        gsm_start_ack.gsm_id     = data[2];
        gsm_start_ack.status     = data[3];
        gsm_start_ack.error_code = data[4];

        if (gsm_start_ack.received)
        {
            switch (gsm_start_ack.status)
            {
            case GSM_START_ACK_OK:
                gsm_start_ctx.state = GSM_FSM_ACK_OK;
                break;

            case GSM_START_ACK_BUSY:
            case GSM_START_ACK_REJECTED:
            case GSM_START_ACK_HW_FAULT:
            default:
                gsm_start_ctx.state = GSM_FSM_ACK_REJECTED;
                break;
            }
            gsm_start_ack_reset();
            return;
        }
    }

    /* Only accept AUTH KEY after successful START */
    if (gsm_start_ctx.state != GSM_FSM_ACK_OK ||
        gsm_start_ctx.gsm_id != data[1])   /* GSM ID in byte[1] */
    {
        return;
    }

    /* ---------- AUTH KEY ---------- */
    if (data[0] != MSG_TYPE_GSM_AUTH_KEY)
        return;

    uint8_t seq_info     = data[2];
    uint8_t key_instance = data[3];
    uint8_t seq_total    = (seq_info >> 4) & 0x0F;
    uint8_t seq_index    = seq_info & 0x0F;

    if (seq_total == 0 || seq_total > GSM_MAX_FRAGMENTS)
        return;

    if (seq_index >= seq_total)
        return;

    if (!gsm_ctx.active || gsm_ctx.key_instance != key_instance)
    {
        gsm_ctx_reset();
        gsm_ctx.active       = 1;
        gsm_ctx.key_instance = key_instance;
        gsm_ctx.seq_total    = seq_total;
        gsm_ctx.start_time   = seconds_uptime;
    }

    if (gsm_ctx.received_mask & (1U << seq_index))
        return;

    uint8_t offset = seq_index * 4;

    gsm_ctx.key[offset + 0] = data[4];
    gsm_ctx.key[offset + 1] = data[5];
    gsm_ctx.key[offset + 2] = data[6];
    gsm_ctx.key[offset + 3] = data[7];

    gsm_ctx.received_mask |= (1U << seq_index);

    if (gsm_ctx.received_mask == ((1U << seq_total) - 1U))
    {
        auth_key_ready = 1;
        gsm_ctx.active = 0;
    }
}

/* ============================================================
 * START REQUEST API
 * ============================================================ */

void gsm_start_request(uint8_t gsm_id)
{
    gsm_ctx_reset();
    auth_key_ready = 0;

    send_gsm_start_req(gsm_id, GSM_ACTION_START);

    gsm_start_ctx.state       = GSM_FSM_WAIT_ACK;
    gsm_start_ctx.gsm_id      = gsm_id;
    gsm_start_ctx.retry_count = 0;
    gsm_start_ctx.start_time  = seconds_uptime;

    gsm_start_ack_reset();
}

/* ============================================================
 * FSM POLLING (CALL EVERY 1s)
 * ============================================================ */

static void gsm_auth_key_poll_1s(void)
{
    if (!gsm_ctx.active)
        return;

    if ((seconds_uptime - gsm_ctx.start_time) >= GSM_AUTH_KEY_TIMEOUT_SEC)
    {
        gsm_fault_flags |= GSM_FAULT_AUTH_TIMEOUT;
        gsm_ctx_reset();
    }
}

void gsm_start_poll_1s(void)
{
    if (gsm_start_ctx.state != GSM_FSM_WAIT_ACK)
        return;

    if ((seconds_uptime - gsm_start_ctx.start_time) >= GSM_START_ACK_TIMEOUT_SEC)
    {
        gsm_start_ctx.state = GSM_FSM_ACK_TIMEOUT;
    }
}

void gsm_manager_process(void)
{
    switch (gsm_start_ctx.state)
    {
        case GSM_FSM_ACK_OK:
            /* GSM ready */
            break;

        case GSM_FSM_ACK_REJECTED:
        case GSM_FSM_ACK_TIMEOUT:
        {
            if (gsm_start_ctx.retry_count < GSM_MAX_RETRIES_PER_UNIT)
            {
                gsm_start_ctx.retry_count++;
                send_gsm_start_req(gsm_start_ctx.gsm_id, GSM_ACTION_START);
                gsm_start_ctx.start_time = seconds_uptime;
                gsm_start_ctx.state = GSM_FSM_WAIT_ACK;
            }
            else
            {
                if (gsm_start_ctx.gsm_id == GSM_1)
                {
                    gsm_fault_flags |= GSM_FAULT_GSM1_FAILED;
                    gsm_start_request(GSM_2);
                }
                else
                {
                    gsm_fault_flags |= GSM_FAULT_GSM2_FAILED;
                    gsm_fault_flags |= GSM_FAULT_NO_GSM_AVAIL;
                    gsm_start_ctx.state = GSM_FSM_IDLE;
                }
            }
            break;
        }

        default:
            break;
    }

    gsm_auth_key_poll_1s();
}

/* ============================================================
 * QUERY APIS
 * ============================================================ */

uint8_t gsm_auth_key_available(void)
{
    return auth_key_ready;
}

const uint8_t *gsm_get_auth_key(void)
{
    return gsm_ctx.key;
}

uint8_t gsm_start_ack_received(void)
{
    return gsm_start_ack.received;
}

uint8_t gsm_get_start_ack_status(void)
{
    return gsm_start_ack.status;
}

uint8_t gsm_get_start_ack_gsm_id(void)
{
    return gsm_start_ack.gsm_id;
}

void gsm_clear_start_ack(void)
{
    gsm_start_ack_reset();
}
