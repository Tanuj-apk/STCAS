#include "NMS.h"
#include "can.h"
#include "can_if.h"
#include "gps.h"
#include <stdlib.h>
#include <string.h>

//static uint16_t nms_msg_seq = 0;

nms_tx_ctx_t nms_ctx;
static void set_bits(uint8_t *buf, uint16_t bit, uint8_t len, uint32_t value)
{
    for(uint8_t i = 0; i < len; i++)
    {
        uint16_t b = bit + i;
        uint8_t byte = b >> 3;
        uint8_t shift = b & 7;

        if (value & (1UL << i))
            buf[byte] |= (1U << shift);

        else
            buf[byte] &= ~(1U << shift);
    }
}

static void nms_build_fragment(uint8_t *can_frame, uint8_t pkt_type,uint8_t seq_total, uint8_t seq_index) 
{
    uint8_t seq_total_lsb;
    uint8_t seq_total_msb;
    seq_total_lsb = seq_total & 0x0F;
    seq_total_msb = (seq_total >> 4) & 0x03;
    can_frame[0] = (seq_total_lsb << 4) | (pkt_type & 0x0F);
    can_frame[1] = ((seq_index & 0x3F) << 2) | seq_total_msb;
    uint16_t payload_offset = seq_index * NMS_PAYLOAD_BYTES;

    for (uint8_t i = 0; i < NMS_PAYLOAD_BYTES; i++)
    {
        if ((payload_offset + i) < nms_ctx.payload_len) 
            can_frame[2 + i] = nms_ctx.payload[payload_offset + i];
        else
            can_frame[2 + i] = 0x00;
    }
}

// BUILD PAYLOAD FUNCTIONS FOR NMS
static uint16_t build_info_payload_to_nms(uint8_t *buf)
{
    uint16_t bit_index = 0;
    nms_kavach_info_t *h = &nms_ctx.kavach_info;
    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);

    /* =========================================================
     * FIELD 1
     * Start of Frame
     * 2 bytes
     * Value = 0xAAAA
     * ========================================================= */
    h->SOF = 0xAAAA;
    set_bits(buf, bit_index, 16, h->SOF);
    bit_index += 16;

    /* =========================================================
     * FIELD 2
     * Message Type
     * 1 byte
     * Value = 0x11
     * ========================================================= */
    h->msg_type = 0x11;
    set_bits(buf, bit_index, 8, h->msg_type);
    bit_index += 8;


    /* =========================================================
     * FIELD 3
     * Message Length
     * This field is the number of bytes from Message Type
     * through CRC, inclusive.
     *
     * Fixed portion after Message Type:
     * Message Length          2
     * Message Sequence        2
     * Stationary KAVACH ID    2
     * NMS System ID           2
     * System Version          1
     * Date                    3
     * Time                    3
     * Station Active Radio    1
     * SOF Tx Byte 1           1
     * SOF Tx Byte 2           1
     *
     * Variable station packet = station_packet_len
     *
     * CRC                     4
     *
     * Therefore:
     *
     * Message Length =
     *     22 + station_packet_len
     *
     * NOTE:
     * The 22 bytes include the Message Length field itself.
     * ========================================================= */
    h->msg_len = (uint16_t)(23U + h->station_packet_len);
    set_bits(buf, bit_index, 16, h->msg_len);
    bit_index += 16;

    /* =========================================================
     * FIELD 4
     * Message Sequence
     * 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16, h->message_sequence);
    bit_index += 16;

    /* =========================================================
     * FIELD 5
     * Stationary KAVACH ID
     * 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16, h->stationary_kavach_id);
    bit_index += 16;

    /* =========================================================
     * FIELD 6
     * NMS System ID
     * 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16, h->nms_system_id);
    bit_index += 16;

    /* =========================================================
     * FIELD 7
     * System Version
     * 0x00 = Version 3.2
     * 0x01 = Version 4.0
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->system_version);
    bit_index += 8;

    /* =========================================================
     * FIELD 8
     * Date
     * DD / MM / YY
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->date_day);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->date_month);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->date_year);
    bit_index += 8;

    /* =========================================================
     * FIELD 9
     * Time
     * HH / MM / SS
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->time_hour);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->time_min);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->time_sec);
    bit_index += 8;

    /* =========================================================
     * FIELD 10
     * Station Active Radio
     * 0xF1 = Radio 1
     * 0xF2 = Radio 2
     * 0xE1 = Ethernet 1
     * 0xE2 = Ethernet 2
     * Other = Active radio unknown
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->station_active_radio);
    bit_index += 8;

    /* =========================================================
     * FIELD 11
     * SOF Tx Byte 1
     * Value = 0xA5
     * ========================================================= */
    h->sof_tx_byte1 = 0xA5;
    set_bits(buf, bit_index, 8, h->sof_tx_byte1);
    bit_index += 8;

    /* =========================================================
     * FIELD 12
     * SOF Tx Byte 2
     * Value = 0xC3
     * ========================================================= */
    h->sof_tx_byte2 = 0xC3;
    set_bits(buf, bit_index, 8, h->sof_tx_byte2);
    bit_index += 8;

    /* =========================================================
     * TO DO:
     * VARIABLE STATION PACKET
     * Station Regular /
     * Access Authority /
     * Additional Emergency Packet /
     * etc.
     * Contents are according to KAVACH Radio Communication
     * Protocol.
     * ========================================================= */

    for (uint16_t i = 0; i < h->station_packet_len; i++)
    {
        set_bits(buf, bit_index, 8, h->station_packet[i]);
        bit_index += 8;
    }

    /*
     * CRC is NOT generated here.
     *
     * Downstream CAN/NMS interface device
     * will append CRC.
     */

    /* =========================================================
     * RETURN TOTAL MESSAGE SIZE
     * ========================================================= */

    return (uint16_t)((bit_index + 7U) / 8U);
}


/* ============================================================
 * BUILD STATIONARY KAVACH HEALTH MESSAGE FOR NMS
 *
 * Message:
 *
 * Field 1  : SOF                  2 bytes
 * Field 2  : Message Type         1 byte
 * Field 3  : Message Length       2 bytes
 * Field 4  : Message Sequence     2 bytes
 * Field 5  : Stationary KAVACH ID 2 bytes
 * Field 6  : NMS System ID        2 bytes
 * Field 7  : System Version       1 byte
 * Field 8  : Date                 3 bytes
 * Field 9  : Time                 3 bytes
 * Field 10 : Event Count          1 byte
 * Field 11 : Event ID             2 bytes
 * Field 12 : Event Data           59 bytes
 * Field 13 : CRC                  4 bytes
 *
 * Message Length = fields 2 through 13 inclusive
 *                = 82 bytes when all 45 Event Data fields
 *                  are included.
 *
 * CRC excludes SOF.
 * ============================================================ */

static uint8_t build_health_payload_to_nms(uint8_t *buf)
{
    uint16_t bit_index = 0;
    nms_health_event_t *h = &nms_ctx.health;
    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);

    /* =========================================================
     * HEADER
     * ========================================================= */
    /* ---------------------------------------------------------
     * Field 1: Start of Frame
     * Size: 2 bytes
     * Value: 0xAAAA
     * --------------------------------------------------------- */
    h->SOF = 0xAAAA;
    set_bits(buf, bit_index, 16, h->SOF);
    bit_index += 16;

    /* ---------------------------------------------------------
     * Field 2: Message Type
     * Size: 1 byte
     * Value: 0x17
     * --------------------------------------------------------- */
    h->msg_type = 0x17;
    set_bits(buf, bit_index, 8, h->msg_type);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Field 3: Message Length
     * Bytes from Message Type through CRC inclusive.
     * For all 45 Event Data fields:
     * Event Data = 59 bytes
     * Message Type        = 1
     * Message Length      = 2
     * Message Sequence    = 2
     * Stationary KAVACH ID = 2
     * NMS System ID       = 2
     * System Version      = 1
     * Date                = 3
     * Time                = 3
     * Event Count         = 1
     * Event ID            = 2
     * Event Data          = 59
     * Total = 80 bytes = 0x0052
     * --------------------------------------------------------- */
    h->msg_len = 82U;
    set_bits(buf, bit_index, 16, h->msg_len);
    bit_index += 16;

    /* ---------------------------------------------------------
     * Field 4: Message Sequence
     * Size: 2 bytes
     * Range: 0 - 65535
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 16, h->message_sequence);
    bit_index += 16;

    /* ---------------------------------------------------------
     * Field 5: Stationary KAVACH ID
     * Size: 2 bytes
     * Valid values: 1 - 65535
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 16, h->stationary_kavach_id);
    bit_index += 16;

    /* ---------------------------------------------------------
     * Field 6: NMS System ID
     * Size: 2 bytes
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 16, h->nms_system_id);
    bit_index += 16;

    /* ---------------------------------------------------------
     * Field 7: System Version
     * Size: 1 byte
     * Value: 1
     * --------------------------------------------------------- */
    h->system_version = 1U;
    set_bits(buf, bit_index, 8, h->system_version);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Field 8: Date
     * Size: 3 bytes
     * DD / MM / YY
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 8, h->date_day);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->date_month);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->date_year);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Field 9: Time
     * Size: 3 bytes
     * HH : MM : SS
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 8, h->time_hour);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->time_min);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->time_sec);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Field 10: Event Count
     * Size: 1 byte
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 8, h->event_count);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Field 11: Event ID
     * Size: 2 bytes
     * Stationary KAVACH Health
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 16, h->event_id);
    bit_index += 16;

    /* =========================================================
     * FIELD 12: EVENT DATA
     * Stationary KAVACH Health Event Fields 1 - 45
     * ========================================================= */
    /* ---------------------------------------------------------
     * Event Field 1: System Temperature
     * Size: 1 byte
     * Signed
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 8, (uint8_t)h->system_temperature);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Event Field 2: Active Radio Number
     * 0 = Not used
     * 1 = Radio 1
     * 2 = Radio 2
     * 3 = Both Radio active
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 8, h->active_radio_number);
    bit_index += 8;

    /* ---------------------------------------------------------
     * Event Field 3: Radio-1 Health
     * 1 = OK
     * 2 = Diagnostic Link Fail
     * 3 = Radio Fail
     * --------------------------------------------------------- */
    set_bits(buf, bit_index, 8, h->radio1_health);
    bit_index += 8;

    /* Event Field 4: Radio-2 Health */
    set_bits(buf, bit_index, 8, h->radio2_health);
    bit_index += 8;

    /* Event Field 5: Radio-1 Input Supply */
    set_bits(buf, bit_index, 8, h->radio1_input_supply);
    bit_index += 8;

    /* Event Field 6: Radio-2 Input Supply */
    set_bits(buf, bit_index, 8, h->radio2_input_supply);
    bit_index += 8;

    /* Event Field 7: Radio-1 Temperature */
    set_bits(buf, bit_index, 8, (uint8_t)h->radio1_temperature);
    bit_index += 8;

    /* Event Field 8: Radio-2 Temperature */
    set_bits(buf, bit_index, 8, (uint8_t)h->radio2_temperature);
    bit_index += 8;

    /* Event Field 9: Radio-1 PA Temperature */
    set_bits(buf, bit_index, 8, h->radio1_pa_temperature);
    bit_index += 8;

    /* Event Field 10: Radio-2 PA Temperature */
    set_bits(buf, bit_index, 8, h->radio2_pa_temperature);
    bit_index += 8;

    /* Event Field 11: Radio-1 PA Supply Voltage */
    set_bits(buf, bit_index, 8, h->radio1_pa_supply_voltage);
    bit_index += 8;

    /* Event Field 12: Radio-2 PA Supply Voltage */
    set_bits(buf, bit_index, 8, h->radio2_pa_supply_voltage);
    bit_index += 8;

    /* Event Field 13: Radio-1 Tx PA Current */
    set_bits(buf, bit_index, 8, h->radio1_tx_pa_current);
    bit_index += 8;

    /* Event Field 14: Radio-2 Tx PA Current */
    set_bits(buf, bit_index, 8, h->radio2_tx_pa_current);
    bit_index += 8;

    /* Event Field 15: Radio-1 Reverse Power */
    set_bits(buf, bit_index, 8, h->radio1_reverse_power);
    bit_index += 8;

    /* Event Field 16: Radio-2 Reverse Power */
    set_bits(buf, bit_index, 8, h->radio2_reverse_power);
    bit_index += 8;

    /* Event Field 17: Radio-1 Forward Power */
    set_bits(buf, bit_index, 8, h->radio1_forward_power);
    bit_index += 8;

    /* Event Field 18: Radio-2 Forward Power */
    set_bits(buf, bit_index, 8, h->radio2_forward_power);
    bit_index += 8;

    /* Event Field 19: Current Running Key */
    set_bits(buf, bit_index, 8, h->current_running_key);
    bit_index += 8;

    /* Event Field 20: Remaining Number of Keys */
    set_bits(buf, bit_index, 8, h->remaining_keys);
    bit_index += 8;

    /* Event Field 21: Session Key Checksum
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->session_key_checksum);
    bit_index += 16;

    /* Event Field 22: Allocated Time Slot for New Loco
     * Size: 1 byte */
    set_bits(buf, bit_index, 8, h->allocated_time_slot);
    bit_index += 8;

    /* Event Field 23: New Loco Regular Packet Time Offset
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->new_loco_regular_pkt_time_offset);
    bit_index += 16;

    /* Event Field 24: Loco Count */
    set_bits(buf, bit_index, 8, h->loco_count);
    bit_index += 8;

    /* Event Field 25: Radio-1 Rx Packet Count */
    set_bits(buf, bit_index, 8, h->radio1_rx_packet_count);
    bit_index += 8;

    /* Event Field 26: Radio-2 Rx Packet Count */
    set_bits(buf, bit_index, 8, h->radio2_rx_packet_count);
    bit_index += 8;

    /* Event Field 27: Active GPS Number */
    set_bits(buf, bit_index, 8, h->active_gps_number);
    bit_index += 8;

    /* Event Field 28: GPS-1 View */
    set_bits(buf, bit_index, 8, h->gps1_view);
    bit_index += 8;

    /* Event Field 29: GPS-2 View */
    set_bits(buf, bit_index, 8, h->gps2_view);
    bit_index += 8;

    /* Event Field 30: GPS-1 Seconds */
    set_bits(buf, bit_index, 8, h->gps1_seconds);
    bit_index += 8;

    /* Event Field 31: GPS-2 Seconds */
    set_bits(buf, bit_index, 8, h->gps2_seconds);
    bit_index += 8;

    /* Event Field 32: GPS-1 Satellites in View */
    set_bits(buf, bit_index, 8, h->gps1_satellites);
    bit_index += 8;

    /* Event Field 33: GPS-1 CNO Max */
    set_bits(buf, bit_index, 8, h->gps1_cno_max);
    bit_index += 8;

    /* Event Field 34: GPS-2 Satellites in View */
    set_bits(buf, bit_index, 8, h->gps2_satellites);
    bit_index += 8;

    /* Event Field 35: GPS-2 CNO Max */
    set_bits(buf, bit_index, 8, h->gps2_cno_max);
    bit_index += 8;

    /* Event Field 36: GSM-1 RSSI */
    set_bits(buf, bit_index, 8, h->gsm1_rssi);
    bit_index += 8;

    /* Event Field 37: GSM-2 RSSI */
    set_bits(buf, bit_index, 8, h->gsm2_rssi);
    bit_index += 8;

    /* Event Field 38: Missing RFID
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->missing_rfid);
    bit_index += 16;

    /* Event Field 39: Invalid RFID
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->invalid_rfid);
    bit_index += 16;

    /* Event Field 40: Conflict Route RFID
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->conflict_route_rfid);
    bit_index += 16;

    /* Event Field 41: Conflicting TIN
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->conflicting_tin);
    bit_index += 16;

    /* Event Field 42: Missing TIN
     * Size: 2 bytes */
    set_bits(buf, bit_index, 16, h->missing_tin);
    bit_index += 16;

    /* Event Field 43: Loco Specific SoS
     * Size: 4 bytes
     * B3-B1 = Loco ID
     * B0     = SoS Code */
    set_bits(buf, bit_index, 32, h->loco_specific_sos);
    bit_index += 32;

    /* Event Field 44: Train Exit Mode
     * Size: 4 bytes
     * B3-B1 = Loco ID
     * B0     = Exit Code */
    set_bits(buf, bit_index, 32, h->train_exit_mode);
    bit_index += 32;

    /* Event Field 45: Station Modules Health
     * Size: 2 bytes
     * b15-b4 = Module ID
     * b3-b0  = Module Health */
    set_bits(buf, bit_index, 16, h->station_modules_health);
    bit_index += 16;

    /*
     * CRC is NOT generated here.
     *
     * Downstream CAN/NMS interface device
     * will append CRC.
     */

    /* =========================================================
     * FINAL LENGTH
     * 2 bytes SOF are NOT included in Message Length.
     * Total returned bytes:
     *     SOF + Message Length
     *     = 2 + 82
     *     = 84 bytes
     * ========================================================= */

    return (uint8_t)((bit_index + 7U) / 8U);
}

//static uint8_t build_health_payload_to_nms(uint8_t *buf)
//{
//    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);
//    uint16_t bit_index = 0;
//    nms_health_event_t *h = &nms_ctx.health;
//
//    /* =========================================================
//     * HEADER FIELDS (Spec Fields 1–7)
//     * ========================================================= */
//
//    /* Field 1: Start of Frame (SOF) — 2 bytes = 16 bits */
//    h->SOF = 0xBBBB;
//    set_bits(buf, bit_index, 16, h->SOF);
//    bit_index += 16;
//
//    /* Field 2: Message Type — 1 byte = 8 bits */
//    h->msg_type = 0x18;
//    set_bits(buf, bit_index, 8, h->msg_type);
//    bit_index += 8;
//
//    /* Field 3: Message Length — 2 bytes = 16 bits
//       (from "Message Type" to "CRC", inclusive) */
//    h->msg_len = 0x006F;
//    set_bits(buf, bit_index, 16, h->msg_len);
//    bit_index += 16;
//
//    /* Field 4: Message Sequence — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->message_sequence);
//    bit_index += 16;
//
//    /* Field 5: Onboard KAVACH ID — 3 bytes = 24 bits */
//    set_bits(buf, bit_index, 24, h->loco_kavach_id & 0xFFFFFF);
//    bit_index += 24;
//
//    /* Field 6: NMS System ID — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->nms_system_id);
//    bit_index += 16;
//
//    /* Field 7: System Version — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->system_version);
//    bit_index += 8;
//
//    /* =========================================================
//     * Field 8: Date — 3 bytes = 24 bits
//     *   DD/MM/YY (IST Time-Configurable)
//     *   Byte[0] = Day   (01–31; 0x00 = not used; 0xFF = unknown)
//     *   Byte[1] = Month (01–12; 0x00 = not used; 0xFF = unknown)
//     *   Byte[2] = Year  (00–99 official; 100–254 = not used; 0xFF = unknown)
//     *   e.g. 27/04/18 → 0x1B-0x04-0x12
//     * ========================================================= */
//    set_bits(buf, bit_index, 8, h->date_day);   bit_index += 8;
//    set_bits(buf, bit_index, 8, h->date_month); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->date_year);  bit_index += 8;
//
//    /* =========================================================
//     * Field 9: Time — 3 bytes = 24 bits
//     *   HH:MM:SS (IST Time-Configurable)
//     *   Byte[0] = Hours   (00–23; 24–254 = not used; 0xFF = unknown)
//     *   Byte[1] = Minutes (00–59; 60–254 = not used; 0xFF = unknown)
//     *   Byte[2] = Seconds (00–59; 60–254 = not used; 0xFF = unknown)
//     *   e.g. 06:36:10 → 0x06-0x24-0x0A
//     * ========================================================= */
//    set_bits(buf, bit_index, 8, h->time_hour); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->time_min);  bit_index += 8;
//    set_bits(buf, bit_index, 8, h->time_sec);  bit_index += 8;
//
//    /* Field 10: Event Count — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->event_count);
//    bit_index += 8;
//
//    /* Field 11: Event ID — 2 bytes = 16 bits (Onboard KAVACH Health) */
//    set_bits(buf, bit_index, 16, h->event_id);
//    bit_index += 16;
//
//    /* Field 12: Event Data — m bytes (variable, Onboard KAVACH Health) */
//    /* Event Data is message-specific; encoded below as Loco KAVACH Health fields */
//
//    /* =========================================================
//     * LOCO KAVACH HEALTH — EVENT DATA FIELDS (Fields 1–57)
//     * ========================================================= */
//
//    /* Loco Field 1: Radio-1 Health — 1 byte
//     *   1=OK, 2=Diagnostic Link Fail, 3=Radio Fail */
//    set_bits(buf, bit_index, 8, h->radio1_health);
//    bit_index += 8;
//
//    /* Loco Field 2: Radio-2 Health — 1 byte
//     *   1=OK, 2=Diagnostic Link Fail, 3=Radio Fail */
//    set_bits(buf, bit_index, 8, h->radio2_health);
//    bit_index += 8;
//
//    /* Loco Field 3: Radio-1 Input Supply — 1 byte
//     *   Value: 10V–30V; on change of voltage by 1V */
//    set_bits(buf, bit_index, 8, h->radio1_input_supply);
//    bit_index += 8;
//
//    /* Loco Field 4: Radio-2 Input Supply — 1 byte
//     *   Value: 10V–30V; on change of voltage by 1V */
//    set_bits(buf, bit_index, 8, h->radio2_input_supply);
//    bit_index += 8;
//
//    /* Loco Field 5: Radio-1 PA Temperature — 1 byte
//     *   Value: -30°C to 70°C (1 byte Signed); on change by 3°C */
//    set_bits(buf, bit_index, 8, (uint8_t)h->radio1_temperature);
//    bit_index += 8;
//
//    /* Loco Field 6: Radio-2 PA Temperature — 1 byte
//     *   Value: -30°C to 70°C (1 byte Signed); on change by 3°C */
//    set_bits(buf, bit_index, 8, (uint8_t)h->radio2_temperature);
//    bit_index += 8;
//
//    /* Loco Field 7: Radio-1 PA Temperature (2nd instance) — 1 byte
//     *   Value: 20°C–100°C; on change of temperature by 3°C */
//    set_bits(buf, bit_index, 8, h->radio1_pa_temperature);
//    bit_index += 8;
//
//    /* Loco Field 8: Radio-2 PA Temperature (2nd instance) — 1 byte
//     *   Value: 20°C–100°C; on change of temperature by 3°C */
//    set_bits(buf, bit_index, 8, h->radio2_pa_temperature);
//    bit_index += 8;
//
//    /* Loco Field 9: Radio-1 PA Supply Voltage — 1 byte
//     *   Value: 11V–13V; on change of voltage by 1V */
//    set_bits(buf, bit_index, 8, h->radio1_pa_voltage);
//    bit_index += 8;
//
//    /* Loco Field 10: Radio-2 PA Supply Voltage — 1 byte
//     *   Value: 11V–13V; on change of voltage by 1V */
//    set_bits(buf, bit_index, 8, h->radio2_pa_voltage);
//    bit_index += 8;
//
//    /* Loco Field 11: Radio-1 Tx PA Current — 1 byte
//     *   Value: 1.5A to 3.2A; on change of current */
//    set_bits(buf, bit_index, 8, h->radio1_tx_pa_current);
//    bit_index += 8;
//
//    /* Loco Field 12: Radio-2 Tx PA Current — 1 byte
//     *   Value: 1.5A to 3.2A; on change of current */
//    set_bits(buf, bit_index, 8, h->radio2_tx_pa_current);
//    bit_index += 8;
//
//    /* Loco Field 13: Radio-1 Reverse Power — 1 byte
//     *   Value received from Radio
//     *   e.g. 0x01 = 0.1W */
//    set_bits(buf, bit_index, 8, h->radio1_reverse_power);
//    bit_index += 8;
//
//    /* Loco Field 14: Radio-2 Reverse Power — 1 byte
//     *   e.g. 0x0F = 1.5W */
//    set_bits(buf, bit_index, 8, h->radio2_reverse_power);
//    bit_index += 8;
//
//    /* Loco Field 15: Radio-1 Forward Power — 1 byte
//     *   e.g. 0x36 = 5.4W */
//    set_bits(buf, bit_index, 8, h->radio1_forward_power);
//    bit_index += 8;
//
//    /* Loco Field 16: Radio-2 Forward Power — 1 byte
//     *   e.g. 0x78 = 12W */
//    set_bits(buf, bit_index, 8, h->radio2_forward_power);
//    bit_index += 8;
//
//    /* Loco Field 17: Stationary Regular Packet Received Time Offset — 2 bytes
//     *   0–2000 ms (while establishing comms with Stationary KAVACH) */
//    set_bits(buf, bit_index, 16, h->stationary_pkt_time_offset);
//    bit_index += 16;
//
//    /* Loco Field 18: Active GPS Number — 1 byte
//     *   GPS used for frame number calculation
//     *   0=No Active GPS, 1=GPS1, 2=GPS2, 3=Both GPS
//     *   on change of GPS */
//    set_bits(buf, bit_index, 8, h->active_gps_number);
//    bit_index += 8;
//
//    /* Loco Field 19: GPS-1 View Status — 1 byte
//     *   0=No Data, 1=V, 2=A; on detection of change of event */
//    set_bits(buf, bit_index, 8, h->gps1_view_status);
//    bit_index += 8;
//
//    /* Loco Field 20: GPS-2 View Status — 1 byte
//     *   0=No Data, 1=V, 2=A; on detection of change of event */
//    set_bits(buf, bit_index, 8, h->gps2_view_status);
//    bit_index += 8;
//
//    /* Loco Field 21: GPS-1 Seconds — 1 byte
//     *   0 to 59 seconds; on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gps1_seconds);
//    bit_index += 8;
//
//    /* Loco Field 22: GPS-2 Seconds — 1 byte
//     *   0 to 59 seconds; on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gps2_seconds);
//    bit_index += 8;
//
//    /* Loco Field 23: GPS-1 Satellites in View — 1 byte
//     *   Value received from GPS receiver
//     *   on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gps1_satellites);
//    bit_index += 8;
//
//    /* Loco Field 24: GPS-1 CNO Max — 1 byte
//     *   Maximum CNO value received from GPS receiver
//     *   on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gps1_cno_max);
//    bit_index += 8;
//
//    /* Loco Field 25: GPS-2 Satellites in View — 1 byte
//     *   on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gps2_satellites);
//    bit_index += 8;
//
//    /* Loco Field 26: GPS-2 CNO Max — 1 byte
//     *   Maximum CNO value received from GPS receiver
//     *   on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gps2_cno_max);
//    bit_index += 8;
//
//    /* Loco Field 27: GPS-1 Link Status — 2 bytes
//     *   0=Both GPS link and PPS fail
//     *   1=GPS link fail and PPS ok
//     *   2=GPS link ok and PPS fail
//     *   3=GPS link ok and PPS ok
//     *   on change of event for every one hour */
//    set_bits(buf, bit_index, 16, h->gps1_link_status);
//    bit_index += 16;
//
//    /* Loco Field 28: GPS-2 Link Status — 2 bytes
//     *   (same encoding as GPS-1 link status) */
//    set_bits(buf, bit_index, 16, h->gps2_link_status);
//    bit_index += 16;
//
//    /* Loco Field 29: GSM-1 RSSI — 1 byte
//     *   Value received from GSM module
//     *   on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gsm1_rssi);
//    bit_index += 8;
//
//    /* Loco Field 30: GSM-2 RSSI — 1 byte
//     *   Value received from GSM module
//     *   on change of value for every one hour */
//    set_bits(buf, bit_index, 8, h->gsm2_rssi);
//    bit_index += 8;
//
//    /* Loco Field 31: Current Running Key — 1 byte
//     *   0=Default key set, 1–30=KMS key set
//     *   on change of Key Set */
//    set_bits(buf, bit_index, 8, h->current_running_key);
//    bit_index += 8;
//
//    /* Loco Field 32: Remaining Number of Keys — 1 byte
//     *   0=No keys, 1–30=Remaining KMS key sets
//     *   on change of value */
//    set_bits(buf, bit_index, 8, h->remaining_keys);
//    bit_index += 8;
//
//    /* Loco Field 33: Session Key Checksum — 2 bytes
//     *   Checksum of 16 bytes session key
//     *   for every 2s at the time of Authentication only */
//    set_bits(buf, bit_index, 16, h->session_key_checksum);
//    bit_index += 16;
//
//    /* Loco Field 34: DMI-1 Link Status — 2 bytes
//     *   0=NOT OK, 1=OK; on change of value */
//    set_bits(buf, bit_index, 16, h->dmi1_link_status);
//    bit_index += 16;
//
//    /* Loco Field 35: DMI-2 Link Status — 2 bytes
//     *   0=NOT OK, 1=OK; on change of value */
//    set_bits(buf, bit_index, 16, h->dmi2_link_status);
//    bit_index += 16;
//
//    /* Loco Field 36: RFID Reader-1 Link Status — 2 bytes
//     *   0=NOT OK, 1=OK; on change of event */
//    set_bits(buf, bit_index, 16, h->rfid1_link_status);
//    bit_index += 16;
//
//    /* Loco Field 37: RFID Reader-2 Link Status — 2 bytes
//     *   0=NOT OK, 1=OK; on change of event */
//    set_bits(buf, bit_index, 16, h->rfid2_link_status);
//    bit_index += 16;
//
//    /* Loco Field 38: Duplicate Missing RFID Tag — 2 bytes
//     *   RFID Tag Number */
//    set_bits(buf, bit_index, 16, h->duplicate_missing_rfid_tag);
//    bit_index += 16;
//
//    /* Loco Field 39: Missing Linked RFID Tag — 4 bytes
//     *   B3–B1: Linked RFID Tag
//     *   B0:    Linking direction */
//    set_bits(buf, bit_index, 32, h->missing_linked_rfid_tag);
//    bit_index += 32;
//
//    /* Loco Field 40: Computed TLM Status — 4 bytes
//     *   B3–B2: Station ID
//     *   B1–B0: TLM Status
//     *   b11–b0: Computed TLM Value
//     *   b15–b12: TLM Status
//     *   TLM Status: 0=TLM Failed, 1=TLM Updated */
//    set_bits(buf, bit_index, 32, h->computed_tlm_status);
//    bit_index += 32;
//
//    /* Loco Field 41: Train Configuration Change — 1 byte
//     *   0=No, 1=Yes */
//    set_bits(buf, bit_index, 8, h->train_configuration_change);
//    bit_index += 8;
//
//    /* Loco Field 42: Bootup Sequence Error — 1 byte
//     *   0=Brake Test failed, 1=MR not available
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->bootup_sequence_error);
//    bit_index += 8;
//
//    /* Loco Field 43: Selected Train Formation — 1 byte
//     *   1  = Light Engine (120kmph)
//     *   2  = Light Engine Multi (120kmph)
//     *   3  = Passenger Train 3 to 7 Coach (120kmph)
//     *   4  = Passenger Train 8 to 13 Coach (120kmph)
//     *   5  = Passenger Train 14 to 20 Coach (120kmph)
//     *   6  = Passenger Train 21 to 27 Coach (120kmph)
//     *   7  = Goods 59 BOXN Empty (1000–1999 Ton, 75kmph)
//     *   8  = Goods 59 BOXN Half Load (2000–3499 Ton, 75kmph)
//     *   9  = Goods 59 BOXN Full Load (3500–5500 Ton, 60kmph)
//     *   10 = Goods 42 BCN Empty (1000–1999 Ton, 75kmph)
//     *   11 = Goods 42 BCN Half Load (2000–3499 Ton, 75kmph)
//     *   12 = Goods 42 BCN Full Load (3500–5500 Ton, 60kmph)
//     *   13 = Light Engine WAP5 (170kmph)
//     *   14 = WAP5-8LHB Coaches (170kmph)
//     *   15 = Light Engine WAP7 (140kmph)
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->selected_train_formation);
//    bit_index += 8;
//
//    /* Loco Field 44: Selected Cab — 1 byte
//     *   0=No Cab Selected, 1=Cab1 Selected
//     *   2=Cab2 Selected, 3=Both Cabs Selected
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->selected_cab);
//    bit_index += 8;
//
//    /* Loco Field 45: Brake Application Reason — 1 byte
//     *   0  = Not used
//     *   1  = Reverse movement detected
//     *   2  = Unusual stoppage detected
//     *   3  = Overspeed
//     *   4  = Rollback detected
//     *   5  = MHT selected
//     *   6  = No LP Acknowledge
//     *   7  = MA Shortened
//     *   8  = Headon collision detected
//     *   9  = Rearend collision detected
//     *   10 = Loco Specific SoS received
//     *   11 = Station General SoS received
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->brake_application_reason);
//    bit_index += 8;
//
//    /* Loco Field 46: Station General SoS — 3 bytes
//     *   B2–B1: Station ID
//     *   B0: General SoS status (1=Received, 2=Cancelled)
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->station_general_sos[2]); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->station_general_sos[1]); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->station_general_sos[0]); bit_index += 8;
//
//    /* Loco Field 47: Station Loco Specific SoS — 3 bytes
//     *   B2–B1: Station ID
//     *   B0: Specific SoS status (1=Received, 2=Cancelled)
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->station_loco_specific_sos[2]); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->station_loco_specific_sos[1]); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->station_loco_specific_sos[0]); bit_index += 8;
//
//    /* Loco Field 48: Collision Detection — 4 bytes
//     *   B3–B1: Loco ID
//     *   B0:    SoS code
//     *   Values:
//     *     1 = Manual SoS received
//     *     2 = Manual SoS cancelled
//     *     3 = Unusual stoppage detected
//     *     4 = Unusual stopage end
//     *     5 = Head-on collision detected
//     *     6 = Head-on collision end
//     *     7 = Rear-end collision detected
//     *     8 = Rear-end collision end
//     *     9 = Train parting detected
//     *    10 = Train parting end
//     *   on detection of event */
//    set_bits(buf, bit_index, 32, h->collision_detection);
//    bit_index += 32;
//
//    /* Loco Field 49: Loco Self SoS — 1 byte
//     *   1 = Manual SoS
//     *   2 = Manual SoS end
//     *   3 = Unusual stopage start
//     *   4 = Unusual stopage end
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->loco_self_sos);
//    bit_index += 8;
//
//    /* Loco Field 50: KAVACH Connection — 1 byte
//     *   1 = KAVACH Isolated
//     *   2 = KAVACH Connected */
//    set_bits(buf, bit_index, 8, h->kavach_connection);
//    bit_index += 8;
//
//    /* Loco Field 51: BIU Isolated — 1 byte
//     *   0 = on detection of event
//     *   1 = BIU Isolated
//     *   2 = BIU Connected */
//    set_bits(buf, bit_index, 8, h->biu_isolated);
//    bit_index += 8;
//
//    /* Loco Field 52: EB Bypassed — 1 byte
//     *   1 = EB Connected
//     *   2 = EB Bypassed
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->eb_bypassed);
//    bit_index += 8;
//
//    /* Loco Field 53: KAVACH Territory — 1 byte
//     *   1 = KAVACH Entry
//     *   2 = KAVACH Exit
//     *   3 = ETCS Entry
//     *   4 = ETCS Exit
//     *   on detection of event */
//    set_bits(buf, bit_index, 8, h->kavach_territory);
//    bit_index += 8;
//
//    /* Loco Field 54: Brake Interface Error — 1 byte
//     *   IRAB / CCB / E70 */
//    set_bits(buf, bit_index, 8, h->brake_interface_error);
//    bit_index += 8;
//
//    /* Loco Field 55: Onboard KAVACH Modules Health — 2 bytes
//     *   b15–b4: Module ID
//     *   b3–b0:  Module Health
//     *   Module Health: 0=NOT OK, 1=OK
//     *   on detection of event */
//    set_bits(buf, bit_index, 16, h->onboard_modules_health);
//    bit_index += 16;
//
//    /* Loco Field 56: Conflict Route RFID — 2 bytes
//     *   on detection of conflicting route RFID */
//    set_bits(buf, bit_index, 16, h->conflict_route_rfid);
//    bit_index += 16;
//
//    /* Loco Field 57: Train Configuration Data Checksum — 4 bytes
//     *   This is Train Configuration Checksum selected by LP */
//    set_bits(buf, bit_index, 32, h->train_configuration_checksum);
//    bit_index += 32;
//
//    /* =========================================================
//     * Fields 58–199: Reserved — leave as 0 (already memset)
//     * Fields 200–254: Firm specific events — 2 bytes each
//     *                 (field info specific to KAVACH firm)
//     * Field 255:      Specific value — Not to be used
//     * ========================================================= */
//
//    /* Field 13 (Header): CRC — 4 bytes
//     *   CCITT-32 Bit CRC (0x04C11DB7)
//     *   Excluding SOF field */
//    return (uint8_t)((bit_index + 7) / 8); /* return total byte length */
//}

static uint8_t build_lkavach_position_info_payload_to_nms(uint8_t *buf)
{
    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);
    uint16_t bit_index = 0;
    nms_kavach_postion_t *h = &nms_ctx.kavach_postion;

    /* Field 1: Start of Frame (SOF) — 2 bytes = 16 bits */
    h->SOF = 0xAAAA;
    set_bits(buf, bit_index, 16, h->SOF);
    bit_index += 16;

    /* Field 2: Message Type — 1 byte = 8 bits */
    h->msg_type = 0x12;
    set_bits(buf, bit_index, 8, h->msg_type);
    bit_index += 8;

    /* Field 3: Message Length — 2 bytes = 16 bits
       (from "Message Type" to "CRC", inclusive) */
    h->msg_len = 0x006F;
    set_bits(buf, bit_index, 16, h->msg_len);
    bit_index += 16;

    /* Field 4: Message Sequence — 2 bytes = 16 bits */
    set_bits(buf, bit_index, 16, h->message_sequence);
    bit_index += 16;

    /* Field 5: Stationary KAVACH ID — 3 bytes = 24 bits */
    set_bits(buf, bit_index, 24, h->stationary_kavach_id & 0xFFFFFF);
    bit_index += 24;

    /* Field 6: NMS System ID — 2 bytes = 16 bits */
    set_bits(buf, bit_index, 16, h->nms_system_id);
    bit_index += 16;

    /* Field 7: System Version — 1 byte = 8 bits */
    set_bits(buf, bit_index, 8, h->system_version);
    bit_index += 8;

    /* =========================================================
     * Field 8: Date — 3 bytes = 24 bits
     *   DD/MM/YY (IST Time-Configurable)
     *   Byte[0] = Day   (01–31; 0x00 = not used; 0xFF = unknown)
     *   Byte[1] = Month (01–12; 0x00 = not used; 0xFF = unknown)
     *   Byte[2] = Year  (00–99 official; 100–254 = not used; 0xFF = unknown)
     *   e.g. 27/04/18 → 0x1B-0x04-0x12
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->date_day);   bit_index += 8;
    set_bits(buf, bit_index, 8, h->date_month); bit_index += 8;
    set_bits(buf, bit_index, 8, h->date_year);  bit_index += 8;

    /* =========================================================
     * Field 9: Time — 3 bytes = 24 bits
     *   HH:MM:SS (IST Time-Configurable)
     *   Byte[0] = Hours   (00–23; 24–254 = not used; 0xFF = unknown)
     *   Byte[1] = Minutes (00–59; 60–254 = not used; 0xFF = unknown)
     *   Byte[2] = Seconds (00–59; 60–254 = not used; 0xFF = unknown)
     *   e.g. 06:36:10 → 0x06-0x24-0x0A
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->time_hour); bit_index += 8;
    set_bits(buf, bit_index, 8, h->time_min);  bit_index += 8;
    set_bits(buf, bit_index, 8, h->time_sec);  bit_index += 8;

    /* Field 10: Onboard Active Radio — 1 byte = 8 bits */
    set_bits(buf, bit_index, 8, h->onboard_active_radio);
    bit_index += 8;

    /* Field 11: SOF Tx Byte 1 — 1 byte = 8 bits */
    set_bits(buf, bit_index, 8, h->sof_tx_byte1);
    bit_index += 8;

    /* Field 12: SOF Tx Byte 2 — 1 byte = 8 bits */
    set_bits(buf, bit_index, 8, h->sof_tx_byte2);
    bit_index += 8;

    /* Field 13: MA Section Count — 1 byte = 8 bits */
    set_bits(buf, bit_index, 8, h->no_of_ma_section_count);
    bit_index += 8;

    /* Field 14: Route ID — 1 byte = 8 bits */
    set_bits(buf, bit_index, 8, h->route_id);
    bit_index += 8;

    return (uint8_t)((bit_index + 7) / 8); /* return total byte length */
}
//static uint8_t build_skavach_rssi_msg_payload_to_nms(uint8_t *buf)
//{
//    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);
//    uint16_t bit_index = 0;
//    nms_lkavach_rssi_t *h = &nms_ctx.kavach_rssi;
//
//    /* Field 1: Start of Frame (SOF) — 2 bytes = 16 bits */
//    h->SOF = 0xAAAA;
//    set_bits(buf, bit_index, 16, h->SOF);
//    bit_index += 16;
//
//    /* Field 2: Message Type — 1 byte = 8 bits */
//    h->msg_type = 0x20;
//    set_bits(buf, bit_index, 8, h->msg_type);
//    bit_index += 8;
//
//    /* Field 3: Message Length — 2 bytes = 16 bits
//       (from "Message Type" to "CRC", inclusive) */
//    h->msg_len = 0x006F;
//    set_bits(buf, bit_index, 16, h->msg_len);
//    bit_index += 16;
//
//    /* Field 4: Message Sequence — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->message_sequence);
//    bit_index += 16;
//
//    /* Field 5: LOCO KAVACH ID — 3 bytes = 24 bits */
//    set_bits(buf, bit_index, 24, h->loco_kavach_id & 0xFFFFFF);
//    bit_index += 24;
//
//    /* Field 6: NMS System ID — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->nms_system_id);
//    bit_index += 16;
//
//    /* Field 7: System Version — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->system_version);
//    bit_index += 8;
//
//    /* =========================================================
//     * Field 8: Date — 3 bytes = 24 bits
//     *   DD/MM/YY (IST Time-Configurable)
//     *   Byte[0] = Day   (01–31; 0x00 = not used; 0xFF = unknown)
//     *   Byte[1] = Month (01–12; 0x00 = not used; 0xFF = unknown)
//     *   Byte[2] = Year  (00–99 official; 100–254 = not used; 0xFF = unknown)
//     *   e.g. 27/04/18 → 0x1B-0x04-0x12
//     * ========================================================= */
//    set_bits(buf, bit_index, 8, h->date_day);   bit_index += 8;
//    set_bits(buf, bit_index, 8, h->date_month); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->date_year);  bit_index += 8;
//
//    /* =========================================================
//     * Field 9: Time — 3 bytes = 24 bits
//     *   HH:MM:SS (IST Time-Configurable)
//     *   Byte[0] = Hours   (00–23; 24–254 = not used; 0xFF = unknown)
//     *   Byte[1] = Minutes (00–59; 60–254 = not used; 0xFF = unknown)
//     *   Byte[2] = Seconds (00–59; 60–254 = not used; 0xFF = unknown)
//     *   e.g. 06:36:10 → 0x06-0x24-0x0A
//     * ========================================================= */
//    set_bits(buf, bit_index, 8, h->time_hour); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->time_min);  bit_index += 8;
//    set_bits(buf, bit_index, 8, h->time_sec);  bit_index += 8;
//
//    /* Field 10: KAVACH Subsystem Type — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->kavach_subsystem_type);
//    bit_index += 16;
//
//    /* Field 11: Station Radio-1 RSSI Sample Count — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->staton_radio1_rssi_sample_count);
//    bit_index += 8;
//
//    /* Field 12: Ref_RFID Tag — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->ref_rfid_tag);
//    bit_index += 16;
//
//    /* Field 13: Abs_Ref_RFID Tag — 3 bytes = 24 bits */
//    set_bits(buf, bit_index, 24, h->abs_ref_rfid_tag & 0xFFFFFF);
//    bit_index += 24;
//
//    /* Field 14: RSSI Value — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->rssi_value);
//    bit_index += 16;
//
//    /* Field 15: Station Radio-2 RSSI Sample Count — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->staton_radio2_rssi_sample_count);
//    bit_index += 8;
//
//    /* Field 16: Ref_RFID Tag 2 — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->ref_rfid_tag2);
//    bit_index += 16;
//
//    /* Field 17: Abs_Ref_RFID Tag 2 — 3 bytes = 24 bits */
//    set_bits(buf, bit_index, 24, h->abs_ref_rfid_tag2 & 0xFFFFFF);
//    bit_index += 24;
//
//    /* Field 18: RSSI Value 2 — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->rssi_value2);
//    bit_index += 16;
//
//    return (uint8_t)((bit_index + 7) / 8); /* return total byte length */
//}

static uint8_t build_skavach_rssi_msg_payload_to_nms(uint8_t *buf)
{
    uint16_t bit_index = 0U;
    nms_skavach_rssi_t *h = &nms_ctx.skavach_rssi;
    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);

    /* =========================================================
     * Field 1: Start of Frame
     * Size: 2 bytes
     * Value: 0xAAAA
     * ========================================================= */
    h->SOF = 0xAAAAU;
    set_bits(buf, bit_index, 16U, h->SOF);
    bit_index += 16U;

    /* =========================================================
     * Field 2: Message Type
     * Size: 1 byte
     * Value: 0x21
     * ========================================================= */
    h->msg_type = 0x21U;
    set_bits(buf, bit_index, 8U, h->msg_type);
    bit_index += 8U;

    /* =========================================================
     * Field 3: Message Length
     *
     * Message Length = bytes from Message Type
     * through CRC inclusive.
     *
     * Total = 39 bytes = 0x0027
     * ========================================================= */
    h->msg_len = 39U;
    set_bits(buf, bit_index, 16U, h->msg_len);
    bit_index += 16U;

    /* =========================================================
     * Field 4: Message Sequence
     * Size: 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16U, h->message_sequence);
    bit_index += 16U;

    /* =========================================================
     * Field 5: Stationary KAVACH ID
     * Size: 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16U, h->stationary_kavach_id);
    bit_index += 16U;

    /* =========================================================
     * Field 6: NMS System ID
     * Size: 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16U, h->nms_system_id);
    bit_index += 16U;

    /* =========================================================
     * Field 7: System Version
     * Size: 1 byte
     * Value: 1
     * ========================================================= */
    h->system_version = 1U;
    set_bits(buf, bit_index, 8U, h->system_version);
    bit_index += 8U;

    /* =========================================================
     * Field 8: Date
     * DD / MM / YY
     * ========================================================= */
    set_bits(buf, bit_index, 8U, h->date_day);
    bit_index += 8U;

    set_bits(buf, bit_index, 8U, h->date_month);
    bit_index += 8U;

    set_bits(buf, bit_index, 8U, h->date_year);
    bit_index += 8U;

    /* =========================================================
     * Field 9: Time
     * HH / MM / SS
     * ========================================================= */
    set_bits(buf, bit_index, 8U, h->time_hour);
    bit_index += 8U;

    set_bits(buf, bit_index, 8U, h->time_min);
    bit_index += 8U;

    set_bits(buf, bit_index, 8U, h->time_sec);
    bit_index += 8U;

    /* =========================================================
     * Field 10: Loco KAVACH ID
     * Size: 3 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 24U, h->loco_kavach_id & 0xFFFFFFUL);
    bit_index += 24U;

    /* =========================================================
     * Field 11: Onboard Radio-1 RSSI Sample Count
     * Size: 1 byte
     * ========================================================= */
    set_bits(buf, bit_index, 8U, h->onboard_radio1_rssi_sample_count);
    bit_index += 8U;

    /* =========================================================
     * Field 12: Ref_RFID Tag
     * Size: 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16U, h->ref_rfid_tag1);
    bit_index += 16U;

    /* =========================================================
     * Field 13: Absolute Location
     * Size: 3 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 24U, h->abs_location1 & 0xFFFFFFUL);
    bit_index += 24U;

    /* =========================================================
     * Field 14: RSSI Value
     * Size: 2 bytes
     * 16-bit signed value
     * ========================================================= */
    set_bits(buf, bit_index, 16U, (uint16_t)h->rssi_value1);
    bit_index += 16U;

    /* =========================================================
     * Field 15: Onboard Radio-2 RSSI Sample Count
     * Size: 1 byte
     * ========================================================= */
    set_bits(buf, bit_index, 8U, h->onboard_radio2_rssi_sample_count);
    bit_index += 8U;

    /* =========================================================
     * Field 16: Ref_RFID Tag
     * Size: 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16U, h->ref_rfid_tag2);
    bit_index += 16U;

    /* =========================================================
     * Field 17: Absolute Location
     * Size: 3 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 24U, h->abs_location2 & 0xFFFFFFUL);
    bit_index += 24U;

    /* =========================================================
     * Field 18: RSSI Value
     * Size: 2 bytes
     * 16-bit signed value
     * ========================================================= */
    set_bits(buf, bit_index, 16U, (uint16_t)h->rssi_value2);
    bit_index += 16U;

    /** Field 19: CRC
     * CRC is NOT generated here.
     * Downstream CAN/NMS interface device
     * will append the 4-byte CRC. */

    /* Payload currently contains:
     * SOF through RSSI Value 2
     * 37 bytes before CRC. */
    return (uint8_t)((bit_index + 7U) / 8U);
}

//static uint8_t build_lkavach_fault_msg_payload_to_nms(uint8_t *buf)
//{
//    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);
//    uint16_t bit_index = 0;
//    nms_kavach_fault_msg_t *h = &nms_ctx.kavach_fault_msg;
//
//    /* Field 1: Start of Frame (SOF) — 2 bytes = 16 bits */
//    h->SOF = 0xAAAA;
//    set_bits(buf, bit_index, 16, h->SOF);
//    bit_index += 16;
//
//    /* Field 2: Message Type — 1 byte = 8 bits */
//    h->msg_type = 0x19;
//    set_bits(buf, bit_index, 8, h->msg_type);
//    bit_index += 8;
//
//    /* Field 3: Message Length — 2 bytes = 16 bits
//       (from "Message Type" to "CRC", inclusive) */
//    h->msg_len = 0x006F;
//    set_bits(buf, bit_index, 16, h->msg_len);
//    bit_index += 16;
//
//    /* Field 4: Message Sequence — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->message_sequence);
//    bit_index += 16;
//
//    /* Field 5: KAVACH Subsystem ID — 3 bytes = 24 bits */
//    set_bits(buf, bit_index, 24, h->kavach_subsystem_id & 0xFFFFFF);
//    bit_index += 24;
//
//    /* Field 6: NMS System ID — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->nms_system_id);
//    bit_index += 16;
//
//    /* Field 7: System Version — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->system_version);
//    bit_index += 8;
//
//    /* =========================================================
//     * Field 8: Date — 3 bytes = 24 bits
//     *   DD/MM/YY (IST Time-Configurable)
//     *   Byte[0] = Day   (01–31; 0x00 = not used; 0xFF = unknown)
//     *   Byte[1] = Month (01–12; 0x00 = not used; 0xFF = unknown)
//     *   Byte[2] = Year  (00–99 official; 100–254 = not used; 0xFF = unknown)
//     *   e.g. 27/04/18 → 0x1B-0x04-0x12
//     * ========================================================= */
//    set_bits(buf, bit_index, 8, h->date_day);   bit_index += 8;
//    set_bits(buf, bit_index, 8, h->date_month); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->date_year);  bit_index += 8;
//
//    /* =========================================================
//     * Field 9: Time — 3 bytes = 24 bits
//     *   HH:MM:SS (IST Time-Configurable)
//     *   Byte[0] = Hours   (00–23; 24–254 = not used; 0xFF = unknown)
//     *   Byte[1] = Minutes (00–59; 60–254 = not used; 0xFF = unknown)
//     *   Byte[2] = Seconds (00–59; 60–254 = not used; 0xFF = unknown)
//     *   e.g. 06:36:10 → 0x06-0x24-0x0A
//     * ========================================================= */
//    set_bits(buf, bit_index, 8, h->time_hour); bit_index += 8;
//    set_bits(buf, bit_index, 8, h->time_min);  bit_index += 8;
//    set_bits(buf, bit_index, 8, h->time_sec);  bit_index += 8;
//
//    /* Field 10: KAVACH Subsystem Type — 2 bytes = 16 bits */
//    set_bits(buf, bit_index, 16, h->kavach_subsystem_type);
//    bit_index += 16;
//
//    /* Field 11: Total Fault Codes (F) — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->total_fault_codes);
//    bit_index += 8;
//
//    /* Field 12: Module ID — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->module_id);
//    bit_index += 8;
//
//    /* Field 13: Fault Code Type — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->fault_code_type);
//    bit_index += 8;
//
//    /* Field 14: Fault Code — 1 byte = 8 bits */
//    set_bits(buf, bit_index, 8, h->fault_code);
//    bit_index += 8;
//
//    return (uint8_t)((bit_index + 7) / 8); /* return total byte length */
//}

static uint16_t build_skavach_fault_msg_payload_to_nms(uint8_t *buf)
{
    uint16_t bit_index = 0;

    nms_kavach_fault_msg_t *h = &nms_ctx.kavach_fault_msg;
    memset(buf, 0, NMS_MAX_PAYLOAD_LEN);

    /* =========================================================
     * FIELD 1
     * Start of Frame
     * 0xAAAA = E1 / Network channel
     * 0xBBBB = GPRS channel
     * For E1/NMS:
     * 0xAAAA
     * ========================================================= */
    h->SOF = 0xAAAA;
    set_bits(buf, bit_index, 16, h->SOF);
    bit_index += 16;

    /* =========================================================
     * FIELD 2
     * Message Type
     * 0x19
     * ========================================================= */
    h->msg_type = NMS_PKT_TYPE_FAULT;
    set_bits(buf, bit_index, 8, h->msg_type);
    bit_index += 8;

    /* =========================================================
     * FIELD 3
     * Message Length
     * From Message Type through CRC inclusive.
     * Fixed fields:
     * Message Type          1
     * Message Length        2
     * Message Sequence      2
     * KAVACH Subsystem ID   3
     * NMS System ID         2
     * System Version        1
     * Date                  3
     * Time                  3
     * Subsystem Type        1
     * Total Fault Codes     1
     * Fault entry           4 * F
     * CRC                   4
     * Therefore:
     * Message Length = 23 + (4 * F)
     * ========================================================= */

    if (h->total_fault_codes > NMS_MAX_FAULT_CODES)
    {
        h->total_fault_codes = NMS_MAX_FAULT_CODES;
    }

    h->msg_len = (uint16_t)(23U + (4U * h->total_fault_codes));
    set_bits(buf, bit_index, 16, h->msg_len);
    bit_index += 16;

    /* =========================================================
     * FIELD 4
     * Message Sequence
     * 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16, h->message_sequence);
    bit_index += 16;

    /* =========================================================
     * FIELD 5
     * KAVACH Subsystem ID
     * 3 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 24, h->kavach_subsystem_id & 0xFFFFFFUL);
    bit_index += 24;

    /* =========================================================
     * FIELD 6
     * NMS System ID
     * 2 bytes
     * ========================================================= */
    set_bits(buf, bit_index, 16, h->nms_system_id);
    bit_index += 16;

    /* =========================================================
     * FIELD 7
     * System Version
     * 1 byte
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->system_version);
    bit_index += 8;

    /* =========================================================
     * FIELD 8
     * Date
     * DD / MM / YY
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->date_day);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->date_month);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->date_year);
    bit_index += 8;

    /* =========================================================
     * FIELD 9
     * Time
     * HH / MM / SS
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->time_hour);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->time_min);
    bit_index += 8;

    set_bits(buf, bit_index, 8, h->time_sec);
    bit_index += 8;

    /* =========================================================
     * FIELD 10
     * KAVACH Subsystem Type
     * 0x11 = Stationary KAVACH
     * 0x22 = Onboard KAVACH
     * 0x33 = TSRMS
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->kavach_subsystem_type);
    bit_index += 8;

    /* =========================================================
     * FIELD 11
     * Total Fault Codes
     * Maximum = 10
     * ========================================================= */
    set_bits(buf, bit_index, 8, h->total_fault_codes);
    bit_index += 8;

    /* =========================================================
     * FIELDS 12-14
     * Repeat for every fault:
     * Field 12: Module ID       1 byte
     * Field 13: Fault Code Type 1 byte
     * Field 14: Fault Code      2 bytes
     * ========================================================= */

    for (uint8_t i = 0; i < h->total_fault_codes; i++)
    {
        /* Field 12: Module ID */
        set_bits(buf, bit_index, 8, h->fault[i].module_id);
        bit_index += 8;

        /* Field 13: Fault Code Type */
        set_bits(buf, bit_index, 8, h->fault[i].fault_code_type);
        bit_index += 8;

        /* Field 14: Fault Code */
        set_bits(buf, bit_index, 16, h->fault[i].fault_code);
        bit_index += 16;
    }

    /*
     * CRC is NOT generated here.
     *
     * Downstream CAN/NMS interface device
     * will append CRC.
     */

    /* =========================================================
     * RETURN TOTAL MESSAGE LENGTH
     * Includes SOF + Message Type ... CRC
     * ========================================================= */
    return (uint16_t)((bit_index + 7U) / 8U);
}

// FUNCTIONS FOR SENDING MESSEGES TO NMS
void send_skavach_info_msg_to_nms(uint8_t skavach_info_frame_num)
{
    uint8_t can_frame[8];
    nms_ctx.payload_len = build_info_payload_to_nms(nms_ctx.payload);
    nms_ctx.seq_total = (nms_ctx.payload_len + NMS_PAYLOAD_BYTES - 1U) / NMS_PAYLOAD_BYTES;

    if (nms_ctx.seq_total >= NMS_MAX_FRAGMENTS)
        return;

    nms_build_fragment(can_frame,NMS_PKT_TYPE_INFO,nms_ctx.seq_total, skavach_info_frame_num);
    canTransmit(canREG1, NMS_TX_MB, can_frame);
    canTransmit(canREG2, NMS_TX_MB, can_frame);
}

void send_skavach_health_msg_to_nms(uint8_t skavach_health_frame_num)
{
    uint8_t can_frame[8];
    nms_ctx.payload_len = build_health_payload_to_nms(nms_ctx.payload);
    nms_ctx.seq_total = (nms_ctx.payload_len + NMS_PAYLOAD_BYTES - 1U) / NMS_PAYLOAD_BYTES;

    if (nms_ctx.seq_total >= NMS_MAX_FRAGMENTS)
        return;

    nms_build_fragment(can_frame,NMS_PKT_TYPE_HEALTH,nms_ctx.seq_total, skavach_health_frame_num);
    canTransmit(canREG1, NMS_TX_MB, can_frame);
    canTransmit(canREG2, NMS_TX_MB, can_frame);
}

void send_skavach_rssi_msg_to_nms(uint8_t skavach_rssi_frame_num)
{
    uint8_t can_frame[8];
    nms_ctx.payload_len = build_skavach_rssi_msg_payload_to_nms(nms_ctx.payload);
    nms_ctx.seq_total = (nms_ctx.payload_len + NMS_PAYLOAD_BYTES - 1U) / NMS_PAYLOAD_BYTES;

    if (nms_ctx.seq_total >= NMS_MAX_FRAGMENTS)
        return;

    nms_build_fragment(can_frame,NMS_PKT_TYPE_RSSI,nms_ctx.seq_total, skavach_rssi_frame_num);
    canTransmit(canREG1, NMS_TX_MB, can_frame);
    canTransmit(canREG2, NMS_TX_MB, can_frame);
}

void send_skavach_fault_msg_to_nms(uint8_t skavach_fault_frame_num)
{
    uint8_t can_frame[8];
    nms_ctx.payload_len = build_skavach_fault_msg_payload_to_nms(nms_ctx.payload);
    nms_ctx.seq_total = (nms_ctx.payload_len + NMS_PAYLOAD_BYTES - 1U) / NMS_PAYLOAD_BYTES;

    if (nms_ctx.seq_total >= NMS_MAX_FRAGMENTS)
        return;

    nms_build_fragment(can_frame,NMS_PKT_TYPE_FAULT,nms_ctx.seq_total, skavach_fault_frame_num);
    canTransmit(canREG1,NMS_TX_MB , can_frame);
    canTransmit(canREG2,NMS_TX_MB , can_frame);
}

void send_loco_postion_info_to_nms(uint8_t stn_loco_postion_frame_num)
{
    uint8_t can_frame[8];
    nms_ctx.payload_len = build_lkavach_position_info_payload_to_nms(nms_ctx.payload);
    nms_ctx.seq_total = (nms_ctx.payload_len + NMS_PAYLOAD_BYTES - 1U) /NMS_PAYLOAD_BYTES;

    if (nms_ctx.seq_total >= NMS_MAX_FRAGMENTS)
        return;

    nms_build_fragment(can_frame,NMS_PKT_TYPE_POS_INFO,nms_ctx.seq_total, stn_loco_postion_frame_num);
    canTransmit(canREG1,NMS_TX_MB, can_frame);
    canTransmit(canREG2,NMS_TX_MB, can_frame);
}
