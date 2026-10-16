/**
 *
 * oscp_imu.c
 *
 * Assumes little-endian host (Cortex-M, x86, ARM64) — the same byte order as
 * the IMU wire format. For big-endian targets, the packed-struct memcpy
 * approach will silently give wrong values. Replace the decode_* functions
 * with explicit byte-order helpers in that case. Feel free to reach out for help on this regards.
 *
 * Version: 0.2.0
 **/

/**
 * Includes
 **/

#include "oscp_imu.h"
#include "cobs.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

/**
 * Typedefs
 **/

typedef struct {
    const char *suffix;
    oscp_frame_type_t type;
} oscp_cmd_rsp_entry_t;

/**
 * Constants
 **/

static const uint16_t CRC_LUT[256] = {
        0x0000, 0xA2EB, 0xE73D, 0x45D6, 0x6C91, 0xCE7A, 0x8BAC, 0x2947,
        0xD922, 0x7BC9, 0x3E1F, 0x9CF4, 0xB5B3, 0x1758, 0x528E, 0xF065,
        0x10AF, 0xB244, 0xF792, 0x5579, 0x7C3E, 0xDED5, 0x9B03, 0x39E8,
        0xC98D, 0x6B66, 0x2EB0, 0x8C5B, 0xA51C, 0x07F7, 0x4221, 0xE0CA,
        0x215E, 0x83B5, 0xC663, 0x6488, 0x4DCF, 0xEF24, 0xAAF2, 0x0819,
        0xF87C, 0x5A97, 0x1F41, 0xBDAA, 0x94ED, 0x3606, 0x73D0, 0xD13B,
        0x31F1, 0x931A, 0xD6CC, 0x7427, 0x5D60, 0xFF8B, 0xBA5D, 0x18B6,
        0xE8D3, 0x4A38, 0x0FEE, 0xAD05, 0x8442, 0x26A9, 0x637F, 0xC194,
        0x42BC, 0xE057, 0xA581, 0x076A, 0x2E2D, 0x8CC6, 0xC910, 0x6BFB,
        0x9B9E, 0x3975, 0x7CA3, 0xDE48, 0xF70F, 0x55E4, 0x1032, 0xB2D9,
        0x5213, 0xF0F8, 0xB52E, 0x17C5, 0x3E82, 0x9C69, 0xD9BF, 0x7B54,
        0x8B31, 0x29DA, 0x6C0C, 0xCEE7, 0xE7A0, 0x454B, 0x009D, 0xA276,
        0x63E2, 0xC109, 0x84DF, 0x2634, 0x0F73, 0xAD98, 0xE84E, 0x4AA5,
        0xBAC0, 0x182B, 0x5DFD, 0xFF16, 0xD651, 0x74BA, 0x316C, 0x9387,
        0x734D, 0xD1A6, 0x9470, 0x369B, 0x1FDC, 0xBD37, 0xF8E1, 0x5A0A,
        0xAA6F, 0x0884, 0x4D52, 0xEFB9, 0xC6FE, 0x6415, 0x21C3, 0x8328,
        0x8578, 0x2793, 0x6245, 0xC0AE, 0xE9E9, 0x4B02, 0x0ED4, 0xAC3F,
        0x5C5A, 0xFEB1, 0xBB67, 0x198C, 0x30CB, 0x9220, 0xD7F6, 0x751D,
        0x95D7, 0x373C, 0x72EA, 0xD001, 0xF946, 0x5BAD, 0x1E7B, 0xBC90,
        0x4CF5, 0xEE1E, 0xABC8, 0x0923, 0x2064, 0x828F, 0xC759, 0x65B2,
        0xA426, 0x06CD, 0x431B, 0xE1F0, 0xC8B7, 0x6A5C, 0x2F8A, 0x8D61,
        0x7D04, 0xDFEF, 0x9A39, 0x38D2, 0x1195, 0xB37E, 0xF6A8, 0x5443,
        0xB489, 0x1662, 0x53B4, 0xF15F, 0xD818, 0x7AF3, 0x3F25, 0x9DCE,
        0x6DAB, 0xCF40, 0x8A96, 0x287D, 0x013A, 0xA3D1, 0xE607, 0x44EC,
        0xC7C4, 0x652F, 0x20F9, 0x8212, 0xAB55, 0x09BE, 0x4C68, 0xEE83,
        0x1EE6, 0xBC0D, 0xF9DB, 0x5B30, 0x7277, 0xD09C, 0x954A, 0x37A1,
        0xD76B, 0x7580, 0x3056, 0x92BD, 0xBBFA, 0x1911, 0x5CC7, 0xFE2C,
        0x0E49, 0xACA2, 0xE974, 0x4B9F, 0x62D8, 0xC033, 0x85E5, 0x270E,
        0xE69A, 0x4471, 0x01A7, 0xA34C, 0x8A0B, 0x28E0, 0x6D36, 0xCFDD,
        0x3FB8, 0x9D53, 0xD885, 0x7A6E, 0x5329, 0xF1C2, 0xB414, 0x16FF,
        0xF635, 0x54DE, 0x1108, 0xB3E3, 0x9AA4, 0x384F, 0x7D99, 0xDF72,
        0x2F17, 0x8DFC, 0xC82A, 0x6AC1, 0x4386, 0xE16D, 0xA4BB, 0x0650,
};

static const uint8_t FRAME_LEN[8] = {
    61, /* RAW */
    25, /* EULER */
    29, /* QUATERNION */
    49, /* ROT_MAT */
    64, /* GNSS */
    58, /* DEBUG1 */
    58, /* DEBUG2 */
    40, /* STARTUP */
};

static const oscp_cmd_rsp_entry_t OSCP_CMD_RSP_TABLE[] = {
    {"Command succeed.\r\n", OSCP_FRAME_CMD_SUCCESS},
    {"Command failed.\r\n", OSCP_FRAME_CMD_FAILED},
    {"This command is erroneous.\r\n", OSCP_FRAME_CMD_UNKNOWN},
    {"This command is not implemented yet.\r\n", OSCP_FRAME_CMD_NOT_IMPL},
};

static const char * const OSCP_USR_REG_STR[] = {
    "GXB", "GYB", "GZB", "GOB",     /* Gyro bias */
    "AXB", "AYB", "AZB",            /* Accel bias */
    "IXB", "IYB",                   /* Inclinometer bias */
    "MXB", "MYB", "MZB",            /* Mag bias */
    "MXX", "MYX", "MZX",            /* Mag calibration matrix col X */
    "MXY", "MYY", "MZY",            /* Mag calibration matrix col Y */
    "MXZ", "MYZ", "MZZ",            /* Mag calibration matrix col Z */
    "FCO", "FHS", "FRT",            /* AHRS fusion convention, heading src, recovery period */
    "FGA", "FAR", "FMR",            /* AHRS gain, accel rejection, mag rejection */
    "GFI", "GLP", "GHP",            /* Gyro filter, LPF, HPF */
    "AFI", "ALP", "AHP",            /* Accel filter, LPF, HPF */
};

/**
 * Defines
 **/

#define OSCP_CMD_RSP_TABLE_LEN (sizeof(OSCP_CMD_RSP_TABLE) / sizeof(OSCP_CMD_RSP_TABLE[0]))
#define OSCP_USR_REG_TABLE_LEN  (sizeof(OSCP_USR_REG_STR)    / sizeof(OSCP_USR_REG_STR[0]))

/**
 * Functions
 **/

/* ============================================================================
 * SHARED — used by every integration path
 * ==========================================================================*/

/** CRC-16 (koopman poly 0xD175, normal form 0xA2EB, init 0xFFFF, no reflection). Exposed for tooling and tests. */
uint16_t oscp_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        const uint8_t pos = (uint8_t)((uint8_t)(crc >> 8u) ^ data[i]);
        crc = (uint16_t)((uint16_t)(crc << 8u) ^ CRC_LUT[pos]);
    }
    return crc;
}

static bool crc_check(const uint8_t *frame, size_t len) {
    if (len < 3) return false;

    const uint16_t computed = oscp_crc16(frame, len - 2);
    const uint16_t received = (uint16_t)(((uint16_t)frame[len - 1] << 8u) | (uint16_t)frame[len - 2]);
    return computed == received;
}

/** COBS. Used internally by the RS422 path; exposed for tooling. */
oscp_err_t oscp_cobs_decode(const uint8_t *data, const size_t data_len, uint8_t *dec, const size_t dec_max_len, size_t *dec_len) {
    const cobs_decode_result result = cobs_decode(dec, dec_max_len, data, data_len);
    if (result.status != COBS_DECODE_OK) return OSCP_ERR;

    *dec_len = result.out_len;
    return OSCP_OK;
}

oscp_err_t oscp_cobs_encode(const uint8_t *data, const size_t data_len, uint8_t *enc, const size_t enc_max_len, size_t *enc_len) {
    const cobs_encode_result result = cobs_encode(enc, enc_max_len, data, data_len);
    if (result.status != COBS_ENCODE_OK) return OSCP_ERR;

    *enc_len = result.out_len;
    return OSCP_OK;
}

/** DECODER */
static void decode_raw(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.raw, data, sizeof(oscp_raw_t));
}

static void decode_euler(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.euler, data, sizeof(oscp_euler_t));
}

static void decode_quat(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.quat, data, sizeof(oscp_quat_t));
}

static void decode_rot_mat(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.rot_mat, data, sizeof(oscp_rot_mat_t));
}

static void decode_gnss(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.gnss, data, sizeof(oscp_gnss_t));
}

static void decode_debug_1(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.debug1, data, sizeof(oscp_debug_1_t));
}

static void decode_debug_2(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.debug2, data, sizeof(oscp_debug_2_t));
}

static void decode_startup(oscp_frame_t *frame, const uint8_t *data) {
    memcpy(&frame->content.startup, data, sizeof(oscp_startup_t));
}

/** FRAME DISPATCH (transport-agnostic) */
/* Populates a frame from a validated payload by frame type. The payload must be
 * at least FRAME_LEN[frame_type] bytes; CRC and length are checked by the caller. */
static oscp_err_t dispatch_frame(oscp_frame_t *frame, const uint8_t frame_type, const uint8_t *payload) {
    frame->type = (oscp_frame_type_t)frame_type;
    switch (frame_type) {
        case OSCP_FRAME_RAW:
            decode_raw(frame, payload);
            break;
        case OSCP_FRAME_EULER:
            decode_euler(frame, payload);
            break;
        case OSCP_FRAME_QUATERNION:
            decode_quat(frame, payload);
            break;
        case OSCP_FRAME_ROT_MATRIX:
            decode_rot_mat(frame, payload);
            break;
        case OSCP_FRAME_GNSS:
            decode_gnss(frame, payload);
            break;
        case OSCP_FRAME_DEBUG_1:
            decode_debug_1(frame, payload);
            break;
        case OSCP_FRAME_DEBUG_2:
            decode_debug_2(frame, payload);
            break;
        case OSCP_FRAME_STARTUP:
            decode_startup(frame, payload);
            break;
        default:
            return OSCP_ERR;
    }
    return OSCP_OK;
}

/* ============================================================================
 * TRANSPORT A - Byte-stream parser (RS-422 variant, or any stream transport)
 *
 * Use this when bytes arrive WITHOUT message boundaries. The parser syncs on
 * the 0x00 delimiter, COBS-decodes, verifies CRC, and calls callback once
 * per valid frame. Feed it bytes as they arrive (single byte or buffer).
 *
 * CAN-FD users: do NOT use this section - see Transport B below.
 * ==========================================================================*/

/** PARSER */
/* Helpers */
static oscp_frame_type_t match_cmd_response(const uint8_t *dec, const size_t dec_len) {
    for (size_t i = 0; i < OSCP_CMD_RSP_TABLE_LEN; i++) {
        const char  *suffix = OSCP_CMD_RSP_TABLE[i].suffix;
        const size_t suffixLength   = strlen(suffix);
        if (dec_len >= suffixLength && memcmp(dec + dec_len - suffixLength, suffix, suffixLength) == 0) {
            return OSCP_CMD_RSP_TABLE[i].type;
        }
    }
    return (oscp_frame_type_t)0xFF;
}

static void parser_buffer(oscp_parser_t *parser) {
    uint8_t dec[OSCP_FRAME_MAX_LEN];
    size_t dec_len = 0;

    /* Decode */
    if (oscp_cobs_decode(parser->buf, parser->buf_len, dec, sizeof(dec), &dec_len) != OSCP_OK) {
        /* Decode Error */
        parser->stats.cobs_errors++;
        return;
    }

    /* Look for commands response first */
    const oscp_frame_type_t resp_type = match_cmd_response(dec, dec_len);
    if (resp_type != (oscp_frame_type_t)0xFF) {
        oscp_frame_t resp_frame;
        memset(&resp_frame, 0, sizeof(resp_frame));
        resp_frame.type = resp_type;
        parser->stats.frames_ok++;
        if (parser->cb) {
            parser->cb(&resp_frame, parser->ctx);
        }
        return;
    }

    /* Guard against small frame */
    if (dec_len < OSCP_FRAME_MIN_LEN) {
        parser->stats.framing_errors++;
        return;
    }

    /* Extract the frame type from the first byte */
    const uint8_t frame_type = dec[0] & 0x07u;

    /* frame_type is 0..7 after the 3-bit mask */
    if (dec_len != FRAME_LEN[frame_type]) {
        parser->stats.framing_errors++;
        return;
    }

    /* Guard against CRC errors */
    if (!crc_check(dec, dec_len)) {
        parser->stats.crc_errors++;
        return;
    }

    /* Decode a given valid frame */
    oscp_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    if (dispatch_frame(&frame, frame_type, dec) != OSCP_OK) {
        parser->stats.framing_errors++;
        return;
    }
    parser->stats.frames_ok++;

    /* Callback if available */
    if (parser->cb) {
        parser->cb(&frame, parser->ctx);
    }
}

/* Public */
void oscp_parser_init(oscp_parser_t *parser, oscp_frame_callback_t cb, void *ctx) {
    memset(parser, 0, sizeof(oscp_parser_t));
    parser->cb = cb;
    parser->ctx = ctx;
}

void oscp_parser_reset(oscp_parser_t *parser) {
    parser->buf_len = 0;
    parser->synced = false;
}

void oscp_parser_feed(oscp_parser_t *parser, uint8_t byte) {
    /* Delimiter found */
    if (byte == OSCP_FRAME_DELIM) {
        /* When synced and data available, parse buffer */
        if (parser->synced && parser->buf_len > 0) {
            parser_buffer(parser);
        }

        /* In any case, reset the buffer and assert sync */
        parser->buf_len = 0;
        parser->synced = true;
        return;
    }

    /* When not synced, drop */
    if (!parser->synced) return;

    /* Protect from overflow */
    if (parser->buf_len >= OSCP_FRAME_MAX_LEN) {
        parser->stats.overflows++;
        parser->buf_len = 0;
        parser->synced = false;
        return;
    }

    /* Insert the byte in the current buffer */
    parser->buf[parser->buf_len++] = byte;
}

void oscp_parser_feed_buf(oscp_parser_t *parser, const uint8_t *buf, const size_t buf_len) {
    for (size_t i = 0; i < buf_len; i++) oscp_parser_feed(parser, buf[i]);
}

const oscp_stats_t *oscp_parser_stats(const oscp_parser_t *parser) {
    return &parser->stats;
}

void oscp_parser_stats_reset(oscp_parser_t *parser) {
    memset(&parser->stats, 0, sizeof(parser->stats));
}

/* ============================================================================
 * TRANSPORT B - Message decode (CAN-FD variant, or any framed transport)
 *
 * Use this when the transport ALREADY delivers whole messages (e.g. an FD-CAN
 * controller hands a complete payload). Single stateless call:
 * no parser, no ring buffer, no COBS, no callback.
 *
 * RS-422 users: do NOT need this - see Transport A above.
 *
 * ==========================================================================*/

/** DECODER */
/* Public */
oscp_err_t oscp_frame_decode(const uint8_t *payload, const size_t len, oscp_frame_t *frame) {
    if (!payload || !frame) return OSCP_ERR_NULL;
    if (len < 1u) return OSCP_ERR;

    /* The frame type comes from the header byte. The per-type CAN identifier carries
     * the same meaning and can be cross-checked by the caller before calling, but is not required here. */
    const uint8_t frame_type = payload[0] & 0x07u;
    const size_t expected = FRAME_LEN[frame_type];

    /* CAN-FD pads the payload up to the DLC bucket (as an example, a 61-byte RAW frame
     * rides in a 64-byte message), so the received length is >= the frame's own
     * length rather than exactly equal. */
    if (len < expected) return OSCP_ERR;

    /* CRC-16 is retained on CAN-FD. */
    if (!crc_check(payload, expected)) return OSCP_ERR;

    memset(frame, 0, sizeof(*frame));
    return dispatch_frame(frame, frame_type, payload);
}

/* ============================================================================
 * COMMANDS (to unit, both transports layer)
 *
 * Each command encodes an ASCII string into a buffer, framed for a transport:
 *   OSCP_TRANSPORT_RS422: COBS-encoded, 0x00-delimited
 *   OSCP_TRANSPORT_CANFD: Raw ASCII payload; caller set the CAN id + DLC
 *
 * ==========================================================================*/

/** COMMANDS */
/* Helpers */
/* Frames an ASCII command for the selected transport:
 *   - RS422: COBS-encode the ASCII, then append the 0x00 delimiter.
 *   - CAN-FD: Copy the raw ASCII; the CAN-FD controller frames the message,
 *            and the caller sets the command identifier + DLC. No COBS, no delimiter. */
static oscp_err_t frame_ascii_cmd(const char *ascii, uint8_t *enc, const size_t enc_max_len, size_t *enc_len, const oscp_transport_t transport) {
    if (!ascii || !enc || !enc_len) return OSCP_ERR_NULL;
    if (enc_max_len == 0) return OSCP_ERR;

    const size_t ascii_len = strlen(ascii);

    if (transport == OSCP_TRANSPORT_CANFD) {
        if (ascii_len > enc_max_len) return OSCP_ERR;
        memcpy(enc, ascii, ascii_len);
        *enc_len = ascii_len;
        return OSCP_OK;
    }

    /* OSCP_TRANSPORT_RS422 from here */
    size_t cobs_len = 0;
    const oscp_err_t err = oscp_cobs_encode((const uint8_t*)ascii, strlen(ascii), enc, enc_max_len - 1, &cobs_len);
    if (err != OSCP_OK) return err;

    enc[cobs_len] = OSCP_FRAME_DELIM;
    *enc_len = cobs_len + 1;
    return OSCP_OK;
}

/* Publics */
/** COMMANDS - Silent Class, ANY (no response expected, valid in ANY state) */
oscp_err_t oscp_cmd_reset(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("RESET\r\n", cmd, cmd_max_len, cmd_len, transport);
}

/** COMMANDS - Silent Class, CONFIG (no response expected, valid in CONFIG state only) */
oscp_err_t oscp_cmd_exit(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("EXIT\r\n", cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_refs(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("REFS\r\n", cmd, cmd_max_len, cmd_len, transport);
}

/** COMMANDS - Ack Class, OPERATING (reply expected, valid in OPERATING state only) */
oscp_err_t oscp_cmd_config(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("CONFIG\r\n", cmd, cmd_max_len, cmd_len, transport);
}

/** COMMANDS - Ack Class, CONFIG (reply expected, valid in CONFIG state only) */
oscp_err_t oscp_cmd_suf(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("SUF\r\n", cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_of(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_frame_sel_t frame, const oscp_transport_t transport) {
    switch (frame) {
        case OSCP_FRAME_SEL_RAW:
        case OSCP_FRAME_SEL_EULER:
        case OSCP_FRAME_SEL_QUATERNION:
        case OSCP_FRAME_SEL_ROT_MATRIX:
        case OSCP_FRAME_SEL_GNSS:
        case OSCP_FRAME_SEL_DEBUG:
            break;
        default:
            return OSCP_ERR_INVALID;
    }

    char buf[7];
    snprintf(buf, sizeof(buf), "OF%c\r\n", (char)frame);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_om(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_om_sel_t mode, const oscp_transport_t transport) {
    switch (mode) {
        case OSCP_OM_IDLE:
        case OSCP_OM_LOW:
        case OSCP_OM_MEDIUM:
            break;
        default:
            return OSCP_ERR_INVALID;
    }
    char buf[7];
    snprintf(buf, sizeof(buf), "OM%c\r\n", (char)mode);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}


oscp_err_t oscp_cmd_enable_oft(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_frame_sel_t frame, const oscp_transport_t transport) {
    switch (frame) {
        case OSCP_FRAME_SEL_RAW:
        case OSCP_FRAME_SEL_EULER:
        case OSCP_FRAME_SEL_QUATERNION:
        case OSCP_FRAME_SEL_ROT_MATRIX:
        case OSCP_FRAME_SEL_GNSS:
            break;
        case OSCP_FRAME_SEL_DEBUG:
        default:
            return OSCP_ERR_INVALID;
    }
    char buf[9];
    snprintf(buf, sizeof(buf), "EOFT%c\r\n", (char)frame);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_disable_oft(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_frame_sel_t frame, const oscp_transport_t transport) {
    switch (frame) {
        case OSCP_FRAME_SEL_RAW:
        case OSCP_FRAME_SEL_EULER:
        case OSCP_FRAME_SEL_QUATERNION:
        case OSCP_FRAME_SEL_ROT_MATRIX:
        case OSCP_FRAME_SEL_GNSS:
            break;
        case OSCP_FRAME_SEL_DEBUG:
        default:
            return OSCP_ERR_INVALID;
    }
    char buf[9];
    snprintf(buf, sizeof(buf), "DOFT%c\r\n", (char)frame);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}


oscp_err_t oscp_cmd_drg(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_gyro_dr_t dr, const oscp_transport_t transport) {
    switch (dr) {
        case OSCP_GYRO_DR_125DPS:
        case OSCP_GYRO_DR_250DPS:
        case OSCP_GYRO_DR_500DPS:
        case OSCP_GYRO_DR_1000DPS:
        case OSCP_GYRO_DR_2000DPS:
        case OSCP_GYRO_DR_4000DPS:
            break;
        default:
            return OSCP_ERR_INVALID;
    }

    char buf[11];
    snprintf(buf, sizeof(buf), "DRG%04u\r\n", (unsigned int)dr);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_dra(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_accel_dr_t dr, const oscp_transport_t transport) {
    switch (dr) {
        case OSCP_ACCEL_DR_2G:
        case OSCP_ACCEL_DR_4G:
        case OSCP_ACCEL_DR_8G:
        case OSCP_ACCEL_DR_16G:
            break;
        default:
            return OSCP_ERR_INVALID;
    }
    char buf[9];
    snprintf(buf, sizeof(buf), "DRA%02u\r\n", (unsigned int)dr);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_dri(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_incl_dr_t dr, const oscp_transport_t transport) {
    switch (dr) {
        case OSCP_INCL_DR_0G5:
        case OSCP_INCL_DR_1G0:
        case OSCP_INCL_DR_2G0:
        case OSCP_INCL_DR_3G0:
            break;
        default:
            return OSCP_ERR_INVALID;
    }
    const unsigned int tenths = (unsigned int)dr;
    char buf[10];
    snprintf(buf, sizeof(buf), "DRI%u.%u\r\n", tenths / 10u, tenths % 10u);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_enable_mcorr(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("EMCORR\r\n", cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_disable_mcorr(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("DMCORR\r\n", cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_wr(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_usr_reg_t reg, const uint32_t value, const oscp_transport_t transport) {
    if ((size_t)reg >= OSCP_USR_REG_TABLE_LEN) return OSCP_ERR_INVALID;
    /* Format: WR + 3-char mnemonic + 8 uppercase hex digits + \r\n = 15 chars */
    char buf[17];
    snprintf(buf, sizeof(buf), "WR%s%08" PRIX32 "\r\n", OSCP_USR_REG_STR[reg], value);
    return frame_ascii_cmd(buf, cmd, cmd_max_len, cmd_len, transport);
}

oscp_err_t oscp_cmd_save(uint8_t *cmd, const size_t cmd_max_len, size_t *cmd_len, const oscp_transport_t transport) {
    return frame_ascii_cmd("SAVE\r\n", cmd, cmd_max_len, cmd_len, transport);
}