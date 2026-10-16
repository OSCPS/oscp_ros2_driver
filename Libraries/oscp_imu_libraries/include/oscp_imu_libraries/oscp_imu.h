/**
 * oscp_imu.h
 *
 * Supports: MK2 IMU Product Family
 * Integrity: CRC-16, Koopman poly 0xD175 (normal form 0xA2EB), init 0xFFFF, no reflection
 *
 * Thread safety: oscp_parser_t is NOT thread-safe. External synchronization
 * is required if oscp_parser_feed / oscp_parser_feed_buf and the frame
 * callback are called from different execution contexts.
 *
 * Version: 0.2.0
 */

#ifndef OSCP_IMU_H
#define OSCP_IMU_H

/**
 * Includes
 **/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

/**
 * Defines
 **/

/** Constants */
#define OSCP_FRAME_DELIM    0x00U
#define OSCP_FRAME_MIN_LEN  25U
#define OSCP_FRAME_MAX_LEN  72U

/** Status Byte Masks */
#define OSCP_STATUS_OK          0x00U /** No bits are set */
#define OSCP_STATUS_OVERRUN     0x01U /** IMU Real Time Controller Overrun */
#define OSCP_STATUS_MEMS_ERR    0x02U /** MEMS Sensors Error */
#define OSCP_STATUS_INCL_ERR    0x04U /** Inclinometer Error */
#define OSCP_STATUS_MAG_ERR     0x08U /** Magnetometer Error */
#define OSCP_STATUS_TEMP_ERR    0x10U /** Temperature Sensor Error */
#define OSCP_STATUS_OG_ERR      0x40U /** Optical Gyroscope Error */

/* Enabled Frames Masks */
#define OSCP_ENABLE_RAW         0x01U
#define OSCP_ENABLE_EULER       0x02U
#define OSCP_ENABLE_QUAT        0x04U
#define OSCP_ENABLE_ROT_MAT     0x08U
#define OSCP_ENABLE_GNSS        0x10U

/* Compiler-agnostic packed attribute */
#if defined(__GNUC__) || defined(__clang__)
#  define OSCP_PACKED __attribute__((packed))
#elif defined(_MSC_VER)
#  define OSCP_PACKED  /* use #pragma pack in MSVC — see note below */
#  pragma pack(push, 1)
#endif

/** Big-endian guard: the wire format is little-endian. */
#if defined(__BYTE_ORDER__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#  error "oscp-imu-c: big-endian targets not supported - replace decode_* with explicit byte-swap helpers"
#endif

/**
 * Typedefs
 **/

/* Frame types */
typedef enum {
    OSCP_FRAME_RAW          = 0x00, /** Raw Operating Frame */
    OSCP_FRAME_EULER        = 0x01, /** AHRS Euler Angles Operating Frame */
    OSCP_FRAME_QUATERNION   = 0x02, /** AHRS Quaternions Operating Frame */
    OSCP_FRAME_ROT_MATRIX   = 0x03, /** AHRS Rotation Matrix Operating Frame */
    OSCP_FRAME_GNSS         = 0x04, /** GNSS Operating Frame */
    OSCP_FRAME_DEBUG_1      = 0x05, /** Debug Operating Frame 1 */
    OSCP_FRAME_DEBUG_2      = 0x06, /** Debug Operating Frame 2 */
    OSCP_FRAME_STARTUP      = 0x07, /** Startup Frame */

    OSCP_FRAME_CMD_SUCCESS  = 0xA0, /** Command executed successfully */
    OSCP_FRAME_CMD_FAILED   = 0xA1, /** Command rejected by the unit */
    OSCP_FRAME_CMD_UNKNOWN  = 0xA2, /** Command is not recognized by the unit */
    OSCP_FRAME_CMD_NOT_IMPL = 0xA3, /** Command is recognized but not implemented yet */
} oscp_frame_type_t;

/* Operating modes */
typedef enum {
    OSCP_OP_MODE_IDLE   = 0x00,
    OSCP_OP_MODE_LOW    = 0x01,
    OSCP_OP_MODE_MEDIUM = 0x02,
} oscp_operating_mode_t;

/* Misalignment Correction */
typedef enum {
    OSCP_MISALIGNMENT_CORR_DISABLED = 0x00,
    OSCP_MISALIGNMENT_CORR_ENABLED  = 0x01,
} oscp_misalignment_corr_t;

/* Accessors for the header byte */
#define OSCP_HDR_FRAME_TYPE(frame)          ((oscp_frame_type_t)((frame).header_byte & 0x07u))
#define OSCP_HDR_OPERATING_MODE(frame)      ((oscp_operating_mode_t)(((frame).header_byte >> 3) & 0x07u))
#define OSCP_HDR_MISALIGNMENT_CORR(frame)   ((oscp_misalignment_corr_t)(((frame).header_byte >> 6) & 0x03u))

/* Raw Operating Frame - 61 bytes */
typedef struct OSCP_PACKED {
    uint8_t header_byte;    /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;        /** Wrapping frame counter [0,255] */
    uint64_t timestamp_ms;  /** Timestamp in ms since last power up or reset */
    float gyro_x;           /** dps */
    float gyro_y;           /** dps */
    float gyro_z;           /** dps */
    float accel_x;          /** g */
    float accel_y;          /** g */
    float accel_z;          /** g */
    float incl_x;           /** mg */
    float incl_y;           /** mg */
    float mag_x;            /** uT */
    float mag_y;            /** uT */
    float mag_z;            /** uT */
    float temp;             /** degC */
    uint8_t status;         /** Status byte - use OSCP_STATUS_* masks */
    uint16_t crc;           /** Checksum */
} oscp_raw_t;

/* Euler Angles Frame - 25 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;    /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;        /** Wrapping frame counter [0,255] */
    uint64_t timestamp_ms;  /** Timestamp in ms since last power up or reset */
    float roll;             /** deg */
    float pitch;            /** deg */
    float yaw;              /** deg */
    uint8_t status;         /** Status byte - use OSCP_STATUS_* masks */
    uint16_t crc;           /** Checksum */
} oscp_euler_t;

/* Quaternion Frame - 29 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;    /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;        /** Wrapping frame counter [0,255] */
    uint64_t timestamp_ms;  /** Timestamp in ms since last power up or reset */
    float w;
    float x;
    float y;
    float z;
    uint8_t status;         /** Status byte - use OSCP_STATUS_* masks */
    uint16_t crc;           /** Checksum */
} oscp_quat_t;

/* Rotation Matrix Frame - 49 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;    /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;        /** Wrapping frame counter [0,255] */
    uint64_t timestamp_ms;  /** Timestamp in ms since last power up or reset */
    float rm[3][3];
    uint8_t status;         /** Status byte - use OSCP_STATUS_* masks */
    uint16_t crc;           /** Checksum */
} oscp_rot_mat_t;

/* GNSS Frame - 64 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;            /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;                /** Wrapping frame counter [0,255] */
    uint64_t timestamp_ms;          /** Timestamp in ms since last power up or reset */
    uint8_t gnss_fix_type;
    uint8_t num_satellites;
    float longitude;                /** deg */
    float latitude;                 /** deg */
    int32_t height;                 /** mm */
    uint32_t horizontal_accuracy;   /** mm */
    uint32_t vertical_accuracy;     /** mm */
    int32_t velocity_north;         /** mm/s */
    int32_t velocity_east;          /** mm/s */
    int32_t velocity_down;          /** mm/s */
    uint32_t speed_accuracy;        /** mm/s */
    float heading_of_motion;        /** deg */
    float heading_accuracy;         /** deg */
    float pdop;
    uint8_t gnssFixOk : 1;
    uint8_t invalidLLH : 1;
    uint8_t reserved : 2;
    uint8_t lastCorrectionAge : 4;
    uint8_t status;
    uint16_t crc;
} oscp_gnss_t;

/* Debug Frame 1 - 58 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;    /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;        /** Wrapping frame counter [0,255] */
    uint32_t gxb;
    uint32_t gyb;
    uint32_t gzb;
    uint32_t gob;
    uint32_t axb;
    uint32_t ayb;
    uint32_t azb;
    uint32_t ixb;
    uint32_t iyb;
    uint32_t mxb;
    uint32_t myb;
    uint32_t mzb;
    uint8_t gyroFilters : 2;
    uint8_t gyroLPF : 3;
    uint8_t gyroHPF : 3;
    uint8_t accelFilters : 2;
    uint8_t accelLPF : 3;
    uint8_t accelHPF : 3;
    uint16_t reserved_0;
    uint8_t reserved_1;
    uint8_t status;
    uint16_t crc;
} oscp_debug_1_t;

/* Debug Frame 2 - 58 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;    /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    uint8_t counter;        /** Wrapping frame counter [0,255] */
    uint32_t mxx;
    uint32_t myx;
    uint32_t mzx;
    uint32_t mxy;
    uint32_t myy;
    uint32_t mzy;
    uint32_t mxz;
    uint32_t myz;
    uint32_t mzz;
    uint32_t fusionGain;
    uint32_t fusionAccelRejection;
    uint32_t fusionMagRejection;
    uint32_t fusionRecoveryTriggerPeriod;
    uint8_t fusionConvention : 4;
    uint8_t fusionHeadingSource : 4;
    uint8_t status;
    uint16_t crc;
} oscp_debug_2_t;

/* Startup Frame - 40 bytes decoded */
typedef struct OSCP_PACKED {
    uint8_t header_byte;        /** [7:6] Misalignment Correction (2b) | [5:3] Operating Mode (3b) | [2:0] Frame Type (3b) */
    char mark_number[10];
    uint16_t unit_number;
    uint8_t sw_major_ver;
    uint8_t sw_minor_ver;
    uint8_t sw_patch_ver;
    uint8_t enabled_frames;     /** Use OSCP_ENABLE_* masks */
    uint8_t gyro_dr : 4;
    uint8_t accel_dr : 4;
    uint8_t gyro_filters : 2;
    uint8_t gyro_lpf : 3;
    uint8_t gyro_hpf : 3;
    uint8_t accel_filters : 2;
    uint8_t accel_lpf : 3;
    uint8_t accel_hpf : 3;
    uint8_t incl_dr : 4;
    uint8_t ahrs_convention : 2;
    uint8_t ahrs_heading_src : 2;
    float ahrs_gain;
    float ahrs_accel_rej;
    float ahrs_mag_rej;
    uint32_t ahrs_rec_trig_per; /** Recovery Trigger Period in s */
    uint8_t status;             /** Status byte - use OSCP_STATUS_* masks */
    uint16_t crc;               /** Checksum */
} oscp_startup_t;

#if defined(_MSC_VER)
#  pragma pack(pop)
#endif

/* Tagged union for handling frame callback */
typedef struct {
    oscp_frame_type_t type;
    union {
        oscp_raw_t     raw;
        oscp_euler_t   euler;
        oscp_quat_t    quat;
        oscp_rot_mat_t rot_mat;
        oscp_gnss_t    gnss;
        oscp_debug_1_t debug1;
        oscp_debug_2_t debug2;
        oscp_startup_t startup;
    } content;
} oscp_frame_t;

/* Accessors */
#define oscp_raw(frame)     ((frame)->content.raw)
#define oscp_euler(frame)   ((frame)->content.euler)
#define oscp_quat(frame)    ((frame)->content.quat)
#define oscp_rot_mat(frame) ((frame)->content.rot_mat)
#define oscp_gnss(frame)    ((frame)->content.gnss)
#define oscp_debug_1(frame) ((frame)->content.debug1)
#define oscp_debug_2(frame) ((frame)->content.debug2)
#define oscp_startup(frame) ((frame)->content.startup)

/* Callback invoked during valid decoded frame for an IMU */
typedef void (*oscp_frame_callback_t)(const oscp_frame_t *frame, void *ctx);

/* Parser statistics */
typedef struct {
    uint32_t frames_ok;         /** Number of frames successfully parsed */
    uint32_t framing_errors;    /** Number of frames dropped due to framing errors */
    uint32_t crc_errors;        /** Number of frames dropped due to CRC errors */
    uint32_t cobs_errors;       /** Number of frames dropped due to COBS errors  */
    uint32_t overflows;         /** Number of frames dropped due to buffer overflow */
} oscp_stats_t;

typedef struct {
    oscp_frame_callback_t cb;
    void *ctx;
    uint8_t buf[OSCP_FRAME_MAX_LEN];
    size_t buf_len;
    bool synced;
    oscp_stats_t stats;
} oscp_parser_t;

/* Return codes */
typedef enum {
    OSCP_OK             =  0,
    OSCP_ERR            = -1,
    OSCP_ERR_INVALID    = -3,
    OSCP_ERR_NULL       = -2,
} oscp_err_t;

/**
 * Transport selector for command encoding.
 * - OSCP_TRANSPORT_RS422: COBS-encoded and 0x00-delimited (RS422-based units).
 * - OSCP_TRANSPORT_CANFD: Raw ASCII payload for CAN-FD; the driver frames the
 *                        message, and the caller needs to sets the command identifier + DLC.
 */
typedef enum {
    OSCP_TRANSPORT_RS422 = 0,
    OSCP_TRANSPORT_CANFD  = 1,
} oscp_transport_t;

/**
 * Frame type selector — used by oscp_cmd_of, oscp_cmd_enable_oft,
 * oscp_cmd_disable_oft. OSCP_FRAME_SEL_DEBUG is only valid for
 * oscp_cmd_of; passing it to enable/disable OFT returns OSCP_ERR_INVALID.
 */
typedef enum {
    OSCP_FRAME_SEL_RAW        = 'R',
    OSCP_FRAME_SEL_EULER      = 'E',
    OSCP_FRAME_SEL_QUATERNION = 'Q',
    OSCP_FRAME_SEL_ROT_MATRIX = 'M',
    OSCP_FRAME_SEL_GNSS       = 'G',
    OSCP_FRAME_SEL_DEBUG      = 'D', /** oscp_cmd_of only */
} oscp_frame_sel_t;


/** Operating mode selector — used by oscp_cmd_om */
typedef enum {
    OSCP_OM_IDLE   = 'I',
    OSCP_OM_LOW    = 'L',
    OSCP_OM_MEDIUM = 'M',
} oscp_om_sel_t;

/** Gyroscope dynamic range — used by oscp_cmd_drg */
typedef enum {
    OSCP_GYRO_DR_125DPS  = 125,
    OSCP_GYRO_DR_250DPS  = 250,
    OSCP_GYRO_DR_500DPS  = 500,
    OSCP_GYRO_DR_1000DPS = 1000,
    OSCP_GYRO_DR_2000DPS = 2000,
    OSCP_GYRO_DR_4000DPS = 4000,
} oscp_gyro_dr_t;

/** Accelerometer dynamic range — used by oscp_cmd_dra */
typedef enum {
    OSCP_ACCEL_DR_2G  =  2,
    OSCP_ACCEL_DR_4G  =  4,
    OSCP_ACCEL_DR_8G  =  8,
    OSCP_ACCEL_DR_16G = 16,
} oscp_accel_dr_t;

/**
 * Inclinometer dynamic range — used by oscp_cmd_dri.
 * Values are in tenths of g (5 = 0.5 g, 10 = 1.0 g, 20 = 2.0 g, 30 = 3.0 g).
 */
typedef enum {
    OSCP_INCL_DR_0G5 =  5,
    OSCP_INCL_DR_1G0 = 10,
    OSCP_INCL_DR_2G0 = 20,
    OSCP_INCL_DR_3G0 = 30,
} oscp_incl_dr_t;

/**
 * User register mnemonics — used by oscp_cmd_wr.
 * Gyro bias (GXB/GYB/GZB/GOB), accelerometer bias (AXB/AYB/AZB),
 * inclinometer bias (IXB/IYB), magnetometer bias (MXB/MYB/MZB),
 * magnetometer calibration matrix (MXX…MZZ), AHRS fusion parameters
 * (FCO/FHS/FRT/FGA/FAR/FMR), sensor filter settings (GFI/GLP/GHP/AFI/ALP/AHP).
 */
typedef enum {
    OSCP_USR_REG_GXB, /** Gyroscope X axis user bias */
    OSCP_USR_REG_GYB, /** Gyroscope Y axis user bias */
    OSCP_USR_REG_GZB, /** Gyroscope Z axis user bias */
    OSCP_USR_REG_GOB, /** Optical gyroscope user bias (MK2E2 and MK2Z only) */
    OSCP_USR_REG_AXB, /** Accelerometer X axis user bias */
    OSCP_USR_REG_AYB, /** Accelerometer Y axis user bias */
    OSCP_USR_REG_AZB, /** Accelerometer Z axis user bias */
    OSCP_USR_REG_IXB, /** Inclinometer X axis user bias */
    OSCP_USR_REG_IYB, /** Inclinometer Y axis user bias */
    OSCP_USR_REG_MXB, /** Magnetometer X axis user bias */
    OSCP_USR_REG_MYB, /** Magnetometer Y axis user bias */
    OSCP_USR_REG_MZB, /** Magnetometer Z axis user bias */
    OSCP_USR_REG_MXX, /** Magnetometer calibration: X-to-X */
    OSCP_USR_REG_MYX, /** Magnetometer calibration: Y-to-X */
    OSCP_USR_REG_MZX, /** Magnetometer calibration: Z-to-X */
    OSCP_USR_REG_MXY, /** Magnetometer calibration: X-to-Y */
    OSCP_USR_REG_MYY, /** Magnetometer calibration: Y-to-Y */
    OSCP_USR_REG_MZY, /** Magnetometer calibration: Z-to-Y */
    OSCP_USR_REG_MXZ, /** Magnetometer calibration: X-to-Z */
    OSCP_USR_REG_MYZ, /** Magnetometer calibration: Y-to-Z */
    OSCP_USR_REG_MZZ, /** Magnetometer calibration: Z-to-Z */
    OSCP_USR_REG_FCO, /** AHRS fusion convention */
    OSCP_USR_REG_FHS, /** AHRS fusion heading source */
    OSCP_USR_REG_FRT, /** AHRS fusion recovery trigger period */
    OSCP_USR_REG_FGA, /** AHRS fusion gain */
    OSCP_USR_REG_FAR, /** AHRS fusion accelerometer rejection */
    OSCP_USR_REG_FMR, /** AHRS fusion magnetometer rejection */
    OSCP_USR_REG_GFI, /** Gyroscope filters enable bitmask */
    OSCP_USR_REG_GLP, /** Gyroscope low-pass filter setting */
    OSCP_USR_REG_GHP, /** Gyroscope high-pass filter setting */
    OSCP_USR_REG_AFI, /** Accelerometer filters enable bitmask */
    OSCP_USR_REG_ALP, /** Accelerometer low-pass filter setting */
    OSCP_USR_REG_AHP, /** Accelerometer high-pass filter setting */
} oscp_usr_reg_t;

/**
 * Functions
 **/

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * SHARED — used by every integration path
 * ==========================================================================*/

/** CRC-16 (koopman poly 0xD175, normal form 0xA2EB, init 0xFFFF, no reflection). Exposed for tooling and tests. */
uint16_t oscp_crc16(const uint8_t *data, size_t len);

/** COBS. Used internally by the RS422 path; exposed for tooling. */
oscp_err_t oscp_cobs_decode(const uint8_t *data, size_t data_len, uint8_t *dec, size_t dec_max_len, size_t *dec_len);
oscp_err_t oscp_cobs_encode(const uint8_t *data, size_t data_len, uint8_t *enc, size_t enc_max_len, size_t *enc_len);

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
void oscp_parser_init(oscp_parser_t *parser, oscp_frame_callback_t cb, void *ctx);
void oscp_parser_reset(oscp_parser_t *parser);
void oscp_parser_feed(oscp_parser_t *parser, uint8_t byte);
void oscp_parser_feed_buf(oscp_parser_t *parser, const uint8_t *buf, size_t buf_len);
const oscp_stats_t *oscp_parser_stats(const oscp_parser_t *parser);
void oscp_parser_stats_reset(oscp_parser_t *parser);

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
oscp_err_t oscp_frame_decode(const uint8_t *payload, size_t len, oscp_frame_t *frame);

/* ============================================================================
 * COMMANDS (to unit, both transports layer)
 *
 * Each command encodes an ASCII string into a buffer, framed for a transport:
 *   OSCP_TRANSPORT_RS422: COBS-encoded, 0x00-delimited
 *   OSCP_TRANSPORT_CANFD: Raw ASCII payload; caller set the CAN id + DLC
 *
 * ==========================================================================*/

/** COMMANDS - Silent Class, ANY (no response expected, valid in ANY state) */
oscp_err_t oscp_cmd_reset(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);

/** COMMANDS - Silent Class, CONFIG (no response expected, valid in CONFIG state only) */
oscp_err_t oscp_cmd_exit(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);
oscp_err_t oscp_cmd_refs(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);

/** COMMANDS - Ack Class, OPERATING (reply expected, valid in OPERATING state only) */
oscp_err_t oscp_cmd_config(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);

/** COMMANDS - Ack Class, CONFIG (reply expected, valid in CONFIG state only) */
oscp_err_t oscp_cmd_suf(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);
oscp_err_t oscp_cmd_of(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_frame_sel_t frame, oscp_transport_t transport);
oscp_err_t oscp_cmd_om(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_om_sel_t mode, oscp_transport_t transport);
oscp_err_t oscp_cmd_enable_oft(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_frame_sel_t frame, oscp_transport_t transport);
oscp_err_t oscp_cmd_disable_oft(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_frame_sel_t frame, oscp_transport_t transport);
oscp_err_t oscp_cmd_drg(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_gyro_dr_t dr, oscp_transport_t transport);
oscp_err_t oscp_cmd_dra(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_accel_dr_t dr, oscp_transport_t transport);
oscp_err_t oscp_cmd_dri(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_incl_dr_t dr, oscp_transport_t transport);
oscp_err_t oscp_cmd_enable_mcorr(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);
oscp_err_t oscp_cmd_disable_mcorr(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);
oscp_err_t oscp_cmd_wr(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_usr_reg_t reg, uint32_t value, oscp_transport_t transport);
oscp_err_t oscp_cmd_save(uint8_t *cmd, size_t cmd_max_len, size_t *cmd_len, oscp_transport_t transport);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif // OSCP_IMU_H