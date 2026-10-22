/*
 * tlm_proto.h - testbed <-> PC binary protocol (pure C, no HAL).
 *
 * Frame on the wire (all multi-byte fields little-endian):
 *
 *   +------+------+------+-----+-------------+---------+
 *   | 0xA5 | 0x5A | type | len | payload[len]| crc16   |
 *   +------+------+------+-----+-------------+---------+
 *
 *   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type,len,payload.
 *
 * This file must stay in sync with host/tbproto.py. PROTO_VERSION is sent in
 * the HELLO frame so the host can refuse a mismatched firmware.
 */
#ifndef TLM_PROTO_H
#define TLM_PROTO_H

#include <stdint.h>
#include <stddef.h>

#define TLM_PROTO_VERSION   1u

#define TLM_SYNC0           0xA5u
#define TLM_SYNC1           0x5Au
#define TLM_MAX_PAYLOAD     96u
#define TLM_OVERHEAD        6u          /* 2 sync + type + len + 2 crc */
#define TLM_MAX_FRAME       (TLM_MAX_PAYLOAD + TLM_OVERHEAD)

/* ---- frame types: board -> PC ---- */
#define TLM_T_TELEMETRY     0x01u
#define TLM_T_HELLO         0x02u       /* sent on boot and in reply to PING */
#define TLM_T_TEXT          0x03u       /* free-form debug string, not NUL-terminated */

/* ---- frame types: PC -> board ---- */
#define TLM_T_MOTOR_CMD     0x10u
#define TLM_T_ARM           0x11u       /* no payload */
#define TLM_T_DISARM        0x12u       /* no payload */
#define TLM_T_SET_RATE      0x13u       /* uint16 rate_hz */
#define TLM_T_PING          0x14u       /* no payload, board answers HELLO */

/* ---- status flags in telemetry ---- */
#define TLM_F_ARMED         (1u << 0)   /* motor commands are being applied */
#define TLM_F_CMD_TIMEOUT   (1u << 1)   /* no MOTOR_CMD within timeout, motors at idle */
#define TLM_F_ENC_FAULT     (1u << 2)   /* sensor driver reported bad encoder read */
#define TLM_F_IMU_FAULT     (1u << 3)   /* sensor driver reported bad IMU read */
#define TLM_F_TX_OVERRUN    (1u << 4)   /* at least one frame dropped since last sent frame */
#define TLM_F_CLAMPED       (1u << 5)   /* last command was clamped to limits */
#define TLM_F_KILLED        (1u << 6)   /* hardware kill switch latched, reset the board */

#pragma pack(push, 1)

/* 60 bytes. One per sample period. */
typedef struct {
    uint32_t seq;           /* increments every sample, gaps = lost frames */
    uint32_t t_us;          /* board time at sensor sample, microseconds (wraps ~71 min) */
    float    q[2];          /* joint angles, rad */
    float    dq[2];         /* joint rates, rad/s */
    float    acc[3];        /* IMU accel, m/s^2, board frame */
    float    gyro[3];       /* IMU gyro, rad/s, board frame */
    uint16_t motor_us[4];   /* ESC pulse applied right after this sample, us */
    uint16_t cmd_seq;       /* seq of the MOTOR_CMD that produced motor_us */
    uint8_t  flags;         /* TLM_F_* */
    uint8_t  reserved;
} tlm_telemetry_t;

/* 12 bytes. */
typedef struct {
    uint16_t seq;           /* host counter, echoed in telemetry.cmd_seq */
    uint16_t reserved;
    uint16_t motor_us[4];   /* requested pulse per ESC, us */
} tlm_motor_cmd_t;

/* 8 bytes. */
typedef struct {
    uint8_t  proto_version;
    uint8_t  reserved;
    uint16_t rate_hz;       /* current telemetry rate */
    uint16_t us_min;        /* idle/disarmed pulse */
    uint16_t us_max;        /* max pulse the board will accept (clamp) */
} tlm_hello_t;

#pragma pack(pop)

/* Compile-time layout checks (the Python side relies on these sizes). */
typedef char tlm_chk_telem[(sizeof(tlm_telemetry_t) == 60) ? 1 : -1];
typedef char tlm_chk_cmd[(sizeof(tlm_motor_cmd_t) == 12) ? 1 : -1];
typedef char tlm_chk_hello[(sizeof(tlm_hello_t) == 8) ? 1 : -1];

uint16_t tlm_crc16(const uint8_t *data, size_t n, uint16_t crc);

/* Build a frame into out (size >= len + TLM_OVERHEAD). Returns bytes written. */
size_t tlm_encode(uint8_t type, const void *payload, uint8_t len, uint8_t *out);

/* Streaming parser: feed bytes one at a time, it resyncs by itself. */
typedef struct {
    uint8_t  state;
    uint8_t  type;
    uint8_t  len;
    uint8_t  idx;
    uint16_t crc_rx;
    uint8_t  buf[TLM_MAX_PAYLOAD];
    uint32_t crc_errors;    /* diagnostics */
    uint32_t frames_ok;
} tlm_parser_t;

void tlm_parser_init(tlm_parser_t *p);

/* Returns 1 when a complete valid frame is available in p->type/len/buf. */
int tlm_parser_feed(tlm_parser_t *p, uint8_t byte);

#endif /* TLM_PROTO_H */
