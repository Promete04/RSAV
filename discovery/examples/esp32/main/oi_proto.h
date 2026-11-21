/* oi_proto.h - portable Roomba Open Interface protocol helpers (C99).
 * No I/O here: functions fill byte buffers that you send with any transport
 * (Linux serial, ESP32 USB host, ESP32 UART), and parse what comes back. */
#ifndef OI_PROTO_H
#define OI_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Opcodes */
enum {
    OI_RESET = 7, OI_START = 128, OI_SAFE = 131, OI_FULL = 132, OI_POWER = 133,
    OI_SPOT = 134, OI_CLEAN = 135, OI_MAX = 136, OI_DRIVE = 137, OI_MOTORS = 138,
    OI_LEDS = 139, OI_SONG = 140, OI_PLAY = 141, OI_SENSORS = 142, OI_SEEK_DOCK = 143,
    OI_PWM_MOTORS = 144, OI_DRIVE_DIRECT = 145, OI_DRIVE_PWM = 146, OI_STREAM = 148,
    OI_QUERY_LIST = 149, OI_PAUSE_STREAM = 150, OI_STOP = 173
};

/* Packet ids used most often */
enum {
    PKT_BUMPS_DROPS = 7, PKT_WALL = 8, PKT_CLIFF_L = 9, PKT_CLIFF_FL = 10, PKT_CLIFF_FR = 11,
    PKT_CLIFF_R = 12, PKT_OVERCURRENT = 14, PKT_CHARGING_STATE = 21, PKT_VOLTAGE = 22,
    PKT_CURRENT = 23, PKT_TEMPERATURE = 24, PKT_CHARGE = 25, PKT_CAPACITY = 26,
    PKT_CHARGE_SOURCES = 34, PKT_OI_MODE = 35, PKT_ENC_LEFT = 43, PKT_ENC_RIGHT = 44,
    PKT_LIGHT_BUMPER = 45
};

/* Bits of packet 7 */
#define OI_BUMP_RIGHT  0x01
#define OI_BUMP_LEFT   0x02
#define OI_DROP_RIGHT  0x04
#define OI_DROP_LEFT   0x08

/* Special radius values for oi_cmd_drive() */
#define OI_RADIUS_STRAIGHT   0x8000
#define OI_RADIUS_SPIN_LEFT  1      /* counter-clockwise */
#define OI_RADIUS_SPIN_RIGHT (-1)   /* clockwise */

/* Geometry (OI spec, Create 2 figures). Calibrate on your robot. */
#define OI_COUNTS_PER_REV   508.8
#define OI_WHEEL_DIAM_MM    72.0
#define OI_WHEEL_BASE_MM    235.0
#define OI_MM_PER_COUNT     (3.14159265358979 * OI_WHEEL_DIAM_MM / OI_COUNTS_PER_REV)

#define OI_MAX_PACKET 58

/* ---- packets ---- */
int     oi_packet_size(uint8_t id);                 /* bytes, or -1 if unknown */
int32_t oi_decode(uint8_t id, const uint8_t *data); /* big-endian, signed where the spec says */
int16_t oi_enc_delta(uint16_t old_count, uint16_t new_count); /* handles 65535 -> 0 wrap */

/* ---- command builders: write into out[], return number of bytes ---- */
size_t oi_cmd_simple(uint8_t *out, uint8_t opcode);                 /* START, SAFE, CLEAN, ... */
size_t oi_cmd_drive(uint8_t *out, int velocity_mm_s, int radius_mm); /* 5 bytes */
size_t oi_cmd_drive_direct(uint8_t *out, int left_mm_s, int right_mm_s); /* 5 bytes */
size_t oi_cmd_query(uint8_t *out, const uint8_t *ids, uint8_t n);    /* 2 + n bytes */
size_t oi_cmd_stream(uint8_t *out, const uint8_t *ids, uint8_t n);   /* 2 + n bytes */
size_t oi_cmd_pause_stream(uint8_t *out, bool resume);               /* 2 bytes */
size_t oi_cmd_beep(uint8_t *out);                                    /* 11 bytes */

/* Bytes the robot sends back for a Query List of these ids */
size_t oi_query_reply_size(const uint8_t *ids, uint8_t n);
/* Decode a Query List reply into values[id] */
void   oi_decode_query(const uint8_t *ids, uint8_t n, const uint8_t *reply, int32_t *values);

/* ---- stream parser (opcode 148) ---- */
typedef struct {
    int32_t  v[OI_MAX_PACKET + 1];  /* v[id] = latest value */
    uint64_t present;               /* bit id set if this frame carried packet id */
} oi_frame_t;

typedef void (*oi_frame_cb)(const oi_frame_t *frame, void *arg);

typedef struct {
    uint8_t buf[256];
    size_t  len;
    uint32_t good_frames, bad_frames;
} oi_parser_t;

/* Feed any received bytes (log text included); cb runs once per valid frame. */
void oi_parser_feed(oi_parser_t *p, const uint8_t *data, size_t n, oi_frame_cb cb, void *arg);

#endif
