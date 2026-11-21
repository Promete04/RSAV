/* oi_proto.c - portable Roomba Open Interface protocol helpers (C99). */
#include "oi_proto.h"

#include <string.h>

/* Size in bytes of each packet id 7..58 (0 = not a valid id) */
static const uint8_t SIZES[OI_MAX_PACKET + 1] = {
    [7] = 1, [8] = 1, [9] = 1, [10] = 1, [11] = 1, [12] = 1, [13] = 1, [14] = 1, [15] = 1,
    [16] = 1, [17] = 1, [18] = 1, [19] = 2, [20] = 2, [21] = 1, [22] = 2, [23] = 2, [24] = 1,
    [25] = 2, [26] = 2, [27] = 2, [28] = 2, [29] = 2, [30] = 2, [31] = 2, [32] = 1, [33] = 2,
    [34] = 1, [35] = 1, [36] = 1, [37] = 1, [38] = 1, [39] = 2, [40] = 2, [41] = 2, [42] = 2,
    [43] = 2, [44] = 2, [45] = 1, [46] = 2, [47] = 2, [48] = 2, [49] = 2, [50] = 2, [51] = 2,
    [52] = 1, [53] = 1, [54] = 2, [55] = 2, [56] = 2, [57] = 2, [58] = 1,
};

static bool is_signed(uint8_t id)
{
    return id == 19 || id == 20 || id == 23 || id == 24 || (id >= 39 && id <= 42) ||
           (id >= 54 && id <= 57);
}

int oi_packet_size(uint8_t id)
{
    if (id > OI_MAX_PACKET || SIZES[id] == 0) return -1;
    return SIZES[id];
}

int32_t oi_decode(uint8_t id, const uint8_t *d)
{
    if (oi_packet_size(id) == 1) return is_signed(id) ? (int32_t)(int8_t)d[0] : (int32_t)d[0];
    uint16_t u = (uint16_t)((d[0] << 8) | d[1]);      /* big-endian */
    return is_signed(id) ? (int32_t)(int16_t)u : (int32_t)u;
}

int16_t oi_enc_delta(uint16_t old_count, uint16_t new_count)
{
    return (int16_t)(uint16_t)(new_count - old_count); /* modular arithmetic does the wrap */
}

static int clamp(int x, int lo, int hi) { return x < lo ? lo : x > hi ? hi : x; }

static void put16(uint8_t *o, int value)
{
    uint16_t u = (uint16_t)value;                       /* two's complement for negatives */
    o[0] = (uint8_t)(u >> 8);
    o[1] = (uint8_t)(u & 0xFF);
}

size_t oi_cmd_simple(uint8_t *out, uint8_t opcode) { out[0] = opcode; return 1; }

size_t oi_cmd_drive(uint8_t *out, int velocity, int radius)
{
    out[0] = OI_DRIVE;
    put16(out + 1, clamp(velocity, -500, 500));
    put16(out + 3, radius);                            /* 0x8000, 1, -1 or -2000..2000 */
    return 5;
}

size_t oi_cmd_drive_direct(uint8_t *out, int left, int right)
{
    out[0] = OI_DRIVE_DIRECT;
    put16(out + 1, clamp(right, -500, 500));           /* RIGHT wheel first on the wire */
    put16(out + 3, clamp(left, -500, 500));
    return 5;
}

size_t oi_cmd_query(uint8_t *out, const uint8_t *ids, uint8_t n)
{
    out[0] = OI_QUERY_LIST; out[1] = n;
    memcpy(out + 2, ids, n);
    return 2u + n;
}

size_t oi_cmd_stream(uint8_t *out, const uint8_t *ids, uint8_t n)
{
    out[0] = OI_STREAM; out[1] = n;
    memcpy(out + 2, ids, n);
    return 2u + n;
}

size_t oi_cmd_pause_stream(uint8_t *out, bool resume)
{
    out[0] = OI_PAUSE_STREAM; out[1] = resume ? 1 : 0;
    return 2;
}

size_t oi_cmd_beep(uint8_t *out)
{
    static const uint8_t b[] = { OI_SONG, 0, 3, 72, 8, 76, 8, 79, 12, OI_PLAY, 0 };
    memcpy(out, b, sizeof b);
    return sizeof b;
}

size_t oi_query_reply_size(const uint8_t *ids, uint8_t n)
{
    size_t total = 0;
    for (uint8_t i = 0; i < n; i++) {
        int s = oi_packet_size(ids[i]);
        total += s > 0 ? (size_t)s : 0;
    }
    return total;
}

void oi_decode_query(const uint8_t *ids, uint8_t n, const uint8_t *reply, int32_t *values)
{
    size_t at = 0;
    for (uint8_t i = 0; i < n; i++) {
        int s = oi_packet_size(ids[i]);
        if (s < 0) continue;
        values[ids[i]] = oi_decode(ids[i], reply + at);
        at += (size_t)s;
    }
}

/* ---- stream parser ---- */
static bool decode_body(const uint8_t *b, size_t n, oi_frame_t *f)
{
    memset(f, 0, sizeof *f);
    size_t j = 0;
    while (j < n) {
        uint8_t id = b[j];
        int s = oi_packet_size(id);
        if (s < 0 || j + 1 + (size_t)s > n) return false;
        f->v[id] = oi_decode(id, b + j + 1);
        f->present |= 1ULL << id;
        j += 1 + (size_t)s;
    }
    return true;
}

static void consume(oi_parser_t *p, size_t k)
{
    memmove(p->buf, p->buf + k, p->len - k);
    p->len -= k;
}

static void process(oi_parser_t *p, oi_frame_cb cb, void *arg)
{
    for (;;) {
        size_t i = 0;
        while (i < p->len && p->buf[i] != 0x13) i++;   /* skip log text up to a header */
        if (i) consume(p, i);
        if (p->len < 2) return;
        size_t n = p->buf[1];
        if (p->len < n + 3) return;                    /* wait for the rest of the frame */
        uint8_t sum = 0;
        for (size_t k = 0; k < n + 3; k++) sum = (uint8_t)(sum + p->buf[k]);
        oi_frame_t f;
        if (sum == 0 && decode_body(p->buf + 2, n, &f)) {
            p->good_frames++;
            consume(p, n + 3);
            if (cb) cb(&f, arg);
        } else {
            p->bad_frames++;
            consume(p, 1);                             /* false header: resync */
        }
    }
}

void oi_parser_feed(oi_parser_t *p, const uint8_t *data, size_t n, oi_frame_cb cb, void *arg)
{
    while (n) {
        size_t room = sizeof p->buf - p->len;
        if (room == 0) {                               /* never fills with valid data: reset */
            p->len = 0;
            room = sizeof p->buf;
        }
        size_t k = n < room ? n : room;
        memcpy(p->buf + p->len, data, k);
        p->len += k; data += k; n -= k;
        process(p, cb, arg);
    }
}
