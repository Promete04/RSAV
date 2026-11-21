/* roomba_linux.c - Roomba OI examples over the micro-USB port (Linux).
 * Build: gcc -std=c99 -O2 -Wall -Wextra -o roomba roomba_linux.c oi_proto.c -lm
 * Run:   ./roomba sensors | stream | forward | backward | left | right | square | circle | beep
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "oi_proto.h"

static const char *PORT = "/dev/ttyACM0";
static volatile sig_atomic_t g_quit = 0;
static void on_sigint(int s) { (void)s; g_quit = 1; }

/* ---------------- serial transport ---------------- */
static int serial_open(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror(dev); return -1; }
    struct termios t;
    if (tcgetattr(fd, &t) < 0) { perror("tcgetattr"); close(fd); return -1; }
    t.c_iflag = 0;                                  /* raw: no CR/LF or flow translation */
    t.c_oflag = 0;
    t.c_lflag = 0;
    t.c_cflag = CS8 | CREAD | CLOCAL;               /* 8N1, ignore modem lines */
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    if (tcsetattr(fd, TCSANOW, &t) < 0) { perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int send_bytes(int fd, const uint8_t *b, size_t n)
{
    while (n) {
        ssize_t w = write(fd, b, n);
        if (w < 0) { if (errno == EINTR) continue; perror("write"); return -1; }
        b += w; n -= (size_t)w;
    }
    return 0;
}

static int read_exact(int fd, uint8_t *b, size_t n, int timeout_ms)
{
    size_t got = 0;
    while (got < n) {
        struct pollfd p = { .fd = fd, .events = POLLIN };
        int r = poll(&p, 1, timeout_ms);
        if (r <= 0) return -1;                      /* timeout or error */
        ssize_t k = read(fd, b + got, n - got);
        if (k <= 0) return -1;
        got += (size_t)k;
    }
    return 0;
}

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

/* ---------------- OI helpers on top of oi_proto ---------------- */
static int cmd(int fd, uint8_t opcode)
{
    uint8_t b[1];
    int r = send_bytes(fd, b, oi_cmd_simple(b, opcode));
    if (opcode == OI_START || opcode == OI_SAFE || opcode == OI_FULL) sleep_ms(30); /* mode change */
    return r;
}

static int drive_direct(int fd, int left, int right)
{
    uint8_t b[5];
    return send_bytes(fd, b, oi_cmd_drive_direct(b, left, right));
}

static int drive(int fd, int velocity, int radius)
{
    uint8_t b[5];
    return send_bytes(fd, b, oi_cmd_drive(b, velocity, radius));
}

static int stop_wheels(int fd) { return drive_direct(fd, 0, 0); }

/* Query List: values[id] filled for each id. Returns 0 on success. */
static int query(int fd, const uint8_t *ids, uint8_t n, int32_t *values)
{
    uint8_t out[64], in[128];
    size_t want = oi_query_reply_size(ids, n);
    tcflush(fd, TCIFLUSH);                          /* drop log text before the reply */
    if (send_bytes(fd, out, oi_cmd_query(out, ids, n)) < 0) return -1;
    if (read_exact(fd, in, want, 500) < 0) return -1;
    oi_decode_query(ids, n, in, values);
    return 0;
}

/* ---------------- closed-loop moves ---------------- */
enum { MOVE_OK = 0, MOVE_BUMP, MOVE_DROP, MOVE_MODE, MOVE_IO, MOVE_QUIT };
static const char *MOVE_TEXT[] = { "ok", "bumper pressed", "wheel off the floor",
                                   "left Safe/Full mode", "serial error", "interrupted" };

static int guard(const int32_t *v, int check_bump)
{
    if (check_bump && (v[PKT_BUMPS_DROPS] & (OI_BUMP_LEFT | OI_BUMP_RIGHT))) return MOVE_BUMP;
    if (v[PKT_BUMPS_DROPS] & (OI_DROP_LEFT | OI_DROP_RIGHT)) return MOVE_DROP;
    if (v[PKT_OI_MODE] < 2) return MOVE_MODE;
    if (g_quit) return MOVE_QUIT;
    return MOVE_OK;
}

static const uint8_t MOVE_IDS[] = { PKT_ENC_LEFT, PKT_ENC_RIGHT, PKT_BUMPS_DROPS, PKT_OI_MODE };

/* Straight line; negative mm = backwards. *done_mm gets the distance travelled. */
static int move_mm(int fd, double mm, int speed, double *done_mm)
{
    int32_t v0[OI_MAX_PACKET + 1] = {0}, v[OI_MAX_PACKET + 1] = {0};
    int sign = mm >= 0 ? 1 : -1, res;
    if (query(fd, MOVE_IDS, 4, v0) < 0) return MOVE_IO;
    if ((res = guard(v0, sign > 0)) != MOVE_OK) return res;
    drive_direct(fd, sign * speed, sign * speed);
    for (;;) {
        sleep_ms(20);
        if (query(fd, MOVE_IDS, 4, v) < 0) { res = MOVE_IO; break; }
        if ((res = guard(v, sign > 0)) != MOVE_OK) break;
        double dl = oi_enc_delta((uint16_t)v0[PKT_ENC_LEFT], (uint16_t)v[PKT_ENC_LEFT]) * OI_MM_PER_COUNT;
        double dr = oi_enc_delta((uint16_t)v0[PKT_ENC_RIGHT], (uint16_t)v[PKT_ENC_RIGHT]) * OI_MM_PER_COUNT;
        if (done_mm) *done_mm = (dl + dr) / 2;
        if (fabs((dl + dr) / 2) >= fabs(mm)) { res = MOVE_OK; break; }
    }
    stop_wheels(fd);
    return res;
}

/* Spin in place; positive degrees = left (counter-clockwise). */
static int turn_deg(int fd, double deg, int speed, double *done_deg)
{
    int32_t v0[OI_MAX_PACKET + 1] = {0}, v[OI_MAX_PACKET + 1] = {0};
    int sign = deg >= 0 ? 1 : -1, res;
    double arc = 3.14159265358979 * OI_WHEEL_BASE_MM * fabs(deg) / 360.0;
    if (query(fd, MOVE_IDS, 4, v0) < 0) return MOVE_IO;
    if ((res = guard(v0, 0)) != MOVE_OK) return res;
    drive_direct(fd, -sign * speed, sign * speed);  /* left back + right forward = left turn */
    for (;;) {
        sleep_ms(20);
        if (query(fd, MOVE_IDS, 4, v) < 0) { res = MOVE_IO; break; }
        if ((res = guard(v, 0)) != MOVE_OK) break;
        double dl = oi_enc_delta((uint16_t)v0[PKT_ENC_LEFT], (uint16_t)v[PKT_ENC_LEFT]) * OI_MM_PER_COUNT;
        double dr = oi_enc_delta((uint16_t)v0[PKT_ENC_RIGHT], (uint16_t)v[PKT_ENC_RIGHT]) * OI_MM_PER_COUNT;
        if (done_deg) *done_deg = (dr - dl) / OI_WHEEL_BASE_MM * 180.0 / 3.14159265358979;
        if ((fabs(dl) + fabs(dr)) / 2 >= arc) { res = MOVE_OK; break; }
    }
    stop_wheels(fd);
    return res;
}

/* ---------------- examples ---------------- */
static void print_frame(const oi_frame_t *f, void *arg)
{
    double *last = arg;
    if (now_s() - *last < 0.25) return;            /* print 4 times a second */
    *last = now_s();
    printf("voltage=%d mV current=%d mA mode=%d enc=%d,%d\n", (int)f->v[PKT_VOLTAGE],
           (int)f->v[PKT_CURRENT], (int)f->v[PKT_OI_MODE], (int)f->v[PKT_ENC_LEFT],
           (int)f->v[PKT_ENC_RIGHT]);
}

static int ex_sensors(int fd)
{
    static const uint8_t ids[] = { 7, 21, 22, 23, 24, 25, 26, 34, 35, 43, 44, 45 };
    int32_t v[OI_MAX_PACKET + 1] = {0};
    if (query(fd, ids, sizeof ids, v) < 0) { fprintf(stderr, "no reply\n"); return 1; }
    printf("mode=%d voltage=%d mV current=%d mA temp=%d C charge=%d/%d mAh on_dock=%d\n",
           (int)v[35], (int)v[22], (int)v[23], (int)v[24], (int)v[25], (int)v[26], (int)!!(v[34] & 2));
    printf("bump L/R=%d/%d drop L/R=%d/%d light_bumper=0x%02X encoders=%d,%d\n",
           !!(v[7] & OI_BUMP_LEFT), !!(v[7] & OI_BUMP_RIGHT), !!(v[7] & OI_DROP_LEFT),
           !!(v[7] & OI_DROP_RIGHT), (unsigned)(v[45] & 0x3F), (int)v[43], (int)v[44]);
    return 0;
}

static int ex_stream(int fd)
{
    static const uint8_t ids[] = { PKT_VOLTAGE, PKT_CURRENT, PKT_OI_MODE, PKT_ENC_LEFT, PKT_ENC_RIGHT };
    uint8_t out[16], in[256];
    oi_parser_t parser;
    memset(&parser, 0, sizeof parser);
    double last = 0, end = now_s() + 2.0;
    tcflush(fd, TCIFLUSH);
    send_bytes(fd, out, oi_cmd_stream(out, ids, sizeof ids));
    while (now_s() < end && !g_quit) {
        struct pollfd p = { .fd = fd, .events = POLLIN };
        if (poll(&p, 1, 100) > 0) {
            ssize_t k = read(fd, in, sizeof in);
            if (k > 0) oi_parser_feed(&parser, in, (size_t)k, print_frame, &last);
        }
    }
    send_bytes(fd, out, oi_cmd_pause_stream(out, false));
    printf("%u good frames, %u bad in 2 s\n", (unsigned)parser.good_frames, (unsigned)parser.bad_frames);
    return 0;
}

static int report(int res, const char *what, double value, const char *unit)
{
    printf("%s: %s (%.0f %s)\n", what, MOVE_TEXT[res], value, unit);
    return res == MOVE_OK ? 0 : 1;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s sensors|stream|forward|backward|left|right|square|circle|beep\n", argv[0]);
        return 2;
    }
    const char *ex = argv[1];
    signal(SIGINT, on_sigint);
    int fd = serial_open(PORT);
    if (fd < 0) return 1;

    int rc = 0, moving = strcmp(ex, "sensors") && strcmp(ex, "stream");
    double val = 0;
    cmd(fd, OI_START);                              /* always first: Passive */
    if (moving) cmd(fd, OI_SAFE);                   /* wheels need Safe or Full */

    if (!strcmp(ex, "sensors"))       rc = ex_sensors(fd);
    else if (!strcmp(ex, "stream"))   rc = ex_stream(fd);
    else if (!strcmp(ex, "forward"))  rc = report(move_mm(fd, 500, 200, &val), "forward", val, "mm");
    else if (!strcmp(ex, "backward")) rc = report(move_mm(fd, -300, 200, &val), "backward", val, "mm");
    else if (!strcmp(ex, "left"))     rc = report(turn_deg(fd, 90, 100, &val), "left", val, "deg");
    else if (!strcmp(ex, "right"))    rc = report(turn_deg(fd, -90, 100, &val), "right", val, "deg");
    else if (!strcmp(ex, "square")) {
        for (int side = 0; side < 4 && rc == 0; side++) {
            rc = report(move_mm(fd, 400, 200, &val), "side", val, "mm");
            if (rc == 0) rc = report(turn_deg(fd, 90, 100, &val), "corner", val, "deg");
        }
    } else if (!strcmp(ex, "circle")) {             /* open loop: r = 300 mm, 150 mm/s, one lap */
        drive(fd, 150, 300);
        double end = now_s() + 2 * 3.14159265358979 * 300 / 150;
        while (now_s() < end && !g_quit) sleep_ms(20);
    } else if (!strcmp(ex, "beep")) {
        uint8_t b[16];
        send_bytes(fd, b, oi_cmd_beep(b));
        sleep_ms(1000);
    } else {
        fprintf(stderr, "unknown example %s\n", ex);
        rc = 2;
    }

    stop_wheels(fd);                                /* always leave the robot stopped, */
    cmd(fd, OI_START);                              /* in Passive, */
    cmd(fd, OI_STOP);                               /* with the OI closed */
    close(fd);
    return rc;
}
