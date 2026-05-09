#include "gps.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

#ifndef RC_O4_OSD_HOST_TEST
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#endif

namespace gps {
namespace {

static bool starts_with(const char *s, const char *prefix) {
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static int split_fields(char *line, char **fields, int max_fields) {
    int count = 0;
    char *p = line;
    if (*p == '$') ++p;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',' || *p == '*') {
            const char sep = *p;
            *p++ = '\0';
            if (sep == '*') break;
            fields[count++] = p;
        } else {
            ++p;
        }
    }
    return count;
}

static double parse_coord(const char *value, const char *hemisphere) {
    if (!value || !*value) return 0.0;
    const double raw = strtod(value, nullptr);
    const int deg = int(raw / 100.0);
    const double min = raw - double(deg * 100);
    double dec = double(deg) + min / 60.0;
    if (hemisphere && (*hemisphere == 'S' || *hemisphere == 'W')) dec = -dec;
    return dec;
}

static bool checksum_ok(const char *line) {
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) return true;
    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; ++p) checksum ^= uint8_t(*p);
    const uint8_t expected = uint8_t(strtoul(star + 1, nullptr, 16));
    return checksum == expected;
}

static bool parse_rmc(char *line, GpsState &state, uint32_t now_ms) {
    char *f[16] = {};
    split_fields(line, f, 16);
    char *status = f[2];
    char *lat = f[3];
    char *ns = f[4];
    char *lon = f[5];
    char *ew = f[6];
    char *speed_kn = f[7];
    char *course = f[8];

    state.fix_valid = status && *status == 'A';
    state.fix_type = state.fix_valid ? 1 : 0;
    if (state.fix_valid) {
        state.lat_deg = parse_coord(lat, ns);
        state.lon_deg = parse_coord(lon, ew);
        state.speed_kmh = speed_kn ? float(strtod(speed_kn, nullptr) * 1.852) : 0.0f;
        state.course_deg = course ? float(strtod(course, nullptr)) : 0.0f;
        state.last_fix_ms = now_ms;
        state.updated = true;
    }
    return true;
}

static bool parse_gga(char *line, GpsState &state, uint32_t now_ms) {
    char *f[16] = {};
    split_fields(line, f, 16);
    char *lat = f[2];
    char *ns = f[3];
    char *lon = f[4];
    char *ew = f[5];
    char *quality = f[6];
    char *sats = f[7];
    char *hdop = f[8];
    char *alt = f[9];

    const int q = quality ? atoi(quality) : 0;
    state.fix_valid = q > 0;
    state.fix_type = q > 1 ? 2 : (q > 0 ? 1 : 0);
    state.satellites = sats ? uint8_t(atoi(sats)) : 0;
    state.hdop = hdop ? float(strtod(hdop, nullptr)) : 99.9f;
    if (state.fix_valid) {
        state.lat_deg = parse_coord(lat, ns);
        state.lon_deg = parse_coord(lon, ew);
        state.altitude_m = alt ? float(strtod(alt, nullptr)) : 0.0f;
        state.last_fix_ms = now_ms;
        state.updated = true;
    }
    return true;
}

static bool parse_vtg(char *line, GpsState &state) {
    char *f[12] = {};
    split_fields(line, f, 12);
    char *course = f[1];
    char *kmh = f[7];
    if (course && *course) state.course_deg = float(strtod(course, nullptr));
    if (kmh && *kmh) state.speed_kmh = float(strtod(kmh, nullptr));
    return true;
}

static bool parse_line(char *line, GpsState &state, uint32_t now_ms) {
    state.updated = false;
    if (!checksum_ok(line)) return false;
    if (starts_with(line, "$GPRMC") || starts_with(line, "$GNRMC")) return parse_rmc(line, state, now_ms);
    if (starts_with(line, "$GPGGA") || starts_with(line, "$GNGGA")) return parse_gga(line, state, now_ms);
    if (starts_with(line, "$GPVTG") || starts_with(line, "$GNVTG")) return parse_vtg(line, state);
    return false;
}

}

void ubx_checksum(const uint8_t *data, size_t len, uint8_t &ck_a, uint8_t &ck_b) {
    ck_a = 0;
    ck_b = 0;
    for (size_t i = 0; i < len; ++i) {
        ck_a = uint8_t(ck_a + data[i]);
        ck_b = uint8_t(ck_b + ck_a);
    }
}

void NmeaParser::reset() {
    len_ = 0;
    line_[0] = '\0';
}

bool NmeaParser::parse_byte(uint8_t byte, GpsState &state, uint32_t now_ms) {
    if (byte == '$') {
        len_ = 0;
    }
    if (byte == '\r') return false;
    if (byte == '\n') {
        line_[len_] = '\0';
        bool ok = len_ > 0 ? parse_line(line_, state, now_ms) : false;
        len_ = 0;
        return ok;
    }
    if (len_ < sizeof(line_) - 1) {
        line_[len_++] = char(byte);
    } else {
        reset();
    }
    return false;
}

void update_timeout(GpsState &state, uint32_t now_ms) {
    if (!state.fix_valid) return;
    if (state.last_fix_ms == 0 || (now_ms - state.last_fix_ms) > GPS_FIX_TIMEOUT_MS) {
        state.fix_valid = false;
        state.fix_type = 0;
        state.satellites = 0;
        state.speed_kmh = 0.0f;
        state.updated = false;
    }
}

#ifndef RC_O4_OSD_HOST_TEST
namespace {

static void write_ubx_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, size_t len) {
    uint8_t header[6] = {0xB5, 0x62, msg_class, msg_id, uint8_t(len), uint8_t(len >> 8)};
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    ubx_checksum(&header[2], 4, ck_a, ck_b);
    for (size_t i = 0; i < len; ++i) {
        ck_a = uint8_t(ck_a + payload[i]);
        ck_b = uint8_t(ck_b + ck_a);
    }
    uart_write_blocking(uart1, header, sizeof(header));
    if (len > 0) uart_write_blocking(uart1, payload, len);
    const uint8_t checksum[2] = {ck_a, ck_b};
    uart_write_blocking(uart1, checksum, sizeof(checksum));
}

static bool ubx_wait_ack(uint8_t expect_class, uint8_t expect_id, uint32_t timeout_ms) {
    enum class AckState : uint8_t { Sync1, Sync2, Class, Id, Len1, Len2, PayloadClass, PayloadId, Cka, Ckb };
    AckState st = AckState::Sync1;
    uint32_t start = to_ms_since_boot(get_absolute_time());

    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        if (!uart_is_readable(uart1)) {
            watchdog_update();
            tight_loop_contents();
            continue;
        }
        const uint8_t b = uart_getc(uart1);
        switch (st) {
        case AckState::Sync1: st = (b == 0xB5) ? AckState::Sync2 : AckState::Sync1; break;
        case AckState::Sync2: st = (b == 0x62) ? AckState::Class : AckState::Sync1; break;
        case AckState::Class: st = (b == 0x05) ? AckState::Id : AckState::Sync1; break;
        case AckState::Id:
            if (b == 0x00) return false;
            st = (b == 0x01) ? AckState::Len1 : AckState::Sync1;
            break;
        case AckState::Len1: st = (b == 0x02) ? AckState::Len2 : AckState::Sync1; break;
        case AckState::Len2: st = (b == 0x00) ? AckState::PayloadClass : AckState::Sync1; break;
        case AckState::PayloadClass: st = (b == expect_class) ? AckState::PayloadId : AckState::Sync1; break;
        case AckState::PayloadId: st = (b == expect_id) ? AckState::Cka : AckState::Sync1; break;
        case AckState::Cka: st = AckState::Ckb; break;
        case AckState::Ckb: return true;
        }
    }
    return false;
}

static bool ubx_send_and_ack(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, size_t len) {
    write_ubx_frame(msg_class, msg_id, payload, len);
    if (ubx_wait_ack(msg_class, msg_id, 500)) return true;
    write_ubx_frame(msg_class, msg_id, payload, len);
    return ubx_wait_ack(msg_class, msg_id, 500);
}

static void reinit_gps_uart(uint32_t baud) {
    uart_deinit(uart1);
    uart_init(uart1, baud);
    gpio_set_function(PIN_UART1_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART1_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);
}

static bool wait_for_nmea(uint32_t timeout_ms) {
    const uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        if (uart_is_readable(uart1)) {
            const uint8_t b = uart_getc(uart1);
            if (b == '$') return true;
        }
        watchdog_update();
        tight_loop_contents();
    }
    return false;
}

}

void init_uart() {
    reinit_gps_uart(UART_GPS_BAUD_INIT);
}

void poll_uart(NmeaParser &parser, GpsState &state, uint32_t now_ms) {
    while (uart_is_readable(uart1)) {
        parser.parse_byte(uart_getc(uart1), state, now_ms);
    }
}

void send_ubx_startup_config() {
    const uint32_t wait_start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - wait_start) < 1000u) {
        watchdog_update();
        sleep_ms(10);
    }

    const uint8_t disable_gll[] = {0xF0, 0x01, 0, 0, 0, 0, 0, 0};
    const uint8_t disable_gsv[] = {0xF0, 0x03, 0, 0, 0, 0, 0, 0};
    const uint8_t disable_gsa[] = {0xF0, 0x02, 0, 0, 0, 0, 0, 0};
    const uint8_t set_5hz[] = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
    const uint8_t set_baud_115200[] = {
        0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,
        0x00, 0xC2, 0x01, 0x00,
        0x07, 0x00,
        0x03, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    };

    (void)ubx_send_and_ack(0x06, 0x01, disable_gll, sizeof(disable_gll));
    (void)ubx_send_and_ack(0x06, 0x01, disable_gsv, sizeof(disable_gsv));
    (void)ubx_send_and_ack(0x06, 0x01, disable_gsa, sizeof(disable_gsa));
    (void)ubx_send_and_ack(0x06, 0x08, set_5hz, sizeof(set_5hz));

    write_ubx_frame(0x06, 0x00, set_baud_115200, sizeof(set_baud_115200));
    const uint32_t baud_wait_start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - baud_wait_start) < 100u) {
        watchdog_update();
        sleep_ms(5);
    }
    reinit_gps_uart(UART_GPS_BAUD_FAST);
    if (!wait_for_nmea(500)) {
        reinit_gps_uart(UART_GPS_BAUD_INIT);
    }
}
#else
void init_uart() {}
void poll_uart(NmeaParser &, GpsState &, uint32_t) {}
void send_ubx_startup_config() {}
#endif

}
