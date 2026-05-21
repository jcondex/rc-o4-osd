#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#include "gps.h"
#include "config.h"
#include "battery.h"
#include "msp.h"
#include "nav.h"
#include "bmp390.h"
#include "imu_mpu6050.h"
#include "compass_qmc5883l.h"
#include "calibration_flash.h"
#include "displayport.h"
#include "state_machine.h"

namespace bmp390 {
bool baseline_seed_ready(float pressure_pa);
}

namespace {

std::vector<uint8_t> g_written;

void capture_writer(const uint8_t *data, size_t len, void *) {
    g_written.insert(g_written.end(), data, data + len);
}

void make_request(uint8_t cmd, msp::Parser &parser, const TelemetryState &state) {
    const uint8_t frame[] = {'$', 'M', '<', 0x00, cmd, cmd};
    msp::Endpoint endpoint{capture_writer, nullptr};
    for (uint8_t b : frame) parser.parse_byte(b, state, endpoint);
}

void make_request_with_checksum(uint8_t cmd, uint8_t checksum, msp::Parser &parser, const TelemetryState &state) {
    const uint8_t frame[] = {'$', 'M', '<', 0x00, cmd, checksum};
    msp::Endpoint endpoint{capture_writer, nullptr};
    for (uint8_t b : frame) parser.parse_byte(b, state, endpoint);
}

uint8_t response_len() {
    assert(g_written.size() >= 6);
    return g_written[3];
}

uint8_t response_cmd() {
    assert(g_written.size() >= 6);
    return g_written[4];
}

void clear_capture() {
    g_written.clear();
}

void set_rc_switch(TelemetryState &state, RcSwitchPosition pos) {
    state.rc.signal_valid = pos != RcSwitchPosition::Unknown;
    state.rc.failsafe = pos == RcSwitchPosition::Unknown;
    state.rc.switch_position = pos;
    state.rc.arm_switch_low = pos == RcSwitchPosition::Low;
    state.rc.arm_switch_middle = pos == RcSwitchPosition::Middle;
    state.rc.arm_switch_high = pos == RcSwitchPosition::High;
    state.rc.cockpit_mode_requested = pos == RcSwitchPosition::High;
    if (state.rc.arm_switch_low) {
        state.rc.rearm_latched = true;
    }
}

static void put_le_u16(std::vector<uint8_t> &v, size_t offset, uint16_t value) {
    v[offset] = uint8_t(value);
    v[offset + 1] = uint8_t(value >> 8);
}

static void put_le_i32(std::vector<uint8_t> &v, size_t offset, int32_t value) {
    v[offset] = uint8_t(value);
    v[offset + 1] = uint8_t(uint32_t(value) >> 8);
    v[offset + 2] = uint8_t(uint32_t(value) >> 16);
    v[offset + 3] = uint8_t(uint32_t(value) >> 24);
}

static std::vector<uint8_t> ubx_frame(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> frame;
    frame.push_back(0xB5);
    frame.push_back(0x62);
    frame.push_back(msg_class);
    frame.push_back(msg_id);
    frame.push_back(uint8_t(payload.size()));
    frame.push_back(uint8_t(payload.size() >> 8));
    frame.insert(frame.end(), payload.begin(), payload.end());

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < frame.size(); ++i) {
        ck_a = uint8_t(ck_a + frame[i]);
        ck_b = uint8_t(ck_b + ck_a);
    }
    frame.push_back(ck_a);
    frame.push_back(ck_b);
    return frame;
}

static std::string nmea_sentence(const char *body) {
    uint8_t checksum = 0;
    for (const char *p = body; *p; ++p) {
        checksum ^= uint8_t(*p);
    }
    char suffix[8] = {};
    snprintf(suffix, sizeof(suffix), "*%02X\r\n", checksum);
    return std::string("$") + body + suffix;
}

uint16_t le16(const uint8_t *p) {
    return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

int16_t sle16(const uint8_t *p) {
    return int16_t(le16(p));
}

uint32_t le32(const uint8_t *p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

int32_t sle32(const uint8_t *p) {
    return int32_t(le32(p));
}

std::string displayport_row_text(uint8_t row) {
    for (size_t i = 0; i + 6 < g_written.size();) {
        const uint8_t len = g_written[i + 3];
        const uint8_t cmd = g_written[i + 4];
        const uint8_t *payload = &g_written[i + 5];
        if (cmd == msp::MSP_DISPLAYPORT && len >= 4 && payload[0] == 3 && payload[1] == row) {
            return std::string(reinterpret_cast<const char *>(payload + 4), len - 4);
        }
        i += size_t(len) + 6;
    }
    return {};
}

std::string displayport_text_at(uint8_t row, uint8_t col) {
    for (size_t i = 0; i + 6 < g_written.size();) {
        const uint8_t len = g_written[i + 3];
        const uint8_t cmd = g_written[i + 4];
        const uint8_t *payload = &g_written[i + 5];
        if (cmd == msp::MSP_DISPLAYPORT && len >= 4 && payload[0] == 3 && payload[1] == row && payload[2] == col) {
            return std::string(reinterpret_cast<const char *>(payload + 4), len - 4);
        }
        i += size_t(len) + 6;
    }
    return {};
}

int count_displayport_char(char ch, uint8_t *min_col = nullptr, uint8_t *max_col = nullptr) {
    int count = 0;
    uint8_t lo = 255;
    uint8_t hi = 0;
    for (size_t i = 0; i + 6 < g_written.size();) {
        const uint8_t len = g_written[i + 3];
        const uint8_t cmd = g_written[i + 4];
        const uint8_t *payload = &g_written[i + 5];
        if (cmd == msp::MSP_DISPLAYPORT && len == 5 && payload[0] == 3 && payload[4] == uint8_t(ch)) {
            ++count;
            if (payload[2] < lo) lo = payload[2];
            if (payload[2] > hi) hi = payload[2];
        }
        i += size_t(len) + 6;
    }
    if (min_col) *min_col = count ? lo : 0;
    if (max_col) *max_col = count ? hi : 0;
    return count;
}

void assert_displayport_writes_fit_canvas() {
    for (size_t i = 0; i + 6 < g_written.size();) {
        assert(g_written[i] == '$');
        assert(g_written[i + 1] == 'M');
        const uint8_t len = g_written[i + 3];
        const uint8_t cmd = g_written[i + 4];
        assert(cmd == msp::MSP_DISPLAYPORT);
        const uint8_t *payload = &g_written[i + 5];
        if (len >= 4 && payload[0] == 3) {
            assert(payload[1] < OSD_CANVAS_ROWS);
            assert(payload[2] < OSD_CANVAS_COLS);
            assert(len <= 34);
            assert(size_t(payload[2]) + size_t(len - 4) <= OSD_CANVAS_COLS);
        }
        i += size_t(len) + 6;
    }
}

void test_msp_identity() {
    TelemetryState state;
    msp::Parser parser;
    clear_capture();
    make_request(msp::MSP_API_VERSION, parser, state);
    assert(response_cmd() == msp::MSP_API_VERSION);
    assert(response_len() == 3);
    assert(g_written[5] == 0);
    assert(g_written[6] == 1);
    assert(g_written[7] == 45);

    clear_capture();
    make_request(msp::MSP_FC_VERSION, parser, state);
    assert(response_cmd() == msp::MSP_FC_VERSION);
    assert(response_len() == 3);
    assert(g_written[5] == 4);
    assert(g_written[6] == 4);
    assert(g_written[7] == 3);

    clear_capture();
    make_request(msp::MSP_FC_VARIANT, parser, state);
    assert(response_len() == 4);
    assert(memcmp(&g_written[5], "BTFL", 4) == 0);

    clear_capture();
    make_request(msp::MSP_NAME, parser, state);
    assert(response_len() == 9);
    assert(memcmp(&g_written[5], "RC-O4-OSD", 9) == 0);
}

void test_msp_build_info_and_name_exact() {
    TelemetryState state;
    uint8_t out[128] = {};
    size_t len = 0;

    assert(msp::build_response(msp::MSP_BUILD_INFO, state, out, sizeof(out), &len));
    assert(len == 26);
    assert(memcmp(out, "Jan  1 2025", 11) == 0);
    assert(memcmp(out + 11, "00:00:00", 8) == 0);
    assert(memcmp(out + 19, "0000000", 7) == 0);

    memset(out, 0xA5, sizeof(out));
    assert(msp::build_response(msp::MSP_NAME, state, out, sizeof(out), &len));
    assert(len == 9);
    assert(memcmp(out, "RC-O4-OSD", 9) == 0);
}

void test_msp_payload_sizes() {
    TelemetryState state;
    state.battery.volts = 12.6f;
    uint8_t out[128] = {};
    size_t len = 0;
    assert(msp::build_response(msp::MSP_ANALOG, state, out, sizeof(out), &len));
    assert(len == 7);
    assert(out[0] == 126);

    assert(msp::build_response(msp::MSP_BATTERY_STATE, state, out, sizeof(out), &len));
    assert(len == 9);
    assert(out[0] == 0);
    assert(out[3] == 126);

    assert(msp::build_response(msp::MSP_STATUS_EX, state, out, sizeof(out), &len));
    assert(len == 16);
}

void test_msp_dynamic_payloads() {
    TelemetryState state;
    state.app_state = AppState::Armed;
    state.armed = true;
    state.rc.failsafe = false;
    state.cycle_time_us = 1000;
    state.i2c_errors = 7;
    state.mpu6050_ok = true;
    state.imu.valid = true;
    state.bmp390_ok = true;
    state.baro.valid = true;
    state.gps.fix_valid = true;

    state.imu.roll_deg = -2.3f;
    state.imu.pitch_deg = 5.2f;
    state.compass.heading_deg = 87.0f;
    state.baro.relative_altitude_m = 12.34f;
    state.baro.vertical_speed_ms = -1.25f;
    state.gps.fix_type = 1;
    state.gps.satellites = 12;
    state.gps.lat_deg = 37.1234567;
    state.gps.lon_deg = -122.7654321;
    state.gps.altitude_m = 42.0f;
    state.gps.speed_kmh = 36.0f;
    state.gps.course_deg = 271.2f;
    state.nav.home_distance_m = 184.0f;
    state.nav.home_bearing_deg = 245.0f;
    state.gps.updated = true;

    uint8_t out[128] = {};
    size_t len = 0;
    assert(msp::build_response(msp::MSP_STATUS, state, out, sizeof(out), &len));
    assert(len == 11);
    assert(le16(&out[0]) == 1000);
    assert(le16(&out[2]) == 7);
    assert(le16(&out[4]) == ((1u << 0) | (1u << 1) | (1u << 3) | (1u << 5)));
    assert(le32(&out[6]) == 1);

    assert(msp::build_response(msp::MSP_ATTITUDE, state, out, sizeof(out), &len));
    assert(len == 6);
    assert(sle16(&out[0]) == -23);
    assert(sle16(&out[2]) == 52);
    assert(le16(&out[4]) == 87);

    assert(msp::build_response(msp::MSP_ALTITUDE, state, out, sizeof(out), &len));
    assert(len == 6);
    assert(sle32(&out[0]) == 1234);
    assert(sle16(&out[4]) == -125);

    assert(msp::build_response(msp::MSP_COMP_GPS, state, out, sizeof(out), &len));
    assert(len == 5);
    assert(le16(&out[0]) == 184);
    assert(le16(&out[2]) == 245);
    assert(out[4] == 1);

    assert(msp::build_response(msp::MSP_RAW_GPS, state, out, sizeof(out), &len));
    assert(len == 16);
    assert(out[0] == 1);
    assert(out[1] == 12);
    assert(sle32(&out[2]) == 371234567);
    assert(sle32(&out[6]) == -1227654321);
    assert(le16(&out[10]) == 42);
    assert(le16(&out[12]) == 1000);
    assert(le16(&out[14]) == 2712);
}

void test_msp_payload_clamps_and_normalizes() {
    TelemetryState state;
    state.imu.roll_deg = 250.0f;
    state.imu.pitch_deg = -120.0f;
    state.compass.heading_deg = 400.0f;
    state.gps.fix_valid = true;
    state.gps.fix_type = 3;
    state.gps.satellites = 22;
    state.gps.lat_deg = 37.1234567;
    state.gps.lon_deg = -122.7654321;
    state.gps.altitude_m = -42.0f;
    state.gps.speed_kmh = 5000.0f;
    state.gps.course_deg = 725.0f;

    uint8_t out[128] = {};
    size_t len = 0;
    assert(msp::build_response(msp::MSP_ATTITUDE, state, out, sizeof(out), &len));
    assert(len == 6);
    assert(sle16(&out[0]) == 1800);
    assert(sle16(&out[2]) == -900);
    assert(le16(&out[4]) == 359);

    assert(msp::build_response(msp::MSP_RAW_GPS, state, out, sizeof(out), &len));
    assert(len == 16);
    assert(out[0] == 3);
    assert(out[1] == 22);
    assert(le16(&out[10]) == 0);
    assert(le16(&out[12]) == 65535);
    assert(le16(&out[14]) == 50);
}

void test_msp_raw_gps_no_fix_payload_is_zero() {
    TelemetryState state;
    state.gps.fix_valid = false;
    state.gps.fix_type = 2;
    state.gps.satellites = 14;
    state.gps.lat_deg = 37.1234567;
    state.gps.lon_deg = -122.7654321;
    state.gps.altitude_m = 42.0f;
    state.gps.speed_kmh = 36.0f;
    state.gps.course_deg = 271.2f;

    uint8_t out[128] = {};
    size_t len = 0;
    assert(msp::build_response(msp::MSP_RAW_GPS, state, out, sizeof(out), &len));
    assert(len == 16);
    for (size_t i = 0; i < len; ++i) {
        assert(out[i] == 0);
    }
}

void test_msp_parser_rejection_and_ignores() {
    TelemetryState state;
    msp::Parser parser;
    clear_capture();
    make_request_with_checksum(msp::MSP_API_VERSION, 0x00, parser, state);
    assert(g_written.empty());

    clear_capture();
    make_request(200, parser, state);
    assert(g_written.empty());

    clear_capture();
    make_request(msp::MSP_OSD_CONFIG, parser, state);
    assert(response_cmd() == msp::MSP_OSD_CONFIG);
    assert(response_len() == 190);
    assert(g_written[5] == 0x61);
    assert(g_written[6] == 3);
    assert(le16(&g_written[5 + 10 + 2 * 8]) == 2049);
    assert(le16(&g_written[5 + 10 + 2 * 29]) == 2048 + 12 + 8 * 32);

    clear_capture();
    make_request(msp::MSP_OSD_CANVAS, parser, state);
    assert(response_cmd() == msp::MSP_OSD_CANVAS);
    assert(response_len() == 2);
    assert(g_written[5] == OSD_CANVAS_COLS);
    assert(g_written[6] == OSD_CANVAS_ROWS);

    clear_capture();
    make_request(msp::MSP_SET_OSD_CANVAS, parser, state);
    assert(response_cmd() == msp::MSP_SET_OSD_CANVAS);
    assert(response_len() == 0);
}

void test_sensor_mask() {
    TelemetryState state;
    state.mpu6050_ok = true;
    state.imu.valid = true;
    state.bmp390_ok = true;
    state.baro.valid = true;
    state.compass_ok = true;
    state.compass.valid = true;
    state.gps.fix_valid = true;
    const uint16_t mask = msp::active_sensor_mask(state);
    assert(mask & (1u << 0));
    assert(mask & (1u << 1));
    assert(mask & (1u << 2));
    assert(mask & (1u << 3));
    assert(mask & (1u << 5));
}

void test_gps_nmea() {
    gps::NmeaParser parser;
    GpsState state;
    const char *gga = "$GPGGA,123519,3723.2475,N,12158.3416,W,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
    for (const char *p = gga; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1000);
    assert(state.fix_valid);
    assert(state.satellites == 8);
    assert(fabs(state.lat_deg - 37.3874583) < 0.0001);
    assert(fabs(state.lon_deg + 121.97236) < 0.0001);
    assert(parser.stats().parsed_sentences == 1);
    assert(parser.stats().gga_sentences == 1);
    assert(strcmp(parser.stats().last_good_type, "GPGGA") == 0);
}

void test_gps_rmc_and_vtg() {
    gps::NmeaParser parser;
    GpsState state;
    const char *rmc = "$GPRMC,123519,A,3723.2475,N,12158.3416,W,10.0,84.4,230394,003.1,W*71\r\n";
    for (const char *p = rmc; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1000);
    assert(state.fix_valid);
    assert(fabs(state.speed_kmh - 18.52f) < 0.01f);
    assert(fabs(state.course_deg - 84.4f) < 0.01f);

    const char *vtg = "$GPVTG,120.0,T,,M,,N,44.4,K*79\r\n";
    for (const char *p = vtg; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1100);
    assert(fabs(state.course_deg - 120.0f) < 0.01f);
    assert(fabs(state.speed_kmh - 44.4f) < 0.01f);
}

void test_gps_accepts_multi_gnss_talker_ids() {
    gps::NmeaParser parser;
    GpsState state;

    const std::string galileo_gga = nmea_sentence("GAGGA,123519,3723.2475,N,12158.3416,W,1,09,0.8,545.4,M,46.9,M,,");
    for (char c : galileo_gga) parser.parse_byte(uint8_t(c), state, 1000);
    assert(state.fix_valid);
    assert(state.satellites == 9);
    assert(parser.stats().gga_sentences == 1);
    assert(strcmp(parser.stats().last_good_type, "GAGGA") == 0);

    const std::string glonass_vtg = nmea_sentence("GLVTG,120.0,T,,M,,N,44.4,K");
    for (char c : glonass_vtg) parser.parse_byte(uint8_t(c), state, 1100);
    assert(fabs(state.course_deg - 120.0f) < 0.01f);
    assert(fabs(state.speed_kmh - 44.4f) < 0.01f);
    assert(parser.stats().vtg_sentences == 1);
    assert(strcmp(parser.stats().last_good_type, "GLVTG") == 0);
}

void test_gps_coordinate_hemispheres() {
    gps::NmeaParser parser;
    GpsState state;

    const std::string southern_eastern = nmea_sentence("GNGGA,123519,3351.1234,S,15112.5678,E,1,10,0.8,12.3,M,0.0,M,,");
    for (char c : southern_eastern) parser.parse_byte(uint8_t(c), state, 1000);

    assert(state.fix_valid);
    assert(state.satellites == 10);
    assert(fabs(state.lat_deg - (-(33.0 + 51.1234 / 60.0))) < 0.000001);
    assert(fabs(state.lon_deg - (151.0 + 12.5678 / 60.0)) < 0.000001);
}

void test_gps_ubx_nav_pvt() {
    gps::NmeaParser parser;
    GpsState state;
    std::vector<uint8_t> payload(92, 0);
    payload[20] = 3;     // 3D fix
    payload[21] = 0x01;  // gnssFixOK
    payload[23] = 11;    // numSV
    put_le_i32(payload, 24, -1219723600);
    put_le_i32(payload, 28, 373874583);
    put_le_i32(payload, 36, 545400);
    put_le_i32(payload, 60, 12340);
    put_le_i32(payload, 64, 8440000);
    put_le_u16(payload, 76, 95);

    const std::vector<uint8_t> frame = ubx_frame(0x01, 0x07, payload);
    bool parsed = false;
    for (uint8_t b : frame) {
        parsed = parser.parse_byte(b, state, 2000) || parsed;
    }

    assert(parsed);
    assert(state.fix_valid);
    assert(state.fix_type == 3);
    assert(state.satellites == 11);
    assert(fabs(state.lat_deg - 37.3874583) < 0.0001);
    assert(fabs(state.lon_deg + 121.97236) < 0.0001);
    assert(fabs(state.altitude_m - 545.4f) < 0.01f);
    assert(fabs(state.speed_kmh - 44.424f) < 0.01f);
    assert(fabs(state.course_deg - 84.4f) < 0.01f);
    assert(fabs(state.hdop - 0.95f) < 0.01f);
    assert(parser.stats().ubx_frames == 1);
    assert(parser.stats().ubx_nav_pvt == 1);
    assert(parser.stats().checksum_errors == 0);
}

void test_gps_ubx_no_fix_clears_motion_and_preserves_satellites() {
    gps::NmeaParser parser;
    GpsState state;
    state.fix_valid = true;
    state.fix_type = 3;
    state.satellites = 12;
    state.speed_kmh = 55.0f;
    state.course_deg = 123.0f;
    state.altitude_m = 44.0f;

    std::vector<uint8_t> payload(92, 0);
    payload[20] = 0;
    payload[21] = 0;
    payload[23] = 6;
    put_le_u16(payload, 76, 250);

    const std::vector<uint8_t> frame = ubx_frame(0x01, 0x07, payload);
    bool parsed = false;
    for (uint8_t b : frame) {
        parsed = parser.parse_byte(b, state, 2000) || parsed;
    }

    assert(parsed);
    assert(state.receiving);
    assert(!state.fix_valid);
    assert(state.fix_type == 0);
    assert(state.satellites == 6);
    assert(fabs(state.hdop - 2.5f) < 0.01f);
    assert(state.speed_kmh == 0.0f);
    assert(state.course_deg == 0.0f);
    assert(state.altitude_m == 0.0f);
}

void test_gps_mixed_ubx_payload_does_not_create_nmea_crc_noise() {
    gps::NmeaParser parser;
    GpsState state;
    std::vector<uint8_t> payload(92, 0);
    payload[20] = 0;
    payload[21] = 0;
    payload[23] = 6;
    payload[40] = '$';
    payload[41] = 'B';
    payload[42] = 'A';
    payload[43] = 'D';
    payload[44] = '\n';

    const std::vector<uint8_t> frame = ubx_frame(0x01, 0x07, payload);
    for (uint8_t b : frame) {
        parser.parse_byte(b, state, 2000);
    }

    assert(!state.fix_valid);
    assert(state.satellites == 6);
    assert(parser.stats().ubx_frames == 1);
    assert(parser.stats().ubx_nav_pvt == 1);
    assert(parser.stats().checksum_errors == 0);
    assert(parser.stats().sentences == 0);
}

void test_gps_ubx_sync_drops_partial_nmea_line() {
    gps::NmeaParser parser;
    GpsState state;

    const char *partial = "$GPGGA,partial";
    for (const char *p = partial; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1500);

    std::vector<uint8_t> payload(92, 0);
    payload[20] = 0;
    payload[21] = 0;
    payload[23] = 4;
    const std::vector<uint8_t> frame = ubx_frame(0x01, 0x07, payload);
    for (uint8_t b : frame) parser.parse_byte(b, state, 1600);
    parser.parse_byte(uint8_t('\n'), state, 1700);

    assert(!state.fix_valid);
    assert(state.satellites == 4);
    assert(parser.stats().ubx_frames == 1);
    assert(parser.stats().ubx_nav_pvt == 1);
    assert(parser.stats().sentences == 0);
    assert(parser.stats().checksum_errors == 0);
}

void test_gps_oversize_ubx_frame_is_discarded_without_nmea_noise() {
    gps::NmeaParser parser;
    GpsState state;

    std::vector<uint8_t> payload(120, 0);
    payload[10] = '$';
    payload[11] = 'G';
    payload[12] = 'P';
    payload[13] = 'G';
    payload[14] = 'G';
    payload[15] = 'A';
    payload[16] = '\n';
    const std::vector<uint8_t> frame = ubx_frame(0x01, 0x35, payload);
    for (uint8_t b : frame) parser.parse_byte(b, state, 1600);

    assert(parser.stats().ubx_oversize == 1);
    assert(parser.stats().sentences == 0);
    assert(parser.stats().checksum_errors == 0);

    const std::string gga = nmea_sentence("GNGGA,123519,3723.2475,N,12158.3416,W,1,09,0.8,545.4,M,46.9,M,,");
    for (char c : gga) parser.parse_byte(uint8_t(c), state, 1700);
    assert(state.fix_valid);
    assert(state.satellites == 9);
    assert(parser.stats().gga_sentences == 1);
}

void test_gps_fix_timeout() {
    GpsState state;
    state.fix_valid = true;
    state.fix_type = 1;
    state.satellites = 10;
    state.speed_kmh = 25.0f;
    state.last_fix_ms = 1000;
    state.receiving = true;
    state.last_message_ms = 1000;
    gps::update_timeout(state, 1000 + GPS_FIX_TIMEOUT_MS);
    assert(state.fix_valid);
    assert(state.receiving);
    gps::update_timeout(state, 1001 + GPS_FIX_TIMEOUT_MS);
    assert(!state.fix_valid);
    assert(state.fix_type == 0);
    assert(state.satellites == 0);
    assert(state.speed_kmh == 0.0f);
    assert(state.receiving);

    gps::update_timeout(state, 1000 + GPS_LINK_TIMEOUT_MS);
    assert(state.receiving);
    gps::update_timeout(state, 1001 + GPS_LINK_TIMEOUT_MS);
    assert(!state.receiving);
}

void test_gps_invalid_sentences_clear_stale_fix() {
    gps::NmeaParser parser;
    GpsState state;
    const char *valid = "$GPGGA,123519,3723.2475,N,12158.3416,W,1,08,0.9,545.4,M,46.9,M,,*59\r\n";
    for (const char *p = valid; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1000);
    state.speed_kmh = 33.0f;
    state.course_deg = 90.0f;
    assert(state.fix_valid);

    const char *no_fix = "$GPGGA,123520,3723.2475,N,12158.3416,W,0,03,9.9,545.4,M,46.9,M,,*50\r\n";
    for (const char *p = no_fix; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1100);
    assert(!state.fix_valid);
    assert(state.fix_type == 0);
    assert(state.satellites == 3);
    assert(state.speed_kmh == 0.0f);
    assert(state.course_deg == 0.0f);

    const char *bad_checksum = "$GPGGA,123521,3723.2475,N,12158.3416,W,1,08,0.9,545.4,M,46.9,M,,*00\r\n";
    state.updated = true;
    for (const char *p = bad_checksum; *p; ++p) parser.parse_byte(uint8_t(*p), state, 1200);
    assert(!state.fix_valid);
    assert(!state.updated);
    assert(parser.stats().checksum_errors == 1);
    assert(strcmp(parser.stats().last_bad_type, "GPGGA") == 0);
}

void test_nav() {
    const float d = nav::haversine_m(37.0, -122.0, 37.001, -122.0);
    assert(d > 100.0f && d < 120.0f);
    const float b = nav::bearing_deg(37.0, -122.0, 37.001, -122.0);
    assert(b < 1.0f || b > 359.0f);

    assert(fabs(nav::bearing_deg(0.0, 0.0, 0.0, 1.0) - 90.0f) < 0.01f);
    assert(fabs(nav::bearing_deg(0.0, 0.0, -1.0, 0.0) - 180.0f) < 0.01f);
    assert(fabs(nav::bearing_deg(0.0, 0.0, 0.0, -1.0) - 270.0f) < 0.01f);
    assert(fabs(nav::bearing_deg(0.0, 179.9, 0.0, -179.9) - 90.0f) < 0.1f);
}

void test_nav_state_updates_and_jump_rejection() {
    NavState nav_state;
    GpsState gps;
    gps.fix_valid = true;
    gps.lat_deg = 37.0;
    gps.lon_deg = -122.0;
    gps.speed_kmh = 10.0f;
    nav::on_arm(nav_state, gps, 1000);
    assert(nav_state.home_set);

    gps.lat_deg = 37.0001;
    gps.lon_deg = -122.0;
    gps.speed_kmh = 12.0f;
    nav::update(nav_state, gps, true, 2000);
    assert(nav_state.home_distance_m > 10.0f && nav_state.home_distance_m < 12.0f);
    assert(nav_state.trip_distance_m > 10.0f && nav_state.trip_distance_m < 12.0f);
    assert(nav_state.max_speed_kmh == 12.0f);

    const float trip_before_jump = nav_state.trip_distance_m;
    gps.lat_deg = 37.0100;
    nav::update(nav_state, gps, true, 3000);
    assert(fabs(nav_state.trip_distance_m - trip_before_jump) < 0.001f);

    gps.speed_kmh = 300.0f;
    nav::update(nav_state, gps, true, 4000);
    assert(nav_state.max_speed_kmh == 12.0f);
    assert(nav::runtime_seconds(nav_state, true, 61000) == 60);

    nav::on_disarm(nav_state);
    assert(!nav_state.home_set);
    assert(nav_state.home_distance_m == 0.0f);
    assert(nav_state.home_bearing_deg == 0.0f);
}

void test_battery_voltage_conversion() {
    assert(fabs(battery::adc_raw_to_volts(0) - 0.0f) < 0.001f);
    assert(fabs(battery::adc_raw_to_volts(4095) - (3.3f * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR)) < 0.001f);
    assert(fabs(battery::adc_raw_to_volts(2048) - (3.3f * (2048.0f / 4095.0f) * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR)) < 0.001f);
}

void test_imu_aircraft_axis_convention() {
    ImuState pitch_state;
    imu_mpu6050::update_filter_from_sample(pitch_state, 0.0f, 0.0f, 1.0f, 0.0f, 10.0f, 0.0f, 1000);
    imu_mpu6050::update_filter_from_sample(pitch_state, 0.0f, 0.0f, 1.0f, 0.0f, 10.0f, 0.0f, 1100);
    assert(pitch_state.pitch_deg > 0.9f && pitch_state.pitch_deg < 1.1f);
    assert(fabs(pitch_state.roll_deg) < 0.01f);

    ImuState roll_state;
    imu_mpu6050::update_filter_from_sample(roll_state, 0.0f, 0.0f, 1.0f, 10.0f, 0.0f, 0.0f, 1000);
    imu_mpu6050::update_filter_from_sample(roll_state, 0.0f, 0.0f, 1.0f, 10.0f, 0.0f, 0.0f, 1100);
    assert(roll_state.roll_deg > 0.9f && roll_state.roll_deg < 1.1f);
    assert(fabs(roll_state.pitch_deg) < 0.01f);
}

void test_imu_negative_angles_and_zero_crossing() {
    ImuState state;

    // Positive gyro integration first, then negative integration. Pitch/roll
    // must cross below zero instead of clamping or getting stuck at zero.
    imu_mpu6050::update_filter_from_sample(state, 0.0f, 0.0f, 1.0f, 20.0f, 20.0f, 0.0f, 1000);
    imu_mpu6050::update_filter_from_sample(state, 0.0f, 0.0f, 1.0f, 20.0f, 20.0f, 0.0f, 1100);
    assert(state.pitch_deg > 1.9f);
    assert(state.roll_deg > 1.9f);

    for (int i = 0; i < 4; ++i) {
        imu_mpu6050::update_filter_from_sample(state, 0.0f, 0.0f, 1.0f, -20.0f, -20.0f, 0.0f, 1200u + uint32_t(i) * 100u);
    }
    assert(state.pitch_deg < -1.0f);
    assert(state.roll_deg < -1.0f);

    // Accelerometer-only estimates should also support negative values.
    ImuState accel_state;
    for (int i = 0; i < 80; ++i) {
        imu_mpu6050::update_filter_from_sample(accel_state, 0.2f, -0.2f, 0.98f, 0.0f, 0.0f, 0.0f, 1000u + uint32_t(i) * 10u);
    }
    assert(accel_state.pitch_deg < -5.0f);
    assert(accel_state.roll_deg < -5.0f);
}

void test_bmp390_compensation() {
    uint8_t raw_cal[21] = {};
    const auto put_u16 = [&](int idx, uint16_t v) {
        raw_cal[idx] = uint8_t(v);
        raw_cal[idx + 1] = uint8_t(v >> 8);
    };
    put_u16(0, 29710);
    put_u16(2, 25600);
    raw_cal[4] = 0;
    put_u16(5, 16384);
    put_u16(7, 16384);
    raw_cal[9] = 0;
    raw_cal[10] = 0;
    put_u16(11, 12517);
    put_u16(13, 0);
    raw_cal[15] = 0;
    raw_cal[16] = 0;
    put_u16(17, 0);
    raw_cal[19] = 0;
    raw_cal[20] = 0;

    bmp390::Calibration cal;
    bmp390::parse_calibration(raw_cal, cal);
    double t_lin = 0.0;
    const double temp = bmp390::compensate_temperature(8654371, cal, t_lin);
    const double pressure = bmp390::compensate_pressure(5373952, cal, t_lin);
    assert(fabs(temp - 25.0008) < 0.01);
    assert(fabs(pressure - 100136.0) < 1.0);
    assert(fabs(bmp390::pressure_to_altitude(100136.0, 100136.0)) < 0.01);
    assert(bmp390::pressure_to_altitude(100000.0, 101325.0) > 0.0f);
    assert(bmp390::pressure_to_altitude(102000.0, 101325.0) < 0.0f);
}

void test_bmp390_provided_vector() {
    // This coefficient/raw pairing was provided as an "official vector" in
    // project notes. Running the Bosch BMP3 floating-point formula with these
    // exact numbers yields ~28.71 C and ~131788 Pa, not 25 C / 100135 Pa.
    // Keep this test to lock our implementation to Bosch's coefficient scaling
    // and compensation math, while avoiding a false expected value.
    uint8_t raw_cal[21] = {};
    const auto put_u16 = [&](int idx, uint16_t v) {
        raw_cal[idx] = uint8_t(v);
        raw_cal[idx + 1] = uint8_t(v >> 8);
    };
    const auto put_i16 = [&](int idx, int16_t v) {
        put_u16(idx, uint16_t(v));
    };

    put_u16(0, 27402);
    put_u16(2, 18868);
    raw_cal[4] = uint8_t(int8_t(-10));
    put_i16(5, -244);
    put_i16(7, -3254);
    raw_cal[9] = 35;
    raw_cal[10] = 0;
    put_u16(11, 25879);
    put_u16(13, 31477);
    raw_cal[15] = uint8_t(int8_t(-13));
    raw_cal[16] = uint8_t(int8_t(-10));
    put_i16(17, 16342);
    raw_cal[19] = 29;
    raw_cal[20] = uint8_t(int8_t(-60));

    bmp390::Calibration cal;
    bmp390::parse_calibration(raw_cal, cal);
    double t_lin = 0.0;
    const double temp = bmp390::compensate_temperature(8654371, cal, t_lin);
    const double pressure = bmp390::compensate_pressure(5373952, cal, t_lin);
    assert(fabs(temp - 28.7134) < 0.001);
    assert(fabs(pressure - 131788.06) < 0.1);
}

void test_bmp390_zero_reference_recomputes_altitude() {
    BaroState state;
    state.pressure_pa = 90000.0f;
    state.altitude_m = 1000.0f;
    state.relative_altitude_m = 1000.0f;
    state.vertical_speed_ms = 2.0f;

    bmp390::set_zero_reference(state);

    assert(fabs(state.altitude_m) < 0.01f);
    assert(fabs(state.relative_altitude_m) < 0.01f);
    assert(fabs(state.vertical_speed_ms) < 0.01f);
}

void test_bmp390_baseline_seed_rejects_startup_jump() {
    assert(!bmp390::baseline_seed_ready(0.0f));
    assert(!bmp390::baseline_seed_ready(87500.0f));
    assert(!bmp390::baseline_seed_ready(101170.0f));
    assert(!bmp390::baseline_seed_ready(101171.0f));
    assert(!bmp390::baseline_seed_ready(101169.0f));
    assert(!bmp390::baseline_seed_ready(101172.0f));
    assert(bmp390::baseline_seed_ready(101171.0f));
    assert(!bmp390::baseline_seed_ready(0.0f));
}

void test_compass_calibration() {
    compass_qmc5883l::calibration_start();
    for (int i = 0; i < 25; ++i) {
        compass_qmc5883l::calibration_update(-500, -400, -350);
        compass_qmc5883l::calibration_update(300, 400, 450);
    }
    CompassCalibration cal;
    assert(compass_qmc5883l::calibration_finish(cal));
    assert(fabs(cal.x_offset + 100.0f) < 0.001f);
    assert(fabs(cal.y_offset - 0.0f) < 0.001f);
    assert(fabs(cal.z_offset - 50.0f) < 0.001f);
    assert(fabs(cal.x_scale - 1.0f) < 0.001f);
    assert(fabs(cal.y_scale - 1.0f) < 0.001f);
    assert(fabs(cal.z_scale - 1.0f) < 0.001f);
}

void test_compass_heading_formula() {
    CompassCalibration cal;
    cal.x_scale = 1.0f;
    cal.y_scale = 1.0f;
    cal.z_scale = 1.0f;
    ImuState imu;

    assert(fabs(compass_qmc5883l::heading_from_sample(100, 0, 0, cal, imu) - 0.0f) < 0.01f);
    assert(fabs(compass_qmc5883l::heading_from_sample(0, -100, 0, cal, imu) - 90.0f) < 0.01f);
    assert(fabs(compass_qmc5883l::heading_from_sample(-100, 0, 0, cal, imu) - 180.0f) < 0.01f);
    assert(fabs(compass_qmc5883l::heading_from_sample(0, 100, 0, cal, imu) - 270.0f) < 0.01f);

    cal.declination_deg = 15.0f;
    assert(fabs(compass_qmc5883l::heading_from_sample(100, 0, 0, cal, imu) - 15.0f) < 0.01f);
    cal.declination_deg = -20.0f;
    assert(fabs(compass_qmc5883l::heading_from_sample(100, 0, 0, cal, imu) - 340.0f) < 0.01f);

    cal.declination_deg = 0.0f;
    cal.x_offset = 10.0f;
    cal.y_offset = -20.0f;
    assert(fabs(compass_qmc5883l::heading_from_sample(110, -20, 0, cal, imu) - 0.0f) < 0.01f);

    imu.valid = true;
    imu.pitch_deg = 0.0f;
    imu.roll_deg = 0.0f;
    assert(fabs(compass_qmc5883l::heading_from_sample(110, -20, 50, cal, imu) - 0.0f) < 0.01f);
}

void test_compass_calibration_rejects_small_ranges() {
    compass_qmc5883l::calibration_start();
    for (int i = 0; i < 25; ++i) {
        compass_qmc5883l::calibration_update(10, 10, 10);
        compass_qmc5883l::calibration_update(20, 20, 20);
    }
    CompassCalibration cal;
    assert(!compass_qmc5883l::calibration_finish(cal));
    assert(!cal.valid);
}

void test_compass_calibration_rejects_low_sample_count() {
    compass_qmc5883l::calibration_start();
    for (int i = 0; i < 9; ++i) {
        compass_qmc5883l::calibration_update(-500, -500, -500);
        compass_qmc5883l::calibration_update(500, 500, 500);
    }
    CompassCalibration cal;
    assert(!compass_qmc5883l::calibration_finish(cal));
    assert(!cal.valid);
}

void test_compass_calibration_rejects_small_axis_span() {
    compass_qmc5883l::calibration_start();
    for (int i = 0; i < 25; ++i) {
        compass_qmc5883l::calibration_update(-500, -500, -10);
        compass_qmc5883l::calibration_update(500, 500, 10);
    }
    CompassCalibration cal;
    assert(!compass_qmc5883l::calibration_finish(cal));
    assert(!cal.valid);
}

void test_ubx_checksum() {
    const uint8_t payload[] = {0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    gps::ubx_checksum(payload, sizeof(payload), ck_a, ck_b);
    assert(ck_a == 0xDE);
    assert(ck_b == 0x6A);
}

void test_first_signal_after_boot_requires_low_switch_before_arm() {
    TelemetryState state;
    state.app_state = AppState::Failsafe;
    state.armed = false;
    state.rc.has_armed_once = false;

    set_rc_switch(state, RcSwitchPosition::High);
    auto transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(!transition.armed_rising);
    assert(!state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::Low);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(state.rc.rearm_latched);
}

void test_failsafe_recovery_restores_selected_arm_state_after_first_arm() {
    TelemetryState state;
    state.app_state = AppState::Armed;
    state.armed = true;
    state.rc.has_armed_once = true;
    set_rc_switch(state, RcSwitchPosition::Unknown);

    auto transition = state_machine::update(state);
    assert(state.app_state == AppState::Failsafe);
    assert(!state.armed);
    assert(transition.armed_falling);

    set_rc_switch(state, RcSwitchPosition::High);
    state.rc.rearm_latched = false;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(transition.armed_rising);
    assert(!state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::Low);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(state.rc.rearm_latched);
}

void test_state_machine_arm_disarm_and_failsafe_latch() {
    TelemetryState state;
    state.app_state = AppState::Initializing;
    set_rc_switch(state, RcSwitchPosition::High);
    auto transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(!transition.armed_rising);
    assert(!state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::Low);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::Middle);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(transition.armed_rising);
    assert(!state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::High);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(!transition.armed_falling);
    assert(!transition.armed_rising);

    set_rc_switch(state, RcSwitchPosition::Middle);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(!transition.armed_falling);
    assert(!transition.armed_rising);

    set_rc_switch(state, RcSwitchPosition::Low);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(transition.armed_falling);

    set_rc_switch(state, RcSwitchPosition::High);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(transition.armed_rising);
    assert(state.rc.has_armed_once);

    state.app_state = AppState::Initializing;
    state.armed = true;
    set_rc_switch(state, RcSwitchPosition::Unknown);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Failsafe);
    assert(!state.armed);
    assert(transition.armed_falling);

    state.app_state = AppState::Armed;
    state.armed = true;
    state.rc.has_armed_once = true;
    set_rc_switch(state, RcSwitchPosition::Unknown);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Failsafe);
    assert(!state.armed);
    assert(transition.armed_falling);

    set_rc_switch(state, RcSwitchPosition::High);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(transition.armed_rising);
    assert(!state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::Low);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(state.rc.rearm_latched);

    set_rc_switch(state, RcSwitchPosition::High);
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(transition.armed_rising);
}

void test_calibration_flash_codec_and_crc() {
    CompassCalibration cal;
    cal.x_offset = -12.5f;
    cal.y_offset = 34.25f;
    cal.z_offset = 9.75f;
    cal.x_scale = 1.10f;
    cal.y_scale = 0.95f;
    cal.z_scale = 1.03f;
    cal.declination_deg = 13.2f;
    cal.valid = true;

    std::vector<uint8_t> storage(calibration_flash::storage_size());
    assert(calibration_flash::encode(cal, storage.data(), storage.size()));
    assert(calibration_flash::crc32_bytes(storage.data(), storage.size() - sizeof(uint32_t)) != 0);

    CompassCalibration decoded;
    assert(calibration_flash::decode(storage.data(), storage.size(), decoded));
    assert(decoded.valid);
    assert(fabs(decoded.x_offset - cal.x_offset) < 0.001f);
    assert(fabs(decoded.y_offset - cal.y_offset) < 0.001f);
    assert(fabs(decoded.z_offset - cal.z_offset) < 0.001f);
    assert(fabs(decoded.x_scale - cal.x_scale) < 0.001f);
    assert(fabs(decoded.y_scale - cal.y_scale) < 0.001f);
    assert(fabs(decoded.z_scale - cal.z_scale) < 0.001f);
    assert(fabs(decoded.declination_deg - cal.declination_deg) < 0.001f);

    storage[sizeof(uint32_t)] ^= 0x01;
    assert(!calibration_flash::decode(storage.data(), storage.size(), decoded));
    assert(!decoded.valid);
}

void test_displayport_frame_shape() {
    TelemetryState state;
    state.armed = true;
    state.rc.signal_valid = true;
    state.rc.pulse_us = 1875;
    state.battery.volts = 12.2f;
    state.baro.valid = true;
    state.baro.relative_altitude_m = 24.0f;
    state.baro.vertical_speed_ms = 1.2f;
    state.baro.temperature_c = 32.0f;
    state.gps.fix_valid = true;
    state.gps.receiving = true;
    state.gps.last_message_ms = state.now_ms;
    state.gps.satellites = 12;
    state.gps.speed_kmh = 38.0f;
    state.gps.course_deg = 87.0f;
    state.nav.home_set = true;
    state.nav.home_distance_m = 184.0f;
    state.nav.home_bearing_deg = 245.0f;
    state.nav.max_speed_kmh = 52.0f;
    state.nav.trip_distance_m = 1240.0f;
    state.nav.armed_since_ms = 1000;
    state.now_ms = 223000;
    state.imu.valid = true;
    state.imu.pitch_deg = 5.2f;
    state.imu.roll_deg = -2.1f;

    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(g_written.size() > 0);
    assert_displayport_writes_fit_canvas();
    assert(g_written[0] == '$');
    assert(g_written[4] == msp::MSP_DISPLAYPORT);
    assert(g_written[5] == 5); // DisplayPort options subcommand.
    assert(g_written[6] == OSD_DISPLAYPORT_FONT);
    assert(g_written[7] == OSD_DISPLAYPORT_RESOLUTION);

    bool saw_arm = false;
    bool saw_attitude = false;
    bool saw_distance = false;
    for (size_t i = 0; i + 6 < g_written.size();) {
        assert(g_written[i] == '$');
        assert(g_written[i + 1] == 'M');
        const uint8_t len = g_written[i + 3];
        const uint8_t cmd = g_written[i + 4];
        assert(cmd == msp::MSP_DISPLAYPORT);
        const uint8_t *payload = &g_written[i + 5];
        if (len >= 4 && payload[0] == 3) {
            if (payload[1] == 2 && payload[2] == 18) {
                saw_arm = true;
                assert(memmem(payload + 4, len - 4, "ARMED", 5) != nullptr);
            }
            if (payload[1] == OSD_ROWS - 1 && payload[2] == 2) {
                saw_attitude = true;
                assert(memmem(payload + 4, len - 4, "PIT:", 4) != nullptr);
            }
            if (payload[1] == 4 && payload[2] == 18) {
                saw_distance = true;
                assert(memmem(payload + 4, len - 4, "DST:", 4) != nullptr);
            }
        }
        i += size_t(len) + 6;
    }
    assert(saw_arm);
    assert(saw_attitude);
    assert(saw_distance);
}

void test_msp_debug_counters_for_osd_requests() {
    TelemetryState state;
    msp::Parser parser;

    clear_capture();
    make_request(msp::MSP_OSD_CONFIG, parser, state);
    assert(parser.stats().osd_config_requests == 1);

    clear_capture();
    make_request(msp::MSP_OSD_CANVAS, parser, state);
    assert(parser.stats().osd_canvas_requests == 1);

    const uint8_t len = 2;
    const uint8_t cmd = msp::MSP_SET_OSD_CANVAS;
    const uint8_t cols = 50;
    const uint8_t rows = 18;
    const uint8_t checksum = len ^ cmd ^ cols ^ rows;
    const uint8_t frame[] = {'$', 'M', '<', len, cmd, cols, rows, checksum};
    clear_capture();
    msp::Endpoint endpoint{capture_writer, nullptr};
    for (uint8_t b : frame) parser.parse_byte(b, state, endpoint);
    assert(parser.stats().set_osd_canvas_requests == 1);
    assert(parser.stats().last_canvas_cols == cols);
    assert(parser.stats().last_canvas_rows == rows);
}

void test_msp_status_sets_dji_failsafe_flag() {
    TelemetryState state;
    uint8_t out[128] = {};
    size_t len = 0;

    state.armed = true;
    state.app_state = AppState::Armed;
    state.rc.failsafe = false;
    assert(msp::build_response(msp::MSP_STATUS, state, out, sizeof(out), &len));
    assert(le32(&out[6]) == 1);

    state.armed = false;
    state.rc.failsafe = true;
    assert(msp::build_response(msp::MSP_STATUS, state, out, sizeof(out), &len));
    assert(le32(&out[6]) == (1u << 4));
}

void test_displayport_home_direction() {
    TelemetryState state;
    state.nav.home_set = true;
    state.nav.home_distance_m = 50.0f;
    state.baro.valid = true;
    state.compass.valid = true;
    state.gps.receiving = true;

    state.nav.home_bearing_deg = 90.0f;
    state.compass.heading_deg = 0.0f;
    state.gps.fix_valid = true;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_text_at(6, 18).find("090") != std::string::npos);
    assert(displayport_text_at(1, 18).find("HDG:N 000") != std::string::npos);

    state.nav.home_bearing_deg = 0.0f;
    state.compass.heading_deg = 90.0f;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_text_at(6, 18).find("000") != std::string::npos);
    assert(displayport_text_at(1, 18).find("HDG:E 090") != std::string::npos);

    state.nav.home_distance_m = 0.5f;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_text_at(6, 18).find("HOM:  1M 000") != std::string::npos);

    state.nav.home_distance_m = 50.0f;
    state.compass.valid = false;
    state.gps.fix_valid = false;
    state.nav.home_bearing_deg = 225.0f;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_text_at(6, 18).empty());
}

void test_displayport_satellites_without_fix() {
    TelemetryState state;
    state.gps.receiving = true;
    state.gps.fix_valid = false;
    state.gps.satellites = 7;

    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_text_at(0, 18).find("GPS:07") != std::string::npos);
    assert(displayport_text_at(3, 18).empty());
    assert(displayport_text_at(5, 18).empty());
}

void test_displayport_cockpit_mode() {
    TelemetryState state;
    state.armed = true;
    set_rc_switch(state, RcSwitchPosition::Middle);
    state.imu.valid = true;
    state.imu.pitch_deg = 8.0f;
    state.imu.roll_deg = -12.0f;
    state.baro.valid = true;
    state.baro.vertical_speed_ms = 1.0f;

    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_text_at(8, 15).empty());
    assert(displayport_text_at(OSD_ROWS - 1, 2).find("PIT:") != std::string::npos);

    set_rc_switch(state, RcSwitchPosition::High);
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert_displayport_writes_fit_canvas();
    uint8_t min_horizon_col = 0;
    uint8_t max_horizon_col = 0;
    assert(count_displayport_char('-', &min_horizon_col, &max_horizon_col) >= 20);
    assert(min_horizon_col < OSD_COCKPIT_CENTER_COL);
    assert(max_horizon_col > OSD_COCKPIT_CENTER_COL);
    assert((OSD_COCKPIT_CENTER_COL - min_horizon_col) == (max_horizon_col - OSD_COCKPIT_CENTER_COL));
    assert(displayport_text_at(OSD_COCKPIT_CENTER_ROW, OSD_COCKPIT_CENTER_COL).empty());
    assert(displayport_text_at(OSD_ROWS - 3, 2).find("VS:+1.0MS") != std::string::npos);
    assert(displayport_text_at(OSD_ROWS - 2, 2).find("ALT:") != std::string::npos);
    assert(displayport_text_at(OSD_ROWS - 1, 2).find("PIT:") != std::string::npos);
    assert(displayport_text_at(OSD_ROWS - 1, 2).find("ROL:") != std::string::npos);

    state.baro.valid = false;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert_displayport_writes_fit_canvas();
    assert(displayport_text_at(OSD_ROWS - 3, 2).empty());
    assert(displayport_text_at(OSD_ROWS - 2, 2).empty());
    assert(displayport_text_at(OSD_ROWS - 1, 2).find("PIT:") != std::string::npos);
    assert(displayport_text_at(OSD_ROWS - 1, 2).find("ROL:") != std::string::npos);
    assert(count_displayport_char('-') >= 12);
}

void test_displayport_cockpit_extreme_angles_stay_on_canvas() {
    TelemetryState state;
    state.armed = true;
    set_rc_switch(state, RcSwitchPosition::High);
    state.imu.valid = true;
    state.imu.pitch_deg = 89.0f;
    state.imu.roll_deg = 180.0f;
    state.baro.valid = true;
    state.baro.vertical_speed_ms = -12.5f;

    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert_displayport_writes_fit_canvas();
    assert(count_displayport_char('-') >= 4);

    state.imu.pitch_deg = -89.0f;
    state.imu.roll_deg = -180.0f;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert_displayport_writes_fit_canvas();
    assert(count_displayport_char('-') >= 4);
}

void test_displayport_warning_priority() {
    TelemetryState state;
    state.armed = true;
    state.rc.signal_valid = false;
    state.rc.failsafe = true;
    state.cal_needed = true;
    state.baro.valid = false;
    state.compass.valid = false;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_row_text(4).find("RC FAILSAFE") != std::string::npos);

    state.rc.failsafe = false;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_row_text(4).find("RC FAILSAFE") != std::string::npos);

    state.rc.signal_valid = true;
    state.rc.arm_switch_high = true;
    state.rc.rearm_latched = false;
    state.armed = false;
    state.gps.fix_valid = false;
    state.nav.home_set = false;
    state.cal_needed = false;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_row_text(4).find("SWITCH LOW") != std::string::npos);

    state.rc.arm_switch_high = false;
    state.rc.rearm_latched = true;
    state.armed = false;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_row_text(4).empty());

    state.cal_needed = true;
    state.compass.initialized = false;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_row_text(4).empty());

    state.compass.initialized = true;
    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(displayport_row_text(4).find("CAL NEEDED") != std::string::npos);
}

}

int main() {
    test_msp_identity();
    test_msp_build_info_and_name_exact();
    test_msp_payload_sizes();
    test_msp_dynamic_payloads();
    test_msp_payload_clamps_and_normalizes();
    test_msp_raw_gps_no_fix_payload_is_zero();
    test_msp_parser_rejection_and_ignores();
    test_sensor_mask();
    test_gps_nmea();
    test_gps_rmc_and_vtg();
    test_gps_accepts_multi_gnss_talker_ids();
    test_gps_coordinate_hemispheres();
    test_gps_ubx_nav_pvt();
    test_gps_ubx_no_fix_clears_motion_and_preserves_satellites();
    test_gps_mixed_ubx_payload_does_not_create_nmea_crc_noise();
    test_gps_ubx_sync_drops_partial_nmea_line();
    test_gps_oversize_ubx_frame_is_discarded_without_nmea_noise();
    test_gps_fix_timeout();
    test_gps_invalid_sentences_clear_stale_fix();
    test_nav();
    test_nav_state_updates_and_jump_rejection();
    test_battery_voltage_conversion();
    test_imu_aircraft_axis_convention();
    test_imu_negative_angles_and_zero_crossing();
    test_bmp390_compensation();
    test_bmp390_provided_vector();
    test_bmp390_zero_reference_recomputes_altitude();
    test_bmp390_baseline_seed_rejects_startup_jump();
    test_compass_calibration();
    test_compass_heading_formula();
    test_compass_calibration_rejects_small_ranges();
    test_compass_calibration_rejects_low_sample_count();
    test_compass_calibration_rejects_small_axis_span();
    test_ubx_checksum();
    test_first_signal_after_boot_requires_low_switch_before_arm();
    test_failsafe_recovery_restores_selected_arm_state_after_first_arm();
    test_state_machine_arm_disarm_and_failsafe_latch();
    test_calibration_flash_codec_and_crc();
    test_displayport_frame_shape();
    test_msp_debug_counters_for_osd_requests();
    test_msp_status_sets_dji_failsafe_flag();
    test_displayport_home_direction();
    test_displayport_satellites_without_fix();
    test_displayport_cockpit_mode();
    test_displayport_cockpit_extreme_angles_stay_on_canvas();
    test_displayport_warning_priority();
    puts("host tests passed");
    return 0;
}
