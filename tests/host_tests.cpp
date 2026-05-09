#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "gps.h"
#include "config.h"
#include "msp.h"
#include "nav.h"
#include "bmp390.h"
#include "compass_qmc5883l.h"
#include "calibration_flash.h"
#include "displayport.h"
#include "state_machine.h"

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

void test_msp_identity() {
    TelemetryState state;
    msp::Parser parser;
    clear_capture();
    make_request(msp::MSP_API_VERSION, parser, state);
    assert(response_cmd() == msp::MSP_API_VERSION);
    assert(response_len() == 3);
    assert(g_written[5] == 0);
    assert(g_written[6] == 1);
    assert(g_written[7] == 42);

    clear_capture();
    make_request(msp::MSP_FC_VARIANT, parser, state);
    assert(response_len() == 4);
    assert(memcmp(&g_written[5], "BTFL", 4) == 0);

    clear_capture();
    make_request(msp::MSP_NAME, parser, state);
    assert(response_len() == 9);
    assert(memcmp(&g_written[5], "RC-O4-OSD", 9) == 0);
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
    assert(out[0] == 3);
    assert(out[3] == 126);

    assert(msp::build_response(msp::MSP_STATUS_EX, state, out, sizeof(out), &len));
    assert(len == 16);
}

void test_msp_dynamic_payloads() {
    TelemetryState state;
    state.armed = true;
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

void test_gps_fix_timeout() {
    GpsState state;
    state.fix_valid = true;
    state.fix_type = 1;
    state.satellites = 10;
    state.speed_kmh = 25.0f;
    state.last_fix_ms = 1000;
    gps::update_timeout(state, 1000 + GPS_FIX_TIMEOUT_MS);
    assert(state.fix_valid);
    gps::update_timeout(state, 1001 + GPS_FIX_TIMEOUT_MS);
    assert(!state.fix_valid);
    assert(state.fix_type == 0);
    assert(state.satellites == 0);
    assert(state.speed_kmh == 0.0f);
}

void test_nav() {
    const float d = nav::haversine_m(37.0, -122.0, 37.001, -122.0);
    assert(d > 100.0f && d < 120.0f);
    const float b = nav::bearing_deg(37.0, -122.0, 37.001, -122.0);
    assert(b < 1.0f || b > 359.0f);
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

void test_ubx_checksum() {
    const uint8_t payload[] = {0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    gps::ubx_checksum(payload, sizeof(payload), ck_a, ck_b);
    assert(ck_a == 0xDE);
    assert(ck_b == 0x6A);
}

void test_state_machine_arm_disarm_and_failsafe_latch() {
    TelemetryState state;
    state.app_state = AppState::Initializing;
    state.rc.failsafe = false;
    auto transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(!transition.armed_rising);

    state.rc.failsafe = false;
    state.rc.signal_valid = true;
    state.rc.arm_switch_high = true;
    state.rc.arm_switch_low = false;
    state.rc.rearm_latched = true;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(state.armed);
    assert(transition.armed_rising);
    assert(!state.rc.rearm_latched);

    state.rc.arm_switch_high = false;
    state.rc.arm_switch_low = true;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(transition.armed_falling);

    state.rc.arm_switch_high = true;
    state.rc.arm_switch_low = false;
    state.rc.rearm_latched = true;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Armed);
    assert(transition.armed_rising);

    state.app_state = AppState::Initializing;
    state.armed = true;
    state.rc.failsafe = true;
    state.rc.signal_valid = false;
    state.rc.arm_switch_high = false;
    state.rc.arm_switch_low = false;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Failsafe);
    assert(!state.armed);
    assert(transition.armed_falling);

    state.app_state = AppState::Armed;
    state.armed = true;
    state.rc.failsafe = true;
    state.rc.signal_valid = false;
    state.rc.arm_switch_high = false;
    state.rc.arm_switch_low = false;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Failsafe);
    assert(!state.armed);
    assert(transition.armed_falling);

    state.rc.failsafe = false;
    state.rc.signal_valid = true;
    state.rc.arm_switch_high = true;
    state.rc.arm_switch_low = false;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Failsafe);
    assert(!state.armed);
    assert(!transition.armed_rising);

    state.rc.arm_switch_high = false;
    state.rc.arm_switch_low = true;
    transition = state_machine::update(state);
    assert(state.app_state == AppState::Disarmed);
    assert(!state.armed);
    assert(state.rc.rearm_latched);

    state.rc.arm_switch_high = true;
    state.rc.arm_switch_low = false;
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
    state.imu.pitch_deg = 5.2f;
    state.imu.roll_deg = -2.1f;

    clear_capture();
    displayport::send_frame(msp::Endpoint{capture_writer, nullptr}, state);
    assert(g_written.size() > 0);
    assert(g_written[0] == '$');
    assert(g_written[4] == msp::MSP_DISPLAYPORT);
    assert(g_written[5] == 0); // heartbeat subcommand

    bool saw_row0 = false;
    bool saw_row8 = false;
    for (size_t i = 0; i + 6 < g_written.size();) {
        assert(g_written[i] == '$');
        assert(g_written[i + 1] == 'M');
        const uint8_t len = g_written[i + 3];
        const uint8_t cmd = g_written[i + 4];
        assert(cmd == msp::MSP_DISPLAYPORT);
        const uint8_t *payload = &g_written[i + 5];
        if (len >= 4 && payload[0] == 3) {
            assert(payload[2] == 0);
            assert(len <= 34);
            if (payload[1] == 0) {
                saw_row0 = true;
                assert(memmem(payload + 4, len - 4, "ARM:ARMED", 9) != nullptr);
            }
            if (payload[1] == 8) {
                saw_row8 = true;
                assert(memmem(payload + 4, len - 4, "PITCH:", 6) != nullptr);
            }
        }
        i += size_t(len) + 6;
    }
    assert(saw_row0);
    assert(saw_row8);
}

}

int main() {
    test_msp_identity();
    test_msp_payload_sizes();
    test_msp_dynamic_payloads();
    test_msp_parser_rejection_and_ignores();
    test_sensor_mask();
    test_gps_nmea();
    test_gps_rmc_and_vtg();
    test_gps_fix_timeout();
    test_nav();
    test_nav_state_updates_and_jump_rejection();
    test_bmp390_compensation();
    test_bmp390_provided_vector();
    test_compass_calibration();
    test_compass_calibration_rejects_small_ranges();
    test_ubx_checksum();
    test_state_machine_arm_disarm_and_failsafe_latch();
    test_calibration_flash_codec_and_crc();
    test_displayport_frame_shape();
    puts("host tests passed");
    return 0;
}
