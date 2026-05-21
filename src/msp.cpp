#include "msp.h"

#include <math.h>
#include <string.h>
#include "config.h"
#include "msp_osd_config.h"

namespace msp {
namespace {

constexpr uint8_t kProtocolVersion = 0;
constexpr uint8_t kApiMajor = 1;
constexpr uint8_t kApiMinor = 45;
constexpr uint8_t kFcMajor = 4;
constexpr uint8_t kFcMinor = 4;
constexpr uint8_t kFcPatch = 3;
constexpr uint8_t kDjiFlagArm = 0;
constexpr uint8_t kDjiFlagFailsafe = 4;

static void put_u8(uint8_t *out, size_t &n, uint8_t v) { out[n++] = v; }
static void put_u16(uint8_t *out, size_t &n, uint16_t v) { out[n++] = uint8_t(v); out[n++] = uint8_t(v >> 8); }
static void put_i16(uint8_t *out, size_t &n, int16_t v) { put_u16(out, n, uint16_t(v)); }
static void put_u32(uint8_t *out, size_t &n, uint32_t v) { out[n++] = uint8_t(v); out[n++] = uint8_t(v >> 8); out[n++] = uint8_t(v >> 16); out[n++] = uint8_t(v >> 24); }
static void put_i32(uint8_t *out, size_t &n, int32_t v) { put_u32(out, n, uint32_t(v)); }
static void put_data(uint8_t *out, size_t &n, const void *data, size_t len) { memcpy(out + n, data, len); n += len; }

static uint8_t vbat_deci(const TelemetryState &state) {
    float v = state.battery.volts * 10.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 255.0f) v = 255.0f;
    return uint8_t(lroundf(v));
}

static uint8_t gps_fix_type(const GpsState &gps) {
    if (!gps.fix_valid) return 0;
    return gps.fix_type == 0 ? 1 : gps.fix_type;
}

static uint32_t flight_mode_flags(const TelemetryState &state) {
    uint32_t flags = 0;
    if (state.armed) {
        flags |= (1u << kDjiFlagArm);
    }
    if (state.app_state == AppState::Failsafe || state.rc.failsafe) {
        flags |= (1u << kDjiFlagFailsafe);
    }
    return flags;
}

static void build_raw_gps_payload(const TelemetryState &state, uint8_t *out, size_t &n) {
    const GpsState &gps = state.gps;
    if (!gps.fix_valid) {
        put_u8(out, n, 0);
        put_u8(out, n, 0);
        put_i32(out, n, 0);
        put_i32(out, n, 0);
        put_u16(out, n, 0);
        put_u16(out, n, 0);
        put_u16(out, n, 0);
        return;
    }

    const int32_t lat = int32_t(gps.lat_deg * 10000000.0);
    const int32_t lon = int32_t(gps.lon_deg * 10000000.0);
    const uint16_t alt = uint16_t(fmaxf(0.0f, fminf(65535.0f, gps.altitude_m)));
    const uint16_t speed = uint16_t(fmaxf(0.0f, fminf(65535.0f, gps.speed_kmh * 27.7778f)));
    float course = gps.course_deg;
    while (course < 0.0f) course += 360.0f;
    while (course >= 360.0f) course -= 360.0f;

    put_u8(out, n, gps_fix_type(gps));
    put_u8(out, n, gps.satellites);
    put_i32(out, n, lat);
    put_i32(out, n, lon);
    put_u16(out, n, alt);
    put_u16(out, n, speed);
    put_u16(out, n, uint16_t(course * 10.0f));
}

static bool is_empty_ack(uint8_t cmd) {
    switch (cmd) {
    case MSP_SET_OSD_CANVAS:
    case MSP_FILTER_CONFIG:
    case MSP_PID_ADVANCED:
    case MSP_RC:
    case MSP_RC_TUNING:
    case MSP_PID:
    case MSP_ESC_SENSOR_DATA:
    case MSP_RTC:
        return true;
    default:
        return false;
    }
}

}

uint16_t active_sensor_mask(const TelemetryState &state) {
    // Betaflight/DJI MSP sensor bit order:
    // ACC=bit0, BARO=bit1, MAG=bit2, GPS=bit3, GYRO=bit5.
    uint16_t sensors = 0;
    if (state.mpu6050_ok && state.imu.valid) sensors |= (1u << 0);
    if (state.bmp390_ok && state.baro.valid) sensors |= (1u << 1);
    if (state.compass_ok && state.compass.valid) sensors |= (1u << 2);
    if (state.gps.fix_valid) sensors |= (1u << 3);
    if (state.mpu6050_ok && state.imu.valid) sensors |= (1u << 5);
    return sensors;
}

void send_packet(Endpoint endpoint, uint8_t cmd, const uint8_t *payload, size_t len) {
    if (!endpoint.writer || len > 255) return;

    uint8_t frame[260] = {'$', 'M', '>', uint8_t(len), cmd};
    uint8_t checksum = uint8_t(len) ^ cmd;
    for (size_t i = 0; i < len; ++i) {
        frame[5 + i] = payload[i];
        checksum ^= payload[i];
    }
    frame[5 + len] = checksum;
    endpoint.writer(frame, len + 6, endpoint.ctx);
    if (endpoint.stats) {
        ++endpoint.stats->tx_packets;
    }
}

void send_empty(Endpoint endpoint, uint8_t cmd) {
    send_packet(endpoint, cmd, nullptr, 0);
}

void send_displayport(Endpoint endpoint, const uint8_t *payload, size_t len) {
    if (endpoint.stats) {
        ++endpoint.stats->displayport_packets;
    }
    send_packet(endpoint, MSP_DISPLAYPORT, payload, len);
}

void send_raw_gps(Endpoint endpoint, const TelemetryState &state) {
    uint8_t payload[16] = {};
    size_t n = 0;
    build_raw_gps_payload(state, payload, n);
    send_packet(endpoint, MSP_RAW_GPS, payload, n);
}

bool build_response(uint8_t cmd, const TelemetryState &state, uint8_t *out, size_t out_cap, size_t *out_len) {
    (void)out_cap;
    size_t n = 0;

    switch (cmd) {
    case MSP_API_VERSION:
        put_u8(out, n, kProtocolVersion);
        put_u8(out, n, kApiMajor);
        put_u8(out, n, kApiMinor);
        break;
    case MSP_FC_VARIANT:
        put_data(out, n, "BTFL", 4);
        break;
    case MSP_FC_VERSION:
        put_u8(out, n, kFcMajor);
        put_u8(out, n, kFcMinor);
        put_u8(out, n, kFcPatch);
        break;
    case MSP_BOARD_INFO:
        put_data(out, n, "PICO", 4);
        put_u16(out, n, 0);
        put_u8(out, n, 0);
        put_u8(out, n, 0);
        put_u8(out, n, 4);
        put_data(out, n, "PICO", 4);
        break;
    case MSP_BUILD_INFO:
        put_data(out, n, "Jan  1 2025", 11);
        put_data(out, n, "00:00:00", 8);
        put_data(out, n, "0000000", 7);
        break;
    case MSP_NAME:
        put_data(out, n, "RC-O4-OSD", 9);
        break;
    case MSP_STATUS:
        put_u16(out, n, state.cycle_time_us);
        put_u16(out, n, state.i2c_errors);
        put_u16(out, n, active_sensor_mask(state));
        put_u32(out, n, flight_mode_flags(state));
        put_u8(out, n, 0);
        break;
    case MSP_STATUS_EX:
        put_u16(out, n, state.cycle_time_us);
        put_u16(out, n, state.i2c_errors);
        put_u16(out, n, active_sensor_mask(state));
        put_u32(out, n, flight_mode_flags(state));
        put_u8(out, n, 0);
        put_u16(out, n, 10);
        put_u16(out, n, 0);
        put_u8(out, n, 0);
        break;
    case MSP_RAW_GPS:
        build_raw_gps_payload(state, out, n);
        break;
    case MSP_COMP_GPS:
        put_u16(out, n, uint16_t(fmaxf(0.0f, fminf(65535.0f, state.nav.home_distance_m))));
        put_u16(out, n, uint16_t(fmaxf(0.0f, fminf(359.0f, state.nav.home_bearing_deg))));
        put_u8(out, n, state.gps.updated ? 1 : 0);
        break;
    case MSP_ATTITUDE:
        put_i16(out, n, int16_t(fmaxf(-1800.0f, fminf(1800.0f, state.imu.roll_deg * 10.0f))));
        put_i16(out, n, int16_t(fmaxf(-900.0f, fminf(900.0f, state.imu.pitch_deg * 10.0f))));
        put_u16(out, n, uint16_t(fmaxf(0.0f, fminf(359.0f, state.compass.heading_deg))));
        break;
    case MSP_ALTITUDE:
        put_i32(out, n, int32_t(state.baro.relative_altitude_m * 100.0f));
        put_i16(out, n, int16_t(state.baro.vertical_speed_ms * 100.0f));
        break;
    case MSP_ANALOG:
        // API 1.42 legacy payload: 7 bytes. ArduPilot's current DJI backend
        // appends centivolts, but we keep the pinned-API form unless Stage 2
        // goggles testing proves O4 needs the extended variant.
        put_u8(out, n, vbat_deci(state));
        put_u16(out, n, 0);
        put_u16(out, n, 0);
        put_i16(out, n, 0);
        break;
    case MSP_BATTERY_STATE:
        // API 1.42 legacy payload: 9 bytes. The final state byte is present
        // in Betaflight/ArduPilot legacy forms even though the project spec's
        // field list originally only summed to 8.
        put_u8(out, n, BAT_CELLS);
        put_u16(out, n, 0);
        put_u8(out, n, vbat_deci(state));
        put_u16(out, n, 0);
        put_i16(out, n, 0);
        put_u8(out, n, 0);
        break;
    case MSP_OSD_CONFIG:
        build_osd_config_payload(out, n);
        break;
    case MSP_OSD_CANVAS:
        put_u8(out, n, OSD_CANVAS_COLS);
        put_u8(out, n, OSD_CANVAS_ROWS);
        break;
    default:
        if (is_empty_ack(cmd)) {
            n = 0;
            break;
        }
        return false;
    }

    *out_len = n;
    return true;
}

void send_startup_burst(Endpoint endpoint, const TelemetryState &state) {
    const uint8_t ids[] = {
        MSP_API_VERSION, MSP_FC_VARIANT, MSP_FC_VERSION,
        MSP_BOARD_INFO, MSP_BUILD_INFO, MSP_NAME,
        MSP_STATUS, MSP_STATUS_EX,
    };
    uint8_t payload[64] = {};
    for (uint8_t cmd : ids) {
        size_t len = 0;
        if (build_response(cmd, state, payload, sizeof(payload), &len)) {
            send_packet(endpoint, cmd, payload, len);
        }
    }
}

void Parser::reset() {
    state_ = RxState::WaitStart;
    direction_ = 0;
    len_ = 0;
    cmd_ = 0;
    checksum_ = 0;
    offset_ = 0;
}

void Parser::parse_byte(uint8_t b, const TelemetryState &telemetry, Endpoint endpoint) {
    switch (state_) {
    case RxState::WaitStart:
        if (b == '$') state_ = RxState::WaitM;
        break;
    case RxState::WaitM:
        state_ = (b == 'M') ? RxState::WaitDir : RxState::WaitStart;
        break;
    case RxState::WaitDir:
        if (b == '<' || b == '>') {
            direction_ = b;
            state_ = RxState::WaitLen;
        } else {
            reset();
        }
        break;
    case RxState::WaitLen:
        len_ = b;
        checksum_ = b;
        offset_ = 0;
        if (len_ > sizeof(payload_)) {
            reset();
        } else {
            state_ = RxState::WaitCmd;
        }
        break;
    case RxState::WaitCmd:
        cmd_ = b;
        checksum_ ^= b;
        state_ = len_ ? RxState::ReadPayload : RxState::ReadChecksum;
        break;
    case RxState::ReadPayload:
        payload_[offset_++] = b;
        checksum_ ^= b;
        if (offset_ >= len_) state_ = RxState::ReadChecksum;
        break;
    case RxState::ReadChecksum:
        if (b == checksum_ && direction_ == '<') {
            ++stats_.rx_packets;
            stats_.last_cmd = cmd_;
            if (cmd_ == MSP_NAME) {
                ++stats_.name_requests;
            } else if (cmd_ == MSP_OSD_CONFIG) {
                ++stats_.osd_config_requests;
            } else if (cmd_ == MSP_OSD_CANVAS) {
                ++stats_.osd_canvas_requests;
            } else if (cmd_ == MSP_SET_OSD_CANVAS) {
                ++stats_.set_osd_canvas_requests;
                if (len_ >= 2) {
                    stats_.last_canvas_cols = payload_[0];
                    stats_.last_canvas_rows = payload_[1];
                }
            }
            uint8_t response[256] = {};
            size_t response_len = 0;
            if (build_response(cmd_, telemetry, response, sizeof(response), &response_len)) {
                send_packet(endpoint, cmd_, response, response_len);
            } else {
                ++stats_.unknown_commands;
            }
        } else if (direction_ == '<') {
            ++stats_.checksum_errors;
            stats_.last_bad_cmd = cmd_;
            stats_.last_bad_len = len_;
        }
        reset();
        break;
    }
}

}
