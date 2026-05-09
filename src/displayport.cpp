#include "displayport.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "nav.h"

namespace displayport {
namespace {

constexpr uint8_t DP_HEARTBEAT = 0;
constexpr uint8_t DP_CLEAR = 2;
constexpr uint8_t DP_WRITE = 3;
constexpr uint8_t DP_DRAW = 4;
constexpr uint8_t DP_ATTR_NORMAL = 0x00;
constexpr uint8_t DP_ATTR_BLINK = 0x40;

static void send_subcmd(msp::Endpoint endpoint, uint8_t subcmd) {
    msp::send_displayport(endpoint, &subcmd, 1);
}

static void write_row(msp::Endpoint endpoint, uint8_t row, const char *text, uint8_t attr = DP_ATTR_NORMAL) {
    uint8_t payload[4 + OSD_COLS] = {};
    payload[0] = DP_WRITE;
    payload[1] = row;
    payload[2] = 0;
    payload[3] = attr;
    size_t len = strnlen(text, OSD_COLS);
    memcpy(&payload[4], text, len);
    msp::send_displayport(endpoint, payload, 4 + len);
}

static const char *armed_string(const TelemetryState &state) {
    return state.armed ? "ARMED" : "DISARMED";
}

static const char *heading_source(const TelemetryState &state) {
    if (state.gps.fix_valid && state.gps.speed_kmh >= HDG_GPS_MIN_KMH) return "GPS";
    if (state.compass.valid) return "MAG";
    return "---";
}

static int heading_deg(const TelemetryState &state) {
    if (state.gps.fix_valid && state.gps.speed_kmh >= HDG_GPS_MIN_KMH) return int(state.gps.course_deg);
    if (state.compass.valid) return int(state.compass.heading_deg);
    return 0;
}

static const char *warning(const TelemetryState &state) {
    if (state.app_state == AppState::Failsafe || state.rc.failsafe) return "FAILSAFE  PWM LOST";
    if (!state.rc.signal_valid) return "PWM LOST";
    if (state.armed && !state.gps.fix_valid) return "GPS LOST";
    if (state.armed && !state.nav.home_set) return "NO GPS HOME";
    if (!state.baro.valid) return "ALT ERR";
    if (!state.compass.valid) return "COMPASS ERR";
    if (state.cal_needed) return "CAL NEEDED";
    if (state.battery.low) return "LOW BATTERY";
    return "";
}

}

void send_message(msp::Endpoint endpoint, const char *message) {
    char row[31] = {};
    snprintf(row, sizeof(row), "%-30s", message);
    send_subcmd(endpoint, DP_HEARTBEAT);
    send_subcmd(endpoint, DP_CLEAR);
    write_row(endpoint, 7, row, DP_ATTR_BLINK);
    send_subcmd(endpoint, DP_DRAW);
}

void send_frame(msp::Endpoint endpoint, const TelemetryState &state) {
    char row[31] = {};
    const int spd_kmh = int(lroundf(state.gps.speed_kmh));
    const int max_kmh = int(lroundf(state.nav.max_speed_kmh));
    const int alt_m = int(lroundf(state.baro.relative_altitude_m));
    const int temp_c = int(lroundf(state.baro.temperature_c));
    const int home_m = state.nav.home_set ? int(lroundf(state.nav.home_distance_m)) : 0;
    const int brg = state.nav.home_set ? int(lroundf(state.nav.home_bearing_deg)) : 0;
    const uint32_t runtime = nav::runtime_seconds(state.nav, state.armed, state.now_ms);
    const int time_min = int(runtime / 60u);
    const int time_sec = int(runtime % 60u);
    const float trip_km = state.nav.trip_distance_m / 1000.0f;

    send_subcmd(endpoint, DP_HEARTBEAT);
    static bool first = true;
    if (first) {
        send_subcmd(endpoint, DP_CLEAR);
        first = false;
    }

    snprintf(row, sizeof(row), "ARM:%-9s GPS:%2d SATS  ", armed_string(state), state.gps.satellites);
    write_row(endpoint, 0, row);
    snprintf(row, sizeof(row), "VBAT:%5.1fV     PWM: %4dus ", state.battery.volts, state.rc.pulse_us);
    write_row(endpoint, 1, row);
    snprintf(row, sizeof(row), "ALT:%6dm      VS:%+6.1fm/s", alt_m, state.baro.vertical_speed_ms);
    write_row(endpoint, 2, row);
    snprintf(row, sizeof(row), "SPD:%4dkm/h    MAX:%4dkm/h ", spd_kmh, max_kmh);
    write_row(endpoint, 3, row);
    if (state.nav.home_set) {
        snprintf(row, sizeof(row), "HOME:%6dm     BRG:%6d  ", home_m, brg);
    } else {
        snprintf(row, sizeof(row), "HOME:---       BRG:---     ");
    }
    write_row(endpoint, 4, row);
    snprintf(row, sizeof(row), "HDG: %3d %-3s    TMP:%5dC   ", heading_deg(state), heading_source(state), temp_c);
    write_row(endpoint, 5, row);
    snprintf(row, sizeof(row), "TIME:%3d:%02d      TRIP:%5.2fkm", time_min, time_sec, trip_km);
    write_row(endpoint, 6, row);
    snprintf(row, sizeof(row), "%-30s", warning(state));
    write_row(endpoint, 7, row, warning(state)[0] ? DP_ATTR_BLINK : DP_ATTR_NORMAL);
    snprintf(row, sizeof(row), "PITCH:%+6.1f    ROLL:%+6.1f", state.imu.pitch_deg, state.imu.roll_deg);
    write_row(endpoint, 8, row);
    send_subcmd(endpoint, DP_DRAW);
}

}
