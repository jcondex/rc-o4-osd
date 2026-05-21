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
constexpr uint8_t DP_OPTIONS = 5;
constexpr uint8_t DP_ATTR_NORMAL = 0x00;
constexpr uint8_t DP_ATTR_BLINK = 0x40;

constexpr int FULL_CIRCLE_DEG = 360;
constexpr uint8_t OSD_LEFT_COL = 2;
constexpr uint8_t OSD_RIGHT_COL = 18;
constexpr uint8_t OSD_SAFE_ROWS = OSD_ROWS;
constexpr uint8_t OSD_GPS_ROW = 0;
constexpr uint8_t OSD_WARNING_ROW = 4;
constexpr uint8_t OSD_VERTICAL_SPEED_ROW = OSD_SAFE_ROWS - 3;
constexpr uint8_t OSD_ALTITUDE_ROW = OSD_SAFE_ROWS - 2;
constexpr uint8_t OSD_ATTITUDE_ROW = OSD_SAFE_ROWS - 1;

static float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float normalize_degrees(float deg) {
    while (deg < 0.0f) deg += float(FULL_CIRCLE_DEG);
    while (deg >= float(FULL_CIRCLE_DEG)) deg -= float(FULL_CIRCLE_DEG);
    return deg;
}

static int normalized_heading_int(float deg) {
    return int(lroundf(normalize_degrees(deg))) % FULL_CIRCLE_DEG;
}

static uint8_t heading_to_direction(float deg) {
    const int heading = normalized_heading_int(deg);
    return uint8_t(((heading * 8) + FULL_CIRCLE_DEG / 2) / FULL_CIRCLE_DEG) % 8u;
}

static const char *cardinal_from_heading(float deg) {
    static const char *dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    return dirs[heading_to_direction(deg)];
}

static const char *armed_string(const TelemetryState &state) {
    return state.armed ? "ARMED" : "DISARMED";
}

static float heading_value_deg(const TelemetryState &state) {
    if (state.gps.fix_valid && state.gps.speed_kmh >= HDG_GPS_MIN_KMH) return state.gps.course_deg;
    if (state.compass.valid) return state.compass.heading_deg;
    return 0.0f;
}

static const char *warning_text(const TelemetryState &state) {
    if (state.rc.failsafe || !state.rc.signal_valid) return "RC FAILSAFE";
    if (!state.rc.rearm_latched && (state.rc.arm_switch_middle || state.rc.arm_switch_high) && !state.armed) return "SWITCH LOW";
    if (state.cal_needed && state.compass.initialized) return "CAL NEEDED";
    return "";
}

static bool gps_recently_receiving(const TelemetryState &state) {
    if (!state.gps.receiving) return false;
    if (state.now_ms < state.gps.last_message_ms) return true;
    return (state.now_ms - state.gps.last_message_ms) <= GPS_LINK_TIMEOUT_MS;
}

static bool cockpit_active(const TelemetryState &state) {
#if OSD_COCKPIT_MODE
    return state.armed && state.rc.cockpit_mode_requested && state.imu.valid;
#else
    (void)state;
    return false;
#endif
}

class DisplayPortWriter {
public:
    explicit DisplayPortWriter(msp::Endpoint endpoint) : endpoint_(endpoint) {}

    void begin_frame() const {
        send_options();
        send_subcmd(DP_HEARTBEAT);
        send_subcmd(DP_CLEAR);
    }

    void end_frame() const {
        send_subcmd(DP_DRAW);
    }

    void write_left(uint8_t row, const char *text, uint8_t attr = DP_ATTR_NORMAL) const {
        write_at(OSD_LEFT_COL, row, text, attr);
    }

    void write_right(uint8_t row, const char *text, uint8_t attr = DP_ATTR_NORMAL) const {
        write_at(OSD_RIGHT_COL, row, text, attr);
    }

    void write_centered(uint8_t row, const char *text, uint8_t attr = DP_ATTR_NORMAL) const {
        const size_t len = strnlen(text, OSD_COLS);
        const uint8_t col = len >= OSD_COLS ? 0 : uint8_t((OSD_COLS - len) / 2u);
        write_at(col, row, text, attr);
    }

    void write_char(uint8_t col, uint8_t row, char ch, uint8_t attr = DP_ATTR_NORMAL) const {
        if (col >= OSD_COLS || row >= OSD_ROWS) return;
        const uint8_t payload[5] = {DP_WRITE, row, col, attr, uint8_t(ch)};
        msp::send_displayport(endpoint_, payload, sizeof(payload));
    }

    void write_canvas_char(uint8_t col, uint8_t row, char ch, uint8_t attr = DP_ATTR_NORMAL) const {
        if (col >= OSD_CANVAS_COLS || row >= OSD_CANVAS_ROWS) return;
        const uint8_t payload[5] = {DP_WRITE, row, col, attr, uint8_t(ch)};
        msp::send_displayport(endpoint_, payload, sizeof(payload));
    }

private:
    void send_subcmd(uint8_t subcmd) const {
        msp::send_displayport(endpoint_, &subcmd, 1);
    }

    void send_options() const {
        const uint8_t payload[] = {DP_OPTIONS, OSD_DISPLAYPORT_FONT, OSD_DISPLAYPORT_RESOLUTION};
        msp::send_displayport(endpoint_, payload, sizeof(payload));
    }

    void write_at(uint8_t col, uint8_t row, const char *text, uint8_t attr) const {
        if (col >= OSD_COLS || row >= OSD_ROWS) return;

        uint8_t payload[4 + OSD_COLS] = {};
        payload[0] = DP_WRITE;
        payload[1] = row;
        payload[2] = col;
        payload[3] = attr;

        const size_t max_len = size_t(OSD_COLS - col);
        const size_t len = strnlen(text, max_len);
        memcpy(&payload[4], text, len);
        msp::send_displayport(endpoint_, payload, 4 + len);
    }

    msp::Endpoint endpoint_;
};

static void render_symbol_test(const DisplayPortWriter &osd) {
    char row[31] = {};
    osd.begin_frame();

    snprintf(row, sizeof(row), "BETAFONT SYMBOL TEST");
    osd.write_left(0, row);

    memset(row, ' ', OSD_COLS);
    row[OSD_COLS] = '\0';
    memcpy(row, "CARDINAL:", 9);
    memcpy(row + 10, "N NE E SE S SW W NW", 19);
    osd.write_left(1, row);

    snprintf(row, sizeof(row), "DISABLE OSD_SYMBOL_TEST_MODE");
    osd.write_left(OSD_WARNING_ROW, row, DP_ATTR_BLINK);
    osd.end_frame();
}

static void render_arm_and_battery(const DisplayPortWriter &osd, const TelemetryState &state) {
    char row[31] = {};

#if OSD_SHOW_ARM_GPS
    snprintf(row, sizeof(row), "%s", armed_string(state));
    osd.write_right(2, row);
#endif

#if OSD_SHOW_BATTERY_PWM
    if (state.battery.volts >= OSD_BATTERY_MIN_VALID_VOLTS) {
        snprintf(row, sizeof(row), "VBAT:%4.1fV", state.battery.volts);
        osd.write_left(2, row);
    }
#endif
}

static void render_warning(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_WARNINGS
    const char *warn = warning_text(state);
    if (warn[0]) {
        char row[31] = {};
        snprintf(row, sizeof(row), "%-30s", warn);
        osd.write_left(OSD_WARNING_ROW, row, DP_ATTR_BLINK);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_gps(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_SPEED
    char row[31] = {};
    if (gps_recently_receiving(state)) {
        snprintf(row, sizeof(row), "GPS:%02u", unsigned(state.gps.satellites));
        osd.write_right(OSD_GPS_ROW, row);
    }

    if (state.gps.fix_valid) {
        snprintf(row, sizeof(row), "SPD:%3d", int(lroundf(state.gps.speed_kmh)));
        osd.write_right(3, row);

        snprintf(row, sizeof(row), "DST:%4.1f", state.nav.trip_distance_m / 1000.0f);
        osd.write_right(4, row);

        snprintf(row, sizeof(row), "MAX:%3dKMH", int(lroundf(state.nav.max_speed_kmh)));
        osd.write_right(5, row);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_home(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_HOME_ARROW
    if (state.gps.fix_valid && state.nav.home_set) {
        char row[31] = {};
        snprintf(row, sizeof(row), "HOM:%3dM %03d",
            int(lroundf(state.nav.home_distance_m)),
            normalized_heading_int(state.nav.home_bearing_deg));
        osd.write_right(6, row);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_heading(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_HEADING_TEMP
    if (state.compass.valid || (state.gps.fix_valid && state.gps.speed_kmh >= HDG_GPS_MIN_KMH)) {
        char row[31] = {};
        const float hdg = heading_value_deg(state);
        snprintf(row, sizeof(row), "HDG:%s %03d", cardinal_from_heading(hdg), normalized_heading_int(hdg));
        osd.write_right(1, row);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_baro(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_ALTITUDE
    if (state.baro.valid) {
        char row[31] = {};
        snprintf(row, sizeof(row), "ALT:%+4dM P:%4d",
            int(lroundf(state.baro.relative_altitude_m)),
            int(lroundf(state.baro.pressure_pa / 100.0f)));
        osd.write_left(OSD_ALTITUDE_ROW, row);

        snprintf(row, sizeof(row), "VS:%+4.1fMS", state.baro.vertical_speed_ms);
        osd.write_left(OSD_VERTICAL_SPEED_ROW, row);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_runtime(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_TIME_TRIP
    const uint32_t runtime = nav::runtime_seconds(state.nav, state.armed, state.now_ms);
    if (state.armed || runtime > 0) {
        char row[31] = {};
        snprintf(row, sizeof(row), "TIME:%3lu:%02lu",
            static_cast<unsigned long>(runtime / 60u),
            static_cast<unsigned long>(runtime % 60u));
        osd.write_right(7, row);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_attitude(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_ATTITUDE
    if (state.imu.valid) {
        char row[31] = {};
        snprintf(row, sizeof(row), "PIT:%+5.1f ROL:%+5.1f", state.imu.pitch_deg, state.imu.roll_deg);
        osd.write_left(OSD_ATTITUDE_ROW, row);
    }
#else
    (void)osd;
    (void)state;
#endif
}

static void render_cockpit_horizon(const DisplayPortWriter &osd, const TelemetryState &state) {
    const float roll_deg = clampf(state.imu.roll_deg, -OSD_COCKPIT_MAX_ROLL_DEG, OSD_COCKPIT_MAX_ROLL_DEG);
    const float pitch_deg = clampf(state.imu.pitch_deg, -OSD_COCKPIT_MAX_PITCH_DEG, OSD_COCKPIT_MAX_PITCH_DEG);
    const int pitch_rows = int(lroundf(pitch_deg / OSD_COCKPIT_PITCH_DEG_PER_ROW));
    const int max_left = int(OSD_COCKPIT_CENTER_COL);
    const int max_right = int(OSD_CANVAS_COLS) - 1 - int(OSD_COCKPIT_CENTER_COL);
    const int half_width = int(clampf(float(OSD_COCKPIT_HALF_WIDTH), 1.0f, float(max_left < max_right ? max_left : max_right)));
    const float rows_per_col = roll_deg / (float(half_width) * OSD_COCKPIT_ROLL_DEG_PER_ROW_AT_EDGE);

    for (int dx = -half_width; dx <= half_width; ++dx) {
        if (dx >= -OSD_COCKPIT_CENTER_GAP_HALF_WIDTH && dx <= OSD_COCKPIT_CENTER_GAP_HALF_WIDTH) {
            continue;
        }
        const int roll_rows = int(lroundf(-rows_per_col * float(dx)));
        const int row = int(OSD_COCKPIT_CENTER_ROW) - pitch_rows + roll_rows;
        const int col = int(OSD_COCKPIT_CENTER_COL) + dx;
        if (row >= 1 && row < int(OSD_CANVAS_ROWS) - 2 && col >= 0 && col < OSD_CANVAS_COLS) {
            osd.write_canvas_char(uint8_t(col), uint8_t(row), '-');
        }
    }
}

static void render_imu_debug(const DisplayPortWriter &osd, const TelemetryState &state) {
#if OSD_SHOW_IMU_DEBUG
    char row[31] = {};
    snprintf(row, sizeof(row), "GYR:%+4d %+4d %+4d d/s",
        int(lroundf(state.imu.gyro_dps[0])),
        int(lroundf(state.imu.gyro_dps[1])),
        int(lroundf(state.imu.gyro_dps[2])));
    osd.write_left(10, row);

    snprintf(row, sizeof(row), "ACC:%+4.1f %+4.1f %+4.1fg",
        state.imu.accel_g[0],
        state.imu.accel_g[1],
        state.imu.accel_g[2]);
    osd.write_left(11, row);
#else
    (void)osd;
    (void)state;
#endif
}

}

void send_message(msp::Endpoint endpoint, const char *message) {
    DisplayPortWriter osd(endpoint);
    osd.begin_frame();
    osd.write_centered(7, message, DP_ATTR_BLINK);
    osd.end_frame();
}

void send_frame(msp::Endpoint endpoint, const TelemetryState &state) {
    DisplayPortWriter osd(endpoint);

#if OSD_SYMBOL_TEST_MODE
    (void)state;
    render_symbol_test(osd);
    return;
#else
    osd.begin_frame();
    render_arm_and_battery(osd, state);
    render_warning(osd, state);
    render_gps(osd, state);
    render_home(osd, state);
    render_heading(osd, state);
    render_baro(osd, state);
    render_runtime(osd, state);
    render_attitude(osd, state);
    if (cockpit_active(state)) {
        render_cockpit_horizon(osd, state);
    }
    render_imu_debug(osd, state);
    osd.end_frame();
#endif
}

}
