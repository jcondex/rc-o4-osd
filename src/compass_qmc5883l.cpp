#include "compass_qmc5883l.h"

#include <math.h>
#include "config.h"
#ifndef RC_O4_OSD_HOST_TEST
#include "i2c_utils.h"
#endif

namespace compass_qmc5883l {
namespace {
constexpr uint8_t REG_DATA = 0x00;
constexpr uint8_t REG_CONTROL = 0x09;
constexpr uint8_t REG_SET_RESET = 0x0B;
constexpr uint8_t QMC5883L_SET_RESET_PERIOD = 0x01;
constexpr uint8_t QMC5883L_OSR512_8G_200HZ_CONTINUOUS = 0x1D;
constexpr float RAD_PER_DEG = 0.0174532925f;
constexpr float DEG_PER_RAD = 57.29578f;

struct CalState {
    int16_t x_min = 32767;
    int16_t x_max = -32768;
    int16_t y_min = 32767;
    int16_t y_max = -32768;
    int16_t z_min = 32767;
    int16_t z_max = -32768;
    bool collecting = false;
    uint32_t sample_count = 0;
};

CalState g_cal;

static int16_t le16(const uint8_t *p) {
    return int16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
}

static float wrap_degrees(float degrees) {
    while (degrees >= 360.0f) degrees -= 360.0f;
    while (degrees < 0.0f) degrees += 360.0f;
    return degrees;
}

static bool valid_raw_sample(int16_t raw_x, int16_t raw_y, int16_t raw_z) {
    return !(raw_x == 0 && raw_y == 0 && raw_z == 0);
}
}

float heading_from_sample(int16_t raw_x, int16_t raw_y, int16_t raw_z, const CompassCalibration &cal, const ImuState &imu) {
    const float x = (float(raw_x) - cal.x_offset) * cal.x_scale;
    const float y = (float(raw_y) - cal.y_offset) * cal.y_scale;
    const float z = (float(raw_z) - cal.z_offset) * cal.z_scale;

    float heading = 0.0f;
    if (imu.valid) {
        const float pitch = imu.pitch_deg * RAD_PER_DEG;
        const float roll = imu.roll_deg * RAD_PER_DEG;
        const float xh = x * cosf(pitch) + y * sinf(roll) * sinf(pitch) - z * cosf(roll) * sinf(pitch);
        const float yh = y * cosf(roll) + z * sinf(roll);
        heading = atan2f(-yh, xh) * DEG_PER_RAD;
    } else {
        heading = atan2f(-y, x) * DEG_PER_RAD;
    }

    return wrap_degrees(heading + cal.declination_deg);
}

bool init(CompassState &state) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)state;
    return false;
#else
    state.initialized = false;
    state.valid = false;
    bool ok = true;
    ok &= i2c_write_reg_u8(COMPASS_I2C_PORT, QMC5883L_ADDR, REG_SET_RESET, QMC5883L_SET_RESET_PERIOD);
    ok &= i2c_write_reg_u8(COMPASS_I2C_PORT, QMC5883L_ADDR, REG_CONTROL, QMC5883L_OSR512_8G_200HZ_CONTINUOUS);
    state.initialized = ok;
    return ok;
#endif
}

bool update(CompassState &state, const CompassCalibration &cal, const ImuState &imu, uint32_t now_ms) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)state;
    (void)cal;
    (void)imu;
    (void)now_ms;
    return false;
#else
    if (!state.initialized && !init(state)) return false;

    uint8_t buf[6] = {};
    if (!i2c_read_regs(COMPASS_I2C_PORT, QMC5883L_ADDR, REG_DATA, buf, sizeof(buf))) {
        state.valid = (now_ms - state.last_read_ms) < SENSOR_STALE_MS;
        return false;
    }

    state.raw_x = le16(&buf[0]);
    state.raw_y = le16(&buf[2]);
    state.raw_z = le16(&buf[4]);
    state.heading_deg = heading_from_sample(state.raw_x, state.raw_y, state.raw_z, cal, imu);
    state.last_read_ms = now_ms;
    state.valid = valid_raw_sample(state.raw_x, state.raw_y, state.raw_z);
    return state.valid;
#endif
}

void calibration_start() {
    g_cal = CalState{};
    g_cal.collecting = true;
}

void calibration_update(int16_t raw_x, int16_t raw_y, int16_t raw_z) {
    if (!g_cal.collecting) return;
    if (raw_x < g_cal.x_min) g_cal.x_min = raw_x;
    if (raw_x > g_cal.x_max) g_cal.x_max = raw_x;
    if (raw_y < g_cal.y_min) g_cal.y_min = raw_y;
    if (raw_y > g_cal.y_max) g_cal.y_max = raw_y;
    if (raw_z < g_cal.z_min) g_cal.z_min = raw_z;
    if (raw_z > g_cal.z_max) g_cal.z_max = raw_z;
    ++g_cal.sample_count;
}

bool calibration_finish(CompassCalibration &out) {
    g_cal.collecting = false;
    const int x_span = int(g_cal.x_max) - int(g_cal.x_min);
    const int y_span = int(g_cal.y_max) - int(g_cal.y_min);
    const int z_span = int(g_cal.z_max) - int(g_cal.z_min);
    if (g_cal.sample_count < 20 || x_span < 200 || y_span < 200 || z_span < 200) {
        out.valid = false;
        return false;
    }

    const float x_range = float(x_span) / 2.0f;
    const float y_range = float(y_span) / 2.0f;
    const float z_range = float(z_span) / 2.0f;
    const float avg_range = (x_range + y_range + z_range) / 3.0f;

    out.x_offset = float(int(g_cal.x_max) + int(g_cal.x_min)) / 2.0f;
    out.y_offset = float(int(g_cal.y_max) + int(g_cal.y_min)) / 2.0f;
    out.z_offset = float(int(g_cal.z_max) + int(g_cal.z_min)) / 2.0f;
    out.x_scale = avg_range / x_range;
    out.y_scale = avg_range / y_range;
    out.z_scale = avg_range / z_range;
    out.declination_deg = MAG_DECLINATION;
    out.valid = true;
    return true;
}

uint32_t calibration_sample_count() {
    return g_cal.sample_count;
}

}
