#include "bmp390.h"

#include <math.h>
#include "config.h"

#ifndef RC_O4_OSD_HOST_TEST
#include "i2c_utils.h"
#endif

namespace bmp390 {
namespace {
#ifndef RC_O4_OSD_HOST_TEST
uint8_t g_addr = BMP390_ADDR_PRIMARY;
#endif
Calibration g_calib = {};
float g_zero_altitude_m = 0.0f;
double g_reference_pressure_pa = 101325.0;
float g_prev_filtered_m = 0.0f;
bool g_filter_seeded = false;

constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_DATA = 0x04;
constexpr uint8_t REG_CALIB = 0x31;
constexpr uint8_t REG_PWR_CTRL = 0x1B;
constexpr uint8_t REG_OSR = 0x1C;
constexpr uint8_t REG_ODR = 0x1D;
constexpr uint8_t REG_CONFIG = 0x1F;

#ifndef RC_O4_OSD_HOST_TEST
static bool probe(uint8_t addr) {
    uint8_t id = 0;
    return i2c_read_regs(I2C_PORT, addr, REG_CHIP_ID, &id, 1) && (id == 0x60 || id == 0x50);
}

static bool read_calibration(uint8_t addr) {
    uint8_t raw[21] = {};
    if (!i2c_read_regs(I2C_PORT, addr, REG_CALIB, raw, sizeof(raw))) return false;
    parse_calibration(raw, g_calib);
    return true;
}
#endif
}

void parse_calibration(const uint8_t raw[21], Calibration &cal) {
    cal.par_t1 = uint16_t(uint16_t(raw[1]) << 8 | raw[0]);
    cal.par_t2 = uint16_t(uint16_t(raw[3]) << 8 | raw[2]);
    cal.par_t3 = int8_t(raw[4]);
    cal.par_p1 = int16_t(uint16_t(raw[6]) << 8 | raw[5]);
    cal.par_p2 = int16_t(uint16_t(raw[8]) << 8 | raw[7]);
    cal.par_p3 = int8_t(raw[9]);
    cal.par_p4 = int8_t(raw[10]);
    cal.par_p5 = uint16_t(uint16_t(raw[12]) << 8 | raw[11]);
    cal.par_p6 = uint16_t(uint16_t(raw[14]) << 8 | raw[13]);
    cal.par_p7 = int8_t(raw[15]);
    cal.par_p8 = int8_t(raw[16]);
    cal.par_p9 = int16_t(uint16_t(raw[18]) << 8 | raw[17]);
    cal.par_p10 = int8_t(raw[19]);
    cal.par_p11 = int8_t(raw[20]);
}

double compensate_temperature(uint32_t raw_temp, const Calibration &cal, double &t_lin) {
    const double par_t1 = double(cal.par_t1) / 0.00390625;
    const double par_t2 = double(cal.par_t2) / 1073741824.0;
    const double par_t3 = double(cal.par_t3) / 281474976710656.0;
    const double pd1 = double(raw_temp) - par_t1;
    const double pd2 = pd1 * par_t2;
    t_lin = pd2 + (pd1 * pd1) * par_t3;
    return t_lin;
}

double compensate_pressure(uint32_t raw_press, const Calibration &cal, double t_lin) {
    const double par_p1 = double(cal.par_p1 - 16384) / 1048576.0;
    const double par_p2 = double(cal.par_p2 - 16384) / 536870912.0;
    const double par_p3 = double(cal.par_p3) / 4294967296.0;
    const double par_p4 = double(cal.par_p4) / 137438953472.0;
    const double par_p5 = double(cal.par_p5) / 0.125;
    const double par_p6 = double(cal.par_p6) / 64.0;
    const double par_p7 = double(cal.par_p7) / 256.0;
    const double par_p8 = double(cal.par_p8) / 32768.0;
    const double par_p9 = double(cal.par_p9) / 281474976710656.0;
    const double par_p10 = double(cal.par_p10) / 281474976710656.0;
    const double par_p11 = double(cal.par_p11) / 36893488147419103232.0;

    const double pd1 = par_p6 * t_lin;
    const double pd2 = par_p7 * (t_lin * t_lin);
    const double pd3 = par_p8 * (t_lin * t_lin * t_lin);
    const double po1 = par_p5 + pd1 + pd2 + pd3;
    const double pd4 = par_p2 * t_lin;
    const double pd5 = par_p3 * (t_lin * t_lin);
    const double pd6 = par_p4 * (t_lin * t_lin * t_lin);
    const double po2 = double(raw_press) * (par_p1 + pd4 + pd5 + pd6);
    const double pd7 = double(raw_press) * double(raw_press);
    const double pd8 = par_p9 + par_p10 * t_lin;
    const double pd9 = pd7 * pd8;
    const double pd10 = pd9 + pd7 * double(raw_press) * par_p11;
    return po1 + po2 + pd10;
}

float pressure_to_altitude(double pressure_pa, double reference_pa) {
    if (pressure_pa <= 0.0 || reference_pa <= 0.0) return 0.0f;
    return 44330.0f * (1.0f - powf(float(pressure_pa / reference_pa), 1.0f / 5.255f));
}

bool init(BaroState &state) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)state;
    return false;
#else
    state.initialized = false;
    state.valid = false;
    if (probe(BMP390_ADDR_PRIMARY)) {
        g_addr = BMP390_ADDR_PRIMARY;
    } else if (probe(BMP390_ADDR_ALT)) {
        g_addr = BMP390_ADDR_ALT;
    } else {
        return false;
    }

    bool ok = true;
    ok &= i2c_write_reg_u8(I2C_PORT, g_addr, REG_PWR_CTRL, 0x33);
    ok &= i2c_write_reg_u8(I2C_PORT, g_addr, REG_OSR, 0x03);
    ok &= i2c_write_reg_u8(I2C_PORT, g_addr, REG_ODR, 0x04);
    ok &= i2c_write_reg_u8(I2C_PORT, g_addr, REG_CONFIG, 0x02);
    ok &= read_calibration(g_addr);
    state.initialized = ok;
    return ok;
#endif
}

bool update(BaroState &state, uint32_t now_ms) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)state;
    (void)now_ms;
    return false;
#else
    if (!state.initialized && !init(state)) return false;

    uint8_t buf[6] = {};
    if (!i2c_read_regs(I2C_PORT, g_addr, REG_DATA, buf, sizeof(buf))) {
        state.valid = (now_ms - state.last_read_ms) < SENSOR_STALE_MS;
        return false;
    }

    const uint32_t raw_press = uint32_t(buf[0]) | (uint32_t(buf[1]) << 8) | (uint32_t(buf[2]) << 16);
    const uint32_t raw_temp = uint32_t(buf[3]) | (uint32_t(buf[4]) << 8) | (uint32_t(buf[5]) << 16);
    double t_lin = 0.0;
    state.temperature_c = float(compensate_temperature(raw_temp, g_calib, t_lin));
    state.pressure_pa = float(compensate_pressure(raw_press, g_calib, t_lin));
    const float altitude = pressure_to_altitude(state.pressure_pa, g_reference_pressure_pa);

    float dt = 1.0f / float(BMP390_SAMPLE_HZ);
    if (state.last_read_ms != 0 && now_ms > state.last_read_ms) {
        dt = float(now_ms - state.last_read_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 1.0f) dt = 1.0f / float(BMP390_SAMPLE_HZ);
    }

    if (!g_filter_seeded) {
        state.altitude_m = altitude;
        g_prev_filtered_m = altitude;
        g_zero_altitude_m = altitude;
        g_filter_seeded = true;
    } else {
        state.altitude_m = BARO_SMOOTH_ALPHA * altitude + (1.0f - BARO_SMOOTH_ALPHA) * state.altitude_m;
    }

    const float vs = (state.altitude_m - g_prev_filtered_m) / dt;
    state.vertical_speed_ms = VS_SMOOTH_ALPHA * vs + (1.0f - VS_SMOOTH_ALPHA) * state.vertical_speed_ms;
    g_prev_filtered_m = state.altitude_m;
    state.relative_altitude_m = state.altitude_m - g_zero_altitude_m;
    state.last_read_ms = now_ms;
    state.valid = state.pressure_pa > 10000.0f;
    return state.valid;
#endif
}

void set_zero_reference(BaroState &state) {
    if (state.pressure_pa > 10000.0f) {
        g_reference_pressure_pa = state.pressure_pa;
    }
    g_zero_altitude_m = state.altitude_m;
    state.relative_altitude_m = 0.0f;
    state.vertical_speed_ms = 0.0f;
    g_prev_filtered_m = state.altitude_m;
}

}
