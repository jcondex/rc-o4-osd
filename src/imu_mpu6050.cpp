#include "imu_mpu6050.h"

#include <math.h>
#include "config.h"
#ifndef RC_O4_OSD_HOST_TEST
#include "i2c_utils.h"
#endif

namespace imu_mpu6050 {
namespace {
#ifndef RC_O4_OSD_HOST_TEST
constexpr uint8_t REG_SMPRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_START = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr float MPU6050_ACCEL_LSB_PER_G = 16384.0f;
constexpr float MPU6050_GYRO_LSB_PER_DPS = 131.0f;

static int16_t be16(const uint8_t *p) {
    return int16_t((uint16_t(p[0]) << 8) | p[1]);
}
#endif
constexpr float DEG_PER_RAD = 57.29578f;

static bool valid_accel_sample(float ax, float ay, float az) {
    return !(ax == 0.0f && ay == 0.0f && az == 0.0f);
}
}

void update_filter_from_sample(ImuState &state, float ax, float ay, float az, float gx, float gy, float gz, uint32_t now_ms) {
    float dt = 0.01f;
    if (state.last_read_ms != 0 && now_ms > state.last_read_ms) {
        dt = float(now_ms - state.last_read_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;
    }

    const float accel_pitch = IMU_PITCH_SIGN * atan2f(-ax, sqrtf(ay * ay + az * az)) * DEG_PER_RAD;
    const float accel_roll = IMU_ROLL_SIGN * atan2f(ay, az) * DEG_PER_RAD;
    const float gyro_pitch = IMU_PITCH_SIGN * gy;
    const float gyro_roll = IMU_ROLL_SIGN * gx;

    state.pitch_deg = IMU_COMP_FILTER_ALPHA * (state.pitch_deg + gyro_pitch * dt)
        + (1.0f - IMU_COMP_FILTER_ALPHA) * accel_pitch;
    state.roll_deg = IMU_COMP_FILTER_ALPHA * (state.roll_deg + gyro_roll * dt)
        + (1.0f - IMU_COMP_FILTER_ALPHA) * accel_roll;

    state.accel_g[0] = ax;
    state.accel_g[1] = ay;
    state.accel_g[2] = az;
    state.gyro_dps[0] = gx;
    state.gyro_dps[1] = gy;
    state.gyro_dps[2] = gz;
    state.last_read_ms = now_ms;
    state.valid = valid_accel_sample(ax, ay, az);
}

bool init(ImuState &state) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)state;
    return false;
#else
    state.initialized = false;
    state.valid = false;

    bool ok = true;
    ok &= i2c_write_reg_u8(SENSOR_I2C_PORT, MPU6050_ADDR, REG_PWR_MGMT_1, 0x00);
    sleep_ms(10);
    ok &= i2c_write_reg_u8(SENSOR_I2C_PORT, MPU6050_ADDR, REG_GYRO_CONFIG, 0x00);
    ok &= i2c_write_reg_u8(SENSOR_I2C_PORT, MPU6050_ADDR, REG_ACCEL_CONFIG, 0x00);
    ok &= i2c_write_reg_u8(SENSOR_I2C_PORT, MPU6050_ADDR, REG_CONFIG, 0x03);
    ok &= i2c_write_reg_u8(SENSOR_I2C_PORT, MPU6050_ADDR, REG_SMPRT_DIV, 0x09);
    state.initialized = ok;
    return ok;
#endif
}

bool update(ImuState &state, uint32_t now_ms) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)state;
    (void)now_ms;
    return false;
#else
    if (!state.initialized && !init(state)) return false;

    uint8_t buf[14] = {};
    if (!i2c_read_regs(SENSOR_I2C_PORT, MPU6050_ADDR, REG_ACCEL_START, buf, sizeof(buf))) {
        state.valid = (now_ms - state.last_read_ms) < SENSOR_STALE_MS;
        return false;
    }

    const float ax = IMU_ACCEL_X_SIGN * float(be16(&buf[0])) / MPU6050_ACCEL_LSB_PER_G;
    const float ay = IMU_ACCEL_Y_SIGN * float(be16(&buf[2])) / MPU6050_ACCEL_LSB_PER_G;
    const float az = IMU_ACCEL_Z_SIGN * float(be16(&buf[4])) / MPU6050_ACCEL_LSB_PER_G;
    const float gx = IMU_GYRO_X_SIGN * float(be16(&buf[8])) / MPU6050_GYRO_LSB_PER_DPS;
    const float gy = IMU_GYRO_Y_SIGN * float(be16(&buf[10])) / MPU6050_GYRO_LSB_PER_DPS;
    const float gz = IMU_GYRO_Z_SIGN * float(be16(&buf[12])) / MPU6050_GYRO_LSB_PER_DPS;

    update_filter_from_sample(state, ax, ay, az, gx, gy, gz, now_ms);
    return state.valid;
#endif
}

}
