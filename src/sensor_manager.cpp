#include "sensor_manager.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "battery.h"
#include "bmp390.h"
#include "compass_qmc5883l.h"
#include "config.h"
#include "imu_mpu6050.h"

namespace sensor_manager {
namespace {

uint32_t g_last_imu_ms = 0;
uint32_t g_last_baro_ms = 0;
uint32_t g_last_compass_ms = 0;
uint32_t g_last_battery_ms = 0;
uint32_t g_last_probe_ms = 0;

}

void init_i2c() {
    i2c_init(SENSOR_I2C_PORT, SENSOR_I2C_SPEED_HZ);
    gpio_set_function(PIN_SENSOR_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SENSOR_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_SENSOR_SDA);
    gpio_pull_up(PIN_SENSOR_SCL);

    i2c_init(COMPASS_I2C_PORT, COMPASS_I2C_SPEED_HZ);
    gpio_set_function(PIN_COMPASS_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_COMPASS_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_COMPASS_SDA);
    gpio_pull_up(PIN_COMPASS_SCL);
}

void init_all(TelemetryState &state) {
    battery::init();
    state.mpu6050_ok = imu_mpu6050::init(state.imu);
    state.bmp390_ok = bmp390::init(state.baro);
    state.compass_ok = compass_qmc5883l::init(state.compass);
}

void update_all(TelemetryState &state, CompassCalibration &compass_cal, uint32_t now_ms) {
    if (now_ms - g_last_imu_ms >= 10u) {
        g_last_imu_ms = now_ms;
        if (state.imu.initialized) {
            state.mpu6050_ok = imu_mpu6050::update(state.imu, now_ms);
            if (!state.mpu6050_ok && (now_ms - state.imu.last_read_ms) >= SENSOR_STALE_MS) {
                ++state.sensor_failures.imu;
                state.imu.initialized = false;
            }
        }
    }

    if (now_ms - g_last_baro_ms >= 40u) {
        g_last_baro_ms = now_ms;
        if (state.baro.initialized) {
            state.bmp390_ok = bmp390::update(state.baro, now_ms);
            if (!state.bmp390_ok && (now_ms - state.baro.last_read_ms) >= SENSOR_STALE_MS) {
                ++state.sensor_failures.baro;
                state.baro.initialized = false;
            }
        }
    }

    if (now_ms - g_last_compass_ms >= 40u) {
        g_last_compass_ms = now_ms;
        if (state.compass.initialized) {
            state.compass_ok = compass_qmc5883l::update(state.compass, compass_cal, state.imu, now_ms);
            if (!state.compass_ok && (now_ms - state.compass.last_read_ms) >= SENSOR_STALE_MS) {
                ++state.sensor_failures.compass;
                state.compass.initialized = false;
            }
        }
    }

    if (now_ms - g_last_battery_ms >= 125u) {
        g_last_battery_ms = now_ms;
        battery::update(state.battery);
    }
}

void reprobe_optional(TelemetryState &state, uint32_t now_ms) {
    if (now_ms - g_last_probe_ms < SENSOR_REPROBE_MS) return;
    g_last_probe_ms = now_ms;

    if (!state.imu.initialized) state.mpu6050_ok = imu_mpu6050::init(state.imu);
    if (!state.baro.initialized) state.bmp390_ok = bmp390::init(state.baro);
    if (!state.compass.initialized) state.compass_ok = compass_qmc5883l::init(state.compass);
}

}
