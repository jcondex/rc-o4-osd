#include <stdio.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "battery.h"
#include "bmp390.h"
#include "calibration_flash.h"
#include "compass_qmc5883l.h"
#include "config.h"
#include "displayport.h"
#include "gps.h"
#include "imu_mpu6050.h"
#include "msp.h"
#include "nav.h"
#include "rc_pwm_pio.h"
#include "state_machine.h"

namespace {

TelemetryState g_state;
CompassCalibration g_compass_cal;
msp::Parser g_msp_parser;
gps::NmeaParser g_gps_parser;

uint32_t g_last_imu_ms = 0;
uint32_t g_last_baro_ms = 0;
uint32_t g_last_compass_ms = 0;
uint32_t g_last_battery_ms = 0;
uint32_t g_last_osd_ms = 0;
uint32_t g_last_gps_push_ms = 0;
uint32_t g_last_probe_ms = 0;
uint32_t g_last_debug_ms = 0;

void uart0_writer(const uint8_t *data, size_t len, void *) {
    uart_write_blocking(uart0, data, len);
}

msp::Endpoint o4_endpoint() {
    return msp::Endpoint{uart0_writer, nullptr};
}

uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

void init_uart0_o4() {
    uart_init(uart0, UART_O4_BAUD);
    gpio_set_function(PIN_UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART0_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(uart0, false, false);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
}

void init_i2c() {
    i2c_init(I2C_PORT, I2C_SPEED_HZ);
    gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_I2C1_SDA);
    gpio_pull_up(PIN_I2C1_SCL);
}

void init_gpio() {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);
    gpio_init(PIN_CAL_TRIGGER);
    gpio_set_dir(PIN_CAL_TRIGGER, GPIO_IN);
    gpio_pull_up(PIN_CAL_TRIGGER);
}

void poll_msp() {
    while (uart_is_readable(uart0)) {
        g_msp_parser.parse_byte(uart_getc(uart0), g_state, o4_endpoint());
    }
}

void reprobe_optional_sensors(uint32_t now) {
    if (now - g_last_probe_ms < SENSOR_REPROBE_MS) return;
    g_last_probe_ms = now;

    if (!g_state.imu.initialized) g_state.mpu6050_ok = imu_mpu6050::init(g_state.imu);
    if (!g_state.baro.initialized) g_state.bmp390_ok = bmp390::init(g_state.baro);
    if (!g_state.compass.initialized) g_state.compass_ok = compass_qmc5883l::init(g_state.compass);
}

void update_sensors(uint32_t now) {
    if (now - g_last_imu_ms >= 10u) {
        g_last_imu_ms = now;
        g_state.mpu6050_ok = imu_mpu6050::update(g_state.imu, now);
        if (!g_state.mpu6050_ok && (now - g_state.imu.last_read_ms) >= SENSOR_STALE_MS) {
            g_state.imu.initialized = false;
        }
    }

    if (now - g_last_baro_ms >= 40u) {
        g_last_baro_ms = now;
        g_state.bmp390_ok = bmp390::update(g_state.baro, now);
        if (!g_state.bmp390_ok && (now - g_state.baro.last_read_ms) >= SENSOR_STALE_MS) {
            g_state.baro.initialized = false;
        }
    }

    if (now - g_last_compass_ms >= 40u) {
        g_last_compass_ms = now;
        g_state.compass_ok = compass_qmc5883l::update(g_state.compass, g_compass_cal, g_state.imu, now);
        if (!g_state.compass_ok && (now - g_state.compass.last_read_ms) >= SENSOR_STALE_MS) {
            g_state.compass.initialized = false;
        }
    }

    if (now - g_last_battery_ms >= 125u) {
        g_last_battery_ms = now;
        battery::update(g_state.battery);
    }
}

void print_debug(uint32_t now) {
#if DEBUG_USB_SERIAL
    const uint32_t interval_ms = 1000u / DEBUG_PRINT_HZ;
    if (now - g_last_debug_ms < interval_ms) return;
    g_last_debug_ms = now;
    printf("state=%u armed=%u pwm=%u rc=%u fs=%u gps=%u sats=%u vbat=%.2f baro=%u imu=%u mag=%u alt=%.1f pitch=%.1f roll=%.1f hdg=%.1f\n",
        unsigned(g_state.app_state),
        g_state.armed ? 1u : 0u,
        unsigned(g_state.rc.pulse_us),
        g_state.rc.signal_valid ? 1u : 0u,
        g_state.rc.failsafe ? 1u : 0u,
        g_state.gps.fix_valid ? 1u : 0u,
        unsigned(g_state.gps.satellites),
        double(g_state.battery.volts),
        g_state.baro.valid ? 1u : 0u,
        g_state.imu.valid ? 1u : 0u,
        g_state.compass.valid ? 1u : 0u,
        double(g_state.baro.relative_altitude_m),
        double(g_state.imu.pitch_deg),
        double(g_state.imu.roll_deg),
        double(g_state.compass.heading_deg));
#else
    (void)now;
#endif
}

void handle_state_machine(uint32_t now) {
    const state_machine::TransitionResult transition = state_machine::update(g_state);
    if (transition.armed_rising) {
        bmp390::set_zero_reference(g_state.baro);
        nav::on_arm(g_state.nav, g_state.gps, now);
    } else if (transition.armed_falling) {
        nav::on_disarm(g_state.nav);
    }

    gpio_put(PIN_LED, g_state.armed ? 1 : 0);
}

void early_init() {
    stdio_init_all();
    init_gpio();
    init_uart0_o4();
    gps::init_uart();
    init_i2c();
    battery::init();

    g_state.app_state = AppState::EarlyInit;
    g_state.mpu6050_ok = imu_mpu6050::init(g_state.imu);
    g_state.bmp390_ok = bmp390::init(g_state.baro);
    g_state.compass_ok = compass_qmc5883l::init(g_state.compass);
    g_state.cal_needed = !calibration_flash::load(g_compass_cal);

    if (!g_compass_cal.valid) {
        g_compass_cal = calibration_flash::defaults();
    }
}

void initializing() {
    g_state.app_state = AppState::Initializing;
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    rc_pwm::init();
    gps::send_ubx_startup_config();
    msp::send_startup_burst(o4_endpoint(), g_state);
    displayport::send_message(o4_endpoint(), "INIT...");
}

}

int main() {
    early_init();

    if (gpio_get(PIN_CAL_TRIGGER) == 0) {
        g_state.app_state = AppState::Calibrating;
        compass_qmc5883l::calibration_start();
        const uint32_t cal_start_ms = millis();
        uint32_t last_cal_osd_ms = 0;
        while ((millis() - cal_start_ms) < 15000u) {
            const uint32_t now = millis();
            g_state.now_ms = now;
            poll_msp();
            if (now - g_last_compass_ms >= 40u) {
                g_last_compass_ms = now;
                if (compass_qmc5883l::update(g_state.compass, g_compass_cal, g_state.imu, now)) {
                    compass_qmc5883l::calibration_update(
                        g_state.compass.raw_x,
                        g_state.compass.raw_y,
                        g_state.compass.raw_z);
                }
            }
            if (now - last_cal_osd_ms >= 250u) {
                last_cal_osd_ms = now;
                char msg[31] = {};
                snprintf(msg, sizeof(msg), "CAL:%5lu SAMPLES", compass_qmc5883l::calibration_sample_count());
                displayport::send_message(o4_endpoint(), msg);
            }
            if (gpio_get(PIN_CAL_TRIGGER) != 0 && (now - cal_start_ms) > 2000u) {
                break;
            }
            sleep_ms(2);
        }

        if (compass_qmc5883l::calibration_finish(g_compass_cal)) {
            calibration_flash::save(g_compass_cal);
            g_state.cal_needed = false;
            displayport::send_message(o4_endpoint(), "CAL SAVED");
        } else {
            g_compass_cal = calibration_flash::defaults();
            g_state.cal_needed = true;
            displayport::send_message(o4_endpoint(), "CAL FAILED - ROTATE MORE");
            sleep_ms(2000);
        }
    }

    initializing();

    while (true) {
        const uint32_t now = millis();
        g_state.now_ms = now;

        poll_msp();
        gps::poll_uart(g_gps_parser, g_state.gps, now);
        gps::update_timeout(g_state.gps, now);
        rc_pwm::update(g_state.rc, now);
        update_sensors(now);
        reprobe_optional_sensors(now);
        handle_state_machine(now);
        nav::update(g_state.nav, g_state.gps, g_state.armed, now);
        print_debug(now);

        if (now - g_last_osd_ms >= (1000u / OSD_UPDATE_HZ)) {
            g_last_osd_ms = now;
            displayport::send_frame(o4_endpoint(), g_state);
        }

#if MSP_RAW_GPS_PROACTIVE
        if (g_state.gps.fix_valid && now - g_last_gps_push_ms >= (1000u / UART_GPS_UPDATE_HZ)) {
            g_last_gps_push_ms = now;
            msp::send_raw_gps(o4_endpoint(), g_state);
        }
#endif

        watchdog_update();
        tight_loop_contents();
    }
}
