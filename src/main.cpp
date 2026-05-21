#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "bmp390.h"
#include "calibration_flash.h"
#include "compass_qmc5883l.h"
#include "config.h"
#include "displayport.h"
#include "gps.h"
#include "msp.h"
#include "nav.h"
#include "pwm_self_test.h"
#include "rc_pwm_pio.h"
#include "sensor_manager.h"
#include "state_machine.h"
#include "telemetry_debug.h"

namespace {

TelemetryState g_state;
CompassCalibration g_compass_cal;
msp::Parser g_msp_parser;
msp::Stats g_msp_stats;
gps::NmeaParser g_gps_parser;

uint32_t g_last_osd_ms = 0;
uint32_t g_last_gps_push_ms = 0;

void uart0_writer(const uint8_t *data, size_t len, void *) {
    uart_write_blocking(uart0, data, len);
}

msp::Endpoint o4_endpoint() {
    return msp::Endpoint{uart0_writer, nullptr, &g_msp_stats};
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

void handle_state_machine(uint32_t now) {
    const state_machine::TransitionResult transition = state_machine::update(g_state);
    if (transition.armed_rising) {
        bmp390::set_zero_reference(g_state.baro);
        nav::on_arm(g_state.nav, g_state.gps, now);
    } else if (transition.armed_falling) {
        nav::on_disarm(g_state.nav);
    }

#if PWM_DEBUG_INPUT_PULLUP
    gpio_put(PIN_LED, g_state.rc.pwm_pin_level ? 1 : 0);
#else
    gpio_put(PIN_LED, g_state.armed ? 1 : 0);
#endif
}

void early_init() {
    stdio_init_all();
    init_gpio();
    init_uart0_o4();
    gps::init_uart();
    sensor_manager::init_i2c();
    pwm_self_test::init();

    g_state.app_state = AppState::EarlyInit;
    sensor_manager::init_all(g_state);
    g_state.cal_needed = !calibration_flash::load(g_compass_cal);

    if (!g_compass_cal.valid) {
        g_compass_cal = calibration_flash::defaults();
    }
}

void initializing() {
    g_state.app_state = AppState::Initializing;
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    rc_pwm::init();
#if GPS_UBX_STARTUP_CONFIG
    gps::send_ubx_startup_config();
#endif
    msp::send_startup_burst(o4_endpoint(), g_state);
#if OSD_DISPLAYPORT_ENABLE
    displayport::send_message(o4_endpoint(), "RC-O4-OSD INIT");
#endif
}

}

int main() {
    early_init();

    if (gpio_get(PIN_CAL_TRIGGER) == 0) {
        g_state.app_state = AppState::Calibrating;
        compass_qmc5883l::calibration_start();
        const uint32_t cal_start_ms = millis();
        uint32_t last_cal_compass_ms = 0;
        uint32_t last_cal_osd_ms = 0;
        while ((millis() - cal_start_ms) < 15000u) {
            const uint32_t now = millis();
            g_state.now_ms = now;
            poll_msp();
            if (now - last_cal_compass_ms >= 40u) {
                last_cal_compass_ms = now;
                if (compass_qmc5883l::update(g_state.compass, g_compass_cal, g_state.imu, now)) {
                    compass_qmc5883l::calibration_update(
                        g_state.compass.raw_x,
                        g_state.compass.raw_y,
                        g_state.compass.raw_z);
                }
            }
            if (now - last_cal_osd_ms >= 250u) {
                last_cal_osd_ms = now;
#if OSD_DISPLAYPORT_ENABLE
                char msg[31] = {};
                snprintf(msg, sizeof(msg), "CAL:%5lu SAMPLES", compass_qmc5883l::calibration_sample_count());
                displayport::send_message(o4_endpoint(), msg);
#endif
            }
            if (gpio_get(PIN_CAL_TRIGGER) != 0 && (now - cal_start_ms) > 2000u) {
                break;
            }
            sleep_ms(2);
        }

        if (compass_qmc5883l::calibration_finish(g_compass_cal)) {
            calibration_flash::save(g_compass_cal);
            g_state.cal_needed = false;
#if OSD_DISPLAYPORT_ENABLE
            displayport::send_message(o4_endpoint(), "CAL SAVED");
#endif
        } else {
            g_compass_cal = calibration_flash::defaults();
            g_state.cal_needed = true;
#if OSD_DISPLAYPORT_ENABLE
            displayport::send_message(o4_endpoint(), "CAL FAILED - ROTATE MORE");
#endif
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
        pwm_self_test::update(now);
        sensor_manager::update_all(g_state, g_compass_cal, now);
        sensor_manager::reprobe_optional(g_state, now);
        handle_state_machine(now);
        nav::update(g_state.nav, g_state.gps, g_state.armed, now);
        telemetry_debug::print(now, g_state, g_msp_parser.stats(), g_msp_stats, g_gps_parser.stats());

        if (now - g_last_osd_ms >= (1000u / OSD_UPDATE_HZ)) {
            g_last_osd_ms = now;
#if OSD_DISPLAYPORT_ENABLE
            displayport::send_frame(o4_endpoint(), g_state);
#endif
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
