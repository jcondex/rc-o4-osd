#include "rc_pwm_pio.h"

#include "config.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "rc_pwm.pio.h"

namespace rc_pwm {
namespace {
PIO g_pio = pio0;
uint g_sm = 0;
bool g_ready = false;
bool g_last_level = false;
uint32_t g_last_rise_us = 0;
volatile uint32_t g_irq_last_rise_us = 0;
volatile uint32_t g_irq_pulse_us = 0;
volatile uint32_t g_irq_edges = 0;
volatile uint32_t g_irq_pulses = 0;
uint32_t g_last_accepted_irq_pulses = 0;

void accept_pulse(RcState &state, uint32_t now_ms, uint32_t pulse) {
    state.pulse_us = uint16_t(pulse);
    state.last_valid_ms = now_ms;
    state.signal_valid = true;
    state.failsafe = false;
    state.arm_switch_low = pulse < PWM_DISARM_THRESHOLD;
    state.arm_switch_high = pulse > PWM_ARM_THRESHOLD;
    state.arm_switch_middle = !state.arm_switch_low && !state.arm_switch_high;
    state.cockpit_mode_requested = state.arm_switch_high;
    if (state.arm_switch_low) {
        state.switch_position = RcSwitchPosition::Low;
        state.rearm_latched = true;
    } else if (state.arm_switch_high) {
        state.switch_position = RcSwitchPosition::High;
    } else {
        state.switch_position = RcSwitchPosition::Middle;
    }
}

void gpio_irq_callback(uint gpio, uint32_t events) {
    if (gpio != PIN_PWM_IN) return;

    const uint32_t now_us = time_us_32();
    ++g_irq_edges;
    if (events & GPIO_IRQ_EDGE_RISE) {
        g_irq_last_rise_us = now_us;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) && g_irq_last_rise_us != 0) {
        g_irq_pulse_us = now_us - g_irq_last_rise_us;
        ++g_irq_pulses;
    }
}
}

bool init() {
    uint offset = pio_add_program(g_pio, &rc_pwm_program);
    g_sm = pio_claim_unused_sm(g_pio, true);
    rc_pwm_program_init(g_pio, g_sm, offset, PIN_PWM_IN);
    pio_sm_set_enabled(g_pio, g_sm, true);
    gpio_set_irq_enabled_with_callback(PIN_PWM_IN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_irq_callback);
    g_ready = true;
    return true;
}

void update(RcState &state, uint32_t now_ms) {
    if (!g_ready) return;

    const bool level = gpio_get(PIN_PWM_IN);
    const uint32_t now_us = time_us_32();
    state.pwm_pin_level = level;
    state.sw_edges = g_irq_edges;
    state.irq_pulses = g_irq_pulses;
    const uint32_t irq_pulse = g_irq_pulse_us;
    if (irq_pulse <= 0xFFFFu) {
        state.sw_pulse_us = uint16_t(irq_pulse);
    } else {
        state.sw_pulse_us = 0xFFFFu;
    }
    if (level != g_last_level) {
        if (level) {
            g_last_rise_us = now_us;
        } else if (g_last_rise_us != 0) {
            const uint32_t sw_pulse = now_us - g_last_rise_us;
            if (sw_pulse <= 0xFFFFu) {
                state.sw_pulse_us = uint16_t(sw_pulse);
            }
        }
        g_last_level = level;
    }

    while (!pio_sm_is_rx_fifo_empty(g_pio, g_sm)) {
        const uint32_t raw = pio_sm_get(g_pio, g_sm);
        const uint32_t pulse = 0xFFFFFFFFu - raw;
        if (pulse <= 0xFFFFu) {
            state.last_raw_pulse_us = uint16_t(pulse);
        } else {
            state.last_raw_pulse_us = 0xFFFFu;
        }
        ++state.pio_pulses;
        if (pulse >= PWM_MIN_US && pulse <= PWM_MAX_US) {
            accept_pulse(state, now_ms, pulse);
        } else {
            ++state.pio_rejected;
        }
    }

    const uint32_t irq_pulses = g_irq_pulses;
    if (irq_pulses != g_last_accepted_irq_pulses && irq_pulse >= PWM_MIN_US && irq_pulse <= PWM_MAX_US) {
        g_last_accepted_irq_pulses = irq_pulses;
        accept_pulse(state, now_ms, irq_pulse);
    }

    if (state.last_valid_ms == 0 || (now_ms - state.last_valid_ms) > PWM_FAILSAFE_MS) {
        state.signal_valid = false;
        state.failsafe = true;
        state.arm_switch_high = false;
        state.arm_switch_middle = false;
        state.arm_switch_low = false;
        state.cockpit_mode_requested = false;
        state.switch_position = RcSwitchPosition::Unknown;
        state.pulse_us = 0;
    }
}

}
