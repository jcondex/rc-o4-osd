#include "rc_pwm_pio.h"

#include "config.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "rc_pwm.pio.h"

namespace rc_pwm {
namespace {
PIO g_pio = pio0;
uint g_sm = 0;
bool g_ready = false;
}

bool init() {
    uint offset = pio_add_program(g_pio, &rc_pwm_program);
    g_sm = pio_claim_unused_sm(g_pio, true);
    rc_pwm_program_init(g_pio, g_sm, offset, PIN_PWM_IN);
    pio_sm_set_enabled(g_pio, g_sm, true);
    g_ready = true;
    return true;
}

void update(RcState &state, uint32_t now_ms) {
    if (!g_ready) return;

    while (!pio_sm_is_rx_fifo_empty(g_pio, g_sm)) {
        const uint32_t raw = pio_sm_get(g_pio, g_sm);
        const uint32_t pulse = 0xFFFFFFFFu - raw;
        if (pulse >= PWM_MIN_US && pulse <= PWM_MAX_US) {
            state.pulse_us = uint16_t(pulse);
            state.last_valid_ms = now_ms;
            state.signal_valid = true;
            state.failsafe = false;
            state.arm_switch_high = pulse > PWM_ARM_THRESHOLD;
            state.arm_switch_low = pulse < PWM_DISARM_THRESHOLD;
            if (state.arm_switch_low) {
                state.rearm_latched = true;
            }
        }
    }

    if (state.last_valid_ms == 0 || (now_ms - state.last_valid_ms) > PWM_FAILSAFE_MS) {
        state.signal_valid = false;
        state.failsafe = true;
        state.arm_switch_high = false;
    }
}

}
