#include "pwm_self_test.h"

#include "config.h"

#ifndef RC_O4_OSD_HOST_TEST
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#endif

namespace pwm_self_test {
namespace {
#ifndef RC_O4_OSD_HOST_TEST
uint32_t g_last_update_ms = 0;
uint g_slice = 0;
uint g_channel = 0;
#endif
}

void init() {
#if PWM_SELF_TEST_OUTPUT
#if PWM_SELF_TEST_STATIC_HIGH
    gpio_init(PIN_PWM_SELF_TEST);
    gpio_set_dir(PIN_PWM_SELF_TEST, GPIO_OUT);
    gpio_put(PIN_PWM_SELF_TEST, 1);
#else
    gpio_set_function(PIN_PWM_SELF_TEST, GPIO_FUNC_PWM);
    g_slice = pwm_gpio_to_slice_num(PIN_PWM_SELF_TEST);
    g_channel = pwm_gpio_to_channel(PIN_PWM_SELF_TEST);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, float(clock_get_hz(clk_sys)) / 1000000.0f);
    pwm_config_set_wrap(&cfg, 20000u - 1u);
    pwm_init(g_slice, &cfg, true);
    pwm_set_chan_level(g_slice, g_channel, 1000u);
#endif
#endif
}

void update(uint32_t now_ms) {
#if PWM_SELF_TEST_OUTPUT && !PWM_SELF_TEST_STATIC_HIGH
    if (now_ms - g_last_update_ms < 3000u) return;
    g_last_update_ms = now_ms;
    const uint32_t phase = (now_ms / 3000u) % 3u;
    const uint16_t pulse_us = phase == 0u ? 1000u : (phase == 1u ? 1500u : 2000u);
    pwm_set_chan_level(g_slice, g_channel, pulse_us);
#else
    (void)now_ms;
#endif
}

}
