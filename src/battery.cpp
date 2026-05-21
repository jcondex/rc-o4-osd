#include "battery.h"

#include "config.h"
#ifndef RC_O4_OSD_HOST_TEST
#include "hardware/adc.h"
#endif

namespace battery {

float adc_raw_to_volts(uint16_t raw_adc) {
    const float vadc = (float(raw_adc) / 4095.0f) * 3.3f;
    return vadc * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;
}

void init() {
#ifndef RC_O4_OSD_HOST_TEST
    adc_init();
    adc_gpio_init(PIN_BAT_ADC);
#endif
}

void update(BatteryState &state) {
#ifdef RC_O4_OSD_HOST_TEST
    state.volts = adc_raw_to_volts(state.raw_adc);
#else
    adc_select_input(0);
    uint32_t sum = 0;
    for (int i = 0; i < 8; ++i) {
        sum += adc_read();
    }
    state.raw_adc = uint16_t(sum / 8u);
    state.volts = adc_raw_to_volts(state.raw_adc);
#endif
    state.low = false;
}

}
