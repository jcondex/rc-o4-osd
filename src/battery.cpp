#include "battery.h"

#include "config.h"
#include "hardware/adc.h"

namespace battery {

void init() {
    adc_init();
    adc_gpio_init(PIN_BAT_ADC);
}

void update(BatteryState &state) {
    adc_select_input(0);
    uint32_t sum = 0;
    for (int i = 0; i < 8; ++i) {
        sum += adc_read();
    }
    state.raw_adc = uint16_t(sum / 8u);
    const float vadc = (float(state.raw_adc) / 4095.0f) * 3.3f;
    state.volts = vadc * BAT_DIVIDER_RATIO * BAT_CAL_FACTOR;
    state.low = state.volts > 1.0f && state.volts < BAT_WARN_VOLTS;
}

}
