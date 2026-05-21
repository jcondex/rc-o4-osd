#pragma once

#include "types.h"

namespace battery {
float adc_raw_to_volts(uint16_t raw_adc);
void init();
void update(BatteryState &state);
}
