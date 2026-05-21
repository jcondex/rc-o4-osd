#pragma once

#include <stdint.h>
#include "types.h"

namespace sensor_manager {

void init_i2c();
void init_all(TelemetryState &state);
void update_all(TelemetryState &state, CompassCalibration &compass_cal, uint32_t now_ms);
void reprobe_optional(TelemetryState &state, uint32_t now_ms);

}
