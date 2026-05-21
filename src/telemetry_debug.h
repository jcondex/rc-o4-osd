#pragma once

#include <stdint.h>

#include "gps.h"
#include "msp.h"
#include "types.h"

namespace telemetry_debug {

void print(uint32_t now_ms, const TelemetryState &state, const msp::Stats &msp_rx, const msp::Stats &msp_tx, const gps::Stats &gps_stats);

}
