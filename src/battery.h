#pragma once

#include "types.h"

namespace battery {
void init();
void update(BatteryState &state);
}
