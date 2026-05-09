#pragma once

#include "types.h"

namespace rc_pwm {
bool init();
void update(RcState &state, uint32_t now_ms);
}
