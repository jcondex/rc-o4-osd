#pragma once

#include <stdint.h>

namespace pwm_self_test {

void init();
void update(uint32_t now_ms);

}
