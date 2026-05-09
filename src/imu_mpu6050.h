#pragma once

#include "types.h"

namespace imu_mpu6050 {
bool init(ImuState &state);
bool update(ImuState &state, uint32_t now_ms);
}
