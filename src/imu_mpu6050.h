#pragma once

#include "types.h"

namespace imu_mpu6050 {
bool init(ImuState &state);
bool update(ImuState &state, uint32_t now_ms);
void update_filter_from_sample(ImuState &state, float ax, float ay, float az, float gx, float gy, float gz, uint32_t now_ms);
}
