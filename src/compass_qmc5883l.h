#pragma once

#include "types.h"

namespace compass_qmc5883l {
bool init(CompassState &state);
bool update(CompassState &state, const CompassCalibration &cal, const ImuState &imu, uint32_t now_ms);
void calibration_start();
void calibration_update(int16_t raw_x, int16_t raw_y, int16_t raw_z);
bool calibration_finish(CompassCalibration &out);
uint32_t calibration_sample_count();
}
