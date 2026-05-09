#pragma once

#include <stddef.h>
#include <stdint.h>

#include "types.h"

namespace calibration_flash {

bool load(CompassCalibration &cal);
bool save(const CompassCalibration &cal);
CompassCalibration defaults();
CompassCalibration collect_compass_calibration(uint32_t duration_ms);
size_t storage_size();
uint32_t crc32_bytes(const uint8_t *data, size_t len);
bool encode(const CompassCalibration &cal, uint8_t *out, size_t out_len);
bool decode(const uint8_t *data, size_t len, CompassCalibration &cal);

}
