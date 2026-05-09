#include "calibration_flash.h"

#include <string.h>
#include <stddef.h>
#include "config.h"

#ifndef RC_O4_OSD_HOST_TEST
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#endif

namespace calibration_flash {
namespace {

struct StoredCal {
    uint32_t magic;
    float x_offset;
    float y_offset;
    float z_offset;
    float x_scale;
    float y_scale;
    float z_scale;
    float declination;
    uint32_t crc32;
};

static CompassCalibration from_stored(const StoredCal &s, bool valid) {
    CompassCalibration cal;
    cal.x_offset = s.x_offset;
    cal.y_offset = s.y_offset;
    cal.z_offset = s.z_offset;
    cal.x_scale = s.x_scale;
    cal.y_scale = s.y_scale;
    cal.z_scale = s.z_scale;
    cal.declination_deg = s.declination;
    cal.valid = valid;
    return cal;
}

}

uint32_t crc32_bytes(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

CompassCalibration defaults() {
    CompassCalibration cal;
    cal.x_offset = QMC_X_OFFSET;
    cal.y_offset = QMC_Y_OFFSET;
    cal.z_offset = QMC_Z_OFFSET;
    cal.x_scale = QMC_X_SCALE;
    cal.y_scale = QMC_Y_SCALE;
    cal.z_scale = QMC_Z_SCALE;
    cal.declination_deg = MAG_DECLINATION;
    cal.valid = false;
    return cal;
}

size_t storage_size() {
    return sizeof(StoredCal);
}

bool encode(const CompassCalibration &cal, uint8_t *out, size_t out_len) {
    if (!out || out_len < sizeof(StoredCal)) {
        return false;
    }
    StoredCal stored = {};
    stored.magic = FLASH_CAL_MAGIC;
    stored.x_offset = cal.x_offset;
    stored.y_offset = cal.y_offset;
    stored.z_offset = cal.z_offset;
    stored.x_scale = cal.x_scale;
    stored.y_scale = cal.y_scale;
    stored.z_scale = cal.z_scale;
    stored.declination = cal.declination_deg;
    stored.crc32 = crc32_bytes(reinterpret_cast<const uint8_t *>(&stored), offsetof(StoredCal, crc32));
    memset(out, 0, out_len);
    memcpy(out, &stored, sizeof(stored));
    return true;
}

bool decode(const uint8_t *data, size_t len, CompassCalibration &cal) {
    if (!data || len < sizeof(StoredCal)) {
        cal = defaults();
        return false;
    }
    StoredCal stored = {};
    memcpy(&stored, data, sizeof(stored));
    if (stored.magic != FLASH_CAL_MAGIC) {
        cal = defaults();
        return false;
    }
    const uint32_t expected = crc32_bytes(reinterpret_cast<const uint8_t *>(&stored), offsetof(StoredCal, crc32));
    if (expected != stored.crc32) {
        cal = defaults();
        return false;
    }
    cal = from_stored(stored, true);
    return true;
}

bool save(const CompassCalibration &cal) {
#ifdef RC_O4_OSD_HOST_TEST
    (void)cal;
    return false;
#else
    uint8_t sector[FLASH_SECTOR_SIZE] = {};
    if (!encode(cal, sector, sizeof(sector))) {
        return false;
    }
    const uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_CAL_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_CAL_OFFSET, sector, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    return true;
#endif
}

bool load(CompassCalibration &cal) {
#ifdef RC_O4_OSD_HOST_TEST
    cal = defaults();
    return false;
#else
    const auto *stored = reinterpret_cast<const uint8_t *>(XIP_BASE + FLASH_CAL_OFFSET);
    return decode(stored, sizeof(StoredCal), cal);
#endif
}

CompassCalibration collect_compass_calibration(uint32_t) {
    CompassCalibration cal = defaults();
    cal.valid = false;
    return cal;
}

}
