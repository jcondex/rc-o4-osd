#include "msp_osd_config.h"

namespace msp {
namespace {

constexpr uint8_t kOsdFlagsFeature = (1u << 0);
constexpr uint8_t kOsdFlagsDeviceDetected = (1u << 5);
constexpr uint8_t kOsdFlagsMspDevice = (1u << 6);
constexpr uint8_t kVideoSystemHd = 3;
constexpr uint8_t kOsdUnitMetric = 1;
constexpr uint8_t kOsdItemCount = 58;
constexpr uint8_t kOsdStatCount = 24;
constexpr uint8_t kOsdTimerCount = 2;
constexpr uint8_t kOsdWarningCount = 16;
constexpr uint32_t kOsdEnabledWarnings = (1u << 9); // FAIL_SAFE in ArduPilot MSP OSD ordering.
constexpr uint16_t kOsdPosVisible = 2048;

static void put_u8(uint8_t *out, size_t &n, uint8_t v) { out[n++] = v; }
static void put_u16(uint8_t *out, size_t &n, uint16_t v) {
    out[n++] = uint8_t(v);
    out[n++] = uint8_t(v >> 8);
}
static void put_u32(uint8_t *out, size_t &n, uint32_t v) {
    out[n++] = uint8_t(v);
    out[n++] = uint8_t(v >> 8);
    out[n++] = uint8_t(v >> 16);
    out[n++] = uint8_t(v >> 24);
}

static uint16_t osd_pos(uint8_t x, uint8_t y) {
    return uint16_t(kOsdPosVisible + x + uint16_t(y) * 32u);
}

static uint16_t native_dji_osd_item_pos(uint8_t item) {
    enum : uint8_t {
        OSD_MAIN_BATT_VOLTAGE = 1,
        OSD_FLYMODE = 7,
        OSD_CRAFT_NAME = 8,
        OSD_GPS_SPEED = 13,
        OSD_GPS_SATS = 14,
        OSD_ALTITUDE = 15,
        OSD_WARNINGS = 21,
        OSD_PITCH_ANGLE = 26,
        OSD_ROLL_ANGLE = 27,
        OSD_DISARMED = 29,
        OSD_HOME_DIR = 30,
        OSD_HOME_DIST = 31,
        OSD_NUMERICAL_VARIO = 33,
    };

    switch (item) {
    case OSD_CRAFT_NAME: return osd_pos(1, 0);
    case OSD_MAIN_BATT_VOLTAGE: return osd_pos(1, 1);
    case OSD_GPS_SPEED: return osd_pos(1, 2);
    case OSD_GPS_SATS: return osd_pos(1, 3);
    case OSD_ALTITUDE: return osd_pos(1, 4);
    case OSD_HOME_DIR: return osd_pos(1, 5);
    case OSD_HOME_DIST: return osd_pos(4, 5);
    case OSD_NUMERICAL_VARIO: return osd_pos(1, 6);
    case OSD_PITCH_ANGLE: return osd_pos(1, 7);
    case OSD_ROLL_ANGLE: return osd_pos(15, 7);
    case OSD_FLYMODE: return osd_pos(1, 8);
    case OSD_DISARMED: return osd_pos(12, 8);
    case OSD_WARNINGS: return osd_pos(6, 10);
    default: return 0;
    }
}

}

void build_osd_config_payload(uint8_t *out, size_t &n) {
    put_u8(out, n, kOsdFlagsFeature | kOsdFlagsDeviceDetected | kOsdFlagsMspDevice);
    put_u8(out, n, kVideoSystemHd);
    put_u8(out, n, kOsdUnitMetric);
    put_u8(out, n, 0); // RSSI alarm disabled.
    put_u16(out, n, 0); // Capacity alarm disabled.
    put_u8(out, n, 0);
    put_u8(out, n, kOsdItemCount);
    put_u16(out, n, 0); // Altitude alarm disabled.
    for (uint8_t i = 0; i < kOsdItemCount; ++i) {
        put_u16(out, n, native_dji_osd_item_pos(i));
    }
    put_u8(out, n, kOsdStatCount);
    for (uint8_t i = 0; i < kOsdStatCount; ++i) {
        put_u16(out, n, 0);
    }
    put_u8(out, n, kOsdTimerCount);
    for (uint8_t i = 0; i < kOsdTimerCount; ++i) {
        put_u16(out, n, 0);
    }
    put_u16(out, n, uint16_t(kOsdEnabledWarnings & 0xFFFFu));
    put_u8(out, n, kOsdWarningCount);
    put_u32(out, n, kOsdEnabledWarnings);
    put_u8(out, n, 1); // Available profiles.
    put_u8(out, n, 1); // Selected profile.
    put_u8(out, n, 0); // Stick overlay unavailable.
}

}
