#pragma once

#include <stddef.h>
#include <stdint.h>
#include "types.h"

namespace msp {

enum Command : uint8_t {
    MSP_API_VERSION = 1,
    MSP_FC_VARIANT = 2,
    MSP_FC_VERSION = 3,
    MSP_BOARD_INFO = 4,
    MSP_BUILD_INFO = 5,
    MSP_NAME = 10,
    MSP_OSD_CONFIG = 84,
    MSP_FILTER_CONFIG = 92,
    MSP_PID_ADVANCED = 94,
    MSP_STATUS = 101,
    MSP_RC = 105,
    MSP_RAW_GPS = 106,
    MSP_COMP_GPS = 107,
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_ANALOG = 110,
    MSP_RC_TUNING = 111,
    MSP_PID = 112,
    MSP_BATTERY_STATE = 130,
    MSP_ESC_SENSOR_DATA = 134,
    MSP_STATUS_EX = 150,
    MSP_DISPLAYPORT = 182,
    MSP_SET_OSD_CANVAS = 188,
    MSP_OSD_CANVAS = 189,
    MSP_RTC = 247,
};

using Writer = void (*)(const uint8_t *data, size_t len, void *ctx);

struct Stats {
    uint32_t rx_packets = 0;
    uint32_t tx_packets = 0;
    uint32_t checksum_errors = 0;
    uint32_t unknown_commands = 0;
    uint32_t name_requests = 0;
    uint32_t osd_config_requests = 0;
    uint32_t osd_canvas_requests = 0;
    uint32_t set_osd_canvas_requests = 0;
    uint32_t displayport_packets = 0;
    uint8_t last_cmd = 0;
    uint8_t last_bad_cmd = 0;
    uint8_t last_bad_len = 0;
    uint8_t last_canvas_cols = 0;
    uint8_t last_canvas_rows = 0;
};

struct Endpoint {
    Writer writer = nullptr;
    void *ctx = nullptr;
    Stats *stats = nullptr;
};

class Parser {
public:
    void reset();
    void parse_byte(uint8_t b, const TelemetryState &state, Endpoint endpoint);
    const Stats &stats() const { return stats_; }

private:
    enum class RxState : uint8_t {
        WaitStart,
        WaitM,
        WaitDir,
        WaitLen,
        WaitCmd,
        ReadPayload,
        ReadChecksum,
    };

    RxState state_ = RxState::WaitStart;
    uint8_t direction_ = 0;
    uint8_t len_ = 0;
    uint8_t cmd_ = 0;
    uint8_t checksum_ = 0;
    uint8_t offset_ = 0;
    uint8_t payload_[192] = {};
    Stats stats_;
};

void send_packet(Endpoint endpoint, uint8_t cmd, const uint8_t *payload, size_t len);
void send_empty(Endpoint endpoint, uint8_t cmd);
void send_startup_burst(Endpoint endpoint, const TelemetryState &state);
void send_raw_gps(Endpoint endpoint, const TelemetryState &state);
void send_displayport(Endpoint endpoint, const uint8_t *payload, size_t len);

uint16_t active_sensor_mask(const TelemetryState &state);
bool build_response(uint8_t cmd, const TelemetryState &state, uint8_t *out, size_t out_cap, size_t *out_len);

}
