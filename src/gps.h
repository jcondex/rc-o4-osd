#pragma once

#include <stddef.h>
#include <stdint.h>
#include "types.h"

namespace gps {

class NmeaParser {
public:
    bool parse_byte(uint8_t byte, GpsState &state, uint32_t now_ms);
    void reset();

private:
    char line_[128] = {};
    size_t len_ = 0;
};

void init_uart();
void poll_uart(NmeaParser &parser, GpsState &state, uint32_t now_ms);
void update_timeout(GpsState &state, uint32_t now_ms);
void send_ubx_startup_config();
void ubx_checksum(const uint8_t *data, size_t len, uint8_t &ck_a, uint8_t &ck_b);

}
