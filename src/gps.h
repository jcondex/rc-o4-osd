#pragma once

#include <stddef.h>
#include <stdint.h>
#include "types.h"

namespace gps {

struct Stats {
    uint32_t bytes = 0;
    uint32_t dollar_signs = 0;
    uint32_t newlines = 0;
    uint32_t sentences = 0;
    uint32_t parsed_sentences = 0;
    uint32_t rmc_sentences = 0;
    uint32_t gga_sentences = 0;
    uint32_t vtg_sentences = 0;
    uint32_t unsupported_sentences = 0;
    uint32_t checksum_errors = 0;
    uint32_t overflows = 0;
    uint32_t uart_rx_drops = 0;
    uint32_t ubx_frames = 0;
    uint32_t ubx_nav_pvt = 0;
    uint32_t ubx_checksum_errors = 0;
    uint32_t ubx_oversize = 0;
    char last_good_type[6] = {};
    char last_bad_type[6] = {};
};

class NmeaParser {
public:
    bool parse_byte(uint8_t byte, GpsState &state, uint32_t now_ms);
    void reset();
    void add_uart_rx_drops(uint32_t count) { stats_.uart_rx_drops += count; }
    const Stats &stats() const { return stats_; }

private:
    char line_[128] = {};
    size_t len_ = 0;
    Stats stats_;
    enum class UbxParseState : uint8_t {
        Sync1,
        Sync2,
        Class,
        Id,
        Len1,
        Len2,
        Payload,
        ChecksumA,
        ChecksumB,
        DiscardOversize,
    };
    UbxParseState ubx_state_ = UbxParseState::Sync1;
    uint8_t ubx_class_ = 0;
    uint8_t ubx_id_ = 0;
    uint16_t ubx_len_ = 0;
    uint16_t ubx_pos_ = 0;
    uint8_t ubx_ck_a_ = 0;
    uint8_t ubx_ck_b_ = 0;
    uint8_t ubx_payload_[100] = {};
};

void init_uart();
void poll_uart(NmeaParser &parser, GpsState &state, uint32_t now_ms);
void update_timeout(GpsState &state, uint32_t now_ms);
void send_ubx_startup_config();
void ubx_checksum(const uint8_t *data, size_t len, uint8_t &ck_a, uint8_t &ck_b);

}
