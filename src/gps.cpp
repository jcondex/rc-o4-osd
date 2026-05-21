#include "gps.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

namespace gps {
namespace {

static bool sentence_has_type(const char *type, const char *suffix) {
    return strlen(type) >= 5 && strcmp(type + 2, suffix) == 0;
}

static void copy_sentence_type(const char *line, char out[6]) {
    out[0] = '\0';
    if (line[0] != '$') return;
    size_t n = 0;
    for (const char *p = line + 1; *p && *p != ',' && *p != '*' && n < 5; ++p) {
        out[n++] = *p;
    }
    out[n] = '\0';
}

static int split_fields(char *line, char **fields, int max_fields) {
    int count = 0;
    char *p = line;
    if (*p == '$') ++p;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',' || *p == '*') {
            const char sep = *p;
            *p++ = '\0';
            if (sep == '*') break;
            fields[count++] = p;
        } else {
            ++p;
        }
    }
    return count;
}

static double parse_coord(const char *value, const char *hemisphere) {
    if (!value || !*value) return 0.0;
    const double raw = strtod(value, nullptr);
    const int deg = int(raw / 100.0);
    const double min = raw - double(deg * 100);
    double dec = double(deg) + min / 60.0;
    if (hemisphere && (*hemisphere == 'S' || *hemisphere == 'W')) dec = -dec;
    return dec;
}

static bool checksum_ok(const char *line) {
    if (line[0] != '$') return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; ++p) checksum ^= uint8_t(*p);
    const uint8_t expected = uint8_t(strtoul(star + 1, nullptr, 16));
    return checksum == expected;
}

static int32_t le_i32(const uint8_t *p) {
    return int32_t(uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24));
}

static uint16_t le_u16(const uint8_t *p) {
    return uint16_t(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
}

static float wrap_degrees(float degrees) {
    while (degrees >= 360.0f) degrees -= 360.0f;
    while (degrees < 0.0f) degrees += 360.0f;
    return degrees;
}

static void mark_receiving(GpsState &state, uint32_t now_ms) {
    state.receiving = true;
    state.last_message_ms = now_ms;
}

static void clear_fix(GpsState &state, bool preserve_satellites = false) {
    state.fix_valid = false;
    state.fix_type = 0;
    if (!preserve_satellites) {
        state.satellites = 0;
    }
    state.altitude_m = 0.0f;
    state.speed_kmh = 0.0f;
    state.course_deg = 0.0f;
    state.updated = false;
}

static bool parse_rmc(char *line, GpsState &state, uint32_t now_ms) {
    char *f[16] = {};
    split_fields(line, f, 16);
    mark_receiving(state, now_ms);
    char *status = f[2];
    char *lat = f[3];
    char *ns = f[4];
    char *lon = f[5];
    char *ew = f[6];
    char *speed_kn = f[7];
    char *course = f[8];

    state.fix_valid = status && *status == 'A';
    state.fix_type = state.fix_valid ? 1 : 0;
    if (!state.fix_valid) {
        clear_fix(state, true);
        return true;
    }

    state.lat_deg = parse_coord(lat, ns);
    state.lon_deg = parse_coord(lon, ew);
    state.speed_kmh = speed_kn ? float(strtod(speed_kn, nullptr) * 1.852) : 0.0f;
    state.course_deg = course ? wrap_degrees(float(strtod(course, nullptr))) : 0.0f;
    state.last_fix_ms = now_ms;
    state.updated = true;
    return true;
}

static bool parse_gga(char *line, GpsState &state, uint32_t now_ms) {
    char *f[16] = {};
    split_fields(line, f, 16);
    mark_receiving(state, now_ms);
    char *lat = f[2];
    char *ns = f[3];
    char *lon = f[4];
    char *ew = f[5];
    char *quality = f[6];
    char *sats = f[7];
    char *hdop = f[8];
    char *alt = f[9];

    const int q = quality ? atoi(quality) : 0;
    state.fix_valid = q > 0;
    state.fix_type = q > 1 ? 2 : (q > 0 ? 1 : 0);
    state.satellites = sats ? uint8_t(atoi(sats)) : 0;
    state.hdop = hdop ? float(strtod(hdop, nullptr)) : 99.9f;
    if (!state.fix_valid) {
        clear_fix(state, true);
        return true;
    }

    state.lat_deg = parse_coord(lat, ns);
    state.lon_deg = parse_coord(lon, ew);
    state.altitude_m = alt ? float(strtod(alt, nullptr)) : 0.0f;
    state.last_fix_ms = now_ms;
    state.updated = true;
    return true;
}

static bool parse_vtg(char *line, GpsState &state, uint32_t now_ms) {
    char *f[12] = {};
    split_fields(line, f, 12);
    mark_receiving(state, now_ms);
    char *course = f[1];
    char *kmh = f[7];
    if (course && *course) state.course_deg = wrap_degrees(float(strtod(course, nullptr)));
    if (kmh && *kmh) state.speed_kmh = float(strtod(kmh, nullptr));
    return true;
}

static bool parse_line(char *line, GpsState &state, uint32_t now_ms) {
    state.updated = false;
    if (!checksum_ok(line)) return false;
    char sentence_type[6] = {};
    copy_sentence_type(line, sentence_type);
    if (sentence_has_type(sentence_type, "RMC")) return parse_rmc(line, state, now_ms);
    if (sentence_has_type(sentence_type, "GGA")) return parse_gga(line, state, now_ms);
    if (sentence_has_type(sentence_type, "VTG")) return parse_vtg(line, state, now_ms);
    return false;
}

static bool parse_ubx_nav_pvt(const uint8_t *payload, uint16_t len, GpsState &state, uint32_t now_ms) {
    if (len < 78) return false;

    const uint8_t fix_type = payload[20];
    const uint8_t flags = payload[21];
    const bool fix_ok = (flags & 0x01u) != 0 && fix_type >= 2;

    state.satellites = payload[23];
    state.hdop = float(le_u16(&payload[76])) / 100.0f;
    mark_receiving(state, now_ms);

    if (!fix_ok) {
        clear_fix(state, true);
        return true;
    }

    state.fix_valid = true;
    state.fix_type = fix_type;
    state.lon_deg = double(le_i32(&payload[24])) * 1.0e-7;
    state.lat_deg = double(le_i32(&payload[28])) * 1.0e-7;
    state.altitude_m = float(le_i32(&payload[36])) / 1000.0f;
    state.speed_kmh = float(le_i32(&payload[60])) * 0.0036f;
    state.course_deg = wrap_degrees(float(le_i32(&payload[64])) / 100000.0f);
    state.last_fix_ms = now_ms;
    state.updated = true;
    return true;
}

}

void ubx_checksum(const uint8_t *data, size_t len, uint8_t &ck_a, uint8_t &ck_b) {
    ck_a = 0;
    ck_b = 0;
    for (size_t i = 0; i < len; ++i) {
        ck_a = uint8_t(ck_a + data[i]);
        ck_b = uint8_t(ck_b + ck_a);
    }
}

void NmeaParser::reset() {
    len_ = 0;
    line_[0] = '\0';
    ubx_state_ = UbxParseState::Sync1;
    ubx_class_ = 0;
    ubx_id_ = 0;
    ubx_len_ = 0;
    ubx_pos_ = 0;
    ubx_ck_a_ = 0;
    ubx_ck_b_ = 0;
}

bool NmeaParser::parse_byte(uint8_t byte, GpsState &state, uint32_t now_ms) {
    ++stats_.bytes;
    switch (ubx_state_) {
    case UbxParseState::Sync1:
        if (byte == 0xB5) {
            len_ = 0;
            line_[0] = '\0';
            ubx_state_ = UbxParseState::Sync2;
            return false;
        }
        break;
    case UbxParseState::Sync2:
        if (byte == 0x62) {
            ubx_state_ = UbxParseState::Class;
            ubx_ck_a_ = 0;
            ubx_ck_b_ = 0;
            return false;
        }
        ubx_state_ = UbxParseState::Sync1;
        if (byte == 0xB5) {
            ubx_state_ = UbxParseState::Sync2;
            return false;
        }
        break;
    case UbxParseState::Class:
        ubx_class_ = byte;
        ubx_ck_a_ = uint8_t(ubx_ck_a_ + byte);
        ubx_ck_b_ = uint8_t(ubx_ck_b_ + ubx_ck_a_);
        ubx_state_ = UbxParseState::Id;
        return false;
    case UbxParseState::Id:
        ubx_id_ = byte;
        ubx_ck_a_ = uint8_t(ubx_ck_a_ + byte);
        ubx_ck_b_ = uint8_t(ubx_ck_b_ + ubx_ck_a_);
        ubx_state_ = UbxParseState::Len1;
        return false;
    case UbxParseState::Len1:
        ubx_len_ = byte;
        ubx_ck_a_ = uint8_t(ubx_ck_a_ + byte);
        ubx_ck_b_ = uint8_t(ubx_ck_b_ + ubx_ck_a_);
        ubx_state_ = UbxParseState::Len2;
        return false;
    case UbxParseState::Len2:
        ubx_len_ |= uint16_t(byte) << 8;
        ubx_ck_a_ = uint8_t(ubx_ck_a_ + byte);
        ubx_ck_b_ = uint8_t(ubx_ck_b_ + ubx_ck_a_);
        ubx_pos_ = 0;
        if (ubx_len_ > sizeof(ubx_payload_)) {
            ++stats_.ubx_oversize;
            ubx_pos_ = uint16_t(ubx_len_ + 2u);
            ubx_state_ = UbxParseState::DiscardOversize;
            return false;
        }
        ubx_state_ = ubx_len_ == 0 ? UbxParseState::ChecksumA : UbxParseState::Payload;
        return false;
    case UbxParseState::Payload:
        ubx_payload_[ubx_pos_++] = byte;
        ubx_ck_a_ = uint8_t(ubx_ck_a_ + byte);
        ubx_ck_b_ = uint8_t(ubx_ck_b_ + ubx_ck_a_);
        if (ubx_pos_ >= ubx_len_) {
            ubx_state_ = UbxParseState::ChecksumA;
        }
        return false;
    case UbxParseState::ChecksumA:
        if (byte != ubx_ck_a_) {
            ++stats_.ubx_checksum_errors;
            ubx_state_ = UbxParseState::Sync1;
            return false;
        }
        ubx_state_ = UbxParseState::ChecksumB;
        return false;
    case UbxParseState::ChecksumB: {
        ubx_state_ = UbxParseState::Sync1;
        if (byte != ubx_ck_b_) {
            ++stats_.ubx_checksum_errors;
            return false;
        }
        ++stats_.ubx_frames;
        if (ubx_class_ == 0x01 && ubx_id_ == 0x07 && parse_ubx_nav_pvt(ubx_payload_, ubx_len_, state, now_ms)) {
            ++stats_.ubx_nav_pvt;
            return true;
        }
        return false;
    }
    case UbxParseState::DiscardOversize:
        if (ubx_pos_ > 0) {
            --ubx_pos_;
        }
        if (ubx_pos_ == 0) {
            ubx_state_ = UbxParseState::Sync1;
        }
        return false;
    }

    if (byte == '$') {
        ++stats_.dollar_signs;
        len_ = 0;
    }
    if (byte == '\r') return false;
    if (byte == '\n') {
        ++stats_.newlines;
        line_[len_] = '\0';
        bool ok = false;
        if (len_ > 0) {
            state.updated = false;
            ++stats_.sentences;
            char sentence_type[6] = {};
            copy_sentence_type(line_, sentence_type);
            const bool has_bad_checksum = line_[0] == '$' && !checksum_ok(line_);
            if (has_bad_checksum) {
                ++stats_.checksum_errors;
                memcpy(stats_.last_bad_type, sentence_type, sizeof(stats_.last_bad_type));
            } else {
                ok = parse_line(line_, state, now_ms);
                if (ok) {
                    ++stats_.parsed_sentences;
                    memcpy(stats_.last_good_type, sentence_type, sizeof(stats_.last_good_type));
                    if (sentence_has_type(sentence_type, "RMC")) {
                        ++stats_.rmc_sentences;
                    } else if (sentence_has_type(sentence_type, "GGA")) {
                        ++stats_.gga_sentences;
                    } else if (sentence_has_type(sentence_type, "VTG")) {
                        ++stats_.vtg_sentences;
                    }
                } else if (sentence_type[0]) {
                    ++stats_.unsupported_sentences;
                }
            }
        }
        len_ = 0;
        return ok;
    }
    if (len_ < sizeof(line_) - 1) {
        line_[len_++] = char(byte);
    } else {
        ++stats_.overflows;
        reset();
    }
    return false;
}

void update_timeout(GpsState &state, uint32_t now_ms) {
    if (state.receiving && (state.last_message_ms == 0 || (now_ms - state.last_message_ms) > GPS_LINK_TIMEOUT_MS)) {
        state.receiving = false;
    }
    if (!state.fix_valid) return;
    if (state.last_fix_ms == 0 || (now_ms - state.last_fix_ms) > GPS_FIX_TIMEOUT_MS) {
        clear_fix(state);
    }
}

}
