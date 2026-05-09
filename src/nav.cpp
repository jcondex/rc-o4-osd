#include "nav.h"

#include <math.h>
#include "config.h"

namespace nav {
namespace {
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;
}

float haversine_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    const double lat1 = lat1_deg * kDegToRad;
    const double lat2 = lat2_deg * kDegToRad;
    const double dlat = (lat2_deg - lat1_deg) * kDegToRad;
    const double dlon = (lon2_deg - lon1_deg) * kDegToRad;
    const double a = sin(dlat / 2.0) * sin(dlat / 2.0)
        + cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);
    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return float(kEarthRadiusM * c);
}

float bearing_deg(double from_lat_deg, double from_lon_deg, double to_lat_deg, double to_lon_deg) {
    const double lat1 = from_lat_deg * kDegToRad;
    const double lat2 = to_lat_deg * kDegToRad;
    const double dlon = (to_lon_deg - from_lon_deg) * kDegToRad;
    const double y = sin(dlon) * cos(lat2);
    const double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    double brg = atan2(y, x) * kRadToDeg;
    while (brg < 0.0) brg += 360.0;
    while (brg >= 360.0) brg -= 360.0;
    return float(brg);
}

void on_arm(NavState &nav_state, const GpsState &gps, uint32_t now_ms) {
    nav_state.armed_since_ms = now_ms;
    nav_state.trip_distance_m = 0.0f;
    nav_state.max_speed_kmh = 0.0f;
    nav_state.last_trip_fix_valid = false;
    nav_state.home_distance_m = 0.0f;
    nav_state.home_bearing_deg = 0.0f;

    if (gps.fix_valid) {
        nav_state.home_set = true;
        nav_state.home_lat_deg = gps.lat_deg;
        nav_state.home_lon_deg = gps.lon_deg;
        nav_state.last_trip_lat_deg = gps.lat_deg;
        nav_state.last_trip_lon_deg = gps.lon_deg;
        nav_state.last_trip_fix_valid = true;
    } else {
        nav_state.home_set = false;
    }
}

void on_disarm(NavState &nav_state) {
    nav_state.home_set = false;
    nav_state.home_distance_m = 0.0f;
    nav_state.home_bearing_deg = 0.0f;
}

void update(NavState &nav_state, const GpsState &gps, bool armed, uint32_t) {
    if (!armed || !gps.fix_valid) {
        return;
    }

    if (nav_state.home_set) {
        nav_state.home_distance_m = haversine_m(gps.lat_deg, gps.lon_deg, nav_state.home_lat_deg, nav_state.home_lon_deg);
        nav_state.home_bearing_deg = bearing_deg(gps.lat_deg, gps.lon_deg, nav_state.home_lat_deg, nav_state.home_lon_deg);
    }

    if (gps.speed_kmh <= GPS_MAX_SPEED_KMH && gps.speed_kmh > nav_state.max_speed_kmh) {
        nav_state.max_speed_kmh = gps.speed_kmh;
    }

    if (!nav_state.last_trip_fix_valid) {
        nav_state.last_trip_lat_deg = gps.lat_deg;
        nav_state.last_trip_lon_deg = gps.lon_deg;
        nav_state.last_trip_fix_valid = true;
        return;
    }

    const float delta = haversine_m(nav_state.last_trip_lat_deg, nav_state.last_trip_lon_deg, gps.lat_deg, gps.lon_deg);
    if (delta <= GPS_MAX_JUMP_M) {
        nav_state.trip_distance_m += delta;
    }
    nav_state.last_trip_lat_deg = gps.lat_deg;
    nav_state.last_trip_lon_deg = gps.lon_deg;
}

uint32_t runtime_seconds(const NavState &nav_state, bool armed, uint32_t now_ms) {
    if (!armed || nav_state.armed_since_ms == 0) return 0;
    return (now_ms - nav_state.armed_since_ms) / 1000u;
}

}
