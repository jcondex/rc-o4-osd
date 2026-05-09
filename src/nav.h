#pragma once

#include "types.h"

namespace nav {

float haversine_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg);
float bearing_deg(double from_lat_deg, double from_lon_deg, double to_lat_deg, double to_lon_deg);
void on_arm(NavState &nav, const GpsState &gps, uint32_t now_ms);
void on_disarm(NavState &nav);
void update(NavState &nav, const GpsState &gps, bool armed, uint32_t now_ms);
uint32_t runtime_seconds(const NavState &nav, bool armed, uint32_t now_ms);

}
