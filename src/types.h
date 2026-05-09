#pragma once

#include <stdint.h>

enum class AppState : uint8_t {
    EarlyInit,
    Calibrating,
    Initializing,
    Disarmed,
    Armed,
    Failsafe,
};

struct BatteryState {
    uint16_t raw_adc = 0;
    float volts = 0.0f;
    bool low = false;
};

struct BaroState {
    bool initialized = false;
    bool valid = false;
    float pressure_pa = 101325.0f;
    float temperature_c = 0.0f;
    float altitude_m = 0.0f;
    float relative_altitude_m = 0.0f;
    float vertical_speed_ms = 0.0f;
    uint32_t last_read_ms = 0;
};

struct ImuState {
    bool initialized = false;
    bool valid = false;
    float accel_g[3] = {0.0f, 0.0f, 0.0f};
    float gyro_dps[3] = {0.0f, 0.0f, 0.0f};
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    uint32_t last_read_ms = 0;
};

struct CompassCalibration {
    float x_offset = 0.0f;
    float y_offset = 0.0f;
    float z_offset = 0.0f;
    float x_scale = 1.0f;
    float y_scale = 1.0f;
    float z_scale = 1.0f;
    float declination_deg = 0.0f;
    bool valid = false;
};

struct CompassState {
    bool initialized = false;
    bool valid = false;
    int16_t raw_x = 0;
    int16_t raw_y = 0;
    int16_t raw_z = 0;
    float heading_deg = 0.0f;
    uint32_t last_read_ms = 0;
};

struct GpsState {
    bool fix_valid = false;
    uint8_t fix_type = 0;
    uint8_t satellites = 0;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    float altitude_m = 0.0f;
    float speed_kmh = 0.0f;
    float course_deg = 0.0f;
    float hdop = 99.9f;
    bool updated = false;
    uint32_t last_fix_ms = 0;
};

struct RcState {
    bool signal_valid = false;
    bool failsafe = true;
    bool arm_switch_high = false;
    bool arm_switch_low = false;
    bool rearm_latched = true;
    uint16_t pulse_us = 0;
    uint32_t last_valid_ms = 0;
};

struct NavState {
    bool home_set = false;
    double home_lat_deg = 0.0;
    double home_lon_deg = 0.0;
    double last_trip_lat_deg = 0.0;
    double last_trip_lon_deg = 0.0;
    bool last_trip_fix_valid = false;
    float home_distance_m = 0.0f;
    float home_bearing_deg = 0.0f;
    float trip_distance_m = 0.0f;
    float max_speed_kmh = 0.0f;
    uint32_t armed_since_ms = 0;
};

struct TelemetryState {
    AppState app_state = AppState::EarlyInit;
    bool armed = false;
    bool cal_needed = false;
    bool bmp390_ok = false;
    bool mpu6050_ok = false;
    bool compass_ok = false;
    uint16_t i2c_errors = 0;
    uint16_t cycle_time_us = 1000;
    uint32_t now_ms = 0;
    BatteryState battery;
    BaroState baro;
    ImuState imu;
    CompassState compass;
    GpsState gps;
    RcState rc;
    NavState nav;
};
