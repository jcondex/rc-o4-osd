# Configuration

Edit [src/config.h](../src/config.h) before building.

## Battery

Default voltage measurement uses this divider:

```c
#define BAT_R1_OHMS      33000.0f
#define BAT_R2_OHMS      10000.0f
#define BAT_CELLS        0
```

`BAT_CELLS` is set to 0 because the firmware does not infer pack cell count from voltage.

For 4S, use a divider suitable for 16.8 V.

Measure the real pack voltage and tune `BAT_CAL_FACTOR` if needed.

## Compass

Set magnetic declination for your area:

```c
#define MAG_DECLINATION 0.0f
```

Hold GP16 low at boot to run compass calibration.

Calibration data is stored in the last flash sector.

## GPS

The HGLRC M100-5883 GPS manual lists 115200 baud and 10 Hz navigation update as defaults.

The firmware starts UART1 at 115200 baud. Startup UBX reconfiguration is currently disabled with:

```c
#define GPS_UBX_STARTUP_CONFIG 0
```

The parser accepts NMEA RMC/GGA/VTG sentences and UBX NAV-PVT frames.

GPS fix data expires after:

```c
#define GPS_FIX_TIMEOUT_MS 1500
```

## I2C

The sensor I2C buses run at 100 kHz.

* `i2c1` on GP6/GP7 is for BMP390 and MPU-6050.
* `i2c0` on GP8/GP9 is for the M100-5883 compass.

Do not change these to 400 kHz unless the pullups on the sensor boards have been checked.

## MSP

`MSP_RAW_GPS_PROACTIVE` controls proactive GPS packets:

```c
#define MSP_RAW_GPS_PROACTIVE 1
```

Use 0 if the O4 setup behaves better with request only GPS.

## OSD

The normal OSD layout is controlled with compile-time switches in `src/config.h`:

```c
#define OSD_SHOW_HOME_ARROW 1
#define OSD_SHOW_ATTITUDE   1
#define OSD_SHOW_IMU_DEBUG  0
```

Set `OSD_SHOW_IMU_DEBUG` to 1 during bench testing to add raw MPU-6050 gyro and accelerometer rows.

To verify DJI O4/Goggles 3 render Betaflight symbol bytes correctly, temporarily enable:

```c
#define OSD_SYMBOL_TEST_MODE 1
```

This replaces the normal OSD with a symbol test page showing the Betaflight arrow glyph range, home flag, over-home symbol, and no-home symbol. Set it back to 0 before normal driving tests.

## Sensors

Sensor data expires after:

```c
#define SENSOR_STALE_MS 500
```

Missing sensors are retried every:

```c
#define SENSOR_REPROBE_MS 1500
```

This lets the firmware keep running when a barometer, IMU, or compass is unplugged and recover after it is plugged back in.

## IMU Mounting

The MPU-6050 axis signs can be changed in `src/config.h`:

```c
#define IMU_ACCEL_X_SIGN  1.0f
#define IMU_ACCEL_Y_SIGN  1.0f
#define IMU_ACCEL_Z_SIGN  1.0f
#define IMU_GYRO_X_SIGN   1.0f
#define IMU_GYRO_Y_SIGN   1.0f
#define IMU_GYRO_Z_SIGN   1.0f
#define IMU_PITCH_SIGN    1.0f
#define IMU_ROLL_SIGN     1.0f
```

Use `-1.0f` for an axis that moves the wrong way.

## Debug Output

USB serial debug output is currently enabled for bench testing.

```c
#define DEBUG_USB_SERIAL 1
#define DEBUG_PRINT_HZ   1
```

Set `DEBUG_USB_SERIAL` to 0 for quieter release-style builds.

## Units

Metric is the default.

```c
#define USE_IMPERIAL 0
```
