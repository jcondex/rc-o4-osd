# Configuration

Edit [src/config.h](../src/config.h) before building.

## Battery

Default configuration is for a 3S LiPo:

```c
#define BAT_R1_OHMS      33000.0f
#define BAT_R2_OHMS      10000.0f
#define BAT_CELLS        3
#define BAT_WARN_VOLTS   10.5f
```

For 4S, use a divider suitable for 16.8 V and update the warning voltage.

Measure the real pack voltage and tune `BAT_CAL_FACTOR` if needed.

## Compass

Set magnetic declination for your area:

```c
#define MAG_DECLINATION 0.0f
```

Hold GP22 low at boot to run compass calibration.

Calibration data is stored in the last flash sector.

## GPS

The GPS starts at 9600 baud.

The firmware sends UBX commands to set 5 Hz output and 115200 baud. If the module does not ACK, it falls back to 9600 baud.

GPS fix data expires after:

```c
#define GPS_FIX_TIMEOUT_MS 1500
```

## I2C

The shared I2C bus runs at 100 kHz.

Do not change this to 400 kHz unless the pullups on the sensor boards have been checked.

## MSP

`MSP_RAW_GPS_PROACTIVE` controls proactive GPS packets:

```c
#define MSP_RAW_GPS_PROACTIVE 1
```

Use 0 if the O4 setup behaves better with request only GPS.

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

USB serial debug output is off by default.

```c
#define DEBUG_USB_SERIAL 0
#define DEBUG_PRINT_HZ   1
```

Set `DEBUG_USB_SERIAL` to 1 during bench testing.

## Units

Metric is the default.

```c
#define USE_IMPERIAL 0
```
