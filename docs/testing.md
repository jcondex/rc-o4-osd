# Testing

Run host tests first.

```sh
tests/run_host_tests.sh
```

Then test hardware in stages.

For bench work, enable USB serial debug output in `src/config.h`:

```c
#define DEBUG_USB_SERIAL 1
```

## Stage 1: PWM

* Connect the receiver AUX PWM signal to GP15
* Confirm low, middle, and high switch pulse widths
* Confirm high arms
* Confirm low disarms
* Remove signal and confirm failsafe after 250 ms
* Restore signal low and confirm it returns to disarmed

## Stage 2: DJI O4 MSP

* Connect UART0 to the O4 MSP UART
* Power the O4 with cooling
* Confirm the goggles see `RC-O4-OSD`
* Confirm ARM shows armed and disarmed
* Confirm failsafe reports disarmed
* Confirm the battery bar responds to MSP_ANALOG

If the battery bar does not work, test alternate battery payload sizes before changing the default.

## Stage 3: Battery

* Measure pack voltage with a meter
* Compare with OSD VBAT
* Adjust divider values or `BAT_CAL_FACTOR`
* Confirm LOW BATTERY appears at the configured voltage

## Stage 4: BMP390

* Confirm pressure, temperature, altitude, and vertical speed update
* Arm and confirm altitude resets to zero
* Disconnect the sensor and confirm ALT ERR
* Reconnect the sensor and confirm recovery

## Stage 5: GPS

* Confirm NMEA is parsed at 9600 baud
* Confirm UBX config switches to 115200 baud if supported
* Confirm fix, satellites, speed, course, and altitude
* Arm with GPS fix and confirm home is set
* Arm without GPS fix and confirm NO GPS HOME
* Confirm trip distance rejects large jumps

## Stage 6: MPU-6050

* Confirm pitch and roll move in the expected direction
* Confirm MSP_ATTITUDE changes
* Confirm active sensor bits include gyro and accelerometer
* If signs are wrong, adjust axis mapping in `src/imu_mpu6050.cpp`

## Stage 7: QMC5883L

* Hold GP22 low at boot
* Rotate the vehicle slowly through 360 degrees
* Confirm calibration is saved
* Confirm heading source is MAG while stationary
* Confirm heading source is GPS above 5 km/h

## Final Test

* Run the full OSD layout
* Check all warnings
* Check sensor unplug and reconnect behavior
* Check watchdog reset returns disarmed
* Check O4 operation with cooling
