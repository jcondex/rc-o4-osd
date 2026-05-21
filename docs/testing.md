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

The debug line includes MSP RX/TX packet counts, MSP checksum and unknown command counts, GPS sentence and checksum counts, and IMU/baro/compass I2C failure counts.

## Stage 1: PWM

* Connect the receiver AUX PWM signal to GP13
* Confirm the debug `pwm=` value is near 1000 us at low switch
* Confirm the debug `pwm=` value is near 1500 us at center switch
* Confirm the debug `pwm=` value is near 2000 us at high switch
* Confirm high switch above 1700 us arms
* Confirm low switch below 1300 us disarms
* Remove signal and confirm failsafe after 250 ms
* Restore signal while the switch is still high and confirm it stays disarmed/failsafe latched
* Move the switch low and confirm it returns to disarmed
* Move the switch high again and confirm it can rearm

## Stage 2: DJI O4 MSP

* Connect UART0 to the O4 MSP UART
* Power the O4 with cooling
* For DJI O4 Air Unit, use a 3.7 V to 13.2 V supply capable of at least 10 W, for example 5 V at 2 A
* Optional: set `OSD_SYMBOL_TEST_MODE` to 1 and confirm Betaflight arrow/home symbols render correctly, then set it back to 0
* Confirm `msp_rx=` increases when the O4 queries MSP
* Confirm `msp_tx=` increases when the firmware responds
* Confirm `msp_crc=` stays at 0 during normal operation
* Confirm the goggles see `RC-O4-OSD`
* Confirm Canvas Mode OSD appears
* Confirm ARM shows armed and disarmed
* Confirm failsafe reports disarmed
* Confirm the battery bar responds to MSP_ANALOG

If the battery bar does not work, test alternate battery payload sizes before changing the default.

## Stage 3: Battery

* Measure pack voltage with a meter
* Compare with OSD VBAT
* Adjust divider values or `BAT_CAL_FACTOR`
* Confirm the displayed voltage is stable through throttle/load changes

## Stage 4: BMP390

* Confirm pressure, temperature, altitude, and vertical speed update
* Arm and confirm altitude resets to zero
* Disconnect the sensor and confirm ALT ERR
* Reconnect the sensor and confirm recovery

## Stage 5: GPS

* Confirm the GPS is running at 115200 baud
* Confirm whether the GPS is outputting at its documented 10 Hz default or the firmware-configured 5 Hz rate
* Confirm NMEA sentences are present after startup
* If only UBX binary data is present, confirm `gps_ubx=` and `gps_pvt=` increase
* Confirm UBX config ACK behavior if supported
* Confirm fix, satellites, speed, course, and altitude
* Arm with GPS fix and confirm home is set
* Confirm the OSD shows home distance and bearing when armed with a GPS fix
* Move away from home and confirm the bearing changes plausibly
* Arm without GPS fix and confirm NO GPS HOME
* Confirm trip distance rejects large jumps

## Stage 6: MPU-6050

* Confirm pitch and roll move in the expected direction
* Optional: set `OSD_SHOW_IMU_DEBUG` to 1 and confirm raw gyro/accelerometer rows change when the vehicle is moved
* Confirm MSP_ATTITUDE changes
* Confirm active sensor bits include gyro and accelerometer
* If signs are wrong, adjust the IMU sign defines in `src/config.h`

## Stage 7: QMC5883L

* Hold GP16 low at boot
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
