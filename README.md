# RC O4 OSD

RC O4 OSD is firmware for a Raspberry Pi Pico 2 that presents a small Betaflight compatible MSP telemetry device to a DJI O4 Air Unit.

It is for RC vehicle camera and telemetry use. It is not a flight controller and it does not drive motors or servos.

Copyright 2025.

[![Host Tests](https://github.com/jdelcond/rc-o4-osd/actions/workflows/host-tests.yml/badge.svg)](https://github.com/jdelcond/rc-o4-osd/actions/workflows/host-tests.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)

## Features

* MSP v1 parser and responder for DJI O4
* Betaflight identity responses for O4 Canvas Mode
* MSP DisplayPort OSD output on a 30 x 16 canvas
* PWM arm channel input using RP2350 PIO
* Failsafe disarm on PWM signal loss
* GPS parsing and MSP_RAW_GPS output
* BMP390 barometer support
* MPU-6050 attitude support
* QMC5883L compass support with flash calibration
* Battery voltage ADC support
* Hardware watchdog
* Host tests for MSP, GPS, baro math, compass calibration, flash storage, OSD frames, and state transitions

## Hardware

Target board:

* Raspberry Pi Pico 2, RP2350

Tested design targets:

* DJI O4 Air Unit
* DJI Goggles 3
* PWM RC receiver
* Adafruit BMP390
* HGLRC M100-5883 GPS with QMC5883L compass
* HiLetgo GY-521 MPU-6050
* Battery voltage divider on ADC0

Pin map:

| Pico pin | Function |
| --- | --- |
| GP0 | UART0 TX to DJI O4 RX |
| GP1 | UART0 RX from DJI O4 TX |
| GP4 | UART1 TX to GPS RX |
| GP5 | UART1 RX from GPS TX |
| GP6 | I2C1 SDA |
| GP7 | I2C1 SCL |
| GP15 | RC PWM input |
| GP22 | Compass calibration trigger |
| GP25 | Onboard LED |
| GP26 | Battery ADC input |

See [docs/wiring.md](docs/wiring.md) for wiring notes.

## Status

The firmware builds and the host test suite passes.

Hardware validation is still required for:

* DJI O4 handshake behavior
* DisplayPort timing on real goggles
* Battery widget payload size
* GPS baud change and UBX ACK handling
* Sensor hot plug behavior
* Compass and IMU mounting orientation

## Build

Requirements:

* Pico SDK 2.x
* CMake 3.28 or newer
* Arm GNU toolchain for RP2350
* C++17 compiler

Build:

```sh
cmake -S . -B build -DPICO_BOARD=pico2
cmake --build build
```

The UF2 file is written to:

```text
build/rc-o4-osd.uf2
```

## Host Tests

Host tests do not require the Pico SDK.

```sh
tests/run_host_tests.sh
```

These tests cover packet formats and pure logic only. They do not replace bench testing with the receiver, sensors, GPS, or DJI O4 hardware.

## Configuration

Main settings are in [src/config.h](src/config.h).

Common settings to check before powering hardware:

* Battery divider values
* Battery cell count
* Low battery warning voltage
* Magnetic declination
* GPS proactive MSP_RAW_GPS setting
* I2C sensor addresses

The I2C bus is set to 100 kHz because the sensor boards commonly have pullups fitted.

## Safety

This firmware always boots disarmed.

PWM signal loss forces failsafe and reports disarmed over MSP. After failsafe, the arm switch must return low before arming is allowed again.

Pico GPIO is 3.3 V only. Use a divider or level shifter if the receiver PWM output is 5 V.

Keep the DJI O4 Air Unit cooled during bench testing.

## Documentation

* [Wiring](docs/wiring.md)
* [Testing](docs/testing.md)
* [MSP notes](docs/msp.md)
* [Configuration](docs/configuration.md)
* [Contributing](CONTRIBUTING.md)
* [Code of Conduct](CODE_OF_CONDUCT.md)
* [Security](SECURITY.md)

## License

This project is licensed under the GNU General Public License version 3.

See [LICENSE](LICENSE).
