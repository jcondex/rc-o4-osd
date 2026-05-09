# Wiring

All grounds must be common.

The Pico 2 GPIO pins are not 5 V tolerant.

## DJI O4 Air Unit

| Pico | O4 |
| --- | --- |
| GP0 UART0 TX | UART RX |
| GP1 UART0 RX | UART TX |
| GND | GND |

Power the O4 from a suitable supply for the exact O4 model.

Use a fan or airflow on the bench.

## RC Receiver

| Receiver | Pico |
| --- | --- |
| AUX PWM signal | GP15 |
| GND | GND |

If the receiver signal is 5 V, use a divider or level shifter before GP15.

## GPS and Compass Module

| Module | Pico |
| --- | --- |
| GPS TX | GP5 UART1 RX |
| GPS RX | GP4 UART1 TX |
| SDA | GP6 I2C1 SDA |
| SCL | GP7 I2C1 SCL |
| GND | GND |

Check the module voltage requirements before connecting VCC.

## BMP390

| BMP390 | Pico |
| --- | --- |
| VIN | 3V3 |
| GND | GND |
| SDA | GP6 |
| SCL | GP7 |

## MPU-6050 GY-521

| GY-521 | Pico |
| --- | --- |
| VCC | 3V3 |
| GND | GND |
| SDA | GP6 |
| SCL | GP7 |
| AD0 | GND |
| INT | Not connected |
| XDA | Not connected |
| XCL | Not connected |

## Battery Divider

For a 3S LiPo:

| Part | Value |
| --- | --- |
| R1 | 33 kOhm |
| R2 | 10 kOhm |

For a 4S LiPo:

| Part | Value |
| --- | --- |
| R1 | 47 kOhm |
| R2 | 10 kOhm |

Wire:

```text
Battery positive -> R1 -> ADC junction -> R2 -> GND
ADC junction -> Pico GP26
Battery negative -> Pico GND
```

Update `src/config.h` to match the divider and cell count.

## Calibration Trigger

Connect GP22 to a momentary switch to GND.

Hold GP22 low at boot to start compass calibration.

## I2C Pullups

The BMP390 board, GY-521 board, and M100-5883 module may all have pullups fitted.

The firmware uses 100 kHz I2C. If the bus is unstable, remove pullups from one or two boards so one pullup set remains.
