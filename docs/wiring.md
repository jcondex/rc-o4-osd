# Wiring

All grounds must be common.

The Pico 2 GPIO pins are not 5 V tolerant.

Pico 2 GPIO is fixed at 3.3 V. Keep external loads powered from the Pico 3V3 pin below 300 mA total.

## DJI O4 Air Unit

| Pico | O4 |
| --- | --- |
| GP0 UART0 TX | UART RX |
| GP1 UART0 RX | UART TX |
| GND | GND |

Power the O4 from a suitable supply for the exact O4 model.

For DJI O4 Air Unit:

* Red VCC: 3.7 V to 13.2 V
* BEC/supply output power: at least 10 W, for example 5 V at 2 A
* Do not power the O4 from the Pico
* If using a 1S battery, keep cable voltage drop low so the O4 input stays above 3.7 V

Use a fan or airflow on the bench. The regular O4 Air Unit has a short cold-start standby time before thermal shutdown, and can overheat or interrupt video transmission without airflow.

## RC Receiver

| Receiver | Pico |
| --- | --- |
| AUX PWM signal | GP13 |
| GND | GND |

If the receiver signal is 5 V, use a divider or level shifter before GP13.

## GPS and Compass Module

| Module | Pico |
| --- | --- |
| GPS TX | GP5 UART1 RX |
| GPS RX | GP4 UART1 TX |
| Compass SDA | GP8 I2C0 SDA |
| Compass SCL | GP9 I2C0 SCL |
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

This divider maps a fully charged 3S pack, 12.6 V, to about 2.93 V at GP26.

For a 4S LiPo:

| Part | Value |
| --- | --- |
| R1 | 47 kOhm |
| R2 | 10 kOhm |

This divider maps a fully charged 4S pack, 16.8 V, to about 2.95 V at GP26.

Wire:

```text
Battery positive -> R1 -> ADC junction -> R2 -> GND
ADC junction -> Pico GP26
Battery negative -> Pico GND
```

For best ADC behavior, connect the divider ground close to Pico AGND, pin 33, while still keeping all system grounds common.

Do not let the ADC junction exceed 3.3 V. GP26 is an ADC-capable pin with a diode to the Pico 3V3 rail, so avoid applying battery sense voltage when the Pico is unpowered.

Update `src/config.h` to match the divider and cell count.

## Calibration Trigger

Connect GP16 to a momentary switch to GND.

Hold GP16 low at boot to start compass calibration.

GP20 is reserved for a future OSD/config button and is not implemented yet.

## I2C Pullups

The BMP390 board, GY-521 board, and M100-5883 module may all have pullups fitted.

The firmware uses two 100 kHz I2C buses:

* GP6/GP7 I2C1 for BMP390 and MPU-6050
* GP8/GP9 I2C0 for the M100-5883 compass

If either bus is unstable, remove excess pullups so one pullup set remains on that bus.

Any I2C pullups must pull to 3.3 V, not 5 V.
