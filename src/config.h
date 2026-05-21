#pragma once

#ifndef RC_O4_OSD_HOST_TEST
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#endif

#define PIN_UART0_TX             0
#define PIN_UART0_RX             1
#define PIN_UART1_TX             4
#define PIN_UART1_RX             5
#define PIN_SENSOR_SDA           6
#define PIN_SENSOR_SCL           7
#define PIN_COMPASS_SDA          8
#define PIN_COMPASS_SCL          9
#define PIN_PWM_IN               13
#define PIN_CAL_TRIGGER          16
#define PIN_OSD_CONFIG_BUTTON    20
#define PIN_LED                  25
#define PIN_BAT_ADC              26

#define UART_O4_BAUD             115200
#define UART_GPS_BAUD_INIT       115200
#define UART_GPS_BAUD_FAST       115200
#define UART_GPS_UPDATE_HZ       5
#define GPS_UART_RX_BUFFER_SIZE  2048
#define GPS_UBX_STARTUP_CONFIG   0
#define GPS_FIX_TIMEOUT_MS       1500
#define GPS_LINK_TIMEOUT_MS      3000

#define SENSOR_I2C_PORT          i2c1
#define SENSOR_I2C_SPEED_HZ      100000
#define COMPASS_I2C_PORT         i2c0
#define COMPASS_I2C_SPEED_HZ     100000
#define BMP390_ADDR_PRIMARY      0x77
#define BMP390_ADDR_ALT          0x76
#define QMC5883L_ADDR            0x0D
#define MPU6050_ADDR             0x68

#define PWM_MIN_US               800
#define PWM_MAX_US               2200
#define PWM_ARM_THRESHOLD        1700
#define PWM_DISARM_THRESHOLD     1300
#define PWM_FAILSAFE_MS          250
#define PWM_DEBUG_INPUT_PULLUP   0
#define PWM_SELF_TEST_OUTPUT     0
#define PWM_SELF_TEST_STATIC_HIGH 0
#define PIN_PWM_SELF_TEST        14

#define OSD_COLS                 30
#define OSD_ROWS                 16
#define OSD_CANVAS_COLS          60
#define OSD_CANVAS_ROWS          22
#define OSD_UPDATE_HZ            10
#define OSD_DISPLAYPORT_ENABLE   1
#define OSD_DISPLAYPORT_FONT     0
// ArduPilot/DJI MSP DisplayPort option: 0=30x16, 1=50x18, 2=60x22.
// DJI O4/Goggles 3 currently render most predictably when all fields stay
// inside the 30x16-safe area.
#define OSD_DISPLAYPORT_RESOLUTION 0
#define OSD_SYMBOL_TEST_MODE     0
#define OSD_SHOW_ARM_GPS         1
#define OSD_SHOW_BATTERY_PWM     1
#define OSD_SHOW_ALTITUDE        1
#define OSD_SHOW_SPEED           1
#define OSD_SHOW_HOME_ARROW      1
#define OSD_SHOW_HEADING_TEMP    1
#define OSD_SHOW_TIME_TRIP       0
#define OSD_SHOW_WARNINGS        1
#define OSD_SHOW_ATTITUDE        1
#define OSD_SHOW_IMU_DEBUG       0
#define OSD_COCKPIT_MODE         1
#define OSD_COCKPIT_CENTER_COL   26
#define OSD_COCKPIT_CENTER_ROW   10
#define OSD_COCKPIT_HALF_WIDTH   11
#define OSD_COCKPIT_CENTER_GAP_HALF_WIDTH 0
#define OSD_COCKPIT_ROLL_DEG_PER_ROW_AT_EDGE 15.0f
#define OSD_COCKPIT_PITCH_DEG_PER_ROW 8.0f
#define OSD_COCKPIT_MAX_PITCH_DEG 24.0f
#define OSD_COCKPIT_MAX_ROLL_DEG 60.0f
#define OSD_BATTERY_MIN_VALID_VOLTS 3.3f
#define MSP_RAW_GPS_PROACTIVE    1

#define BAT_R1_OHMS              33000.0f
#define BAT_R2_OHMS              10000.0f
#define BAT_CELLS                0
#define BAT_CAL_FACTOR           1.0f
#define BAT_DIVIDER_RATIO        ((BAT_R1_OHMS + BAT_R2_OHMS) / BAT_R2_OHMS)

#define QMC_X_OFFSET             0.0f
#define QMC_Y_OFFSET             0.0f
#define QMC_Z_OFFSET             0.0f
#define QMC_X_SCALE              1.0f
#define QMC_Y_SCALE              1.0f
#define QMC_Z_SCALE              1.0f
#define MAG_DECLINATION          0.0f
#define HDG_GPS_MIN_KMH          5.0f

#define GPS_MAX_JUMP_M           50.0f
#define GPS_MAX_SPEED_KMH        250.0f

#define BMP390_SAMPLE_HZ         25
#define BMP390_DEBUG_REGS        0
#define BARO_SMOOTH_ALPHA        0.10f
#define VS_SMOOTH_ALPHA          0.05f
#define SENSOR_STALE_MS          500
#define SENSOR_REPROBE_MS        1500

#define MPU6050_SAMPLE_HZ        100
#define IMU_COMP_FILTER_ALPHA    0.98f
#define IMU_ACCEL_X_SIGN         1.0f
#define IMU_ACCEL_Y_SIGN         1.0f
#define IMU_ACCEL_Z_SIGN         1.0f
#define IMU_GYRO_X_SIGN          1.0f
#define IMU_GYRO_Y_SIGN          1.0f
#define IMU_GYRO_Z_SIGN          1.0f
#define IMU_PITCH_SIGN           1.0f
#define IMU_ROLL_SIGN            1.0f

#define WATCHDOG_TIMEOUT_MS      500
#define DEBUG_USB_SERIAL         1
#define DEBUG_PRINT_HZ           1

#define FLASH_CAL_MAGIC          0xCAFEBABEu
#define FLASH_CAL_OFFSET         (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

#define USE_IMPERIAL             0
#define SHOW_GPS_ALTITUDE        0
