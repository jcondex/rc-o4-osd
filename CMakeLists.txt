# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.28)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rc-o4-osd C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name.

add_executable(rc-o4-osd
        src/main.cpp
        src/battery.cpp
        src/bmp390.cpp
        src/calibration_flash.cpp
        src/compass_qmc5883l.cpp
        src/displayport.cpp
        src/gps.cpp
        src/gps_uart.cpp
        src/imu_mpu6050.cpp
        src/msp.cpp
        src/msp_osd_config.cpp
        src/nav.cpp
        src/pwm_self_test.cpp
        src/rc_pwm_pio.cpp
        src/sensor_manager.cpp
        src/state_machine.cpp
        src/telemetry_debug.cpp
)

pico_set_program_name(rc-o4-osd "rc-o4-osd")
pico_set_program_version(rc-o4-osd "2025.1")

# Generate PIO header
pico_generate_pio_header(rc-o4-osd ${CMAKE_CURRENT_LIST_DIR}/src/rc_pwm.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rc-o4-osd 0)
pico_enable_stdio_usb(rc-o4-osd 1)

# Add the standard library to the build
target_link_libraries(rc-o4-osd
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(rc-o4-osd PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Add any user requested libraries
target_link_libraries(rc-o4-osd 
        hardware_adc
        hardware_flash
        hardware_i2c
        hardware_pio
        hardware_pwm
        hardware_sync
        hardware_uart
        hardware_watchdog
        )

pico_add_extra_outputs(rc-o4-osd)
