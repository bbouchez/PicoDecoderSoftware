cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(DCCLocoDecoder C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(DCC_LOCODECODER_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

add_executable(
        DCCLocoDecoder
		RP2040_DCC_Decoder.c
        DCCLocoDecoder_main.c)

# Multiplier to lengthen XOSC startup delay to accommodate slow-starting oscillator
target_compile_definitions(
        DCCLocoDecoder PUBLIC
        PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
)

# Add pico_multicore which is required for multicore functionality
target_link_libraries(
        DCCLocoDecoder
        pico_stdlib
        hardware_pwm
        hardware_adc
        hardware_flash
        pico_multicore
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(DCCLocoDecoder 1)
pico_enable_stdio_uart(DCCLocoDecoder 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(DCCLocoDecoder)
