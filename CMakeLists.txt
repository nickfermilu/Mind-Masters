cmake_minimum_required(VERSION 3.16)
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Use prebuilt pioasm (avoids needing a host C++ compiler)
set(pioasm_DIR "$ENV{USERPROFILE}/.pico-sdk/tools/2.2.0/pioasm" CACHE PATH "pioasm location")

include(pico_sdk_import.cmake)

project(brain_stimulator C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(brain_stimulator
    main.c
    dac.c
    waveform.c
    ble.c
    adc_reader.c
    pid.c
    safety.c
)

target_link_libraries(brain_stimulator
    pico_stdlib
    hardware_pio
    hardware_i2c
    hardware_timer
    hardware_gpio
    pico_cyw43_arch_none
    pico_btstack_ble
    pico_btstack_cyw43
)

# Generate GATT header from .gatt profile definition
pico_btstack_make_gatt_header(brain_stimulator PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/brain_stimulator.gatt
)

# Include project root so btstack_config.h / lwipopts.h are found
target_include_directories(brain_stimulator PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Generate PIO header from .pio program
pico_generate_pio_header(brain_stimulator ${CMAKE_CURRENT_LIST_DIR}/dac_spi.pio)

pico_enable_stdio_usb(brain_stimulator 1)
pico_enable_stdio_uart(brain_stimulator 0)
pico_add_extra_outputs(brain_stimulator)
