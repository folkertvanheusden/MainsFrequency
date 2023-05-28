cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT on)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mainsdata C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_definitions(CYW43_ARCH_DEBUG_ENABLED)

set(PICO_BOARD pico_w)

pico_sdk_init()

add_executable(mainsdata
	regression.cpp
	statemachine.cpp
	rp2040sampler.cpp)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

target_include_directories(mainsdata PUBLIC .)

pico_add_extra_outputs(mainsdata)

target_link_libraries(mainsdata
	hardware_adc
	pico_lwip_mqtt
	pico_cyw43_arch_lwip_threadsafe_background
	pico_stdlib
        pico_multicore
	hardware_gpio)
