cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_FETCH_FROM_GIT on)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mainsdata C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(mainsdata
	regression.cpp
	statemachine.cpp
	rp2040sampler.cpp)

target_include_directories(mainsdata PUBLIC /opt/pico-sdk/lib/lwip/src/include ./ /opt/pico-sdk/src/rp2_common/pico_lwip/include /opt/pico-sdk/src/rp2_common/pico_rand/include/ /opt/pico-sdk/src/rp2_common/pico_cyw43_arch/include/ /opt/pico-sdk/lib/cyw43-driver/src/ /opt/pico-sdk/src/rp2_common/pico_cyw43_driver/include /opt/pico-sdk/src/rp2_common/pico_async_context/include)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

pico_add_extra_outputs(mainsdata)

target_link_libraries(mainsdata
	pico_lwip_mqtt
	pico_stdlib
        pico_multicore
	hardware_gpio
	FreeRTOS)
