cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Compiler options
set(STM32_MCU_FLAGS  "-mcpu=cortex-m55 -mfpu=fpv5-d16 -mfloat-abi=hard -mcmse " )

#Linker options
set(STM32_LINKER_SCRIPT STM32N657XX_LRUN.ld)
set(STM32_LINKER_OPTION  )

# Set the project name
set(CMAKE_PROJECT_NAME magnecko-sense-fw_Appli)
project(${CMAKE_PROJECT_NAME})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Include mx-generated
include("mx-generated.cmake")

# Board identity is also used by middleware that consumes application headers.
target_include_directories(stm32cubemx INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../common/board")


# In order to use CMake for cross-compiling
set(CMAKE_CXX_COMPILER_FORCED "true")

# Add project symbols (macros)
option(DEV_MODE "Build application for direct SRAM development" OFF)
option(MAGNECKO_DHCP "Try DHCP before the 30-second static IPv4 fallback" ON)
target_compile_definitions(${PROJECT_NAME} PRIVATE MAGNECKO_DHCP=$<BOOL:${MAGNECKO_DHCP}>)
target_sources(netxduo PRIVATE ../Middlewares/ST/netxduo/addons/dhcp/nxd_dhcp_client.c)
target_include_directories(netxduo PUBLIC ../Middlewares/ST/netxduo/addons/dhcp)
target_compile_definitions(netxduo PUBLIC NX_DHCP_CLIENT_USER_CREATE_PACKET_POOL NX_DHCP_CLIENT_SEND_ARP_PROBE)
target_sources(netxduo PRIVATE ../Middlewares/ST/netxduo/addons/mdns/nxd_mdns.c)
target_include_directories(netxduo PUBLIC ../Middlewares/ST/netxduo/addons/mdns)
target_compile_definitions(netxduo PUBLIC NX_MDNS_DISABLE_CLIENT)
# Optimize the complete application and network stack in Release builds.
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    foreach(release_target ${CMAKE_PROJECT_NAME} STM32_Drivers netxduo threadx usbx STM32_USBPD_Library)
        target_compile_options(${release_target} PRIVATE $<$<COMPILE_LANGUAGE:C>:-O3>)
    endforeach()
endif()

option(VL53L9_SENSOR_ONLY "Run VL53L9 without DCMIPP capture for fault isolation" OFF)
if(VL53L9_SENSOR_ONLY)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE VL53L9_SENSOR_ONLY)
endif()

if(DEV_MODE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DEV_MODE)
endif()


target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ETH_PHY_1000MBITS_SUPPORTED
)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ./Libs/lsm6dsv16x-pid
    ./Libs/IMU
    ./Libs/53L9/app
    ./Libs/53L9/interface/vl53l9
    ./Libs/53L9/vl53l9
    ./Libs/PHY
    ./Libs/control
    ../common/led
    ../common/s28hs512t

)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ./Libs/lsm6dsv16x-pid/lsm6dsv16x_reg.c
    ./Libs/IMU/IMU.c
    ./Libs/IMU/platform.c
    ./Libs/53L9/interface/platform/platform_utils.c
    ./Libs/53L9/interface/vl53l9/vl53l9_device.c
    ./Libs/53L9/interface/vl53l9/vl53l9_platform.c
    ./Libs/53L9/interface/vl53l9/vl53l9_utils.c
    ./Libs/53L9/vl53l9/vl53l9.c
    ./Libs/53L9/app/VL53L9_process.c
    ./Libs/PHY/ADIN1300.c
    ./Libs/control/control_server.c
    ./Libs/control/housekeeping.c
    ./Libs/control/stream_manager.c
    ./Libs/control/status_led.c
    ./Libs/control/boot_diagnostics.c
    ./Libs/control/sensor_clock.c
    ./Libs/control/imu_stream.c
    ./Libs/control/imu_control.c
    ./Libs/control/ethernet_network.c
    ./Libs/control/ethernet_mdns.c
    ./Libs/control/ethernet_send.c
    ../common/s28hs512t/semper_flash.c
    ../common/s28hs512t/s28hs512t.c
    ./Libs/control/device_parameters.c
    ../common/firmware/fw_parameters.c
    ../common/firmware/fw_parameters_json.c
    ./Libs/control/tof_control.c
    ./Libs/control/thread_profiler.c
    ./Libs/control/ethernet_stream.c
    ./Libs/control/ethernet_link.c
    ./Libs/control/app_events.c
    ../common/led/LEDController.c
    ./Core/Src/app_clock.c
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    # Add user defined libraries
    stm32cubemx
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()









