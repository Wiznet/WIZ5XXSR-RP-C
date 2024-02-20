# CMake minimum required version
cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
if(NOT DEFINED PICO_SDK_PATH)
    set(PICO_SDK_PATH ${CMAKE_SOURCE_DIR}/libraries/pico-sdk)
    message(STATUS "PICO_SDK_PATH = ${PICO_SDK_PATH}")
endif()

include(WIZ5XXSR-RP_c_sdk_version.cmake)
include(WIZ5XXSR-RP_c-patch.cmake)
include(pico_sdk_import.cmake)

# Set project name
set(PROJECT_NAME WIZ5XXSR-RP)

# Set project informations
project(${PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()


set(WIZNET_CHIP W5100S)
#set(WIZNET_CHIP W5500)

if(${WIZNET_CHIP} STREQUAL W5100S)
    add_definitions(-D_WIZCHIP_=W5100S)
elseif(${WIZNET_CHIP} STREQUAL W5500)
    add_definitions(-D_WIZCHIP_=W5500)
else()
    message(FATAL_ERROR "WIZNET_CHIP is wrong = ${WIZNET_CHIP}")
endif()

message(STATUS "WIZNET_CHIP = ${WIZNET_CHIP}")
# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED BOOT_PORT_DIR)
    set(BOOT_PORT_DIR ${CMAKE_SOURCE_DIR}/port/boot)
    message(STATUS "BOOT_PORT_DIR = ${BOOT_PORT_DIR}")
endif()

if(NOT DEFINED APP_PORT_DIR)
    set(APP_PORT_DIR ${CMAKE_SOURCE_DIR}/port/app)
    message(STATUS "APP_PORT_DIR = ${APP_PORT_DIR}")
endif()

# Turn off mbedtls test mode 
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${APP_PORT_DIR}/mbedtls/inc/SSLConfig.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)
add_definitions(-DPICO_FLASH_SPI_CLKDIV=4)

# Add libraries in subdirectories
add_subdirectory(App)
add_subdirectory(Boot)
add_subdirectory(${MBEDTLS_DIR})
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
add_subdirectory(${BOOT_PORT_DIR})
add_subdirectory(${APP_PORT_DIR})

# Set compile options
add_compile_options(
        -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
