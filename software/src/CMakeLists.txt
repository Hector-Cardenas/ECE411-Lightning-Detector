# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

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
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ThunderBuddy C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ThunderBuddy ThunderBuddy.c )

pico_set_program_name(ThunderBuddy "ThunderBuddy")
pico_set_program_version(ThunderBuddy "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ThunderBuddy 1)
pico_enable_stdio_usb(ThunderBuddy 1)

# Add the standard library to the build
target_link_libraries(ThunderBuddy
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(ThunderBuddy PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(ThunderBuddy 
        hardware_spi
        hardware_i2c
        hardware_dma
        hardware_adc
        hardware_pio
        hardware_interp
        hardware_timer
        hardware_watchdog
        hardware_clocks
        pico_cyw43_arch_none
        )

pico_add_extra_outputs(ThunderBuddy)
option(PRINT "Prints debug messages to console" OFF)
option(FLASH "Flashes onboard LED when attempting to transmit" OFF)
option(FORCE "Forces transmit" OFF)
option(PRINTFLASH "Prints debug messages to console, and flashes onboard LED" OFF)
option(PRINTFORCE "Prints debug messages to console, and forces transmit" OFF)
option(DEBUG "Prints debug messages to console, flashes onboard LED, and forces transmit" OFF)

if(PRINT)
	add_definitions(-DPRINT)
endif()
if(FLASH)
	add_definitions(-DFLASH)
endif()
if(FORCE)
	add_definitions(-DFORCEINT)
endif()
if(PRINTFLASH)
	add_definitions(-DPRINTFLASH)
endif()
if(PRINTFORCE)
	add_definitions(-DPRINTFORCE)
endif()
if(DEBUG)
	add_definitions(-DDEBUG)
endif()
