cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME L4)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/task.c
    GwLib/Src/gwmath.c
    GwLib/Src/i2c.c
    GwLib/Src/kalman.c
    GwLib/Src/pid.c
    Hardware/eMPL/Src/inv_mpu_dmp_motion_driver.c
    Hardware/eMPL/Src/inv_mpu.c
    Hardware/Src/anotc.c
    Hardware/Src/icm20608.c
    Hardware/Src/imu.c
    Hardware/Src/led.c
    Hardware/Src/state.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    GwLib/Inc
    Hardware/Inc
    Hardware/eMPL/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    __GW_USE_CMATH__
    MPU6050
    EMPL_TARGET_STM32
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)

# Add script to convert ELF to HEX
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND arm-none-eabi-objcopy -O ihex ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Converting ELF to HEX"
)
add_custom_target(ELF2HEX ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)