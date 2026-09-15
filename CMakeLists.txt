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
set(CMAKE_PROJECT_NAME h7_framework)

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

# Collect Skywalker sources
file(GLOB_RECURSE SKYWALKER_SOURCES
    "Skywalker/BSP/Src/*.c"
    "Skywalker/Algorithm/Src/*.c"
    "Skywalker/Device/Src/*.c"
    "Skywalker/Application/Src/*.c"
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SKYWALKER_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Skywalker/BSP/Inc
    Skywalker/Algorithm/Inc
    Skywalker/Device/Inc
    Skywalker/Application/Inc
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/DSP/PrivateInclude
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# CMSIS-DSP static library
file(GLOB_RECURSE CMSIS_DSP_SOURCES
    "Drivers/CMSIS/DSP/Source/BasicMathFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/CommonTables/*.c"
    "Drivers/CMSIS/DSP/Source/ComplexMathFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/ControllerFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/FastMathFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/FilteringFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/MatrixFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/StatisticsFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/SupportFunctions/*.c"
    "Drivers/CMSIS/DSP/Source/TransformFunctions/*.c"
)
add_library(CMSIS_DSP STATIC ${CMSIS_DSP_SOURCES})
target_include_directories(CMSIS_DSP PRIVATE
    Drivers/CMSIS/DSP/Include
    Drivers/CMSIS/DSP/PrivateInclude
    Drivers/CMSIS/Include
)
target_compile_definitions(CMSIS_DSP PRIVATE
    ARM_MATH_CM7
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    CMSIS_DSP
)
