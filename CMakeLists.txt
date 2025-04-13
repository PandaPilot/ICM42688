# lib/ICM42688/CMakeLists.txt

# Minimum CMake version required
cmake_minimum_required(VERSION 3.13)

# Define the library
add_library(ICM42688 STATIC
    src/ICM42688.cpp
    src/registers.h
)

# Include the header files
target_include_directories(ICM42688 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link the Pico SDK libraries
target_link_libraries(ICM42688 PUBLIC
    pico_stdlib
    hardware_i2c
)