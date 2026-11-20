cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(robotic_arm_bridge)

# Build every .c / .cpp file in ../src
FILE(GLOB app_sources ../src/*.c ../src/*.cpp)
target_sources(app PRIVATE ${app_sources})
