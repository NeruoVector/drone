cmake_minimum_required(VERSION 3.25)
project(neurovector-drone-test-package LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(neurovector-drone REQUIRED)

add_executable(example src/example.cpp)
target_link_libraries(example PRIVATE neurovector::drone_hal neurovector::nv_pwm neurovector::nv_avoidance)
