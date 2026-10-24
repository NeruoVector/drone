cmake_minimum_required(VERSION 3.25)
project(neurovector-drone LANGUAGES C CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- C++ HAL 라이브러리 ----------------------------------------------------
add_library(drone_hal src/drone_hal.cpp)
add_library(neurovector::drone_hal ALIAS drone_hal)

add_library(nv_mission src/mission.cpp)
add_library(neurovector::nv_mission ALIAS nv_mission)
target_include_directories(nv_mission PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
add_library(nv_avoidance src/avoidance.cpp src/window_navigation.cpp src/fast_reflex.cpp
    src/wifi_sensing.cpp src/delivery_navigation.cpp)
add_library(neurovector::nv_avoidance ALIAS nv_avoidance)
target_include_directories(nv_avoidance PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(nv_route_check app/route_check.cpp)
target_link_libraries(nv_route_check PRIVATE neurovector::nv_mission)

target_include_directories(drone_hal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ---- C 펌웨어 라이브러리 ---------------------------------------------------
add_library(nv_pwm firmware/src/nv_pwm.c)
add_library(neurovector::nv_pwm ALIAS nv_pwm)

target_include_directories(nv_pwm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/firmware/include>
    $<INSTALL_INTERFACE:include>
)

# ---- 실행 파일 -------------------------------------------------------------
add_executable(drone_demo app/main.cpp)
target_link_libraries(drone_demo PRIVATE neurovector::drone_hal neurovector::nv_pwm)
add_executable(nv_delivery_demo app/delivery_demo.cpp)
target_link_libraries(nv_delivery_demo PRIVATE neurovector::nv_avoidance)

# ---- 테스트 ---------------------------------------------------------------
option(NEUROVECTOR_BUILD_TESTS "Build unit tests" ON)

if(NEUROVECTOR_BUILD_TESTS)
    enable_testing()
    add_executable(drone_tests tests/test_drone_hal.cpp)
    target_link_libraries(drone_tests PRIVATE neurovector::drone_hal neurovector::nv_pwm)
    add_test(NAME drone_tests COMMAND drone_tests)
    add_executable(window_navigation_tests tests/test_window_navigation.cpp)
    target_link_libraries(window_navigation_tests PRIVATE neurovector::nv_avoidance)
    add_test(NAME window_navigation_tests COMMAND window_navigation_tests)
    add_executable(fast_reflex_tests tests/test_fast_reflex.cpp)
    target_link_libraries(fast_reflex_tests PRIVATE neurovector::nv_avoidance)
    add_test(NAME fast_reflex_tests COMMAND fast_reflex_tests)
    add_executable(wifi_sensing_tests tests/test_wifi_sensing.cpp)
    target_link_libraries(wifi_sensing_tests PRIVATE neurovector::nv_avoidance)
    add_test(NAME wifi_sensing_tests COMMAND wifi_sensing_tests)
    add_test(NAME delivery_navigation_scenarios COMMAND nv_delivery_demo)
endif()

# ---- 설치 ----------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS drone_hal nv_pwm drone_demo nv_mission nv_avoidance nv_route_check nv_delivery_demo
    EXPORT neurovector-drone-targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY firmware/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT neurovector-drone-targets
    FILE neurovector-drone-targets.cmake
    NAMESPACE neurovector::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/neurovector-drone
)
