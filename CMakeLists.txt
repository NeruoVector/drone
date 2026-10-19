cmake_minimum_required(VERSION 3.25)
project(neurovector-drone LANGUAGES C CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---- C++ HAL 라이브러리 ----------------------------------------------------
add_library(drone_hal src/drone_hal.cpp)
add_library(neurovector::drone_hal ALIAS drone_hal)

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

# ---- 테스트 ---------------------------------------------------------------
option(NEUROVECTOR_BUILD_TESTS "Build unit tests" ON)

if(NEUROVECTOR_BUILD_TESTS)
    enable_testing()
    add_executable(drone_tests tests/test_drone_hal.cpp)
    target_link_libraries(drone_tests PRIVATE neurovector::drone_hal neurovector::nv_pwm)
    add_test(NAME drone_tests COMMAND drone_tests)
endif()

# ---- 설치 ----------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS drone_hal nv_pwm drone_demo
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
