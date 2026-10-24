#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

#include "neurovector/drone_hal.hpp"
#include "nv_pwm.h"

namespace {
void require(bool condition) {
    if (!condition) throw std::runtime_error("HAL test requirement failed");
}

void test_motor_clamps_throttle() {
    nv::Motor motor("m1", 0);
    motor.arm();
    motor.set_throttle(150.0);
    require(std::abs(motor.throttle() - 100.0) < 1e-9);
    motor.set_throttle(-10.0);
    require(std::abs(motor.throttle() - 0.0) < 1e-9);
}

void test_disarm_resets_throttle() {
    nv::Motor motor("m2", 1);
    motor.arm();
    motor.set_throttle(80.0);
    motor.disarm();
    require(!motor.armed());
    require(std::abs(motor.throttle()) < 1e-9);
}

void test_throttle_requires_arm() {
    nv::DroneConfig config;
    nv::FlightController fc(config);
    fc.set_throttle(50.0);
    for (const auto& motor : fc.snapshot()) {
        require(std::abs(motor.throttle()) < 1e-9);
    }
}

void test_mix_sets_four_motors() {
    nv::DroneConfig config;
    nv::FlightController fc(config);
    fc.arm();
    fc.set_throttle(50.0);
    fc.mix(0.0, 0.0, 0.0);
    for (const auto& motor : fc.snapshot()) {
        require(std::abs(motor.throttle() - 50.0) < 1e-9);
    }
}

void test_pwm_requires_arm() {
    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    const int unarmed = nv_pwm_set_throttle(&pwm, 0, 50.0f);
    require(unarmed == -2);
    nv_pwm_arm(&pwm);
    const int armed = nv_pwm_set_throttle(&pwm, 0, 50.0f);
    require(armed == 0);
    require(nv_pwm_pulse_us(&pwm, 0) == 1500);
}

void test_pwm_rejects_bad_channel() {
    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    nv_pwm_arm(&pwm);
    const int invalid = nv_pwm_set_throttle(&pwm, NV_PWM_CHANNELS, 10.0f);
    require(invalid == -1);
}

}  // namespace

int main() {
    test_motor_clamps_throttle();
    test_disarm_resets_throttle();
    test_throttle_requires_arm();
    test_mix_sets_four_motors();
    test_pwm_requires_arm();
    test_pwm_rejects_bad_channel();

    std::cout << "all drone tests passed\n";
    return EXIT_SUCCESS;
}
