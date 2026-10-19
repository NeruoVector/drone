#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "neurovector/drone_hal.hpp"
#include "nv_pwm.h"

namespace {

void test_motor_clamps_throttle() {
    nv::Motor motor("m1", 0);
    motor.arm();
    motor.set_throttle(150.0);
    assert(std::abs(motor.throttle() - 100.0) < 1e-9);
    motor.set_throttle(-10.0);
    assert(std::abs(motor.throttle() - 0.0) < 1e-9);
}

void test_disarm_resets_throttle() {
    nv::Motor motor("m2", 1);
    motor.arm();
    motor.set_throttle(80.0);
    motor.disarm();
    assert(!motor.armed());
    assert(std::abs(motor.throttle()) < 1e-9);
}

void test_throttle_requires_arm() {
    nv::DroneConfig config;
    nv::FlightController fc(config);
    fc.set_throttle(50.0);
    for (const auto& motor : fc.snapshot()) {
        assert(std::abs(motor.throttle()) < 1e-9);
    }
}

void test_mix_sets_four_motors() {
    nv::DroneConfig config;
    nv::FlightController fc(config);
    fc.arm();
    fc.set_throttle(50.0);
    fc.mix(0.0, 0.0, 0.0);
    for (const auto& motor : fc.snapshot()) {
        assert(std::abs(motor.throttle() - 50.0) < 1e-9);
    }
}

void test_pwm_requires_arm() {
    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    assert(nv_pwm_set_throttle(&pwm, 0, 50.0f) == -2);
    nv_pwm_arm(&pwm);
    assert(nv_pwm_set_throttle(&pwm, 0, 50.0f) == 0);
    assert(nv_pwm_pulse_us(&pwm, 0) == 1500);
}

void test_pwm_rejects_bad_channel() {
    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    nv_pwm_arm(&pwm);
    assert(nv_pwm_set_throttle(&pwm, NV_PWM_CHANNELS, 10.0f) == -1);
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
