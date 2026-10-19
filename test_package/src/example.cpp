#include <iostream>

#include "neurovector/drone_hal.hpp"
#include "nv_pwm.h"

int main() {
    nv::FlightController fc(nv::DroneConfig{});
    fc.arm();
    fc.set_throttle(25.0);
    std::cout << "test_package ok: " << fc.config().id << "\n";

    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    std::cout << "pwm min = " << nv_pwm_pulse_us(&pwm, 0) << "us\n";
    return 0;
}
