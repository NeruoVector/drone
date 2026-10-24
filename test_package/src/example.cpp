#include <iostream>

#include "neurovector/drone_hal.hpp"
#include "nv_pwm.h"
#include "neurovector/delivery_navigation.hpp"
#include <stdexcept>

int main() {
    nv::FlightController fc(nv::DroneConfig{});
    fc.arm();
    fc.set_throttle(25.0);
    std::cout << "test_package ok: " << fc.config().id << "\n";

    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    std::cout << "pwm min = " << nv_pwm_pulse_us(&pwm, 0) << "us\n";
    neurovector::navigation::DeliveryNavigator safety;
    const auto decision = safety.update({});
    if (decision.safety != neurovector::navigation::SafetyDirective::delegate_to_autopilot ||
        decision.navigation.desired_speed_mps != 0)
        throw std::runtime_error("packaged safety library allowed missing aircraft state");
    std::cout << "packaged delivery safety API ok\n";
    return 0;
}
