#include "neurovector/drone_hal.hpp"
#include "nv_pwm.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

static void check(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(EXIT_FAILURE); }
}
static bool zero(const nv::FlightController& fc) {
    for (const auto& motor : fc.snapshot()) if (motor.throttle() != 0) return false;
    return true;
}
int main() {
    const double invalid[] = {NAN, INFINITY, -INFINITY};
    nv::Motor motor("test", 0);
    motor.set_throttle(25);
    check(motor.throttle() == 0, "disarmed motor");
    motor.arm();
    for (double value : invalid) {
        motor.set_throttle(50); motor.set_throttle(value);
        check(motor.throttle() == 0, "invalid motor command");
    }
    nv::FlightController fc({});
    for (double value : invalid) check(fc.clamp_power(value) == 0, "invalid power input");
    check(fc.clamp_power(-1) == 0 && fc.clamp_power(150) == 120, "finite power clamp");
    for (double limit : {double(NAN), double(INFINITY), -double(INFINITY), -1.0, 0.0}) {
        nv::DroneConfig config; config.max_power_watts = limit;
        check(nv::FlightController(config).clamp_power(50) == 0, "invalid power limit");
    }
    fc.mix(10, 0, 0);
    check(zero(fc), "disarmed mix");
    fc.arm(); fc.set_throttle(50); fc.mix(2, -1, .5);
    const auto first = fc.snapshot(); fc.mix(2, -1, .5);
    const auto second = fc.snapshot();
    for (std::size_t i = 0; i < first.size(); ++i)
        check(first[i].throttle() == second[i].throttle(), "repeated mix drifts");
    for (double value : invalid) {
        fc.set_throttle(50); fc.mix(value, 0, 0);
        check(zero(fc), "invalid mix");
        fc.mix(0, 0, 0); check(zero(fc), "invalid mix retained stale collective");
        fc.set_throttle(value); check(zero(fc), "invalid collective");
    }
    fc.set_throttle(50); fc.disarm(); fc.set_throttle(80); fc.mix(10, 0, 0);
    check(zero(fc), "disarm cleared output");
    fc.arm(); fc.mix(0, 0, 0); check(zero(fc), "rearm retained stale collective");
    nv_pwm_state_t pwm; nv_pwm_init(&pwm); nv_pwm_arm(&pwm);
    for (double value : invalid) {
        check(nv_pwm_set_throttle(&pwm, 0, 50) == 0, "PWM setup");
        check(nv_pwm_set_throttle(&pwm, 0, static_cast<float>(value)) == -3, "PWM invalid error");
        check(nv_pwm_pulse_us(&pwm, 0) == NV_PWM_MIN_US, "PWM invalid safe pulse");
    }
    nv_pwm_disarm(&pwm); nv_pwm_arm(&pwm);
    check(nv_pwm_pulse_us(&pwm, 0) == NV_PWM_MIN_US, "PWM rearm stale pulse");
    std::cout << "safety tests passed\n";
}
