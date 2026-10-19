#include <cstdlib>
#include <iostream>

#include "neurovector/drone_hal.hpp"
#include "nv_pwm.h"

int main() {
    nv::DroneConfig config{
        .id = "drone-001",
        .max_power_watts = 120.0,
        .max_altitude_m = 120.0,
    };

    nv::FlightController fc(config);
    std::cout << "drone id: " << fc.config().id << "\n";

    fc.arm();
    fc.set_throttle(40.0);
    fc.mix(/*roll=*/2.0, /*pitch=*/-1.0, /*yaw=*/0.5);

    for (const auto& motor : fc.snapshot()) {
        std::cout << motor.name() << " ch=" << static_cast<int>(motor.channel())
                  << " throttle=" << motor.throttle() << "%\n";
    }

    const auto imu = fc.read_imu();
    std::cout << "imu roll=" << imu.roll_deg << " pitch=" << imu.pitch_deg
              << " yaw=" << imu.yaw_deg << "\n";

    // C 펌웨어 레이어 사용 예시
    nv_pwm_state_t pwm;
    nv_pwm_init(&pwm);
    nv_pwm_arm(&pwm);
    if (nv_pwm_set_throttle(&pwm, 0, 40.0f) != 0) {
        std::cerr << "failed to set pwm throttle\n";
        return EXIT_FAILURE;
    }
    std::cout << "pwm ch0 = " << nv_pwm_pulse_us(&pwm, 0) << "us\n";

    fc.disarm();
    return EXIT_SUCCESS;
}
