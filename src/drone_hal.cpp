#include "neurovector/drone_hal.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nv {

Motor::Motor(std::string name, std::uint8_t channel)
    : name_(std::move(name)), channel_(channel) {}

void Motor::set_throttle(double percent) {
    throttle_ = std::clamp(percent, 0.0, 100.0);
}

void Motor::arm() { armed_ = true; }

void Motor::disarm() {
    armed_ = false;
    throttle_ = 0.0;
}

bool Motor::armed() const { return armed_; }

double Motor::throttle() const { return throttle_; }

const std::string& Motor::name() const { return name_; }

std::uint8_t Motor::channel() const { return channel_; }

Imu::Imu(std::uint32_t seed) : rng_(seed) {}

ImuSample Imu::read() {
    std::uniform_real_distribution<double> noise(-0.02, 0.02);
    return ImuSample{
        .roll_deg = roll_ + noise(rng_),
        .pitch_deg = pitch_ + noise(rng_),
        .yaw_deg = yaw_ + noise(rng_),
        .accel_z = 9.81 + noise(rng_),
    };
}

void Imu::simulate_attitude(double roll_deg, double pitch_deg, double yaw_deg) {
    roll_ = roll_deg;
    pitch_ = pitch_deg;
    yaw_ = yaw_deg;
}

FlightController::FlightController(DroneConfig config)
    : config_(std::move(config)), imu_(42) {
    for (std::size_t i = 0; i < 4; ++i) {
        motors_.emplace_back("m" + std::to_string(i + 1), static_cast<std::uint8_t>(i));
    }
}

void FlightController::arm() {
    for (auto& motor : motors_) {
        motor.arm();
    }
    armed_ = true;
}

void FlightController::disarm() {
    for (auto& motor : motors_) {
        motor.disarm();
    }
    armed_ = false;
}

bool FlightController::armed() const { return armed_; }

void FlightController::set_throttle(double percent) {
    // 안전 장치: 시동이 걸리지 않으면 스로틀을 올리지 않는다.
    if (!armed_) {
        return;
    }
    for (auto& motor : motors_) {
        motor.set_throttle(percent);
    }
}

void FlightController::mix(double roll, double pitch, double yaw) {
    // 쿼드콥터 X 프레임 단순 믹싱
    const double base = motors_.empty() ? 0.0 : motors_.front().throttle();
    const double m1 = base + pitch - roll + yaw;
    const double m2 = base + pitch + roll - yaw;
    const double m3 = base - pitch - roll - yaw;
    const double m4 = base - pitch + roll + yaw;

    const double mix_values[4] = {m1, m2, m3, m4};
    for (std::size_t i = 0; i < motors_.size() && i < 4; ++i) {
        motors_[i].set_throttle(mix_values[i]);
    }
}

std::vector<Motor> FlightController::snapshot() const {
    return motors_;
}

ImuSample FlightController::read_imu() { return imu_.read(); }

const DroneConfig& FlightController::config() const { return config_; }

double FlightController::clamp_power(double watts) const {
    return std::clamp(watts, 0.0, config_.max_power_watts);
}

}  // namespace nv
