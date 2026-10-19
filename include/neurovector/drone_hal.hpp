#pragma once

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace nv {

/// 드론 기체 설정
struct DroneConfig {
    std::string id = "drone-001";
    double max_power_watts = 120.0;
    double max_altitude_m = 120.0;
};

/// IMU 한 번의 샘플
struct ImuSample {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double accel_z = 9.81;
};

/// 모터(ESC) 하나를 나타낸다.
class Motor {
public:
    Motor(std::string name, std::uint8_t channel);

    void set_throttle(double percent);
    void arm();
    void disarm();

    bool armed() const;
    double throttle() const;
    const std::string& name() const;
    std::uint8_t channel() const;

private:
    std::string name_;
    std::uint8_t channel_;
    double throttle_ = 0.0;
    bool armed_ = false;
};

/// IMU 센서 (현재는 시뮬레이션)
class Imu {
public:
    explicit Imu(std::uint32_t seed);

    ImuSample read();

    /// 테스트/시뮬레이션용 자세 설정
    void simulate_attitude(double roll_deg, double pitch_deg, double yaw_deg);

private:
    std::mt19937 rng_;
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
};

/// 모터 믹싱과 안전 장치를 담당하는 비행 컨트롤러
class FlightController {
public:
    explicit FlightController(DroneConfig config);

    void arm();
    void disarm();
    bool armed() const;

    void set_throttle(double percent);
    void mix(double roll, double pitch, double yaw);

    std::vector<Motor> snapshot() const;
    ImuSample read_imu();
    const DroneConfig& config() const;

    double clamp_power(double watts) const;

private:
    DroneConfig config_;
    Imu imu_;
    std::vector<Motor> motors_;
    bool armed_ = false;
};

}  // namespace nv
