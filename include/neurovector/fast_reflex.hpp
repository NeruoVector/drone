#pragma once

#include "neurovector/avoidance.hpp"

namespace neurovector::navigation {
struct DisturbanceSample {
  double observed_at_s{};
  avoidance::Vec3 wind_enu_mps;
  // Acceleration after subtracting gravity and commanded aircraft acceleration.
  avoidance::Vec3 acceleration_residual_enu_mps2, angular_rate_deg_s;
  bool calibrated{};
};
struct ReflexConfig {
  double max_age_s{0.05}, recovery_s{1.0};
  double max_wind_mps{8}, max_wind_slew_mps2{12};
  double max_acceleration_residual_mps2{6}, max_angular_rate_deg_s{180};
};
struct ReflexDecision {
  bool hold{true};
  std::string reason;
};
// Evaluate on the high-rate sensor loop, independently of vision/Wi-Fi inference.
class FastReflex {
public:
  explicit FastReflex(ReflexConfig config = {}) : config_(config) {}
  ReflexDecision evaluate(double now_s, const std::optional<DisturbanceSample> &sample);
private:
  ReflexConfig config_;
  std::optional<DisturbanceSample> previous_;
  std::optional<double> calm_since_, last_now_;
};
} // namespace neurovector::navigation
