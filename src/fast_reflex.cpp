#include "neurovector/fast_reflex.hpp"

#include <cmath>

namespace neurovector::navigation {
namespace {
double norm(avoidance::Vec3 v) { return std::hypot(v.x, v.y, v.z); }
bool finite(avoidance::Vec3 v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
} // namespace
ReflexDecision FastReflex::evaluate(double now, const std::optional<DisturbanceSample> &sample) {
  auto hold = [&](const char *reason) {
    calm_since_.reset();
    return ReflexDecision{true, reason};
  };
  const double limits[]{config_.max_age_s, config_.recovery_s, config_.max_wind_mps,
                        config_.max_wind_slew_mps2, config_.max_acceleration_residual_mps2,
                        config_.max_angular_rate_deg_s};
  for (double value : limits)
    if (!std::isfinite(value) || value <= 0)
      return hold("invalid disturbance thresholds");
  if (!std::isfinite(now) || now < 0 || (last_now_ && now < *last_now_)) {
    previous_.reset();
    return hold("disturbance clock invalid or moved backwards");
  }
  last_now_ = now;
  if (!sample || !sample->calibrated || !std::isfinite(sample->observed_at_s) ||
      sample->observed_at_s < 0 || sample->observed_at_s > now ||
      now - sample->observed_at_s > config_.max_age_s ||
      !finite(sample->wind_enu_mps) || !finite(sample->acceleration_residual_enu_mps2) ||
      !finite(sample->angular_rate_deg_s))
    return hold("fast disturbance sensing missing, stale or uncalibrated");
  const auto &s = *sample;
  if (previous_ && s.observed_at_s < previous_->observed_at_s)
    return hold("disturbance sample arrived out of order");
  double slew = 0;
  if (previous_) {
    const double dt = s.observed_at_s - previous_->observed_at_s;
    const auto a = s.wind_enu_mps, b = previous_->wind_enu_mps;
    if (dt > config_.max_age_s)
      calm_since_.reset();
    if (dt > 0 && dt <= config_.max_age_s)
      slew = norm({a.x - b.x, a.y - b.y, a.z - b.z}) / dt;
    if (dt == 0 && (a.x != b.x || a.y != b.y || a.z != b.z ||
                   s.acceleration_residual_enu_mps2.x != previous_->acceleration_residual_enu_mps2.x ||
                   s.acceleration_residual_enu_mps2.y != previous_->acceleration_residual_enu_mps2.y ||
                   s.acceleration_residual_enu_mps2.z != previous_->acceleration_residual_enu_mps2.z ||
                   s.angular_rate_deg_s.x != previous_->angular_rate_deg_s.x ||
                   s.angular_rate_deg_s.y != previous_->angular_rate_deg_s.y ||
                   s.angular_rate_deg_s.z != previous_->angular_rate_deg_s.z))
      return hold("conflicting disturbance samples share a timestamp");
  }
  previous_ = s;
  if (norm(s.wind_enu_mps) >= config_.max_wind_mps ||
      slew >= config_.max_wind_slew_mps2 ||
      norm(s.acceleration_residual_enu_mps2) >= config_.max_acceleration_residual_mps2 ||
      norm(s.angular_rate_deg_s) >= config_.max_angular_rate_deg_s)
    return hold("gust or external physical disturbance; stabilize and hold");
  if (!calm_since_)
    calm_since_ = s.observed_at_s;
  if (s.observed_at_s - *calm_since_ < config_.recovery_s)
    return {true, "waiting for continuous calm disturbance samples"};
  return {false, "fast disturbance sensors stable"};
}
} // namespace neurovector::navigation
