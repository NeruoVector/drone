#include "neurovector/delivery_navigation.hpp"

#include <cmath>

namespace neurovector::navigation {
DeliveryDecision DeliveryNavigator::update(const DeliveryInput &in, const avoidance::Config &config) {
  DeliveryDecision out;
  auto stop = [&](const std::string &reason) {
    out.navigation.action = avoidance::Action::hold;
    out.navigation.reason = reason;
    out.navigation.desired_speed_mps = 0;
    out.navigation.detour.clear();
    return out;
  };
  const auto reflex = reflex_.evaluate(in.flight.now_s, in.disturbance);
  if (reflex.hold)
    return stop(reflex.reason);
  if (in.destination.now_s != in.flight.now_s ||
      !std::isfinite(in.vehicle_position_error_m) || in.vehicle_position_error_m <= 0 ||
      in.vehicle_position_error_m > 10)
    return stop("destination clock or aircraft localization error invalid");
  auto localized = locate_window(in.destination);
  if (!localized.target)
    return stop(localized.reason);
  out.target = localized.target;
  auto flight = in.flight;
  auto limits = config;
  limits.clearance_m += out.target->error_m + in.vehicle_position_error_m;
  if (!std::isfinite(limits.clearance_m) ||
      in.destination.survey.center_standoff_m < out.target->facade.minimum_standoff_m +
          limits.drone_radius_m + limits.clearance_m)
    return stop("window standoff cannot contain aircraft and localization errors");
  if (!in.wifi.healthy || !std::isfinite(in.wifi.observed_at_s) ||
      in.wifi.observed_at_s < 0 || in.wifi.observed_at_s > flight.now_s ||
      flight.now_s - in.wifi.observed_at_s > 0.5 || in.wifi.paths.size() > 128 ||
      flight.obstacles.size() + in.wifi.paths.size() > 128)
    return stop("Wi-Fi sensing unavailable, stale or oversized");
  for (const auto &path : in.wifi.paths)
    flight.obstacles.push_back(localize_wifi(path, flight.now_s).obstacle);
  flight.facade = out.target->facade;
  const bool requested_handoff = flight.window_handoff;
  flight.window_handoff = false;
  // Check all live collision evidence before allowing a handoff, even at zero speed.
  out.navigation = avoidance::decide(flight, limits);
  if (out.navigation.action != avoidance::Action::proceed)
    return requested_handoff ? stop(out.navigation.reason) : out;
  const auto hover = out.target->hover_enu;
  if (flight.remaining_route.empty())
    return stop("validated delivery approach route missing");
  const auto end = flight.remaining_route.back();
  if (std::hypot(end.x - hover.x, end.y - hover.y, end.z - hover.z) > 0.25) {
    out.navigation.action = avoidance::Action::request_replan;
    out.navigation.reason = "validate a replacement mission ending at the resolved window hover";
    out.navigation.desired_speed_mps = 0;
    return out;
  }
  if (requested_handoff) {
    const auto p = flight.position, v = flight.velocity;
    out.handoff_ready = std::hypot(p.x - hover.x, p.y - hover.y, p.z - hover.z) <= 0.25 &&
                         std::hypot(v.x, v.y, v.z) <= 0.1;
    return stop(out.handoff_ready ? "resolved window exterior handoff ready; maintain hold" :
                                   "handoff requires arrival at resolved window and stationary hover");
  }
  return out;
}
} // namespace neurovector::navigation
