#include "neurovector/delivery_navigation.hpp"

#include <cmath>

namespace neurovector::navigation {
const char *safety_name(SafetyDirective directive) {
  switch (directive) {
  case SafetyDirective::hold: return "hold";
  case SafetyDirective::continue_mission: return "continue_mission";
  case SafetyDirective::delegate_to_autopilot: return "delegate_to_autopilot";
  case SafetyDirective::landed: return "landed";
  }
  return "delegate_to_autopilot";
}
DeliveryDecision DeliveryNavigator::update(const DeliveryInput &in, const avoidance::Config &config) {
  DeliveryDecision out;
  const double now = in.flight.now_s;
  auto finish = [&]() {
    if (out.safety == SafetyDirective::hold) {
      if (!hold_since_) hold_since_ = now;
      if (now - *hold_since_ >= limits_.max_wait_s) {
        out.safety = SafetyDirective::delegate_to_autopilot;
        failsafe_latched_ = true;
        out.navigation.action = avoidance::Action::hold;
        out.navigation.desired_speed_mps = 0;
        out.navigation.detour.clear();
        out.navigation.reason = "waiting budget exhausted; native return/alternate-landing failsafe required";
        out.handoff_ready = false;
      }
    } else {
      hold_since_.reset();
    }
    return out;
  };
  auto stop = [&](const std::string &reason, SafetyDirective directive = SafetyDirective::hold) {
    out.navigation.action = avoidance::Action::hold;
    out.navigation.reason = reason;
    out.navigation.desired_speed_mps = 0;
    out.navigation.detour.clear();
    out.safety = directive;
    if (directive == SafetyDirective::delegate_to_autopilot) failsafe_latched_ = true;
    return finish();
  };
  auto fresh = [&](double at) {
    return std::isfinite(at) && at >= 0 && at <= now && now - at <= limits_.max_status_age_s;
  };
  if (!std::isfinite(now) || now < 0 || (last_now_ && now < *last_now_) ||
      !std::isfinite(limits_.max_status_age_s) || limits_.max_status_age_s <= 0 ||
      !std::isfinite(limits_.max_wait_s) || limits_.max_wait_s <= 0 ||
      !std::isfinite(limits_.contact_confirmation_s) || limits_.contact_confirmation_s <= 0 ||
      !avoidance::valid_config(config)) {
    contact_since_.reset(); last_contact_sample_.reset();
    return stop("invalid clock, original navigation configuration or safety limits",
                SafetyDirective::delegate_to_autopilot);
  }
  last_now_ = now;
  if (!avoidance::usable_vehicle_state(in.flight, config)) {
    contact_since_.reset(); last_contact_sample_.reset();
    return stop("aircraft mode, fresh pose, estimator or control authority unavailable",
                SafetyDirective::delegate_to_autopilot);
  }
  const auto &state = in.flight.state;
  if (state.mode == avoidance::FlightMode::landed) {
    const auto v = in.flight.velocity;
    if (!state.ground_contact || !state.landed_confirmed || std::hypot(v.x, v.y, v.z) > 0.1) {
      contact_since_.reset(); last_contact_sample_.reset();
      return stop("landed mode lacks consistent physical ground-contact confirmation",
                  SafetyDirective::delegate_to_autopilot);
    }
    if (!last_contact_sample_ || state.observed_at_s - *last_contact_sample_ > limits_.max_status_age_s ||
        state.observed_at_s < *last_contact_sample_)
      contact_since_ = state.observed_at_s;
    last_contact_sample_ = state.observed_at_s;
    out.disarm_allowed = state.observed_at_s - *contact_since_ >= limits_.contact_confirmation_s;
    return stop(out.disarm_allowed ? "physical touchdown confirmed; autopilot may disarm" :
                                    "waiting for sustained physical touchdown confirmation",
                out.disarm_allowed ? SafetyDirective::landed : SafetyDirective::hold);
  }
  contact_since_.reset(); last_contact_sample_.reset();
  if (failsafe_latched_)
    return stop("native failsafe latched; a newly authorized mission requires a new supervisor",
                SafetyDirective::delegate_to_autopilot);
  const auto &system = in.system;
  if (!fresh(system.observed_at_s) || !system.command_link_healthy ||
      !system.independent_watchdog_enabled || system.native_failsafe_active || system.operator_override)
    return stop("link/watchdog unavailable or native/operator control has priority",
                SafetyDirective::delegate_to_autopilot);
  if (!system.battery_healthy || !std::isfinite(system.usable_energy_wh) ||
      !std::isfinite(system.return_energy_wh) || !std::isfinite(system.reserve_energy_wh) ||
      system.usable_energy_wh <= 0 || system.return_energy_wh < 0 || system.reserve_energy_wh <= 0 ||
      system.usable_energy_wh > 1e6 || system.return_energy_wh > 1e6 || system.reserve_energy_wh > 1e6 ||
      system.usable_energy_wh <= system.return_energy_wh + system.reserve_energy_wh)
    return stop("battery emergency or insufficient return-and-reserve energy",
                SafetyDirective::delegate_to_autopilot);
  const auto reflex = reflex_.evaluate(now, in.disturbance);
  if (reflex.hold) return stop(reflex.reason);
  const bool landing = state.mode == avoidance::FlightMode::landing;
  if ((!landing && state.mode != avoidance::FlightMode::window_delivery) ||
      (!landing && in.destination.now_s != now) ||
      !std::isfinite(in.vehicle_position_error_m) || in.vehicle_position_error_m <= 0 ||
      in.vehicle_position_error_m > 10)
    return stop("delivery mode, destination clock or aircraft localization error invalid");
  auto flight = in.flight;
  auto limits = config;
  limits.clearance_m += in.vehicle_position_error_m;
  if (landing && flight.window_handoff) return stop("landing cannot overlap a window handoff");
  if (!landing) {
    auto localized = locate_window(in.destination);
    if (!localized.target) return stop(localized.reason);
    out.target = localized.target;
    limits.clearance_m += out.target->error_m;
    if (in.destination.survey.center_standoff_m < out.target->facade.minimum_standoff_m +
        limits.drone_radius_m + limits.clearance_m)
      return stop("window standoff cannot contain aircraft and localization errors");
    flight.facade = out.target->facade;
    if (flight.remaining_route.empty()) return stop("validated delivery approach route missing");
    const auto end = flight.remaining_route.back(), hover = out.target->hover_enu;
    if (std::hypot(end.x - hover.x, end.y - hover.y, end.z - hover.z) > 0.25) {
      out.navigation.action = avoidance::Action::request_replan;
      out.navigation.reason = "validate a replacement mission ending at the resolved window hover";
      return finish();
    }
  }
  if (!in.wifi.healthy || !fresh(in.wifi.observed_at_s) || in.wifi.paths.size() > 128 ||
      flight.obstacles.size() + in.wifi.paths.size() > 128)
    return stop("Wi-Fi sensing unavailable, stale or oversized");
  for (const auto &path : in.wifi.paths)
    flight.obstacles.push_back(localize_wifi(path, now).obstacle);
  const bool requested_handoff = flight.window_handoff;
  flight.window_handoff = false;
  out.navigation = avoidance::decide(flight, limits);
  if (out.navigation.action != avoidance::Action::proceed) {
    if (requested_handoff) return stop(out.navigation.reason);
    if (out.navigation.action == avoidance::Action::slow) out.safety = SafetyDirective::continue_mission;
    return finish();
  }
  if (requested_handoff) {
    const auto p = flight.position, v = flight.velocity, hover = out.target->hover_enu;
    const auto &h = in.handoff;
    out.handoff_ready = fresh(h.observed_at_s) && h.target_window_id == out.target->id &&
        h.recipient_confirmed && h.receiving_path_clear &&
        h.rotor_space_clear && h.payload_secure && h.mechanism_healthy &&
        std::hypot(p.x - hover.x, p.y - hover.y, p.z - hover.z) + out.target->error_m +
            in.vehicle_position_error_m <= 0.25 && std::hypot(v.x, v.y, v.z) <= 0.1;
    return stop(out.handoff_ready ? "verified recipient, payload and exterior handoff ready; maintain hold" :
                                   "handoff alignment, recipient, rotor space or payload interlock uncertain");
  }
  out.safety = SafetyDirective::continue_mission;
  return finish();
}
} // namespace neurovector::navigation
