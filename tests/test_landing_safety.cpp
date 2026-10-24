#include "neurovector/avoidance.hpp"
#include "neurovector/delivery_navigation.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace nav = neurovector::avoidance;
namespace delivery = neurovector::navigation;

nav::Input fixture() {
  nav::Input in;
  in.now_s = in.observations_at_s = 10;
  in.sensors_ready = true;
  in.state = {nav::FlightMode::landing, 10, true, true, true, 0, 0, true, true};
  in.position = {0, 0, 5}; in.velocity = {0, 0, -0.3};
  in.remaining_route = {{0, 0, 0.5}};
  in.coverage = {{{{-3, -3, 0}, {3, 3, 6}}, 10}};
  in.landing_surface = nav::LandingSurface{{0, 0, 0}, 3, 3, 0, 0.01, 0.05, 0.99, 10, true, true};
  return in;
}
nav::Obstacle obstacle(nav::Vec3 position) {
  nav::Obstacle o;
  o.id = "downward-hazard"; o.confidence = 0.99; o.observed_at_s = 10;
  o.position_known = o.velocity_known = true;
  o.position = position; o.radius_m = 0.1;
  return o;
}
int main() {
  int passed = 0;
  auto check = [&](const char *name, const nav::Input &in, nav::Action expected) {
    auto d = nav::decide(in);
    if (d.action != expected || (expected == nav::Action::hold &&
        (d.desired_speed_mps != 0 || !d.detour.empty())))
      throw std::runtime_error(std::string(name) + ": " + d.reason);
    std::cout << name << ": " << nav::action_name(d.action) << " (" << d.reason << ")\n";
    ++passed;
    return d;
  };
  check("verified-clear-landing", fixture(), nav::Action::proceed);
  auto in = fixture(); in.obstacles = {obstacle({0, 0, 0.15})};
  for (int priority : {1, 2, 3}) {
    in.obstacles[0].priority = priority;
    auto d = check("person-object-leaf-below", in, nav::Action::hold);
    if (d.obstacle_index != 0) throw std::runtime_error("landing hazard index lost");
  }
  in = fixture(); in.obstacles = {obstacle({10, 0, 0.5})}; in.obstacles[0].velocity = {-1, 0, 0};
  check("person-enters-after-normal-horizon", in, nav::Action::hold);
  in = fixture(); in.obstacles = {obstacle({30, 30, 0.5})};
  check("stationary-hazard-away-from-site", in, nav::Action::proceed);
  in = fixture(); in.landing_surface.reset();
  check("missing-surface", in, nav::Action::hold);
  in = fixture(); in.landing_surface->sensors_ready = false;
  check("downward-sensor-offline", in, nav::Action::hold);
  in = fixture(); in.landing_surface->support_confirmed = false;
  check("water-hole-or-unknown-support", in, nav::Action::hold);
  in = fixture(); in.landing_surface->slope_deg = 15;
  check("steep-ground", in, nav::Action::hold);
  in = fixture(); in.landing_surface->roughness_m = 0.2;
  check("uneven-ground", in, nav::Action::hold);
  in = fixture(); in.landing_surface->half_width_m = 0.8;
  check("small-surface", in, nav::Action::hold);
  in = fixture(); in.coverage.clear();
  check("empty-detections-without-downward-coverage", in, nav::Action::hold);
  in = fixture(); in.coverage[0].bounds.minimum.z = 2;
  check("unobserved-bottom-of-descent", in, nav::Action::hold);
  in = fixture(); in.coverage[0].observed_at_s = 9;
  check("stale-downward-coverage", in, nav::Action::hold);
  in = fixture(); in.landing_surface->observed_at_s = 9;
  check("stale-surface", in, nav::Action::hold);
  in = fixture(); in.landing_surface->observed_at_s = 11;
  check("future-surface", in, nav::Action::hold);
  in = fixture(); in.landing_surface->position_error_m = 0.5;
  check("inaccurate-downward-position", in, nav::Action::hold);
  in = fixture(); in.landing_surface->confidence = 0.5;
  check("uncertain-surface", in, nav::Action::hold);
  in = fixture(); in.landing_surface->slope_deg = std::numeric_limits<double>::quiet_NaN();
  check("malformed-surface", in, nav::Action::hold);
  in = fixture(); in.velocity.z = -1;
  check("excessive-descent-speed", in, nav::Action::hold);
  in = fixture(); in.velocity.x = 1;
  check("excessive-drift", in, nav::Action::hold);
  in = fixture(); in.remaining_route.back().x = 1;
  check("misaligned-touchdown-route", in, nav::Action::hold);
  in = fixture(); in.remaining_route.back().z = 0;
  check("wrong-contact-height", in, nav::Action::hold);
  in = fixture(); in.obstacles = {obstacle({0, 0, 1})}; in.obstacles[0].position_known = false;
  check("unknown-depth-below", in, nav::Action::hold);
  in = fixture(); in.obstacles = {obstacle({10, 0, 1})}; in.obstacles[0].velocity_known = false;
  check("unknown-motion-can-reach-site", in, nav::Action::hold);
  in = fixture(); in.obstacles = {obstacle({0, 0, 1})}; in.obstacles[0].observed_at_s = 9;
  check("stale-hazard", in, nav::Action::hold);
  in = fixture(); in.no_fly_boxes = {{{1.2, -1, 0}, {2, 1, 3}}};
  check("uncertainty-envelope-near-no-fly-zone", in, nav::Action::hold);
  in = fixture(); in.position.x = 0.24;
  check("alignment-includes-position-error", in, nav::Action::hold);
  in = fixture(); in.position.z = 0.5; in.velocity = {};
  check("verified-contact-position", in, nav::Action::proceed);
  in = fixture(); in.coverage[0].bounds.maximum.x = 1;
  check("incomplete-footprint-coverage", in, nav::Action::hold);
  in = fixture(); in.coverage[0].observed_at_s = 10.01;
  check("future-downward-coverage", in, nav::Action::hold);
  nav::Config invalid;
  invalid.landing.max_descent_mps = -1;
  if (nav::decide(fixture(), invalid).action != nav::Action::hold)
    throw std::runtime_error("invalid landing configuration accepted");
  ++passed;
  in = fixture(); in.state.mode = nav::FlightMode::transit; in.landing_surface.reset();
  in.position = {0, 0, 5}; in.velocity = {1, 0, 0}; in.remaining_route = {{10, 0, 5}};
  check("normal-cruise-regression", in, nav::Action::proceed);
  delivery::DeliveryNavigator supervisor;
  delivery::DeliveryInput integrated;
  integrated.flight = fixture();
  integrated.vehicle_position_error_m = 0.1;
  integrated.disturbance = delivery::DisturbanceSample{};
  integrated.disturbance->calibrated = true;
  integrated.wifi.healthy = true;
  integrated.system = {0, 100, 20, 10, true, true, true};
  for (int tick = 0; tick <= 110; ++tick) {
    const double now = tick * 0.01;
    integrated.flight.now_s = integrated.flight.observations_at_s = now;
    integrated.flight.state.observed_at_s = integrated.system.observed_at_s = now;
    integrated.flight.landing_surface->observed_at_s = now;
    integrated.flight.coverage[0].observed_at_s = now;
    integrated.disturbance->observed_at_s = integrated.wifi.observed_at_s = now;
    supervisor.update(integrated);
  }
  auto d = supervisor.update(integrated);
  if (d.navigation.action != nav::Action::proceed || d.handoff_ready || d.target)
    throw std::runtime_error("verified landing requires unrelated window evidence");
  ++passed;
  integrated.flight.obstacles = {obstacle({0, 0, 0.15})};
  integrated.flight.obstacles[0].observed_at_s = integrated.flight.now_s;
  d = supervisor.update(integrated);
  if (d.navigation.action != nav::Action::hold || d.handoff_ready)
    throw std::runtime_error("delivery supervisor bypassed landing hazard");
  ++passed;
  integrated.flight.obstacles.clear();
  integrated.wifi.paths = {delivery::WifiPath{}};
  if (supervisor.update(integrated).navigation.action != nav::Action::hold)
    throw std::runtime_error("landing bypassed unresolved Wi-Fi observation");
  ++passed;
  integrated.wifi.paths.clear();
  integrated.flight.window_handoff = true;
  if (supervisor.update(integrated).navigation.action != nav::Action::hold)
    throw std::runtime_error("landing overlapped window handoff");
  ++passed;
  integrated.flight.window_handoff = false;
  integrated.disturbance->wind_enu_mps.x = 10;
  integrated.disturbance->observed_at_s = integrated.flight.now_s = 1.11;
  if (supervisor.update(integrated).navigation.action != nav::Action::hold)
    throw std::runtime_error("landing bypassed fast gust protection");
  ++passed;
  std::cout << "landing cases passed: " << passed << '\n';
}
