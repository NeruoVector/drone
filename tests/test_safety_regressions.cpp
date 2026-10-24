#include "neurovector/delivery_navigation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace nav = neurovector::avoidance;
namespace delivery = neurovector::navigation;
void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
delivery::DeliveryInput fixture() {
  delivery::DeliveryInput in;
  in.vehicle_position_error_m = 0.1;
  in.flight.sensors_ready = true;
  in.flight.state = {nav::FlightMode::window_delivery, 0, true, true, true, 0, 0, true, true};
  in.flight.position = {0, -10, 7.5}; in.flight.velocity = {0, 1, 0};
  in.flight.remaining_route = {{0, -4, 7.5}};
  in.destination.floor = 3;
  in.destination.destination = {{37, 127, 100}, 0.2, 0, true};
  in.destination.survey.origin = {37, 127, 100};
  in.destination.survey.outward_normal = {0, -1, 0};
  in.destination.survey.first_row_height_m = 1.5;
  in.destination.survey.rows_complete = true;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 2; ++col)
      in.destination.windows.push_back({std::to_string(row) + ":" + std::to_string(col),
          {col * 4.0, 0, 1.5 + row * 3}, 0.05, 0.99, 0});
  in.disturbance = delivery::DisturbanceSample{};
  in.disturbance->calibrated = true;
  in.wifi.healthy = true;
  in.system = {0, 100, 20, 10, true, true, true};
  in.handoff = {0, true, true, true, true, true, "2:0"};
  return in;
}
void stamp(delivery::DeliveryInput &in, double time) {
  in.flight.now_s = in.flight.observations_at_s = in.flight.state.observed_at_s = time;
  in.destination.now_s = in.destination.destination.observed_at_s = time;
  in.system.observed_at_s = in.handoff.observed_at_s = in.wifi.observed_at_s = time;
  in.disturbance->observed_at_s = time;
  for (auto &w : in.destination.windows) w.observed_at_s = time;
}
void warm(delivery::DeliveryNavigator &supervisor, delivery::DeliveryInput &in) {
  for (int i = 0; i <= 110; ++i) { stamp(in, i * 0.01); supervisor.update(in); }
  require(supervisor.update(in).safety == delivery::SafetyDirective::continue_mission,
          "healthy supervisor never permits the authorized approach");
}
int main() {
  nav::Input in;
  in.now_s = in.observations_at_s = 10;
  in.sensors_ready = true;
  in.state = {nav::FlightMode::transit, 10, true, true, true, 0, 0, true, true};
  in.position = {0, -3.5, 7.5}; in.velocity = {0, 5, 0};
  in.remaining_route = {{0, -4, 7.5}};
  in.facade = nav::Facade{{0, 0, 7.5}, {0, -1, 0}, 2};
  require(nav::decide(in).action == nav::Action::hold,
          "momentum toward facade bypassed braking clearance");
  in.velocity = {0, -1, 0};
  require(nav::decide(in).action == nav::Action::proceed, "motion away from facade blocked");
  in.facade.reset(); in.position = {0, 0, 5}; in.velocity = {5, 0, 0};
  in.remaining_route = {{-10, 0, 5}}; in.no_fly_boxes = {{{3, -1, 4}, {4, 1, 6}}};
  require(nav::decide(in).action == nav::Action::hold, "momentum entered no-fly zone");
  in.no_fly_boxes.clear(); in.state.mode = nav::FlightMode::unknown;
  require(nav::decide(in).action == nav::Action::hold, "unknown mode permitted motion");
  in.state.mode = nav::FlightMode::landing; in.remaining_route = {{0, 0, 0.5}};
  in.velocity = {0, 0, -0.3};
  require(nav::decide(in).action == nav::Action::hold, "landing without downward evidence permitted");
  in.landing_surface = nav::LandingSurface{{0, 0, 0}, 3, 3, 0, 0.01, 0.05, 0.99, 10, true, true};
  in.coverage = {{{{-3, -3, 0}, {3, 3, 6}}, 10}};
  nav::Obstacle person;
  person.id = "person"; person.priority = 1; person.confidence = 0.99;
  person.observed_at_s = 10; person.position_known = person.velocity_known = true;
  person.position = {2.5, 0, 0.5}; person.radius_m = 0.3; in.obstacles = {person};
  require(nav::decide(in).action == nav::Action::hold, "stationary person can enter landing perimeter");
  int checked = 6;
  in.state.mode = nav::FlightMode::transit;
  in.position = {0, 0, 5}; in.velocity = {1, 0, 0};
  in.remaining_route = {{1, 0, 5}};
  person.priority = 2; person.position = {1, 3, 5}; person.velocity = {0, -1, 0};
  person.radius_m = 0.1; in.obstacles = {person};
  require(nav::decide(in).action != nav::Action::proceed,
          "moving obstacle reaches the endpoint while aircraft hovers there"); ++checked;
  auto reject = [&](const char *name, auto mutate,
                    delivery::SafetyDirective expected = delivery::SafetyDirective::delegate_to_autopilot) {
    auto input = fixture(); delivery::DeliveryNavigator supervisor; warm(supervisor, input);
    mutate(input);
    const auto d = supervisor.update(input);
    require(d.navigation.action == nav::Action::hold && d.navigation.desired_speed_mps == 0 &&
            d.safety == expected && !d.handoff_ready && !d.disarm_allowed, name);
    std::cout << "PASS " << name << " safety=" << delivery::safety_name(d.safety) << '\n'; ++checked;
  };
  reject("stale-aircraft-pose-with-fresh-camera", [](auto &i) { i.flight.state.observed_at_s = 0; });
  reject("invalid-position-estimate", [](auto &i) { i.flight.state.position_valid = false; });
  reject("invalid-velocity-estimate", [](auto &i) { i.flight.state.velocity_valid = false; });
  reject("invalid-attitude-estimate", [](auto &i) { i.flight.state.attitude_valid = false; });
  reject("excessive-tilt", [](auto &i) { i.flight.state.roll_deg = 30; });
  reject("combined-roll-pitch-tilt", [](auto &i) { i.flight.state.roll_deg = i.flight.state.pitch_deg = 19; });
  reject("motor-failure", [](auto &i) { i.flight.state.motors_healthy = false; });
  reject("insufficient-thrust", [](auto &i) { i.flight.state.thrust_available = false; });
  reject("unknown-autopilot-mode", [](auto &i) { i.flight.state.mode = nav::FlightMode::unknown; });
  reject("battery-emergency", [](auto &i) { i.system.battery_healthy = false; });
  reject("insufficient-return-reserve", [](auto &i) { i.system.usable_energy_wh = 30; });
  reject("invalid-energy", [](auto &i) { i.system.usable_energy_wh = std::numeric_limits<double>::quiet_NaN(); });
  reject("command-link-loss", [](auto &i) { i.system.command_link_healthy = false; });
  reject("missing-independent-watchdog", [](auto &i) { i.system.independent_watchdog_enabled = false; });
  reject("native-failsafe-priority", [](auto &i) { i.system.native_failsafe_active = true; });
  reject("operator-override-priority", [](auto &i) { i.system.operator_override = true; });
  reject("stale-system-status", [](auto &i) { i.system.observed_at_s = 0; });
  reject("future-system-status", [](auto &i) { i.system.observed_at_s = 2; });
  auto input = fixture(); delivery::DeliveryNavigator supervisor; warm(supervisor, input);
  input.flight.remaining_route = {{4, -4, 7.5}};
  person.priority = 2; person.observed_at_s = input.flight.now_s;
  const double length = std::hypot(4, 6);
  person.position = {4 + 8 / length, -4 + 12 / length, 7.5}; input.flight.obstacles = {person};
  auto d = supervisor.update(input);
  require(d.navigation.action == nav::Action::request_replan && d.navigation.desired_speed_mps == 0,
          "slow obstacle decision bypassed wrong-window endpoint validation"); ++checked;
  input = fixture(); delivery::DeliveryNavigator bad_config; warm(bad_config, input);
  nav::Config negative; negative.clearance_m = -0.1;
  d = bad_config.update(input, negative);
  require(d.navigation.action == nav::Action::hold &&
          d.safety == delivery::SafetyDirective::delegate_to_autopilot,
          "negative original clearance accepted after inflation"); ++checked;
  auto handoff_reject = [&](const char *name, auto mutate) {
    reject(name, [&](auto &i) {
      i.flight.position = {0, -4, 7.5}; i.flight.velocity = {}; i.flight.window_handoff = true;
      mutate(i);
    }, delivery::SafetyDirective::hold);
  };
  handoff_reject("handoff-error-bound", [](auto &i) { i.vehicle_position_error_m = 0.8; });
  handoff_reject("unconfirmed-recipient", [](auto &i) { i.handoff.recipient_confirmed = false; });
  handoff_reject("blocked-receiving-path", [](auto &i) { i.handoff.receiving_path_clear = false; });
  handoff_reject("hand-near-rotors", [](auto &i) { i.handoff.rotor_space_clear = false; });
  handoff_reject("unsecured-payload", [](auto &i) { i.handoff.payload_secure = false; });
  handoff_reject("payload-mechanism-failure", [](auto &i) { i.handoff.mechanism_healthy = false; });
  handoff_reject("stale-handoff-confirmations", [](auto &i) { i.handoff.observed_at_s = 0; });
  handoff_reject("confirmation-for-different-window", [](auto &i) { i.handoff.target_window_id = "2:1"; });
  input = fixture(); delivery::DeliveryNavigator waiting({}, {.max_wait_s = 2}); warm(waiting, input);
  input.destination.survey.rows_complete = false;
  for (int tick = 111; tick <= 320; ++tick) { stamp(input, tick * 0.01); d = waiting.update(input); }
  require(d.safety == delivery::SafetyDirective::delegate_to_autopilot, "waiting budget did not expire");
  input.destination.survey.rows_complete = true; stamp(input, 3.21);
  require(waiting.update(input).safety == delivery::SafetyDirective::delegate_to_autopilot,
          "expired waiting resumed an old mission automatically"); checked += 2;
  input = fixture(); delivery::DeliveryNavigator contact; warm(contact, input);
  input.flight.velocity = {}; input.flight.state.mode = nav::FlightMode::landed;
  input.flight.state.ground_contact = input.flight.state.landed_confirmed = true;
  stamp(input, 1.11); d = contact.update(input);
  require(!d.disarm_allowed, "motors could be cut on a single contact sample"); ++checked;
  for (int tick = 112; tick <= 222; ++tick) { stamp(input, tick * 0.01); d = contact.update(input); }
  require(d.disarm_allowed && d.safety == delivery::SafetyDirective::landed &&
          d.navigation.desired_speed_mps == 0 && !d.handoff_ready, "confirmed touchdown never completed"); ++checked;
  for (bool native : {false, true}) {
    auto overridden = input;
    overridden.system.operator_override = !native;
    overridden.system.native_failsafe_active = native;
    delivery::DeliveryNavigator priority;
    for (int tick = 223; tick <= 340; ++tick) {
      stamp(overridden, tick * 0.01); d = priority.update(overridden);
    }
    require(!d.disarm_allowed && d.safety == delivery::SafetyDirective::delegate_to_autopilot,
            "landed confirmation bypassed operator/native control priority"); ++checked;
  }
  input.flight.state.ground_contact = false; stamp(input, 2.23);
  require(!contact.update(input).disarm_allowed, "ground-contact bounce preserved disarm permission"); ++checked;
  std::cout << "safety regression cases passed: " << checked << '\n';
}
