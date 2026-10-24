#include "neurovector/delivery_navigation.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace nav = neurovector::navigation;
namespace avoid = neurovector::avoidance;

nav::DeliveryInput fixture() {
  nav::DeliveryInput in;
  in.vehicle_position_error_m = 0.1;
  in.flight.sensors_ready = true;
  in.flight.state = {avoid::FlightMode::window_delivery, 0, true, true, true, 0, 0, true, true};
  in.system = {0, 100, 20, 10, true, true, true};
  in.handoff = {0, true, true, true, true, true, "3-0"};
  in.flight.position = {0, -10, 7.5}; in.flight.velocity = {0, 1, 0};
  in.flight.remaining_route = {{0, -4, 7.5}};
  in.destination.floor = 3;
  in.destination.destination = {{37, 127, 100}, 0.2, 0, true};
  in.destination.survey.origin = {37, 127, 100};
  in.destination.survey.outward_normal = {0, -1, 0};
  in.destination.survey.first_row_height_m = 1.5;
  in.destination.survey.rows_complete = true;
  // Project synthetic camera detections through the real calibrated ray helper.
  avoid::Camera camera;
  camera.fx = camera.fy = 400; camera.cx = 320; camera.cy = 240;
  camera.width = 640; camera.height = 480; camera.pose_valid = true;
  camera.camera_to_enu = {1, 0, 0, 0, 0, 1, 0, -1, 0};
  camera.origin_enu = {0, -20, 6};
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 2; ++col) {
      const double x = col * 4.0, z = 1.5 + row * 3;
      const double px = 320 + x / 20 * 400, py = 240 - (z - 6) / 20 * 400;
      auto point = avoid::project_detection({px - 5, py - 5, px + 5, py + 5},
                                             std::hypot(x, 20, z - 6), camera);
      if (!point) throw std::runtime_error("synthetic camera projection failed");
      in.destination.windows.push_back({std::to_string(row + 1) + "-" + std::to_string(col),
                                        *point, 0.05, 0.95, 0});
    }
  in.disturbance = nav::DisturbanceSample{};
  in.disturbance->calibrated = true;
  in.wifi.healthy = true;
  return in;
}
void stamp(nav::DeliveryInput &in, double time) {
  in.flight.now_s = in.flight.observations_at_s = time;
  in.flight.state.observed_at_s = in.system.observed_at_s = in.handoff.observed_at_s = time;
  in.destination.now_s = in.destination.destination.observed_at_s = time;
  for (auto &w : in.destination.windows) w.observed_at_s = time;
  in.disturbance->observed_at_s = in.wifi.observed_at_s = time;
  for (auto &p : in.wifi.paths) p.observed_at_s = time;
}
int main() {
  try {
    nav::DeliveryNavigator navigator;
    auto in = fixture();
    for (int i = 0; i < 110; ++i) {
      stamp(in, i * 0.01);
      navigator.update(in);
    }
    int passed = 0;
    auto check = [&](const char *name, avoid::Action action) {
      const auto d = navigator.update(in);
      if (d.navigation.action != action)
        throw std::runtime_error(std::string(name) + ": " + d.navigation.reason);
      std::cout << "{\"case\":" << std::quoted(name) << ",\"action\":"
                << std::quoted(avoid::action_name(d.navigation.action))
                << ",\"safety\":" << std::quoted(nav::safety_name(d.safety))
                << ",\"reason\":" << std::quoted(d.navigation.reason)
                << ",\"handoff_ready\":" << (d.handoff_ready ? "true" : "false") << "}\n";
      ++passed;
      return d;
    };
    auto d = check("camera-rows-gps-approach", avoid::Action::proceed);
    if (!d.target || d.target->floor != 3 || d.target->id != "3-0")
      throw std::runtime_error("wrong delivery floor/window");
    in.flight.remaining_route = {{4, -4, 7.5}};
    check("wrong-window-route", avoid::Action::request_replan);
    in.flight.remaining_route = {{0, -4, 7.5}};
    in.destination.destination.horizontal_error_m = 3;
    check("ambiguous-gps", avoid::Action::hold);
    in.destination.destination.horizontal_error_m = 0.2;
    in.destination.survey.rows_complete = false;
    check("partial-facade", avoid::Action::hold);
    in.destination.survey.rows_complete = true;
    in.wifi.healthy = false;
    check("missing-wifi-heartbeat", avoid::Action::hold);
    in.wifi.healthy = true;
    nav::WifiPath path;
    path.id = "wifi-crossing-object";
    path.transmitter_enu = {-5, -12, 7.5}; path.receiver_enu = {5, -12, 7.5};
    path.azimuth_rad = std::atan2(4, -5); path.elevation_rad = std::atan2(0.2, std::sqrt(41));
    path.excess_delay_s = (2 * std::sqrt(41.04) - 10) / 299792458.0;
    path.delay_error_s = 1e-12; path.angular_error_rad = 0.001; path.anchor_error_m = 0.0001;
    path.confidence = 0.95; path.calibrated = path.path_resolved = true;
    in.wifi.paths = {path}; stamp(in, 1.10);
    const auto radio = nav::localize_wifi(in.wifi.paths[0], 1.10).obstacle;
    if (!radio.position_known || std::hypot(radio.position.x, radio.position.y + 8,
                                           radio.position.z - 7.7) > 1e-8)
      throw std::runtime_error("synthetic Wi-Fi obstacle was not localized");
    check("wifi-obstacle-on-approach", avoid::Action::hold);
    in.wifi.paths[0].path_resolved = false;
    check("ambiguous-wifi-reflection", avoid::Action::hold);
    in.wifi.paths.clear();
    in.flight.window_handoff = true;
    d = check("handoff-before-arrival", avoid::Action::hold);
    if (d.handoff_ready || d.navigation.desired_speed_mps != 0)
      throw std::runtime_error("handoff accepted before arrival");
    in.flight.position = {0, -4, 7.5}; in.flight.velocity = {};
    in.flight.window_handoff = true;
    d = check("correct-floor-exterior-handoff", avoid::Action::hold);
    if (!d.handoff_ready || d.navigation.desired_speed_mps != 0)
      throw std::runtime_error("handoff never stationary and ready");
    in.wifi.paths = {path}; stamp(in, 1.11);
    d = check("wifi-blocks-handoff", avoid::Action::hold);
    if (d.handoff_ready) throw std::runtime_error("obstructed handoff allowed");
    in.wifi.paths.clear();
    in.destination.survey.center_standoff_m = 3;
    check("localization-error-breaks-standoff", avoid::Action::hold);
    in.destination.survey.center_standoff_m = 4;
    in.disturbance->wind_enu_mps.x = 2;
    in.disturbance->observed_at_s = 1.12; in.flight.now_s = 1.12;
    // Vision intentionally stale: the fast disturbance path must still run first.
    in.flight.observations_at_s = 0;
    d = check("gust-before-vision", avoid::Action::hold);
    if (d.navigation.reason.find("disturbance") == std::string::npos || d.handoff_ready)
      throw std::runtime_error("gust did not override handoff");
    std::cout << "{\"synthetic\":true,\"physical_flight\":false,\"passed\":" << passed << "}\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
