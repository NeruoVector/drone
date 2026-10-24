#include "neurovector/wifi_sensing.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace nav = neurovector::navigation;
namespace avoid = neurovector::avoidance;
void require(bool condition) {
  if (!condition) throw std::runtime_error("Wi-Fi sensing regression");
}
nav::WifiPath fixture() {
  nav::WifiPath p;
  p.id = "reflection-1";
  p.transmitter_enu = {0, 0, 0}; p.receiver_enu = {10, 0, 0};
  p.azimuth_rad = std::atan2(4, -5); p.elevation_rad = std::atan2(3, std::sqrt(41));
  p.excess_delay_s = (2 * std::sqrt(50) - 10) / 299792458.0;
  p.delay_error_s = 1e-10; p.angular_error_rad = 0.01; p.anchor_error_m = 0.01;
  p.observed_at_s = 10; p.confidence = 0.95;
  p.calibrated = p.path_resolved = true;
  return p;
}
int main() {
  auto p = fixture();
  auto o = nav::localize_wifi(p, 10).obstacle;
  require(o.position_known && !o.velocity_known && o.radius_m > 0.5);
  require(std::hypot(o.position.x - 5, o.position.y - 4, o.position.z - 3) < 1e-9);
  avoid::Input in;
  in.now_s = in.observations_at_s = 10; in.sensors_ready = true;
  in.position = {0, 4, 3}; in.velocity = {2, 0, 0}; in.remaining_route = {{20, 4, 3}};
  in.obstacles = {o};
  require(avoid::decide(in).action == avoid::Action::hold);
  p.tracked_velocity_enu_mps = avoid::Vec3{};
  require(nav::localize_wifi(p, 10).obstacle.velocity_known);
  p.path_resolved = false;
  o = nav::localize_wifi(p, 10).obstacle;
  require(!o.position_known);
  in.obstacles = {o}; require(avoid::decide(in).action == avoid::Action::hold);
  p = fixture(); p.calibrated = false;
  require(!nav::localize_wifi(p, 10).obstacle.position_known);
  p = fixture(); require(!nav::localize_wifi(p, 11).obstacle.position_known);
  require(!nav::localize_wifi(p, 9).obstacle.position_known);
  p.excess_delay_s = 0;
  require(!nav::localize_wifi(p, 10).obstacle.position_known);
  p = fixture(); p.angular_error_rad = 1;
  require(!nav::localize_wifi(p, 10).obstacle.position_known);
  p = fixture(); p.anchor_error_m = -1;
  require(!nav::localize_wifi(p, 10).obstacle.position_known);
  p = fixture(); p.confidence = std::numeric_limits<double>::quiet_NaN();
  require(!nav::localize_wifi(p, 10).obstacle.position_known);
  p = fixture(); p.tracked_velocity_enu_mps = avoid::Vec3{101, 0, 0};
  require(!nav::localize_wifi(p, 10).obstacle.position_known);
  std::cout << "Wi-Fi sensing checks passed\n";
}
