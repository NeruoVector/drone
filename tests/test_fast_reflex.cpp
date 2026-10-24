#include "neurovector/fast_reflex.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace nav = neurovector::navigation;
void require(bool condition) {
  if (!condition) throw std::runtime_error("fast reflex regression");
}
int main() {
  nav::FastReflex reflex;
  nav::DisturbanceSample s;
  s.calibrated = true;
  require(reflex.evaluate(0, s).hold);
  for (int i = 1; i <= 110; ++i) {
    s.observed_at_s = i * 0.01;
    auto d = reflex.evaluate(s.observed_at_s, s);
    require(d.hold == (i < 100));
  }
  // Detect a sub-threshold wind rise within one 10 ms sample, without vision.
  s.observed_at_s = 1.11; s.wind_enu_mps.x = 1;
  require(reflex.evaluate(1.11, s).hold);
  s.observed_at_s = 1.12; s.wind_enu_mps = {};
  require(reflex.evaluate(1.12, s).hold);
  for (int i = 113; i <= 225; ++i) {
    s.observed_at_s = i * 0.01;
    reflex.evaluate(s.observed_at_s, s);
  }
  require(!reflex.evaluate(2.25, s).hold);
  s.observed_at_s = 2.26; s.acceleration_residual_enu_mps2.y = 7;
  require(reflex.evaluate(2.26, s).hold);
  require(reflex.evaluate(3, s).hold);
  require(reflex.evaluate(3, std::nullopt).hold);
  s.observed_at_s = 3; s.acceleration_residual_enu_mps2 = {};
  s.angular_rate_deg_s.z = 181;
  require(reflex.evaluate(3, s).hold);
  s.observed_at_s = 3.01; s.angular_rate_deg_s = {}; s.wind_enu_mps.x = 9;
  require(reflex.evaluate(3.01, s).hold);
  s.observed_at_s = 3.02; s.wind_enu_mps = {}; s.calibrated = false;
  require(reflex.evaluate(3.02, s).hold);
  s.calibrated = true; s.wind_enu_mps.x = std::numeric_limits<double>::quiet_NaN();
  require(reflex.evaluate(3.02, s).hold);
  s.wind_enu_mps = {}; s.observed_at_s = 4;
  require(reflex.evaluate(3.02, s).hold);
  require(reflex.evaluate(2, s).hold);
  nav::FastReflex invalid({.max_age_s = -1});
  require(invalid.evaluate(4, s).hold);
  std::cout << "fast reflex checks passed\n";
}
