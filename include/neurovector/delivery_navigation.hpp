#pragma once

#include "neurovector/fast_reflex.hpp"
#include "neurovector/wifi_sensing.hpp"
#include "neurovector/window_navigation.hpp"

namespace neurovector::navigation {
struct WifiFrame {
  double observed_at_s{};
  bool healthy{};
  std::vector<WifiPath> paths;
};
struct DeliveryInput {
  avoidance::Input flight;
  WindowRequest destination;
  std::optional<DisturbanceSample> disturbance;
  WifiFrame wifi;
  double vehicle_position_error_m{};
};
struct DeliveryDecision {
  std::optional<WindowTarget> target;
  avoidance::Decision navigation;
  bool handoff_ready{};
};
class DeliveryNavigator {
public:
  explicit DeliveryNavigator(ReflexConfig reflex = {}) : reflex_(reflex) {}
  // Call on each fast sensor tick with the latest cached visual/CSI observations.
  // The caller supplies an already validated/authorized route, never raw motor commands.
  DeliveryDecision update(const DeliveryInput &input, const avoidance::Config &config = {});
private:
  FastReflex reflex_;
};
} // namespace neurovector::navigation
