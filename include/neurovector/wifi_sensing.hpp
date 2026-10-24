#pragma once

#include "neurovector/avoidance.hpp"

namespace neurovector::navigation {
struct WifiPath {
  std::string id;
  avoidance::Vec3 transmitter_enu, receiver_enu;
  // Receiver-to-reflector bearing in ENU: azimuth east toward north, elevation up.
  double azimuth_rad{}, elevation_rad{};
  // Reflected Tx-object-Rx path delay minus direct Tx-Rx delay, in seconds.
  double excess_delay_s{}, delay_error_s{}, angular_error_rad{}, anchor_error_m{};
  double observed_at_s{}, confidence{}, object_radius_m{0.5};
  bool calibrated{}, path_resolved{};
  std::optional<avoidance::Vec3> tracked_velocity_enu_mps;
};
struct WifiResult {
  avoidance::Obstacle obstacle;
  std::string reason;
};
// An unresolved path returns an unlocalized obstacle that blocks nearby navigation.
// This consumes calibrated CSI AoA/ToF estimates, never RSSI or an AP location.
WifiResult localize_wifi(const WifiPath &path, double now_s);
} // namespace neurovector::navigation
