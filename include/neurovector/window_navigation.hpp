#pragma once

#include "neurovector/avoidance.hpp"

namespace neurovector::navigation {
using avoidance::Vec3;

struct GeoPoint {
  double latitude_deg{}, longitude_deg{}, ellipsoid_altitude_m{};
};
struct GpsFix {
  GeoPoint point;
  double horizontal_error_m{}, observed_at_s{};
  bool valid{};
};
struct Window {
  std::string id;
  Vec3 center_enu;
  double error_m{}, confidence{}, observed_at_s{};
};
struct WindowSurvey {
  GeoPoint origin;
  double origin_error_m{};
  Vec3 facade_point, outward_normal;
  double first_row_height_m{}, floor_pitch_m{3.0}, row_tolerance_m{0.35};
  int first_floor{1};
  // All lower rows and every window in the requested row were observed.
  bool rows_complete{};
  double protected_standoff_m{2.0}, center_standoff_m{4.0};
};
struct WindowRequest {
  double now_s{};
  int floor{};
  GpsFix destination;
  WindowSurvey survey;
  std::vector<Window> windows;
};
struct WindowTarget {
  std::string id;
  int floor{};
  Vec3 center_enu, hover_enu;
  avoidance::Facade facade;
  double error_m{};
};
struct WindowResult {
  std::optional<WindowTarget> target;
  std::string reason;
};
// Window centres must come from synchronized, calibrated camera/depth projection.
WindowResult locate_window(const WindowRequest &request);
} // namespace neurovector::navigation
