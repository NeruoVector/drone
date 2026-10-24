#include "neurovector/window_navigation.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <set>

namespace neurovector::navigation {
namespace {
bool finite(Vec3 p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
bool geographic(GeoPoint p) {
  return std::isfinite(p.latitude_deg) && std::abs(p.latitude_deg) <= 90 &&
         std::isfinite(p.longitude_deg) && std::abs(p.longitude_deg) <= 180 &&
         std::isfinite(p.ellipsoid_altitude_m) &&
         std::abs(p.ellipsoid_altitude_m) <= 10000;
}
Vec3 ecef(GeoPoint p) {
  const double lat = p.latitude_deg * std::numbers::pi / 180;
  const double lon = p.longitude_deg * std::numbers::pi / 180;
  constexpr double e2 = 6.6943799901413165e-3;
  const double n = 6378137.0 / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));
  return {(n + p.ellipsoid_altitude_m) * std::cos(lat) * std::cos(lon),
          (n + p.ellipsoid_altitude_m) * std::cos(lat) * std::sin(lon),
          (n * (1 - e2) + p.ellipsoid_altitude_m) * std::sin(lat)};
}
Vec3 horizontal_enu(GeoPoint point, GeoPoint origin) {
  // GPS altitude never selects a floor. Use the surveyed vertical datum instead.
  point.ellipsoid_altitude_m = origin.ellipsoid_altitude_m;
  auto a = ecef(point), b = ecef(origin);
  Vec3 d{a.x - b.x, a.y - b.y, a.z - b.z};
  const double lat = origin.latitude_deg * std::numbers::pi / 180;
  const double lon = origin.longitude_deg * std::numbers::pi / 180;
  return {-std::sin(lon) * d.x + std::cos(lon) * d.y,
          -std::sin(lat) * std::cos(lon) * d.x -
              std::sin(lat) * std::sin(lon) * d.y + std::cos(lat) * d.z,
          0};
}
bool fresh(double now, double at) {
  return std::isfinite(at) && at >= 0 && at <= now && now - at <= 0.5;
}
} // namespace

WindowResult locate_window(const WindowRequest &r) {
  const auto &s = r.survey;
  auto fail = [](const char *reason) { return WindowResult{{}, reason}; };
  const double values[]{s.origin_error_m, s.first_row_height_m, s.floor_pitch_m,
                        s.row_tolerance_m, s.protected_standoff_m,
                        s.center_standoff_m, r.destination.horizontal_error_m};
  for (double v : values)
    if (!std::isfinite(v))
      return fail("invalid window survey or GPS uncertainty");
  if (!std::isfinite(r.now_s) || r.now_s < 0 || !geographic(s.origin) ||
      !geographic(r.destination.point) || !r.destination.valid ||
      !fresh(r.now_s, r.destination.observed_at_s) ||
      r.destination.horizontal_error_m <= 0 || s.origin_error_m < 0 ||
      s.origin_error_m > 10 || r.floor < s.first_floor || r.floor > 200 ||
      s.first_floor < 1 || s.first_floor > 200 || !s.rows_complete ||
      r.windows.empty() || r.windows.size() > 2048 ||
      s.row_tolerance_m <= 0 || s.row_tolerance_m > 1 ||
      s.floor_pitch_m <= 4 * s.row_tolerance_m || s.floor_pitch_m > 10 ||
      s.protected_standoff_m < 2 || s.center_standoff_m < s.protected_standoff_m ||
      s.center_standoff_m > 50 || !finite(s.facade_point) ||
      !finite(s.outward_normal) || std::abs(s.outward_normal.z) > 1e-6 ||
      std::abs(std::hypot(s.outward_normal.x, s.outward_normal.y) - 1) > 1e-6)
    return fail("missing, stale or invalid floor/GPS/facade evidence");
  const Vec3 gps = horizontal_enu(r.destination.point, s.origin);
  if (!finite(gps) || std::hypot(gps.x, gps.y) > 1000)
    return fail("destination outside surveyed local GPS frame");
  struct Candidate { const Window *window; double distance, error; };
  std::vector<Candidate> candidates;
  std::set<int> floors;
  std::set<std::string> ids;
  for (const auto &w : r.windows) {
    if (w.id.empty() || w.id.size() > 128 || !ids.insert(w.id).second ||
        !finite(w.center_enu) || std::hypot(w.center_enu.x, w.center_enu.y) > 1000 ||
        !std::isfinite(w.error_m) || w.error_m <= 0 || w.error_m > s.row_tolerance_m ||
        !std::isfinite(w.confidence) || w.confidence < 0.8 || w.confidence > 1 ||
        !fresh(r.now_s, w.observed_at_s))
      return fail("invalid, duplicate or uncertain window observation");
    const double row = (w.center_enu.z - s.first_row_height_m) / s.floor_pitch_m;
    if (!std::isfinite(row) || row < -0.5 || row > 199 ||
        std::abs(row - std::round(row)) * s.floor_pitch_m + w.error_m >
            s.row_tolerance_m)
      return fail("window rows disagree with surveyed floor spacing");
    const int floor = s.first_floor + static_cast<int>(std::round(row));
    if (floor > 200)
      return fail("floor exceeds local survey limits");
    floors.insert(floor);
    const double plane_distance =
        (w.center_enu.x - s.facade_point.x) * s.outward_normal.x +
        (w.center_enu.y - s.facade_point.y) * s.outward_normal.y;
    if (std::abs(plane_distance) + w.error_m > s.row_tolerance_m)
      return fail("window lies outside selected building facade");
    if (floor == r.floor)
      candidates.push_back({&w, std::hypot(w.center_enu.x - gps.x,
                                           w.center_enu.y - gps.y),
                            w.error_m + s.origin_error_m +
                                r.destination.horizontal_error_m});
  }
  for (int floor = s.first_floor; floor <= r.floor; ++floor)
    if (!floors.contains(floor))
      return fail("missing lower window row; floor count is ambiguous");
  if (candidates.empty())
    return fail("requested floor has no observed window");
  std::sort(candidates.begin(), candidates.end(), [](auto a, auto b) {
    return a.distance < b.distance;
  });
  const auto &best = candidates.front();
  if (best.distance + best.error > 10)
    return fail("GPS destination does not match the observed windows");
  for (std::size_t i = 1; i < candidates.size(); ++i)
    if (candidates[i].distance - best.distance <= best.error + candidates[i].error)
      return fail("GPS uncertainty cannot distinguish neighboring windows");
  const auto &w = *best.window;
  Vec3 hover{w.center_enu.x + s.outward_normal.x * s.center_standoff_m,
             w.center_enu.y + s.outward_normal.y * s.center_standoff_m,
             w.center_enu.z};
  return {WindowTarget{w.id, r.floor, w.center_enu, hover,
                       {w.center_enu, s.outward_normal, s.protected_standoff_m},
                       w.error_m + s.origin_error_m},
          "window row and GPS destination resolved"};
}
} // namespace neurovector::navigation
