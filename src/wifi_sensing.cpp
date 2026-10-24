#include "neurovector/wifi_sensing.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace neurovector::navigation {
namespace {
double norm(avoidance::Vec3 p) { return std::hypot(p.x, p.y, p.z); }
bool finite(avoidance::Vec3 p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
} // namespace
WifiResult localize_wifi(const WifiPath &p, double now) {
  WifiResult out;
  out.obstacle.id = "wifi-unlocalized";
  out.obstacle.observed_at_s = std::isfinite(now) && now >= 0 ? now : 0;
  out.reason = "unresolved, stale or uncalibrated Wi-Fi reflection";
  const double values[]{p.azimuth_rad, p.elevation_rad, p.excess_delay_s,
                        p.delay_error_s, p.angular_error_rad, p.anchor_error_m,
                        p.observed_at_s, p.confidence, p.object_radius_m};
  for (double v : values)
    if (!std::isfinite(v)) return out;
  if (!std::isfinite(now) || now < 0 || p.id.empty() || p.id.size() > 128 ||
      !p.calibrated || !p.path_resolved || !finite(p.transmitter_enu) ||
      !finite(p.receiver_enu) || norm(p.transmitter_enu) > 10000 ||
      norm(p.receiver_enu) > 10000 || p.observed_at_s < 0 ||
      p.observed_at_s > now || now - p.observed_at_s > 0.5 ||
      p.confidence < 0.8 || p.confidence > 1 || p.object_radius_m < 0 ||
      p.object_radius_m > 50 || std::abs(p.azimuth_rad) > std::numbers::pi ||
      std::abs(p.elevation_rad) > std::numbers::pi / 2 ||
      p.delay_error_s <= 0 || p.delay_error_s >= p.excess_delay_s ||
      p.excess_delay_s > 1e-5 || p.angular_error_rad <= 0 ||
      p.angular_error_rad > 0.2 || p.anchor_error_m <= 0 || p.anchor_error_m > 1 ||
      (p.tracked_velocity_enu_mps &&
       (!finite(*p.tracked_velocity_enu_mps) || norm(*p.tracked_velocity_enu_mps) > 100)))
    return out;
  constexpr double c = 299792458.0;
  const avoidance::Vec3 b{p.transmitter_enu.x - p.receiver_enu.x,
                           p.transmitter_enu.y - p.receiver_enu.y,
                           p.transmitter_enu.z - p.receiver_enu.z};
  const double baseline = norm(b);
  const avoidance::Vec3 u{std::cos(p.elevation_rad) * std::cos(p.azimuth_rad),
                           std::cos(p.elevation_rad) * std::sin(p.azimuth_rad),
                           std::sin(p.elevation_rad)};
  const double dot = b.x * u.x + b.y * u.y + b.z * u.z;
  const double length = baseline + c * p.excess_delay_s;
  const double denominator = 2 * (length - dot);
  if (!std::isfinite(denominator) || denominator <= 1e-6)
    return out;
  const double range = (length * length - baseline * baseline) / denominator;
  // Interval enclosure of the bistatic ellipsoid/ray intersection, including
  // antenna survey, delay and bearing error. Reject near-degenerate geometry.
  const double chord = 2 * std::sin(p.angular_error_rad / 2);
  const double bmin = std::max(0.0, baseline - 2 * p.anchor_error_m);
  const double bmax = baseline + 2 * p.anchor_error_m;
  const double lmin = bmin + c * (p.excess_delay_s - p.delay_error_s);
  const double lmax = bmax + c * (p.excess_delay_s + p.delay_error_s);
  const double dmin = dot - baseline * chord - 2 * p.anchor_error_m;
  const double dmax = dot + baseline * chord + 2 * p.anchor_error_m;
  if (lmin <= dmax || lmin <= bmax)
    return out;
  const double rmin = (lmin * lmin - bmax * bmax) / (2 * (lmax - dmin));
  const double rmax = (lmax * lmax - bmin * bmin) / (2 * (lmin - dmax));
  const double error = std::max(std::abs(range - rmin), std::abs(rmax - range)) +
                       rmax * chord + p.anchor_error_m;
  if (!std::isfinite(range) || range <= 0 || range > 1000 ||
      !std::isfinite(error) || error > 10)
    return out;
  auto &o = out.obstacle;
  o.id = p.id;
  o.position = {p.receiver_enu.x + range * u.x,
                p.receiver_enu.y + range * u.y,
                p.receiver_enu.z + range * u.z};
  o.position_known = true;
  o.observed_at_s = p.observed_at_s;
  o.confidence = p.confidence;
  o.radius_m = p.object_radius_m + error;
  // Scalar Doppler is not a full 3D velocity; only a calibrated tracker may set it.
  o.velocity_known = p.tracked_velocity_enu_mps.has_value();
  o.velocity = p.tracked_velocity_enu_mps.value_or(avoidance::Vec3{});
  out.reason = "calibrated Wi-Fi reflection localized with bounded uncertainty";
  return out;
}
} // namespace neurovector::navigation
