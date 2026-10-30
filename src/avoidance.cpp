#include "neurovector/avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace neurovector::avoidance {
namespace {
#include "avoidance_geometry.inc"
#include "avoidance_safety.inc"
} // namespace

Decision decide(const Input &in, const Config &c) {
  Decision out;
  auto hold = [&](const char *reason) {
    out.action = Action::hold;
    out.reason = reason;
    out.desired_speed_mps = 0;
    return out;
  };
  if (!config_valid(c) || !finite(in.now_s) || in.now_s < 0 ||
      !finite(in.position) || !finite(in.velocity) ||
      in.remaining_route.empty() || in.remaining_route.size() > 256 ||
      in.obstacles.size() > 128 || in.no_fly_boxes.size() > 64 ||
      in.coverage.size() > 64)
    return hold("invalid or oversized local navigation input");
  if (!in.sensors_ready || !fresh(in.now_s, in.observations_at_s, c.max_age_s))
    return hold("observation pipeline unavailable or heartbeat stale");
  double speed = norm(in.velocity);
  if (!finite(speed) || speed > c.max_speed_mps)
    return hold("invalid or excessive speed");
  out.braking_distance_m = speed * speed / (2 * c.deceleration_mps2) +
                           speed * c.reaction_s + c.clearance_m;
  for (auto p : in.remaining_route)
    if (!finite(p) || norm(sub(p, in.position)) > 100000)
      return hold("invalid local route coordinate");
  for (const auto &b : in.no_fly_boxes)
    if (!valid_box(b))
      return hold("invalid no-fly geometry");
  for (const auto &coverage : in.coverage)
    if (!valid_box(coverage.bounds) || !finite(coverage.observed_at_s))
      return hold("invalid sensor coverage");
  if (in.facade) {
    auto &f = *in.facade;
    if (!finite(f.window) || !finite(f.outward_normal) ||
        std::abs(norm(f.outward_normal) - 1) > 1e-6 ||
        std::abs(f.outward_normal.z) > 1e-6 || !finite(f.minimum_standoff_m) ||
        f.minimum_standoff_m < 2)
      return hold("invalid exterior window plane");
    if (!outside(in.position, in.facade, c.drone_radius_m + c.clearance_m))
      return hold("window standoff violated");
  }
  if (in.window_handoff)
    return hold("window handoff requires stationary exterior hover");
  std::vector<Vec3> path{in.position};
  path.insert(path.end(), in.remaining_route.begin(), in.remaining_route.end());
  double padding = c.drone_radius_m + c.clearance_m;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!outside(path[i], in.facade, padding))
      return hold("route enters protected window facade");
    for (const auto &zone : in.no_fly_boxes)
      if (segment_box(path[i - 1], path[i], zone, padding))
        return hold("route intersects no-fly box");
  }
  struct Threat {
    std::size_t index;
    bool immediate, uncertain;
    int priority;
    double ttc, distance, along;
  };
  std::vector<Threat> threats;
  for (std::size_t i = 0; i < in.obstacles.size(); ++i) {
    const auto &o = in.obstacles[i];
    if (o.id.size() > 128 || o.priority < 1 || o.priority > 3 ||
        !finite(o.confidence) || o.confidence < 0 || o.confidence > 1 ||
        !finite(o.observed_at_s) || !finite(o.radius_m) || o.radius_m < 0 ||
        o.radius_m > 100 ||
        (o.distance_m && (!finite(*o.distance_m) || *o.distance_m <= 0)) ||
        (o.position_known && !finite(o.position)) ||
        (o.velocity_known && (!finite(o.velocity) || norm(o.velocity) > 100)))
      return hold("invalid obstacle measurement");
    const bool trusted = fresh(in.now_s, o.observed_at_s, c.max_age_s) &&
                         o.confidence >= c.minimum_confidence;
    const double radius = padding + o.radius_m;
    if (!o.position_known) {
      // A scalar range cannot establish that a detection is outside the
      // corridor.
      if (!trusted || !o.distance_m ||
          *o.distance_m <=
              out.braking_distance_m + speed * c.horizon_s + radius)
        threats.push_back(
            {i, false, true, o.priority, 0, o.distance_m.value_or(0), 0});
      continue;
    }
    auto position = o.position;
    if (trusted && o.velocity_known)
      position = add(
          position, mul(o.velocity, std::max(0.0, in.now_s - o.observed_at_s)));
    auto proximity = route_proximity(path, position);
    double distance = norm(sub(position, in.position));
    // Priority changes caution, never the shared physical collision envelope.
    const double caution_margin = o.priority == 1
                                      ? 2 * c.clearance_m
                                      : (o.priority == 2 ? c.clearance_m : 0.0);
    const double caution_horizon =
        c.horizon_s * (o.priority == 1 ? 1.5 : (o.priority == 2 ? 1.0 : 0.75));
    const bool uncertain = !trusted || !o.velocity_known;
    const double age = std::max(0.0, in.now_s - o.observed_at_s);
    const double uncertainty =
        uncertain ? std::min(100000.0, c.unknown_obstacle_speed_mps *
                                           (age + caution_horizon))
                  : 0.0;
    double ttc =
        o.velocity_known && trusted
            ? path_ttc(path, position, o.velocity, speed, radius, c.horizon_s)
            : infinity;
    if (o.velocity_known && trusted)
      ttc = std::min(ttc, path_ttc(path, position, o.velocity, speed,
                                   radius + caution_margin, caution_horizon));
    double physical_ttc =
        o.velocity_known && trusted
            ? collision_time(sub(position, in.position),
                             sub(o.velocity, in.velocity), radius, c.horizon_s)
            : infinity;
    ttc = std::min(ttc, physical_ttc);
    const bool immediate =
        distance <= radius ||
        physical_ttc <= c.reaction_s + speed / c.deceleration_mps2;
    const bool relevant =
        proximity.distance <= radius + caution_margin + uncertainty ||
        finite(ttc) || immediate;
    if (!relevant)
      continue;
    threats.push_back(
        {i, immediate, uncertain, o.priority, ttc, distance,
         std::max(0.0,
                  proximity.along - uncertainty -
                      speed * std::max(0.0, caution_horizon - c.horizon_s))});
  }
  if (threats.empty()) {
    out.action = Action::proceed;
    out.reason = "no relevant obstacle in remaining route corridor";
    out.desired_speed_mps = speed;
    return out;
  }
  std::sort(
      threats.begin(), threats.end(), [](const Threat &a, const Threat &b) {
        return std::tuple{!a.immediate, a.priority, a.ttc, a.distance,
                          a.index} <
               std::tuple{!b.immediate, b.priority, b.ttc, b.distance, b.index};
      });
  for (const auto &threat : threats)
    out.threat_order.push_back(threat.index);
  const auto &first = threats.front();
  out.obstacle_index = first.index;
  // A lower priority uncertain obstacle still blocks a speculative detour.
  if (std::any_of(threats.begin(), threats.end(),
                  [](const Threat &t) { return t.immediate; }))
    return hold("immediate physical collision risk; brake and hold");
  if (std::any_of(threats.begin(), threats.end(), [&](const Threat &t) {
        return t.uncertain &&
               (t.along <= out.braking_distance_m + speed * c.horizon_s ||
                t.distance <= out.braking_distance_m + padding);
      }))
    return hold(
        "relevant obstacle lacks fresh trustworthy depth, pose or motion");
  bool approaching =
      std::any_of(threats.begin(), threats.end(), [&](const Threat &t) {
        return finite(t.ttc) ||
               t.along <= out.braking_distance_m + speed * c.horizon_s;
      });
  if (!approaching) {
    out.action = Action::slow;
    out.reason = "obstacle ahead outside immediate planning horizon";
    out.desired_speed_mps = std::min(speed, c.slow_speed_mps);
    return out;
  }
  out.action = Action::request_replan;
  out.reason = "obstacle blocks route; require a validated replacement mission";
  out.desired_speed_mps = 0;
  const auto &threat = in.obstacles[first.index];
  double travel =
      std::max(first.along + c.detour_offset_m * 2, c.detour_offset_m * 2);
  Vec3 rejoin = at_distance(path, travel);
  auto forward = sub(rejoin, in.position);
  forward.z = 0;
  double length = norm(forward);
  if (length < 1e-6 || std::abs(rejoin.z - in.position.z) > 1e-6 || speed < 0.1)
    return out;
  forward = mul(forward, 1 / length);
  Vec3 left{-forward.y, forward.x, 0};
  const double offset =
      std::max(c.detour_offset_m, padding + threat.radius_m + c.clearance_m);
  for (double side : {1.0, -1.0}) {
    auto shift = mul(left, offset * side);
    std::vector<Vec3> candidate{in.position, add(in.position, shift),
                                add(rejoin, shift), rejoin};
    if (checked_detour(in, c, candidate, std::max(c.slow_speed_mps, speed))) {
      out.detour = std::move(candidate);
      out.reason = "checked lateral candidate available; replacement mission "
                   "validation required";
      break;
    }
  }
  return out;
}

std::optional<Vec3> project_detection(const DetectionBox &box,
                                      std::optional<double> radial,
                                      const Camera &c) {
  if (!radial || !finite(*radial) || *radial <= 0 || *radial > 10000 ||
      !c.pose_valid || !finite(c.origin_enu) || c.width <= 0 || c.height <= 0 ||
      c.width > 100000 || c.height > 100000 || !finite(c.fx) || !finite(c.fy) ||
      c.fx <= 0 || c.fy <= 0 || !finite(c.cx) || !finite(c.cy) ||
      !finite(box.x1) || !finite(box.x2) || !finite(box.y1) ||
      !finite(box.y2) || box.x1 < 0 || box.y1 < 0 || box.x2 > c.width ||
      box.y2 > c.height || box.x2 <= box.x1 || box.y2 <= box.y1)
    return std::nullopt;
  const auto &r = c.camera_to_enu;
  for (double value : r)
    if (!finite(value))
      return std::nullopt;
  for (int row = 0; row < 3; ++row)
    for (int other = 0; other < 3; ++other) {
      double product = 0;
      for (int col = 0; col < 3; ++col)
        product += r[row * 3 + col] * r[other * 3 + col];
      if (std::abs(product - (row == other ? 1.0 : 0.0)) > 1e-6)
        return std::nullopt;
    }
  double det = r[0] * (r[4] * r[8] - r[5] * r[7]) -
               r[1] * (r[3] * r[8] - r[5] * r[6]) +
               r[2] * (r[3] * r[7] - r[4] * r[6]);
  if (std::abs(det - 1) > 1e-6)
    return std::nullopt;
  double pixel_x = (box.x1 + box.x2) / 2;
  if (c.mirrored)
    pixel_x = c.width - 1 - pixel_x;
  Vec3 ray{(pixel_x - c.cx) / c.fx,
           ((box.y1 + box.y2) / 2 - c.cy) / c.fy, 1};
  ray = mul(ray, *radial / norm(ray));
  Vec3 enu{r[0] * ray.x + r[1] * ray.y + r[2] * ray.z,
           r[3] * ray.x + r[4] * ray.y + r[5] * ray.z,
           r[6] * ray.x + r[7] * ray.y + r[8] * ray.z};
  auto result = add(c.origin_enu, enu);
  return finite(result) ? std::optional<Vec3>{result} : std::nullopt;
}
const char *action_name(Action a) {
  switch (a) {
  case Action::proceed:
    return "proceed";
  case Action::slow:
    return "slow";
  case Action::hold:
    return "hold";
  case Action::request_replan:
    return "request_replan";
  }
  return "hold";
}
} // namespace neurovector::avoidance
