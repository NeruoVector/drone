#include "neurovector/avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <tuple>

namespace neurovector::avoidance {
namespace {
constexpr double infinity = std::numeric_limits<double>::infinity();
bool finite(double v) { return std::isfinite(v); }
bool finite(Vec3 v) {
  return finite(v.x) && finite(v.y) && finite(v.z) &&
         std::abs(v.x) <= 1000000 && std::abs(v.y) <= 1000000 &&
         std::abs(v.z) <= 1000000;
}
Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 mul(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
bool valid_box(const Box &b) {
  return finite(b.minimum) && finite(b.maximum) && b.minimum.x <= b.maximum.x &&
         b.minimum.y <= b.maximum.y && b.minimum.z <= b.maximum.z;
}
bool fresh(double now, double observed, double age) {
  return finite(observed) && observed <= now + 0.05 && now - observed <= age;
}
bool contains(const Box &box, Vec3 p, double margin) {
  return p.x >= box.minimum.x + margin && p.x <= box.maximum.x - margin &&
         p.y >= box.minimum.y + margin && p.y <= box.maximum.y - margin &&
         p.z >= box.minimum.z + margin && p.z <= box.maximum.z - margin;
}
bool segment_box(Vec3 a, Vec3 b, const Box &box, double margin) {
  const double starts[]{a.x, a.y, a.z}, ends[]{b.x, b.y, b.z};
  const double mins[]{box.minimum.x - margin, box.minimum.y - margin,
                      box.minimum.z - margin};
  const double maxs[]{box.maximum.x + margin, box.maximum.y + margin,
                      box.maximum.z + margin};
  double lo = 0, hi = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const double d = ends[axis] - starts[axis];
    if (std::abs(d) < 1e-12) {
      if (starts[axis] < mins[axis] || starts[axis] > maxs[axis])
        return false;
    } else {
      double x = (mins[axis] - starts[axis]) / d,
             y = (maxs[axis] - starts[axis]) / d;
      if (x > y)
        std::swap(x, y);
      lo = std::max(lo, x);
      hi = std::min(hi, y);
      if (lo > hi)
        return false;
    }
  }
  return true;
}
bool outside(Vec3 p, const std::optional<Facade> &facade, double margin) {
  return !facade || dot(sub(p, facade->window), facade->outward_normal) >=
                        facade->minimum_standoff_m + margin;
}
struct Proximity {
  double distance{infinity}, along{infinity};
};
Proximity route_proximity(const std::vector<Vec3> &path, Vec3 p) {
  Proximity out;
  double travelled = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    auto delta = sub(path[i], path[i - 1]);
    double length = norm(delta);
    double t =
        length > 1e-9
            ? std::clamp(dot(sub(p, path[i - 1]), delta) / (length * length),
                         0.0, 1.0)
            : 0;
    double d = norm(sub(p, add(path[i - 1], mul(delta, t))));
    if (d < out.distance)
      out = {d, travelled + t * length};
    travelled += length;
  }
  return out;
}
// Entry time into two moving swept spheres, not just time of nearest approach.
double collision_time(Vec3 separation, Vec3 relative_velocity, double radius,
                      double duration) {
  const double c = dot(separation, separation) - radius * radius;
  if (c <= 0)
    return 0;
  const double a = dot(relative_velocity, relative_velocity),
               b = 2 * dot(separation, relative_velocity);
  if (a < 1e-12)
    return infinity;
  double discriminant = b * b - 4 * a * c;
  if (discriminant < 0)
    return infinity;
  double t = (-b - std::sqrt(discriminant)) / (2 * a);
  return t >= 0 && t <= duration ? t : infinity;
}
double path_ttc(const std::vector<Vec3> &path, Vec3 obstacle, Vec3 velocity,
                double speed, double radius, double horizon) {
  if (speed < 1e-6)
    return collision_time(sub(obstacle, path[0]), velocity, radius, horizon);
  double elapsed = 0;
  for (std::size_t i = 1; i < path.size() && elapsed < horizon; ++i) {
    auto delta = sub(path[i], path[i - 1]);
    double length = norm(delta);
    if (length < 1e-9)
      continue;
    double duration = std::min(length / speed, horizon - elapsed);
    auto separation = sub(add(obstacle, mul(velocity, elapsed)), path[i - 1]);
    double t =
        collision_time(separation, sub(velocity, mul(delta, speed / length)),
                       radius, duration);
    if (finite(t))
      return elapsed + t;
    elapsed += length / speed;
  }
  if (elapsed < horizon) {
    const auto separation = sub(add(obstacle, mul(velocity, elapsed)), path.back());
    const double t = collision_time(separation, velocity, radius, horizon - elapsed);
    if (finite(t))
      return elapsed + t;
  }
  return infinity;
}
Vec3 at_distance(const std::vector<Vec3> &path, double distance) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    double leg = norm(sub(path[i], path[i - 1]));
    if (distance <= leg && leg > 1e-9)
      return add(path[i - 1], mul(sub(path[i], path[i - 1]), distance / leg));
    distance -= leg;
  }
  return path.back();
}
bool checked_detour(const Input &in, const Config &c,
                    const std::vector<Vec3> &path, double speed) {
  double padding = c.drone_radius_m + c.clearance_m, total = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!outside(path[i - 1], in.facade, padding) ||
        !outside(path[i], in.facade, padding))
      return false;
    bool covered = false;
    for (const auto &coverage : in.coverage) {
      if (fresh(in.now_s, coverage.observed_at_s, c.max_age_s) &&
          contains(coverage.bounds, path[i - 1], padding) &&
          contains(coverage.bounds, path[i], padding)) {
        covered = true;
        break;
      }
    }
    if (!covered)
      return false;
    for (const auto &zone : in.no_fly_boxes)
      if (segment_box(path[i - 1], path[i], zone, padding))
        return false;
    total += norm(sub(path[i], path[i - 1]));
  }
  if (total / speed > c.horizon_s)
    return false;
  for (const auto &obstacle : in.obstacles) {
    if (!obstacle.position_known || !obstacle.velocity_known ||
        obstacle.confidence < c.minimum_confidence ||
        !fresh(in.now_s, obstacle.observed_at_s, c.max_age_s))
      return false;
    auto position = add(obstacle.position,
                        mul(obstacle.velocity,
                            std::max(0.0, in.now_s - obstacle.observed_at_s)));
    if (finite(path_ttc(path, position, obstacle.velocity, speed,
                        padding + obstacle.radius_m, c.horizon_s)))
      return false;
  }
  return true;
}
bool config_valid(const Config &c) {
  const double values[]{c.drone_radius_m,    c.clearance_m,
                        c.max_age_s,         c.horizon_s,
                        c.deceleration_mps2, c.reaction_s,
                        c.slow_speed_mps,    c.detour_offset_m,
                        c.max_speed_mps,     c.unknown_obstacle_speed_mps,
                        c.max_tilt_deg, c.landing.max_age_s, c.landing.max_slope_deg,
                        c.landing.max_roughness_m, c.landing.max_descent_mps,
                        c.landing.max_horizontal_mps, c.landing.alignment_m,
                        c.landing.contact_height_m, c.landing.max_duration_s};
  for (double v : values)
    if (!finite(v) || v <= 0 || v > 1000)
      return false;
  return c.max_tilt_deg < 90 && c.landing.max_slope_deg < 90 &&
         finite(c.landing.minimum_confidence) && c.landing.minimum_confidence >= 0.5 &&
         c.landing.minimum_confidence <= 1 && finite(c.minimum_confidence) && c.minimum_confidence >= 0 &&
         c.minimum_confidence <= 1;
}
struct LandingCheck {
  const char *reason{};
  std::optional<std::size_t> obstacle_index;
};
LandingCheck check_landing(const Input &in, const Config &c) {
  const auto &l = c.landing;
  if (!in.landing_surface)
    return {"landing requires valid limits and a measured downward surface", {}};
  const auto &s = *in.landing_surface;
  const double measurements[]{s.half_width_m, s.half_length_m, s.slope_deg,
                              s.roughness_m, s.position_error_m, s.confidence,
                              s.observed_at_s};
  for (double v : measurements)
    if (!finite(v))
      return {"invalid downward surface measurement", {}};
  const double age_limit = std::min(c.max_age_s, l.max_age_s);
  if (!s.sensors_ready || !s.support_confirmed || !finite(s.ground_point) ||
      s.observed_at_s < 0 || s.observed_at_s > in.now_s ||
      in.now_s - s.observed_at_s > age_limit ||
      s.confidence < l.minimum_confidence || s.confidence > 1 ||
      s.position_error_m <= 0 || s.position_error_m > l.alignment_m ||
      s.slope_deg < 0 || s.slope_deg > l.max_slope_deg ||
      s.roughness_m < 0 || s.roughness_m > l.max_roughness_m)
    return {"downward sensing or stable landing support is uncertain", {}};
  const double padding = c.drone_radius_m + c.clearance_m +
                         s.position_error_m + l.alignment_m;
  if (s.half_width_m < padding || s.half_length_m < padding)
    return {"landing surface is too small for aircraft clearance", {}};
  if (std::hypot(in.velocity.x, in.velocity.y) > l.max_horizontal_mps ||
      in.velocity.z < -l.max_descent_mps || in.velocity.z > 0.05)
    return {"landing horizontal or descent speed exceeds limits", {}};
  const double contact_z = s.ground_point.z + l.contact_height_m;
  auto aligned = [&](Vec3 p) {
    return std::hypot(p.x - s.ground_point.x, p.y - s.ground_point.y) +
               s.position_error_m <= l.alignment_m && p.z >= contact_z - 0.05;
  };
  if (!aligned(in.position))
    return {"aircraft is not aligned above the measured landing patch", {}};
  if (!outside(in.position, in.facade, padding))
    return {"landing clearance violates the protected facade", {}};
  double previous_z = in.position.z;
  for (auto p : in.remaining_route) {
    if (!aligned(p) || !outside(p, in.facade, padding) || p.z > previous_z + 0.05)
      return {"landing route leaves the vertical touchdown corridor", {}};
    previous_z = p.z;
  }
  if (std::abs(in.remaining_route.back().z - contact_z) > 0.05)
    return {"landing route does not end at calibrated contact height", {}};
  for (const auto &zone : in.no_fly_boxes)
    if (segment_box(in.position, {s.ground_point.x, s.ground_point.y, contact_z}, zone, padding))
      return {"landing clearance envelope intersects a no-fly zone", {}};
  bool covered = false;
  for (const auto &coverage : in.coverage) {
    const auto &b = coverage.bounds;
    if (coverage.observed_at_s >= 0 && coverage.observed_at_s <= in.now_s &&
        in.now_s - coverage.observed_at_s <= age_limit &&
        b.minimum.x <= s.ground_point.x - padding &&
        b.maximum.x >= s.ground_point.x + padding &&
        b.minimum.y <= s.ground_point.y - padding &&
        b.maximum.y >= s.ground_point.y + padding &&
        b.minimum.z <= s.ground_point.z + s.position_error_m &&
        b.maximum.z >= in.position.z)
      covered = true;
  }
  if (!covered)
    return {"downward observations do not cover the whole descent footprint", {}};
  const double height = std::max(0.0, in.position.z - contact_z);
  const double rate = in.velocity.z < 0 ? -in.velocity.z : l.max_descent_mps;
  const double duration = std::max(c.horizon_s, height / rate);
  if (!finite(duration) || duration > l.max_duration_s)
    return {"landing duration exceeds observed planning limits", {}};
  const Box column{{s.ground_point.x - padding, s.ground_point.y - padding,
                     s.ground_point.z - s.position_error_m},
                    {s.ground_point.x + padding, s.ground_point.y + padding,
                     in.position.z + c.drone_radius_m}};
  for (std::size_t i = 0; i < in.obstacles.size(); ++i) {
    const auto &o = in.obstacles[i];
    if (!o.position_known || o.confidence < std::max(c.minimum_confidence, l.minimum_confidence) ||
        o.observed_at_s < 0 || o.observed_at_s > in.now_s ||
        in.now_s - o.observed_at_s > age_limit)
      return {"landing obstacle position or freshness is uncertain", i};
    const double age = in.now_s - o.observed_at_s;
    const Vec3 position = o.velocity_known ? add(o.position, mul(o.velocity, age)) : o.position;
    const Vec3 predicted = o.velocity_known ? add(position, mul(o.velocity, duration)) : position;
    // People and animals can start moving even when their tracked velocity is zero.
    const double uncertainty = o.velocity_known && o.priority != 1 ? 0 :
        c.unknown_obstacle_speed_mps * (age + duration);
    if (segment_box(position, predicted, column, o.radius_m + uncertainty))
      return {"hazard below aircraft or moving into the landing corridor", i};
  }
  return {};
}
} // namespace

bool valid_config(const Config &c) { return config_valid(c); }
bool usable_vehicle_state(const Input &in, const Config &c) {
  const auto &s = in.state;
  switch (s.mode) {
  case FlightMode::transit: case FlightMode::window_delivery:
  case FlightMode::landing: case FlightMode::landed: break;
  case FlightMode::unknown: return false;
  default: return false;
  }
  return finite(in.now_s) && in.now_s >= 0 && finite(s.observed_at_s) &&
         s.observed_at_s >= 0 && s.observed_at_s <= in.now_s &&
         in.now_s - s.observed_at_s <= c.max_age_s && finite(in.position) &&
         finite(in.velocity) && s.position_valid && s.velocity_valid && s.attitude_valid &&
         s.motors_healthy && s.thrust_available && finite(s.roll_deg) && finite(s.pitch_deg) &&
         std::abs(s.roll_deg) <= c.max_tilt_deg && std::abs(s.pitch_deg) <= c.max_tilt_deg &&
         std::cos(s.roll_deg * std::numbers::pi / 180) *
             std::cos(s.pitch_deg * std::numbers::pi / 180) >=
             std::cos(c.max_tilt_deg * std::numbers::pi / 180);
}

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
  if (!usable_vehicle_state(in, c))
    return hold("aircraft mode, pose timestamp, estimator or control health invalid");
  if (in.state.mode == FlightMode::landed)
    return hold("autopilot reports landed; no navigation motion permitted");
  const bool landing = in.state.mode == FlightMode::landing;
  if (!in.sensors_ready || !fresh(in.now_s, in.observations_at_s, c.max_age_s))
    return hold("observation pipeline unavailable or heartbeat stale");
  double speed = norm(in.velocity);
  if (!finite(speed) || speed > c.max_speed_mps)
    return hold("invalid or excessive speed");
  out.braking_distance_m = speed * speed / (2 * c.deceleration_mps2) +
                           speed * c.reaction_s + c.clearance_m;
  const Vec3 stop_position = speed > 1e-6 ?
      add(in.position, mul(in.velocity, (speed * c.reaction_s +
          speed * speed / (2 * c.deceleration_mps2)) / speed)) : in.position;
  for (auto p : in.remaining_route)
    if (!finite(p) || norm(sub(p, in.position)) > 100000)
      return hold("invalid local route coordinate");
  for (const auto &b : in.no_fly_boxes)
    if (!valid_box(b))
      return hold("invalid no-fly geometry");
    else if (segment_box(in.position, stop_position, b, c.drone_radius_m + c.clearance_m))
      return hold("current braking trajectory intersects a no-fly zone");
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
    if (!outside(stop_position, in.facade, c.drone_radius_m + c.clearance_m))
      return hold("insufficient braking distance before protected facade");
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
  if (landing) {
    const auto landing = check_landing(in, c);
    if (landing.reason) {
      out.obstacle_index = landing.obstacle_index;
      return hold(landing.reason);
    }
  }
  if (threats.empty()) {
    out.action = Action::proceed;
    out.reason = landing ? "landing surface and full descent corridor verified" :
                                        "no relevant obstacle in remaining route corridor";
    out.desired_speed_mps = landing ? std::min(speed, c.landing.max_descent_mps) : speed;
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
