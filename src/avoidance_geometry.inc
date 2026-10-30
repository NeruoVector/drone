// Private geometry helpers used by the avoidance decision implementation.
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
