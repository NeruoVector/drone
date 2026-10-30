// Private detour and configuration safety checks for avoidance decisions.
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
                        c.max_speed_mps,     c.unknown_obstacle_speed_mps};
  for (double v : values)
    if (!finite(v) || v <= 0 || v > 1000)
      return false;
  return finite(c.minimum_confidence) && c.minimum_confidence >= 0 &&
         c.minimum_confidence <= 1;
}
