#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace neurovector::avoidance {

// East, north, up, in metres in one fixed local frame.
struct Vec3 {
  double x{}, y{}, z{};
};
struct Box {
  Vec3 minimum, maximum;
};
struct Coverage {
  Box bounds;
  double observed_at_s{};
};
struct Facade {
  Vec3 window;
  Vec3 outward_normal; // Unit horizontal ENU vector, pointing outside.
  double minimum_standoff_m{2.0};
};
struct Obstacle {
  std::string id;
  int priority{2}; // 1 people/body/animals, 2 vehicles/objects, 3 leaves.
  double confidence{};
  double observed_at_s{}; // Same monotonic clock as Input.now_s.
  bool position_known{};
  Vec3 position;
  bool velocity_known{};
  Vec3 velocity;
  std::optional<double> distance_m; // Camera radial distance, never optical Z.
  double radius_m{0.5};
};
struct Config {
  double drone_radius_m{0.5};
  double clearance_m{0.5};
  double max_age_s{0.5};
  double minimum_confidence{0.5};
  double horizon_s{5.0};
  double deceleration_mps2{2.0};
  double reaction_s{0.25};
  double slow_speed_mps{1.0};
  double detour_offset_m{4.0};
  double max_speed_mps{30.0};
  double unknown_obstacle_speed_mps{
      5.0}; // Conservative reachable-region bound.
};
struct Input {
  double now_s{};
  bool sensors_ready{}; // Explicitly false until the observation pipeline is
                        // healthy.
  double observations_at_s{}; // Fresh frame heartbeat, including frames with no
                              // detections.
  Vec3 position, velocity;
  // Remaining waypoints, in forward order, excluding already completed legs.
  std::vector<Vec3> remaining_route;
  std::vector<Obstacle> obstacles;
  std::vector<Box> no_fly_boxes;
  std::vector<Coverage> coverage;
  std::optional<Facade> facade;
  bool window_handoff{}; // Always stationary outside hover; never enter window.
};
enum class Action { proceed, slow, hold, request_replan };
struct Decision {
  Action action{Action::hold};
  std::string reason;
  std::optional<std::size_t> obstacle_index;
  std::vector<std::size_t> threat_order; // Ordered indices into Input.obstacles.
  double desired_speed_mps{};
  double braking_distance_m{};
  // Advisory candidate only. Caller must obtain mission
  // validation/authorization and re-evaluate live obstacles before using it;
  // this layer controls no motors.
  std::vector<Vec3> detour;
};
Decision decide(const Input &input, const Config &config = {});

struct Camera {
  double fx{}, fy{}, cx{}, cy{};
  int width{}, height{};
  // Row-major proper orthonormal rotation, OpenCV camera (right, down, forward)
  // into ENU, and camera centre in that same ENU frame.
  std::array<double, 9> camera_to_enu{};
  Vec3 origin_enu;
  bool pose_valid{};
  bool mirrored{}; // Undo horizontal image mirroring before using physical intrinsics.
};
struct DetectionBox {
  double x1{}, y1{}, x2{}, y2{};
};
// Failure leaves an obstacle unlocalized. Feed position_known=false to
// decide(); missing distance/pose must never become a fictitious zero-distance
// ENU point.
std::optional<Vec3> project_detection(const DetectionBox &box,
                                      std::optional<double> radial_distance_m,
                                      const Camera &camera);
const char *action_name(Action action);
} // namespace neurovector::avoidance
