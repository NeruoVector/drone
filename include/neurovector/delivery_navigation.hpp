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
struct SystemStatus {
  double observed_at_s{};
  double usable_energy_wh{}, return_energy_wh{}, reserve_energy_wh{};
  bool battery_healthy{}, command_link_healthy{}, independent_watchdog_enabled{};
  bool native_failsafe_active{}, operator_override{};
};
struct HandoffStatus {
  double observed_at_s{};
  bool recipient_confirmed{}, receiving_path_clear{}, rotor_space_clear{};
  bool payload_secure{}, mechanism_healthy{};
  std::string target_window_id;
};
struct SafetyLimits {
  double max_status_age_s{0.5}, max_wait_s{30}, contact_confirmation_s{1};
};
enum class SafetyDirective { hold, continue_mission, delegate_to_autopilot, landed };
const char *safety_name(SafetyDirective directive);
struct DeliveryInput {
  avoidance::Input flight;
  WindowRequest destination;
  std::optional<DisturbanceSample> disturbance;
  WifiFrame wifi;
  double vehicle_position_error_m{};
  SystemStatus system;
  HandoffStatus handoff;
};
struct DeliveryDecision {
  std::optional<WindowTarget> target;
  avoidance::Decision navigation;
  bool handoff_ready{};
  SafetyDirective safety{SafetyDirective::hold};
  bool disarm_allowed{}; // Advisory only, after confirmed physical touchdown.
};
class DeliveryNavigator {
public:
  explicit DeliveryNavigator(ReflexConfig reflex = {}, SafetyLimits limits = {})
      : reflex_(reflex), limits_(limits) {}
  // Call on each fast sensor tick with the latest cached visual/CSI observations.
  // The caller supplies an already validated/authorized route, never raw motor commands.
  DeliveryDecision update(const DeliveryInput &input, const avoidance::Config &config = {});
private:
  FastReflex reflex_;
  SafetyLimits limits_;
  std::optional<double> last_now_, hold_since_, contact_since_, last_contact_sample_;
  bool failsafe_latched_{};
};
} // namespace neurovector::navigation
