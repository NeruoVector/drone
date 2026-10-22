#pragma once

#include "neurovector/drone_hal.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace nv {
struct Waypoint { double lat{}, lon{}, altitude_m{}; };
struct Zone { double ceiling_m{-1}; std::vector<Waypoint> polygon; };
struct MissionLimits {
    DroneConfig airframe;
    double max_distance_m{20000}, max_leg_m{5000}, max_speed_mps{15};
    double battery_wh{500}, minimum_power_w{120}, max_duration_s{3600};
    double max_hold_s{600};
};
struct MissionReport {
    std::string drone_id, route_fingerprint;
    bool valid{false}; std::size_t selected_index{};
    double distance_m{}, estimated_duration_s{}, estimated_energy_wh{};
    std::vector<std::string> errors;
    std::string json() const;
};
/// Exact NVPLAN1 bytes are fingerprinted. Limits must come from trusted local configuration.
MissionReport validate_mission(std::string_view text, const MissionLimits& limits,
                               std::int64_t now_unix);

} // namespace nv
