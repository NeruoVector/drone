#pragma once
#include "neurovector/mission.hpp"
#include <cmath>
#include <vector>

namespace nv::mission_detail {
inline constexpr double pi = 3.14159265358979323846;
inline constexpr double earth = 6371000.0;
inline bool finite(double x) { return std::isfinite(x); }
struct P { double x, y; };
double rad(double degrees);
bool geo_valid(const Waypoint& waypoint);
double ground(const Waypoint& a, const Waypoint& b);
Waypoint offset(const Waypoint& waypoint, double bearing, double distance);
bool same(const Waypoint& a, const Waypoint& b);
bool hits(P a, P b, double za, double zb, const std::vector<P>& polygon, double ceiling);
}
