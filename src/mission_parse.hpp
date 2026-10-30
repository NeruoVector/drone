#pragma once
#include "neurovector/mission.hpp"
#include <istream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace nv::mission_detail {
template<class T> T read(std::istream& in) {
    T value{};
    if (!(in >> value)) throw std::runtime_error("malformed_plan");
    return value;
}
double number(std::istream& in);
std::size_t count(std::istream& in, std::size_t min, std::size_t max);
Waypoint waypoint(std::istream& in);
bool identifier(std::string_view s);
std::string fingerprint(std::string_view text);
}
