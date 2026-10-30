#include "mission_parse.hpp"
#include "mission_geometry.hpp"
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace nv::mission_detail {
double number(std::istream& in) {
    double value = read<double>(in);
    if (!finite(value)) throw std::runtime_error("nonfinite_number");
    return value;
}

std::size_t count(std::istream& in, std::size_t min, std::size_t max) {
    auto value = read<long long>(in);
    if (value < static_cast<long long>(min) || value > static_cast<long long>(max))
        throw std::runtime_error("count_limit");
    return static_cast<std::size_t>(value);
}

Waypoint waypoint(std::istream& in) {
    Waypoint value{number(in), number(in), number(in)};
    if (!geo_valid(value)) throw std::runtime_error("invalid_coordinate");
    return value;
}

bool identifier(std::string_view s) {
    return !s.empty() && s.size() <= 128 &&
        std::all_of(s.begin(), s.end(), [](unsigned char c) {
            return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                   (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
        });
}

std::string fingerprint(std::string_view text) {
    std::uint64_t hash = 0xcbf29ce484222325ULL;
    for (unsigned char c : text) {
        hash ^= c;
        hash *= 0x100000001b3ULL;
    }
    std::ostringstream out;
    out << std::hex << std::setfill('0') << std::setw(16) << hash;
    return out.str();
}
}
