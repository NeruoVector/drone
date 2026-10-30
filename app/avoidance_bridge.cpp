#include "neurovector/avoidance.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace a = neurovector::avoidance;
namespace {
double number(std::istream& s) { double v; if (!(s >> v) || !std::isfinite(v)) throw std::runtime_error("invalid number"); return v; }
bool flag(std::istream& s) { int v; if (!(s >> v) || (v != 0 && v != 1)) throw std::runtime_error("invalid flag"); return v == 1; }
std::size_t count(std::istream& s, std::size_t max) { long long n; if (!(s >> n) || n < 0 || static_cast<unsigned long long>(n) > max) throw std::runtime_error("invalid count"); return static_cast<std::size_t>(n); }
a::Vec3 vec(std::istream& s) { return {number(s), number(s), number(s)}; }
a::Box box(std::istream& s) { return {vec(s), vec(s)}; }
void print(const a::Decision& d) {
  std::cout << std::setprecision(17) << "{\"action\":" << std::quoted(a::action_name(d.action))
    << ",\"reason\":" << std::quoted(d.reason) << ",\"obstacle_index\":";
  if (d.obstacle_index) std::cout << *d.obstacle_index; else std::cout << "null";
  std::cout << ",\"threat_order\":[";
  for (std::size_t i=0; i<d.threat_order.size(); ++i) { if (i) std::cout << ','; std::cout << d.threat_order[i]; }
  std::cout << "],\"desired_speed_mps\":" << d.desired_speed_mps << ",\"braking_distance_m\":" << d.braking_distance_m << ",\"detour\":[";
  for (std::size_t i=0; i<d.detour.size(); ++i) { if (i) std::cout << ','; const auto& p=d.detour[i]; std::cout << '[' << p.x << ',' << p.y << ',' << p.z << ']'; }
  std::cout << "]}" << std::endl;
}
void process(const std::string& line) {
  std::istringstream s(line); std::string version; s >> version;
  if (version != "V1") throw std::runtime_error("invalid version");
  a::Input in; in.now_s=number(s); in.sensors_ready=flag(s); in.observations_at_s=number(s); in.position=vec(s); in.velocity=vec(s);
  for (auto n=count(s,256); n; --n) in.remaining_route.push_back(vec(s));
  for (auto n=count(s,128); n; --n) {
    a::Obstacle o; if (!(s >> o.priority)) throw std::runtime_error("invalid priority");
    o.confidence=number(s); o.observed_at_s=number(s); o.position_known=flag(s); o.position=vec(s); o.velocity_known=flag(s); o.velocity=vec(s);
    bool has_distance=flag(s); double distance=number(s); if (has_distance) o.distance_m=distance; o.radius_m=number(s); in.obstacles.push_back(o);
  }
  for (auto n=count(s,64); n; --n) in.no_fly_boxes.push_back(box(s));
  for (auto n=count(s,64); n; --n) { a::Coverage c; c.bounds=box(s); c.observed_at_s=number(s); in.coverage.push_back(c); }
  if (flag(s)) { a::Facade f; f.window=vec(s); f.outward_normal=vec(s); f.minimum_standoff_m=number(s); in.facade=f; }
  in.window_handoff=flag(s); s >> std::ws; if (!s.eof()) throw std::runtime_error("trailing input");
  print(a::decide(in));
}
}
int main() {
  constexpr std::size_t limit=1024*1024; std::string line; line.reserve(4096); bool overflow=false; char c;
  while (std::cin.get(c)) {
    if (c=='\n') {
      try { if (overflow) throw std::runtime_error("line too long"); process(line); }
      catch (...) { a::Decision d; d.reason="bridge invalid input"; print(d); }
      line.clear(); overflow=false;
    } else if (line.size()<limit) line.push_back(c); else overflow=true;
  }
}
