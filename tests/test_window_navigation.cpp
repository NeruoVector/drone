#include "neurovector/window_navigation.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace nav = neurovector::navigation;
void require(bool condition) {
  if (!condition) throw std::runtime_error("window navigation regression");
}
nav::WindowRequest fixture() {
  nav::WindowRequest r;
  r.now_s = 10;
  r.floor = 3;
  r.destination = {{37, 127, 9000}, 0.2, 10, true};
  r.survey.origin = {37, 127, 100};
  r.survey.outward_normal = {0, -1, 0};
  r.survey.first_row_height_m = 1.5;
  r.survey.rows_complete = true;
  for (int floor = 1; floor <= 4; ++floor)
    for (int column = 0; column < 3; ++column)
      r.windows.push_back({std::to_string(floor) + "-" + std::to_string(column),
                            {column * 4.0, 0, 1.5 + 3 * (floor - 1)}, 0.05, 0.95, 10});
  return r;
}
int main() {
  auto r = fixture();
  auto result = nav::locate_window(r);
  require(result.target && result.target->id == "3-0");
  require(result.target->floor == 3 && result.target->hover_enu.z == 7.5);
  require(result.target->hover_enu.y == -4);
  // Latitude selects the facade; longitude chooses another window in the same row.
  r.destination.point.longitude_deg += 4 / (6378137 * std::cos(37 * std::acos(-1.0) / 180)) * 180 / std::acos(-1.0);
  result = nav::locate_window(r);
  require(result.target && result.target->id == "3-1");
  r.destination.horizontal_error_m = 3;
  require(!nav::locate_window(r).target);
  r = fixture();
  r.windows.erase(r.windows.begin(), r.windows.begin() + 3);
  require(!nav::locate_window(r).target);
  r = fixture();
  r.windows.erase(r.windows.begin() + 3, r.windows.begin() + 6);
  require(!nav::locate_window(r).target);
  r = fixture(); r.survey.rows_complete = false;
  require(!nav::locate_window(r).target);
  r = fixture(); r.floor = 5;
  require(!nav::locate_window(r).target);
  r = fixture(); r.destination.observed_at_s = 9;
  require(!nav::locate_window(r).target);
  r = fixture(); r.windows[0].observed_at_s = 11;
  require(!nav::locate_window(r).target);
  r = fixture(); r.windows[0].center_enu.z += 1;
  require(!nav::locate_window(r).target);
  r = fixture(); r.windows[0].center_enu.y = 2;
  require(!nav::locate_window(r).target);
  r = fixture(); r.windows[0].confidence = 0.2;
  require(!nav::locate_window(r).target);
  r = fixture(); r.windows[0].error_m = std::numeric_limits<double>::quiet_NaN();
  require(!nav::locate_window(r).target);
  r = fixture(); r.windows.push_back(r.windows[6]);
  require(!nav::locate_window(r).target);
  r = fixture(); r.survey.origin.longitude_deg = 180; r.destination.point.longitude_deg = -180;
  require(nav::locate_window(r).target.has_value());
  std::cout << "window navigation checks passed\n";
}
