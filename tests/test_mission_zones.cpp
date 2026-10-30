#include "neurovector/mission.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

void check(bool value, const char* message) {
    if (!value) { std::cerr << message << '\n'; std::exit(EXIT_FAILURE); }
}
std::string plan(const std::string& ceiling, const std::string& polygon) {
    return "NVPLAN2 plan drone-001 2000 0 1\n120 20000 5000 10 120 500 0\n"
           "0\n2\n0 0 10\n0 .002 10\n"
           "3\n0 0 10\n0 .001 10\n0 .002 10\n1\n" + ceiling + " 4\n" + polygon;
}
int main() {
    nv::MissionLimits limits;
    const std::string crossing="-.0001 .0014\n-.0001 .0016\n.0001 .0016\n.0001 .0014\n";
    const std::string touching="0 .0014\n0 .0016\n.0001 .0016\n.0001 .0014\n";
    auto blocked=nv::validate_mission(plan("-1",crossing),limits,1000);
    check(!blocked.valid && blocked.errors.size()==1 && blocked.errors[0]=="selected_route_not_minimum_valid_candidate", "second leg crosses zone");
    check(!nv::validate_mission(plan("-1",touching),limits,1000).valid, "zone edge contact must block");
    auto clear=nv::validate_mission(plan("5",crossing),limits,1000);
    check(clear.valid && clear.distance_m>200 && clear.selected_index==0 && clear.errors.empty(), "route above ceiling remains valid");
    check(!nv::validate_mission(plan("10",crossing),limits,1000).valid, "route at ceiling must block");
}
