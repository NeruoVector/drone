#include "neurovector/mission.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

static void check(bool value, const char* message) {
    if (!value) { std::cerr << message << '\n'; std::exit(EXIT_FAILURE); }
}
static std::string plan(const std::string& required, const std::string& candidates, const char* version = "NVPLAN2") {
    return std::string(version) + " plan drone-001 2000 0 1\n120 20000 5000 10 120 500 0\n0\n" + required + candidates + "0\n";
}
int main() {
    const std::string required = "4\n0 0 10\n0 .001 10\n0 .002 10\n0 0 10\n";
    const std::string correct = "4\n0 0 10\n0 .001 10\n0 .002 10\n0 0 10\n";
    const std::string missing = "3\n0 0 10\n0 .002 10\n0 0 10\n";
    const std::string reordered = "4\n0 0 10\n0 .002 10\n0 .001 10\n0 0 10\n";
    nv::MissionLimits limits;
    check(nv::validate_mission(plan(required, correct), limits, 1000).valid, "correct ordered route rejected");
    check(!nv::validate_mission(plan(required, missing), limits, 1000).valid, "missing stop accepted");
    check(!nv::validate_mission(plan(required, reordered), limits, 1000).valid, "reordered stops accepted");
    check(!nv::validate_mission(plan(required, correct, "NVPLAN1"), limits, 1000).valid, "legacy accepted");
    const std::string alternative = "NVPLAN2 plan drone-001 2000 0 2\n120 20000 5000 10 120 500 0\n0\n" + required + correct + missing + "0\n";
    check(!nv::validate_mission(alternative, limits, 1000).valid, "alternative omitted stop");
    std::cout << "mission stop tests passed\n";
}
