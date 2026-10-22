#include "neurovector/mission.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <stdexcept>

int main(int argc,char** argv) {
    nv::MissionLimits limits; std::string expected; bool drone_id_set=false;
    try {
        for(int i=1;i<argc;++i) {
            std::string flag=argv[i];
            if(i+1>=argc) throw std::runtime_error("missing_flag_value");
            std::string value=argv[++i];
            if(flag=="--drone-id") {limits.airframe.id=value;drone_id_set=true;}
            else if(flag=="--fingerprint")expected=value;
            else {
                std::size_t end=0; double v=std::stod(value,&end);
                if(end!=value.size())throw std::runtime_error("invalid_flag_value");
                if(flag=="--max-altitude-m")limits.airframe.max_altitude_m=v;
                else if(flag=="--max-power-w")limits.airframe.max_power_watts=v;
                else if(flag=="--min-power-w")limits.minimum_power_w=v;
                else if(flag=="--battery-wh")limits.battery_wh=v;
                else if(flag=="--max-distance-m")limits.max_distance_m=v;
                else if(flag=="--max-leg-m")limits.max_leg_m=v;
                else if(flag=="--max-speed-mps")limits.max_speed_mps=v;
                else if(flag=="--max-duration-s")limits.max_duration_s=v;
                else if(flag=="--max-hold-s")limits.max_hold_s=v;
                else throw std::runtime_error("unknown_flag");
            }
        }
        if(!drone_id_set)throw std::runtime_error("local_drone_id_required");
        if(expected.size()!=16||expected.find_first_not_of("0123456789abcdef")!=std::string::npos)
            throw std::runtime_error("expected_lowercase_fingerprint_required");
        std::string text; text.reserve(65536); char c;
        while(std::cin.get(c)) {text.push_back(c);if(text.size()>1024*1024)throw std::runtime_error("plan_size_limit");}
        auto now=std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        auto report=nv::validate_mission(text,limits,now);
        if(report.route_fingerprint!=expected) {report.valid=false;report.errors.push_back("expected_fingerprint_mismatch");}
        std::cout<<report.json()<<'\n'; return report.valid?0:2;
    } catch(const std::exception& e) {
        nv::MissionReport report;report.drone_id=limits.airframe.id;report.errors.push_back(e.what());
        std::cout<<report.json()<<'\n';return 2;
    }
}
