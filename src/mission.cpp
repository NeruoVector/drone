#include "neurovector/mission.hpp"
#include "mission_geometry.hpp"
#include "mission_parse.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace nv {
namespace {
using namespace mission_detail;
std::string quote(std::string_view s) {
    std::ostringstream o; o << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') o << '\\' << c;
        else if (c < 32) o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else o << c;
    } return o.str() + '"';
}

}
std::string MissionReport::json() const {
    std::ostringstream o; o<<std::setprecision(17)<<"{\"drone_id\":"<<quote(drone_id)<<",\"route_fingerprint\":"<<quote(route_fingerprint)
      <<",\"valid\":"<<(valid?"true":"false")<<",\"selected_index\":"<<selected_index<<",\"distance_m\":"<<distance_m
      <<",\"estimated_duration_s\":"<<estimated_duration_s<<",\"estimated_energy_wh\":"<<estimated_energy_wh<<",\"errors\":[";
    for(std::size_t i=0;i<errors.size();++i) { if(i)o<<','; o<<quote(errors[i]); } return o.str()+"]}";
}
MissionReport validate_mission(std::string_view text,const MissionLimits& local,std::int64_t now) {
    MissionReport report; report.drone_id=local.airframe.id; report.route_fingerprint=fingerprint(text);
    try {
        if(text.empty()||text.size()>1024*1024) throw std::runtime_error("plan_size_limit");
        std::istringstream in{std::string(text)}; in.imbue(std::locale::classic());
        if(read<std::string>(in)!="NVPLAN2") throw std::runtime_error("unsupported_protocol");
        auto plan_id=read<std::string>(in),drone=read<std::string>(in); auto expires=read<std::int64_t>(in);
        report.selected_index=count(in,0,15); auto n=count(in,1,16);
        if(!identifier(plan_id)||!identifier(drone)||drone!=local.airframe.id) throw std::runtime_error("mission_identity_mismatch");
        if(expires<=now || now<0 || expires-now>1800) throw std::runtime_error("mission_expired_or_invalid_ttl");
        if(report.selected_index>=n) throw std::runtime_error("selected_index_out_of_range");
        double altitude=number(in),maxdist=number(in),maxleg=number(in),speed=number(in),power=number(in),battery=number(in),hold=number(in);
        auto positive=[](double x){return finite(x)&&x>0;};
        if(!positive(local.airframe.max_altitude_m)||!positive(local.airframe.max_power_watts)||
           !positive(local.max_distance_m)||!positive(local.max_leg_m)||!positive(local.max_speed_mps)||
           !positive(local.battery_wh)||!positive(local.minimum_power_w)||!positive(local.max_duration_s)||
           !finite(local.max_hold_s)||local.max_hold_s<0 || local.minimum_power_w>local.airframe.max_power_watts)
            throw std::runtime_error("invalid_local_capabilities");
        if(!positive(altitude)||!positive(maxdist)||!positive(maxleg)||!positive(speed)||speed>100||
           !positive(power)||!positive(battery)||hold<0||hold>3600) throw std::runtime_error("invalid_constraints");
        if(altitude>local.airframe.max_altitude_m || maxdist>local.max_distance_m || maxleg>local.max_leg_m ||
           speed>local.max_speed_mps || power>local.airframe.max_power_watts || power<local.minimum_power_w ||
           battery>local.battery_wh || hold>local.max_hold_s) throw std::runtime_error("constraints_exceed_local_capabilities");
        auto window=count(in,0,1); Waypoint target{}; double bearing=0,standoff=0,approach=0,whold=0;
        if(window) {
            target=waypoint(in); bearing=number(in); standoff=number(in); approach=number(in); whold=number(in);
            if(target.altitude_m<=0||target.altitude_m>altitude||bearing<0||bearing>=360||standoff<2||approach<=standoff||approach>1000||
               whold<1||whold>3600||std::abs(whold-hold)>1e-9) throw std::runtime_error("invalid_window_target");
        } else if(hold!=0) throw std::runtime_error("hold_without_window");
        auto required_count=count(in,2,255);
        std::vector<Waypoint> required; required.reserve(required_count);
        for(std::size_t i=0;i<required_count;++i) required.push_back(waypoint(in));
        std::vector<std::vector<Waypoint>> paths; paths.reserve(n);
        for(std::size_t i=0;i<n;++i) { auto m=count(in,2,256); std::vector<Waypoint> p; p.reserve(m); for(std::size_t j=0;j<m;++j)p.push_back(waypoint(in)); paths.push_back(std::move(p)); }
        auto zn=count(in,0,64); std::vector<Zone> zones; zones.reserve(zn); std::size_t vertices=0;
        for(std::size_t z=0;z<zn;++z) {
            Zone zone; zone.ceiling_m=number(in); if(zone.ceiling_m<0&&zone.ceiling_m!=-1) throw std::runtime_error("invalid_zone_ceiling");
            auto m=count(in,3,256); vertices+=m; if(vertices>126) throw std::runtime_error("zone_vertex_limit");
            zone.polygon.reserve(m);
            for(std::size_t j=0;j<m;++j) { Waypoint w{number(in),number(in),0}; if(!geo_valid(w)) throw std::runtime_error("invalid_zone"); zone.polygon.push_back(w); }
            for(std::size_t j=0;j<m;++j) if(std::abs(zone.polygon[j].lon-zone.polygon[(j+1)%m].lon)>180) throw std::runtime_error("antimeridian_zone");
            zones.push_back(std::move(zone));
        }
        std::string extra; if(in>>extra) throw std::runtime_error("trailing_tokens");
        std::vector<std::vector<P>> projected_zones; projected_zones.reserve(zones.size());
        for (const auto& zone : zones) {
            auto& poly = projected_zones.emplace_back();
            poly.reserve(zone.polygon.size());
            for (const auto& w : zone.polygon) poly.push_back({w.lon, w.lat});
        }
        bool selected_valid=false; double best_distance=std::numeric_limits<double>::infinity();
        for(std::size_t i=0;i<n;++i) {
            const auto& p=paths[i]; bool valid=true; double dist=0;
            if(!same(p.front(),required.front())||!same(p.back(),required.back())) throw std::runtime_error("candidate_endpoints_differ");
            std::size_t stop=0;
            for(const auto& w:p) if(stop<required.size() && same(w,required[stop])) ++stop;
            if(stop!=required.size()) throw std::runtime_error("candidate_missing_required_stops");
            for(const auto& w:p) if(w.altitude_m<=0||w.altitude_m>altitude) valid=false;
            if(window) {
                if(p.size()<3||!same(p[p.size()-2],offset(target,bearing,approach))||!same(p.back(),offset(target,bearing,standoff))) valid=false;
                for(const auto& w:p) {
                    double north=rad(w.lat-target.lat)*earth,east=rad(w.lon-target.lon)*earth*std::cos(rad(target.lat));
                    if(north*std::cos(rad(bearing))+east*std::sin(rad(bearing))<standoff-.05) valid=false;
                }
            }
            for(std::size_t j=1;j<p.size();++j) {
                const auto&a=p[j-1];const auto&b=p[j];double leg=std::hypot(ground(a,b),b.altitude_m-a.altitude_m); dist+=leg;
                if(std::abs(a.lon-b.lon)>180||leg>maxleg) valid=false;
                for(std::size_t z=0; z<zones.size(); ++z) {
                    if(hits({a.lon,a.lat},{b.lon,b.lat},a.altitude_m,b.altitude_m,projected_zones[z],zones[z].ceiling_m)) valid=false;
                }
            }
            double duration=dist/speed+hold,energy=power*duration/3600;
            if(dist>maxdist||energy>battery||duration>local.max_duration_s||!finite(dist)||!finite(duration)||!finite(energy))valid=false;
            if(i==report.selected_index && finite(dist) && finite(duration) && finite(energy)) {
                report.distance_m=dist;report.estimated_duration_s=duration;report.estimated_energy_wh=energy;
            }
            if(i==report.selected_index) selected_valid=valid;
            if(valid&&dist<best_distance) best_distance=dist;
        }
        // Rust and C++ may round equivalent geographic paths differently.
        if(!selected_valid || report.distance_m>best_distance+1e-6) throw std::runtime_error("selected_route_not_minimum_valid_candidate");
        report.valid=true;
    } catch(const std::exception& e) { report.errors.push_back(e.what()); }
    return report;
}
} // namespace nv
