#include "neurovector/mission.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace nv {
namespace {
constexpr double pi = 3.14159265358979323846, earth = 6371000.0;
bool finite(double x) { return std::isfinite(x); }
std::string quote(std::string_view s) {
    std::ostringstream o; o << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') o << '\\' << c;
        else if (c < 32) o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else o << c;
    } return o.str() + '"';
}
double rad(double d) { return d * pi / 180; }
bool geo_valid(const Waypoint& w) {
    return finite(w.lat) && finite(w.lon) && finite(w.altitude_m) &&
           w.lat >= -90 && w.lat <= 90 && w.lon >= -180 && w.lon <= 180;
}
double ground(const Waypoint& a, const Waypoint& b) {
    double x = std::pow(std::sin(rad(b.lat-a.lat)/2),2) +
        std::cos(rad(a.lat))*std::cos(rad(b.lat))*std::pow(std::sin(rad(b.lon-a.lon)/2),2);
    return 2 * earth * std::asin(std::sqrt(std::clamp(x,0.0,1.0)));
}
Waypoint offset(const Waypoint& w, double bearing, double distance) {
    double angle=distance/earth, b=rad(bearing), lat=rad(w.lat), lon=rad(w.lon);
    double l=std::asin(std::sin(lat)*std::cos(angle)+std::cos(lat)*std::sin(angle)*std::cos(b));
    double o=lon+std::atan2(std::sin(b)*std::sin(angle)*std::cos(lat),std::cos(angle)-std::sin(lat)*std::sin(l));
    return {l*180/pi,std::fmod(o*180/pi+540,360)-180,w.altitude_m};
}
bool same(const Waypoint& a,const Waypoint& b) {
    return std::abs(a.lat-b.lat)<1e-10 && std::abs(a.lon-b.lon)<1e-10 && std::abs(a.altitude_m-b.altitude_m)<1e-6;
}
struct P { double x,y; };
P sub(P a,P b) { return {a.x-b.x,a.y-b.y}; }
double cross(P a,P b) { return a.x*b.y-a.y*b.x; }
bool on(P p,P a,P b) {
    return std::abs(cross(sub(b,a),sub(p,a)))<=1e-12 && p.x>=std::min(a.x,b.x)-1e-12 &&
        p.x<=std::max(a.x,b.x)+1e-12 && p.y>=std::min(a.y,b.y)-1e-12 && p.y<=std::max(a.y,b.y)+1e-12;
}
bool inside(P p,const std::vector<P>& poly) {
    bool hit=false;
    for(std::size_t i=0,j=poly.size()-1;i<poly.size();j=i++) {
        P a=poly[j],b=poly[i]; if(on(p,a,b)) return true;
        if((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) hit=!hit;
    } return hit;
}
bool intersect(P p,P q,P c,P d) {
    P r=sub(q,p),s=sub(d,c),delta=sub(c,p); double den=cross(r,s);
    if(std::abs(den)<=1e-12) return on(c,p,q)||on(d,p,q)||on(p,c,d)||on(q,c,d);
    double t=cross(delta,s)/den,u=cross(delta,r)/den;
    return t>=-1e-12 && t<=1+1e-12 && u>=-1e-12 && u<=1+1e-12;
}
bool hits(P a,P b,double za,double zb,const std::vector<P>& poly,double ceiling) {
    double lo=0,hi=1;
    if(ceiling>=0) {
        if(za>ceiling && zb>ceiling) return false;
        double dz=zb-za;
        if(dz>0) hi=std::min(hi,(ceiling-za)/dz);
        if(dz<0) lo=std::max(lo,(ceiling-za)/dz);
        if(lo>hi) return false;
    }
    P p{a.x+(b.x-a.x)*lo,a.y+(b.y-a.y)*lo},q{a.x+(b.x-a.x)*hi,a.y+(b.y-a.y)*hi};
    if(inside(p,poly)||inside(q,poly)) return true;
    for(std::size_t i=0;i<poly.size();++i) if(intersect(p,q,poly[i],poly[(i+1)%poly.size()])) return true;
    return false;
}
template<class T> T read(std::istream& in) { T x{}; if(!(in>>x)) throw std::runtime_error("malformed_plan"); return x; }
double number(std::istream& in) { double v=read<double>(in); if(!finite(v)) throw std::runtime_error("nonfinite_number"); return v; }
std::size_t count(std::istream& in,std::size_t min,std::size_t max) {
    auto n=read<long long>(in); if(n<static_cast<long long>(min)||n>static_cast<long long>(max)) throw std::runtime_error("count_limit"); return static_cast<std::size_t>(n);
}
Waypoint waypoint(std::istream& in) {
    Waypoint w{number(in),number(in),number(in)}; if(!geo_valid(w)) throw std::runtime_error("invalid_coordinate"); return w;
}
bool identifier(std::string_view s) {
    return !s.empty() && s.size()<=128 && std::all_of(s.begin(),s.end(),[](unsigned char c){return (c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.';});
}
std::string fingerprint(std::string_view text) {
    std::uint64_t h=0xcbf29ce484222325ULL;
    for(unsigned char c:text) { h^=c; h*=0x100000001b3ULL; }
    std::ostringstream o; o<<std::hex<<std::setfill('0')<<std::setw(16)<<h; return o.str();
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
        if(read<std::string>(in)!="NVPLAN1") throw std::runtime_error("unsupported_protocol");
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
        std::vector<std::vector<Waypoint>> paths; paths.reserve(n);
        for(std::size_t i=0;i<n;++i) { auto m=count(in,2,256); std::vector<Waypoint> p; for(std::size_t j=0;j<m;++j)p.push_back(waypoint(in)); paths.push_back(std::move(p)); }
        auto zn=count(in,0,64); std::vector<Zone> zones; std::size_t vertices=0;
        for(std::size_t z=0;z<zn;++z) {
            Zone zone; zone.ceiling_m=number(in); if(zone.ceiling_m<0&&zone.ceiling_m!=-1) throw std::runtime_error("invalid_zone_ceiling");
            auto m=count(in,3,256); vertices+=m; if(vertices>126) throw std::runtime_error("zone_vertex_limit");
            for(std::size_t j=0;j<m;++j) { Waypoint w{number(in),number(in),0}; if(!geo_valid(w)) throw std::runtime_error("invalid_zone"); zone.polygon.push_back(w); }
            for(std::size_t j=0;j<m;++j) if(std::abs(zone.polygon[j].lon-zone.polygon[(j+1)%m].lon)>180) throw std::runtime_error("antimeridian_zone");
            zones.push_back(std::move(zone));
        }
        std::string extra; if(in>>extra) throw std::runtime_error("trailing_tokens");
        bool selected_valid=false; double best_distance=std::numeric_limits<double>::infinity();
        for(std::size_t i=0;i<n;++i) {
            const auto& p=paths[i]; bool valid=true; double dist=0;
            if(!same(p.front(),paths[0].front())||!same(p.back(),paths[0].back())) throw std::runtime_error("candidate_endpoints_differ");
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
                for(const auto& zone:zones) {
                    std::vector<P> poly; for(const auto&w:zone.polygon)poly.push_back({w.lon,w.lat});
                    if(hits({a.lon,a.lat},{b.lon,b.lat},a.altitude_m,b.altitude_m,poly,zone.ceiling_m)) valid=false;
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
