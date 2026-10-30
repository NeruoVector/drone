#include "mission_geometry.hpp"
#include <algorithm>

namespace nv::mission_detail {
// Private geographic and polygon geometry helpers used by mission validation.
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
static P sub(P a,P b) { return {a.x-b.x,a.y-b.y}; }
static double cross(P a,P b) { return a.x*b.y-a.y*b.x; }
static bool on(P p,P a,P b) {
    return std::abs(cross(sub(b,a),sub(p,a)))<=1e-12 && p.x>=std::min(a.x,b.x)-1e-12 &&
        p.x<=std::max(a.x,b.x)+1e-12 && p.y>=std::min(a.y,b.y)-1e-12 && p.y<=std::max(a.y,b.y)+1e-12;
}
static bool inside(P p,const std::vector<P>& poly) {
    bool hit=false;
    for(std::size_t i=0,j=poly.size()-1;i<poly.size();j=i++) {
        P a=poly[j],b=poly[i]; if(on(p,a,b)) return true;
        if((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) hit=!hit;
    } return hit;
}
static bool intersect(P p,P q,P c,P d) {
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

}
