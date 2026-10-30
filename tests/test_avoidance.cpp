#include "neurovector/avoidance.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
using namespace neurovector::avoidance;
void check(bool v, const char* message) { if (!v) { std::cerr << message << '\n'; std::exit(1); } }
Input base() { Input in; in.now_s=100; in.observations_at_s=100; in.sensors_ready=true; in.position={0,0,5}; in.velocity={2,0,0}; in.remaining_route={{10,0,5},{20,0,5}}; return in; }
Obstacle obstacle(int priority, Vec3 p) { Obstacle o; o.priority=priority; o.confidence=1; o.observed_at_s=100; o.position_known=true; o.position=p; o.velocity_known=true; o.radius_m=.5; return o; }
int main() {
  auto in=base(); check(decide(in).action==Action::proceed,"clear route");
  in.observations_at_s=99; check(decide(in).action==Action::hold,"stale heartbeat");
  in=base(); in.no_fly_boxes.push_back({{14,-1,4},{15,1,6}}); check(decide(in).action==Action::hold,"second segment no-fly");
  in=base(); in.window_handoff=true; check(decide(in).action==Action::hold,"window handoff");
  in=base(); in.facade=Facade{{0,0,5},{1,0,0},2}; check(decide(in).action==Action::hold,"facade standoff");
  in=base(); in.obstacles={obstacle(3,{8,0,5}),obstacle(1,{9,0,5})}; auto d=decide(in); check(d.action==Action::request_replan,"obstacles require replan"); check(d.threat_order.size()==2 && d.threat_order[0]==1 && d.threat_order[1]==0,"priority order"); check(d.detour.empty(),"no invented free-space coverage");
  in=base(); in.obstacles={obstacle(2,{8,0,5})}; in.coverage={Coverage{{{-10,-10,0},{30,10,10}},100}}; Config cdetour; cdetour.horizon_s=30; d=decide(in,cdetour); check(d.action==Action::request_replan && !d.detour.empty(),"covered candidate remains advisory replan");
  in.coverage[0].observed_at_s=99; d=decide(in,cdetour); check(d.detour.empty(),"stale coverage cannot authorize detour");
  in=base(); in.obstacles={obstacle(2,{1,0,5})}; check(decide(in).action==Action::hold,"immediate collision hold");
  in=base(); auto o=obstacle(2,{8,0,5}); o.observed_at_s=99; in.obstacles={o}; check(decide(in).action==Action::hold,"stale obstacle hold");
  Camera c; c.fx=100; c.fy=100; c.cx=50; c.cy=50; c.width=100; c.height=100; c.camera_to_enu={1,0,0,0,1,0,0,0,1}; c.origin_enu={1,2,3}; c.pose_valid=true;
  auto p=project_detection({40,40,60,60},2,c); check(p && std::abs(p->x-1)<1e-9 && std::abs(p->y-2)<1e-9 && std::abs(p->z-5)<1e-9,"radial projection");
  check(!project_detection({40,40,60,60},std::nullopt,c),"missing depth");
}
