# Downward landing safety gate

Set `avoidance::Input.state.mode=FlightMode::landing` from the autopilot landing mode before
any landing descent, and keep it set through touchdown. `avoidance::decide()` now
requires a measured `landing_surface` and fresh downward `coverage`. An unsafe or
uncertain observation returns `hold`, zero speed and no lateral detour. This asks
the autopilot to brake the descent and stabilize; it does not disarm the motors.

Supply the aircraft, surface, observations and route in one ENU frame with one
monotonic clock. The landing surface includes its ground centre, horizontal patch
half-width/half-length, measured slope, roughness, bounded relative position error,
confidence and timestamp. `sensors_ready` requires a healthy calibrated downward
pipeline. `support_confirmed` requires evidence of a continuous, stable landing
surface over the entire patch. A hole, water, unstable vegetation or unknown support
must leave it false. Empty object detections do not establish support.

The existing `coverage` boxes must represent measured coverage with a complete
hazard inventory, including fresh empty frames. At least one box must cover the
entire horizontal clearance footprint and the descent column down to the measured
surface; do not fabricate this box from a camera's field of view. `obstacles` must
include downward detections, with depth/pose and tracked motion where available.
Use the existing calibrated `project_detection()` for camera observations. The
intended support plane is the surface, not an obstacle; objects on top of it remain
obstacles. Class priority does not excuse an object below the aircraft.

The gate checks small objects at ground level as well as people, animals and
vehicles. It checks moving objects against the entire descent column over the
estimated remaining descent duration (at least the normal planning horizon).
People/animals and unknown motion expand the reachable region using the configured speed bound;
missing, stale or low-confidence positions block landing. Unknown space, narrow
patches, excessive slope/roughness, drift, fast descent, facade and no-fly clearance
violations also block landing. Re-evaluate each sensor/control tick and whenever
the descent rate changes; this is not a one-time clearance to land.

Default `Config.landing` values: surface/coverage age <= 0.2 s, confidence >= 0.9,
slope <= 8 degrees, roughness <= 0.05 m, descent <= 0.5 m/s and horizontal drift
<= 0.3 m/s. Horizontal alignment including position error must fit within 0.25 m.
Configure `contact_height_m` from the aircraft's measured centre-to-landing-gear
height (default 0.5 m); the route ends that far above the surface. Descent exceeding
120 s at its current rate requires a new plan. These defaults need aircraft and
sensor calibration before physical flight.

`DeliveryNavigator` uses this landing gate with the existing fast disturbance and
Wi-Fi checks. During landing it does not require unrelated recipient-window GPS
evidence or compare the route to a window hover point. The caller must still supply
an authorized landing route. Any supplied facade protection remains active.
Landing and window handoff cannot be requested together.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/landing_safety_tests
```

The tests inject synthetic downward measurements and hazards through the public
API. They verify clear landings, hazards below, moving intruders, stale/missing
observations, unknown support, poor terrain, clearance and delivery-supervisor
integration. No downward sensor driver, surface classifier or autopilot connection
is present in this HAL; this change adds the decision gate, not real-world sensing
or flight validation. The caller must connect its landing mode and sensor outputs.
Fresh pose/estimator and control health in `flight.state` are also required;
see [system safety](system-safety.md).
