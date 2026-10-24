# Delivery navigation supervisor

`navigation::DeliveryNavigator` combines window rows/GPS, the fast disturbance
reflex, calibrated Wi-Fi obstacle observations, and existing visual collision checks
through `neurovector::nv_avoidance`. The public header is
`neurovector/delivery_navigation.hpp`. No third-party dependency was added.

Call `update()` on every high-rate sensor tick with cached visual observations and
CSI estimates. Capture and inference must run separately so they cannot delay the
reflex. A startup/recovery hold lasts until one second of continuous calm samples.
Only after that gate does the supervisor resolve the floor/window, add calibrated
localization errors to clearance, merge Wi-Fi obstacles, and check the live route.
Every sensor timestamp must use the same monotonic clock. The live pipeline must
report health even when its detections are empty. Wi-Fi contributes obstacles,
never a claim of observed free space.

The caller must supply an already authorized route validated by the backend and
`nv_route_check`. The supervisor does not infer authorization from waypoints. If
the route's endpoint differs by more than 25 cm from the newly resolved window
hover, it returns `request_replan` with zero speed and the new target separately.
Create, cross-validate and authorize a replacement mission before using it. Camera
and radio poses, survey origin, waypoints and aircraft localization must all share
the same ENU frame. `vehicle_position_error_m` is a calibrated positive error bound.

For handoff, the aircraft must be within 25 cm of the resolved hover and move at
most 0.1 m/s, with all evidence healthy and the collision decision clear. Then
`handoff_ready` becomes true while the requested action remains stationary `hold`
at zero speed. Approaching obstacles, ambiguous GPS/window evidence and gusts block
handoff. This is an advisory condition, not a payload-release command.

## Reproduce the integrated synthetic scenarios

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j4
ctest --test-dir build --output-on-failure
./build/nv_delivery_demo
```

The demo projects window boxes through the existing camera/depth helper, selects
floor 3 using row heights and GPS, and reports JSON lines for approach, wrong-window
route, ambiguous GPS, incomplete facade, Wi-Fi health, localized and ambiguous
reflections, exterior handoff, and gust override. It asserts each expected decision
and exits nonzero on a mismatch. It does not arm or fly a vehicle.

Hardware integration still needs a trained window detector, synchronized metric
depth/camera pose, surveyed floor anchor/facade, sufficiently accurate recipient GPS,
calibrated airflow/IMU, CSI extraction and full bearing/delay estimation, and an
autopilot supervisor that executes hold/replanning. The current HAL supplies a
simulated IMU, not these sensors. There is no claim of real flight, complete
protection from attacks, or the indoor Wi-Fi paper's accuracy in outdoor conditions.

Algorithm details and source papers: [window localization](window-navigation.md),
[fast reflex](fast-reflex.md), [Wi-Fi geometry](wifi-sensing.md).

The supervisor also supports [downward landing checks](landing-safety.md).
Set `flight.state.mode=FlightMode::landing` for landing and supply `flight.landing_surface`,
downward coverage and hazards. This uses the existing disturbance/Wi-Fi gates with
the authorized landing route; recipient-window evidence is not required in this mode.

The supervisor additionally requires fresh aircraft/system state and battery return
reserve, and verifies recipient/payload interlocks for handoff. Read the
[system safety contract](system-safety.md) before integrating it: consumers must
honor `DeliveryDecision.safety`, including native failsafe delegation and confirmed
touchdown. The former landing-request boolean was replaced by actual flight mode.

Verified on 2026-10-24 with GCC 14.2, C++20, Debug and
`-Wall -Wextra -Wpedantic -Werror`: five CTest suites passed, including 12 integrated
delivery cases. The same suites passed AddressSanitizer and UndefinedBehaviorSanitizer.
CMake installation and the installed demo were exercised. Existing school checks
also passed: 16 avoidance cases, two Rust/C++ route cross-checks, three rejected plans.
These are synthetic algorithm checks; Gazebo/PX4, radio capture and physical flight
were not exercised in this run.
