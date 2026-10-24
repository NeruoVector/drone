# Aircraft, mission and payload safety

`DeliveryNavigator` requires actual autopilot state and system status before
permitting a window approach or landing. Defaults block motion. Rebuild consumers
and populate the new fields; never substitute hardcoded healthy flags.

## Required inputs

- `flight.state`: actual `FlightMode`, pose/velocity/attitude validity, sample time,
  roll/pitch, motor health and thrust authority. The timestamp belongs to the same
  pose/velocity snapshot passed in `flight`, not a new camera heartbeat. Convert
  PX4 NED measurements to ENU. `thrust_available` means control capability, not a
  nonzero instantaneous throttle.
- `system`: fresh battery health, usable energy, energy needed to return, a positive
  reserve, command-link health and verified native watchdog enablement. Return
  estimates must account for aircraft load, wind and route. An emergency battery
  report overrides an apparently sufficient energy estimate.
- `handoff`: fresh recipient, clear receiving path, clear rotor space, secured
  payload and healthy mechanism confirmations, bound to `target_window_id`.
  These are verified interlocks, not consequences of geometric alignment.

Unknown mode is rejected. Landing checks are derived from
`flight.state.mode == FlightMode::landing`; the old `landing_requested` boolean was
removed. A transit descent is not automatically a landing. Bind mode to the actual
autopilot mode, and map unsupported/manual modes to unknown with operator override.

The standalone `avoidance::decide` checks fresh aircraft state, combined roll/pitch
tilt, live obstacles and braking geometry. Full energy/link/payload policy belongs
to `DeliveryNavigator`; a geometry result is not flight authorization.

## Required output handling

Inspect `DeliveryDecision.safety` before the navigation result:

| Directive | Required controller behavior |
|---|---|
| `continue_mission` | Follow only the authorized route, respecting the navigation ceiling. |
| `hold` | Ask the autopilot to brake and stabilize; do not stop motors in flight. |
| `delegate_to_autopilot` | Suspend this mission's commands and invoke/retain the configured native failsafe. Do not force position hold with invalid localization. |
| `landed` | Physical touchdown is confirmed; `disarm_allowed` is advisory permission for the autopilot. |

This layer does not operate motors or send PX4 commands. The adapter must implement
this handling and must not use `navigation.action` alone. Native/operator control
always takes priority.

The sibling `sim` checkout now supplies the `nv_safety_bridge` ROS 2 package for
actual PX4 state conversion, typed observations, intervention commands and ACKs.
See `sim/docs/safety-bridge.md` in the platform workspace. Physical observation
producers and the authorized route controller remain required.

Battery capacity must exceed return energy plus reserve. Invalid/stale aircraft
state, battery emergency, insufficient reserve, link loss, absent independent
watchdog, native failsafe and operator override delegate control to the autopilot.
Blocked approaches/handoffs have a waiting budget (default 30 s); expiry delegates
instead of hovering indefinitely. Delegation latches: clearing an input flag cannot
resume the old mission. Start a new supervisor only for an explicitly authorized
new mission after complete healthy preflight evidence.

The native controller must choose return, a validated alternate landing or another
emergency response using its remaining sensing and energy. This library does not
invent an unobserved escape route or declare an alternate landing site safe.

## Geometry and contact

Current-velocity reaction/braking travel is checked against facade half-spaces and
expanded no-fly boxes, even when the next waypoint points away. Calibrate reaction
and deceleration for load and weather. Window endpoints are checked before every
movement-permitting result, including slow. Original configuration is validated
before uncertainty inflation. Handoff alignment includes aircraft and target error
within the 25 cm budget.

Collision prediction includes stationary hover after the final waypoint for the
rest of the prediction horizon. Fresh system status and native/operator priority
also gate landed/disarm permission; physical contact cannot bypass that priority.

People/animals use a reachable-region bound during landing even with tracked zero
velocity. This can block landing for a person outside the immediate footprint;
replan or wait within the energy budget. Other known-motion objects retain the
constant-velocity model. Tune the speed bound to the site; classification/tracking
errors remain sensor limitations.

Nominal contact height or zero vertical speed cannot authorize motor shutdown.
`FlightMode::landed`, `ground_contact` and `landed_confirmed` must come from the
autopilot land detector, agree with low velocity and persist through fresh samples
for at least one second by default. A bounce, stale state or missing contact revokes
permission. No automatic payload release is introduced. Use one supervisor on one
control thread per aircraft/mission with coherent immutable sensor snapshots.

## Independent watchdog boundary

A watchdog in the process that may hang cannot protect against that process dying.
`independent_watchdog_enabled` must reflect read-back of the real autopilot's
configured Offboard/link-loss handling. It is a required precondition, not an
independent watchdog implemented in this C++ process.

For the installed PX4 version, configure and verify Offboard-loss timing/action,
battery return/landing thresholds, position-loss response and land detection.
Official references: [Offboard](https://docs.px4.io/main/en/flight_modes/offboard),
[safety configuration](https://docs.px4.io/main/en/config/safety),
[land detector](https://docs.px4.io/main/en/advanced_config/land_detector).
No parameters were applied to a vehicle. Test companion-process termination,
link loss, estimator loss and battery emergencies in disarmed SITL/HIL before flight.

After a CMake build, run `ctest --test-dir build --output-on-failure`,
`build/safety_regression_tests` and `build/nv_delivery_demo`. Tests inject typed
states through the real public APIs. Actual sensor adapters, watchdog configuration,
recipient recognition and payload hardware still require integration.

## Verification of the current working tree

GCC 14.2/C++20 with `-Wall -Wextra -Wpedantic -Werror`: seven CTest suites passed,
including 42 safety regression cases, 39 landing cases and 12 delivery demo cases.
The same seven suites passed AddressSanitizer and UndefinedBehaviorSanitizer.
The earlier three-case public-API reproduction now reports `failed_acceptance_cases=0/3`.
The school scenario runner passed 16 avoidance cases, two Rust/C++ route checks and
three rejected plans after its aircraft-state fixture was migrated. These results
cover synthetic software behavior, not actual aircraft failsafe execution.
