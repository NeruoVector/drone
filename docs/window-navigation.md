# Window row and GPS localization

`navigation::locate_window()` in `neurovector/window_navigation.hpp` resolves a
requested floor and destination GPS fix into a window centre and exterior hover
point in the surveyed ENU frame. Link `neurovector::nv_avoidance`.

Inputs are window centres produced by `avoidance::project_detection()` with
synchronized camera pose, intrinsics and measured radial depth, plus bounded
position errors. The stock COCO model does not detect windows: a trained window
detector or surveyed window observations are required. Do not count arbitrary
rectangles as windows or estimate floor height from GNSS altitude.

Survey the building's first visible window-row height, its actual floor number,
floor pitch, facade plane and outward normal. `rows_complete` is an explicit
confirmation that every floor from that anchor to the destination was observed.
Each row must agree with the surveyed pitch including its vertical error bound;
missing rows, duplicate IDs, stale observations and windows on other facades fail.
Several windows at one height belong to one floor. Split-level buildings, skipped
floor labels and varying pitches require another survey/model; this regular-row
implementation rejects inconsistent observations.

GPS selects horizontally within the requested row using WGS84 ECEF-to-ENU
coordinates (within 1 km of the origin, including longitude wrap). Supply a fresh
destination fix; a stored address coordinate must be checked against a fresh,
trusted site measurement before use. GPS altitude is ignored for floor selection.
The best window must remain unambiguous under the sum of the fix, survey and
window error bounds. Ordinary GNSS can fail this check near buildings: obtain a
better fix or recipient-specific visual confirmation instead of choosing randomly.
Confidence scores are not position-error bounds; the caller must calibrate both.

The output is a planning target, not a flight authorization. Plan and validate the
route to `hover_enu` through the existing backend and `nv_route_check`; pass the
returned facade to `avoidance::decide`. The centre standoff must cover the protected
distance plus aircraft radius, clearance and localization error. Keep a stationary
exterior handoff. No motor commands or indoor entry are introduced.

Run `cmake -S . -B build && cmake --build build && ctest --test-dir build
--output-on-failure`. Tests use synthetic observations, not a trained window model
or real GPS/depth data.
