# Fast disturbance reflex

`navigation::FastReflex` consumes calibrated wind, residual acceleration and angular
rate measurements before the slower visual navigation loop. It requests immediate
stabilization/hold for wind magnitude, rapid wind changes or physical disturbances.
Missing, future, reordered, conflicting or stale measurements block navigation.
Continuous fresh calm samples are required at startup and after every disturbance;
one quiet sample cannot release the hold. Use one instance per aircraft and clock.

This architecture is inspired by Fuller et al., 2014,
[Flying Drosophila stabilize their vision-based velocity controller by sensing wind
with their antennae](https://doi.org/10.1073/pnas.1323529111).
The study describes a fast antennal feedback loop alongside slower visual feedback.
It does not establish arbitrary attack detection or a drone escape controller.
We apply the parallel sensing principle, not the insect's literal motor response.

Run `evaluate()` on the high-rate sensor/control thread independently of image or
Wi-Fi processing, and immediately deliver `hold` to the autopilot's stabilization
supervisor. Inputs use the same monotonic clock. Wind must account for rotor wash,
vehicle motion and sensor orientation; acceleration must exclude gravity and
commanded acceleration. Raw accelerometer values are not suitable residuals.
Default thresholds are synthetic engineering starting points, not flight-calibrated
limits. The 50 ms age bound is a data freshness limit, not a proven reaction time.

Physical impacts, gusts and abrupt motion can trigger a hold; this cannot identify
intent, prevent an impact that has already happened, detect cyberattacks, or provide
unobserved lateral escape space. Existing collision checking and mission validation
remain necessary. No arbitrary evasive motor mixing is issued.
