# Wi-Fi reflection obstacle observations

`navigation::localize_wifi()` converts a calibrated CSI path estimate into an
`avoidance::Obstacle`. Feed the result to the existing collision decision layer
alongside camera/depth detections; an invalid estimate remains unlocalized and
blocks motion rather than becoming free space. No new dependency is required.

Research checked: Tian et al., 2023,
[A Localization and Tracking System Using Single WiFi Link](https://doi.org/10.3390/rs15092461).
It estimates moving-target AoA, ToF and velocity using calibrated CSI and MUSIC.
Its controlled indoor evaluation and processing costs do not validate outdoor
drone collision avoidance. Static-path removal also means this moving-target
method must not be treated as a complete inventory of stationary obstacles.

This code implements the downstream geometric localization and collision input,
not the paper's CSI extraction, 3D MUSIC, Kalman or particle filtering pipeline.
It requires a CSI-capable antenna array/receiver and hardware-specific phase,
delay, orientation and clock calibration. The radio adapter must deliver a
resolved reflected path with full ENU azimuth and elevation, delay relative to the
direct Tx-Rx path, measurement time and calibrated error bounds. A linear array
that cannot resolve elevation or front/back ambiguity cannot set `path_resolved`.
RSSI, AP GPS coordinates and an ordinary Wi-Fi connection are insufficient.

Let `b = Tx - Rx`, `u` be the receiver-to-reflector unit bearing, and
`L = |b| + c * excess_delay`. Solving `|r*u-b| + r = L` gives
`r = (L*L - |b|*|b|) / (2*(L-dot(b,u)))`, and `position = Rx + r*u`.
The implementation encloses range and bearing uncertainty, including antenna
survey error, in a sphere and adds it to the physical obstacle radius. Degenerate
geometry or error above 10 m fails localization. Supplied error fields must be
actual bounds, not unscaled standard deviations. Moving receiver pose must match
the measurement time and use the same surveyed ENU frame as camera navigation.

Only a full tracked 3D velocity may set `tracked_velocity_enu_mps`. A single Doppler
value does not establish a stationary object or a complete velocity vector. Without
that tracker, existing unknown-motion reachable-region checks remain active.
Never mark camera coverage as free based only on an empty Wi-Fi observation list.
No real CSI hardware capture or outdoor radio accuracy has been verified here.
