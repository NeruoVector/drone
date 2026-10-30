# Persistent native avoidance bridge, V1

Build with `cmake -S drone -B build/drone -DCMAKE_BUILD_TYPE=Release` then `cmake --build build/drone --target nv_avoidance_bridge nv_route_check`. Start `build/drone/nv_avoidance_bridge` once and exchange one newline-terminated request and one JSON response per frame. The process runs the project's `neurovector::avoidance::decide` with its default configuration; it does not drive a vehicle. An invalid, non-finite, oversized, or incomplete request returns a `hold` decision. A caller must also hold on a dead process, timeout, missing response, or malformed response.

Tokens are separated by whitespace; numbers must be finite and flags exactly `0` or `1`. Each request is bounded to 1 MiB:

```
V1 now_s sensors_ready observations_at_s position_xyz velocity_xyz
route_count [waypoint_xyz]...
obstacle_count [priority confidence observed_at_s position_known position_xyz velocity_known velocity_xyz distance_known distance_m radius_m]...
nofly_count [minimum_xyz maximum_xyz]...
coverage_count [minimum_xyz maximum_xyz observed_at_s]...
facade_present [window_xyz outward_normal_xyz minimum_standoff_m]
window_handoff
```

The entire request is one line. XYZ means three numbers. Include placeholder finite coordinates and distance even when their corresponding known flag is zero. The facade fields are present only when `facade_present` is 1. Counts are limited to 256 waypoints, 128 obstacles, 64 no-fly boxes, and 64 coverage boxes. Obstacles are indexed in input order; priority is 1, 2, or 3. Coordinates are in a fixed ENU frame in metres; velocities are metres per second. All timestamps are seconds from the same monotonic clock. A fresh observation heartbeat is required even for a frame with no detections. Coverage asserts fresh observed free space backed by a complete obstacle inventory; never invent it from field of view alone.

For example:

```
printf 'V1 100 1 100 0 0 5 0 0 0 1 10 0 5 0 0 0 0 0\n' | build/drone/nv_avoidance_bridge
```

The JSON response contains `action` (`proceed`, `slow`, `hold`, `request_replan`), `reason`, nullable `obstacle_index`, ordered `threat_order`, `desired_speed_mps`, `braking_distance_m`, and `detour`, an array of ENU XYZ arrays. Speed is an advisory ceiling, hold requires the controller to brake and maintain position, and replan requests a replacement mission. Any detour is advisory only: obtain backend and native mission validation and recheck live obstacles before execution. No-fly and facade constraints, obstacle priority, and fresh observation checks remain active. See [avoidance.md](avoidance.md) for the complete safety semantics.

`nv_route_check` reads the exact NVPLAN2 bytes on stdin, checks their fingerprint against `--fingerprint`, and validates using trusted local limits and `--drone-id`; its JSON result and exit status indicate validity. See [mission.md](mission.md). The caller must retain those exact bytes and must not treat a detour as authorized merely because the avoidance layer produced it.
