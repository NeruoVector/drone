# Independent mission validation

`nv_route_check` is an offline decision tool. It does not download missions, authenticate a server, send HTTP reports, drive the existing HAL, or operate an autopilot. A navigation adapter still needs real position, altitude datum, heading, battery, flight-controller transport, continuously refreshed restrictions, and failsafe behavior before motor commands are possible.

Build without tests:

```bash
cmake -S drone -B build/drone -DNEUROVECTOR_BUILD_TESTS=OFF
cmake --build build/drone
```

The executable reads the backend mission's exact `plan_text` bytes from stdin and emits one JSON object on stdout. Exit status 0 means locally accepted; 2 means rejected. `errors` gives rejection reasons. Other JSON fields match `POST /api/v1/routes/plans/{id}/cross-check`.

Given a locally saved backend response `mission.json`, a trusted HTTP adapter can use this loop:

```bash
fingerprint=$(jq -r '.route_fingerprint' mission.json)
jq -jr '.plan_text' mission.json > plan.txt
build/drone/nv_route_check \
  --drone-id drone-001 --fingerprint "$fingerprint" \
  < plan.txt > cross-check.json
```

`jq -jr` preserves the existing final newline in `plan_text`; adding a newline changes the fingerprint. The adapter submits the saved report to the cross-check endpoint and may request dispatch only after the backend accepts it. A rejected report must never arm or dispatch the drone. Do not replace the locally configured drone ID or capability flags with values copied from an unverified mission.

The FNV-1a 64-bit fingerprint detects differing bytes; it is not authentication or a signature. The HTTP adapter must authenticate both directions and independently trust its expected fingerprint. A replay must also satisfy the mission expiry and local time checks.

## Local capabilities

The validator uses both server constraints and trusted local capabilities. Defaults come from `DroneConfig` where available: drone `drone-001`, altitude 120 m, maximum power 120 W. Additional local defaults are maximum total distance 20,000 m, maximum leg 5,000 m, maximum speed 15 m/s, battery budget 500 Wh, conservative modeled power floor 120 W, maximum mission duration 3,600 s, maximum window hold 600 s. These are configuration defaults for a decision tool, not measured airframe specifications. Deployment must set them to the actual aircraft and available battery including reserves. Supply a conservative power floor; overstating endurance by reducing modeled power is not permitted.

Supported local flags:

| Flag | Meaning |
| --- | --- |
| `--drone-id` | Independently provisioned aircraft identifier |
| `--fingerprint` | Expected 16 lowercase hexadecimal characters |
| `--max-altitude-m` | Maximum allowed altitude in the agreed mission datum |
| `--max-power-w` | Airframe maximum power |
| `--min-power-w` | Conservative power floor used to reject optimistic server energy models |
| `--battery-wh` | Locally available mission energy budget after reserves |
| `--max-distance-m` | Total route distance limit |
| `--max-leg-m` | Single leg distance limit |
| `--max-speed-mps` | Cruise speed capability |
| `--max-duration-s` | Full flight and hover duration limit |
| `--max-hold-s` | Outside-window hover duration limit |

The backend default modeled power is 300 W; the HAL default maximum is 120 W. Such a plan is intentionally rejected. Configure the backend model to match known hardware, or provision different local capabilities only when the actual airframe supports them. The tool never silently changes the aircraft limits to make a plan pass.

## Wire contract and independent checks

`NVPLAN1` is whitespace-delimited, locale-independent, and limited to 1 MiB. Numeric values must be finite. Its token order is:

```text
NVPLAN1 plan_uuid drone_id expires_unix selected_index candidate_count
max_altitude max_distance max_leg speed power battery hold_seconds
window_flag [window_lat window_lon window_alt outward_bearing standoff approach hold]
# Repeat candidate_count times:
waypoint_count
lat lon altitude
...
zone_count
# Repeat zone_count times:
ceiling vertex_count
lat lon
...
```

The comments above document the schema and are not literal protocol tokens. Zone ceiling `-1` means unbounded. Altitude and zone ceiling must use one agreed datum; mixing AGL and AMSL is unsafe and unsupported.

Counts are bounded: 1–16 candidates, 2–256 waypoints per candidate, 0–64 zones, 3–256 polygon vertices and at most 126 vertices overall. Antimeridian route legs and polygons are rejected. A missing zone is not inferred from imagery; the authenticated mission snapshot and local restriction updates must be complete.

The validator independently computes each leg as `hypot(haversine_distance, altitude_delta)` with Earth radius 6,371,000 m. Duration includes hover; energy is `power * duration / 3600`. It checks every leg against every polygon edge and polygon interior, including boundary touch. For a ceiling, it clips the leg to the interval at or below that ceiling before polygon intersection. It does not sample points and assume the space between them is clear.

Every candidate has the same departure and final point including altitude. The chosen index must be the first minimum-distance valid candidate, subject to local limits. Optimality is limited to the supplied bounded candidates; this is not proof of a globally shortest path. Server-supplied metrics are not accepted as evidence.

Missions must name the local aircraft and expire after local current time but within 1,800 s. Mission geometry is validated even when the expected hash matches.

For a window target, the final two waypoints must be the spherical offsets along its outward bearing at approach distance then hover standoff. Standoff is at least 2 m; approach is greater than standoff and at most 1,000 m; hover duration is 1–3,600 s and also subject to the stricter local hold limit. All route points must remain on the exterior side of the facade standoff plane, and the final altitude must equal the window altitude. This checks a planar facade model only: surrounding building geometry must also be supplied as restricted zones. No endpoint is inside the window. Actual handoff requires the separate navigation decision layer to hold outside and must never treat these coordinates as an instruction to enter the building.
