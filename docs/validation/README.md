# Validation reports and per-phase results

| Phase | Report | Topic |
|---|---|---|
| v1.0.1 diagnostic | [`../diagnostics/REPORT.md`](../diagnostics/REPORT.md) | baseline investigation: frames, map, timing, camera, control |
| 1 | [`phase1_station_frames/REPORT.md`](phase1_station_frames/REPORT.md) | station frame correction |
| 2a | [`phase2a_map_unknown_space/REPORT.md`](phase2a_map_unknown_space/REPORT.md) | map unknown-space classification |
| 2b | [`phase2b_slam_remap/REPORT.md`](phase2b_slam_remap/REPORT.md) | SLAM re-map, surveyed goals, `allow_unknown: false` |
| 3 | [`phase3_timing_sync/REPORT.md`](phase3_timing_sync/REPORT.md) | acknowledgement timeout, node clocks, keep-static clears |
| 4 | [`phase4_perception_performance/REPORT.md`](phase4_perception_performance/REPORT.md) | camera bottleneck, pinhole camera, detection envelope, unknown space |
| 4b | [`phase4b_headless_operation/REPORT.md`](phase4b_headless_operation/REPORT.md) | headless mode, Nav2 loop timing semantics |
| 5 | [`phase5_motion_control/REPORT.md`](phase5_motion_control/REPORT.md) | motion smoothness, heading, clearance trade-off |
| 6 | [`phase6_mission_verification/REPORT.md`](phase6_mission_verification/REPORT.md) | settling, fresh marker evidence, signals, shutdown |
| 7 | [`phase7_integrated_validation/REPORT.md`](phase7_integrated_validation/REPORT.md) | six-mission repeatability, v1.1 comparison |

*The per-phase result summaries below were moved verbatim from the README at v1.1 (Phase 7). Figures are as measured in each phase; later phases may qualify them (see each report).*

## Per-phase results

Counted from the saved logs by `scripts/trial_metrics.py`.

| Run | Where | Code | Stations | Nav aborts / retries | Real-time factor |
|---|---|---|---|---|---|
| Trial 10 | Local, WSL2 | `3e8e1b7` (Nav2 tuning) | 5/5 | 0 / 0 | 0.67 |
| Trial 12 | Local, WSL2, heavy load | `0998813` (supervised manager) | 5/5, exit 0 | 1 / 1 (recovered) | 0.31 |
| [CI run 45570293516](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45570293516) | GitHub runner, Xvfb + software GL | `847f31c` | 5/5, exit 0 | 0 / 0 | 0.84 |
| [CI run 45579301270](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270) | GitHub runner, Xvfb + software GL | `d34a0de` | 5/5, exit 0 | - | - |

All four use the same Nav2 parameter file; for Trial 10 that is established from git
history, since its metadata predates the runner's `nav2_params_sha256` field. Both CI runs tested the current mission code;
`d34a0de` is the tree merged to `main`, and `847f31c` differs from it only in tests and
documentation. The trial script exits with the mission's exit code, and 0 means all five
tasks succeeded. The first CI run's logs are kept in
[`audit_logs/ci_run_45570293516/`](../../audit_logs/ci_run_45570293516/PROVENANCE.md). The second
run's logs are only in its GitHub artifact, so its abort and RTF columns are not filled in.

Earlier trials (01–08) record the baseline, a rejected headless experiment, and the failure
that led to the supervision work. Every trial, including unsuccessful and interrupted ones,
is listed with notes in [`docs/trial_results.md`](../../docs/trial_results.md).

### Station positioning (separate diagnostic experiment)

These numbers come from one instrumented diagnostic mission on v1.0.1 (2026-11-25), not from the trials
above. Ground truth is Gazebo's world pose of the robot at the moment the mission logged ARRIVED.

| Station | Distance from intended position | Heading error | Marker verified |
|---|---|---|---|
| Home Charging Dock | 0.50 m | +9.6° | yes |
| Loading Zone | 0.40 m | +9.5° | yes |
| Inventory Scan Station | 0.18 m | +8.9° | yes |
| Dispatch Zone | 0.32 m | −2.4° | yes |

A single rigid map-to-world transform (2.05° rotation plus a translation), fitted only to map geometry,
predicts these distances to within 0.03–0.11 m. The rigid transform explains most, not all, of the
discrepancy. Residual errors remain:

- systematic regional shifts of about 5 cm or less;
- residual position error with a median of 8.8 cm, a 95th percentile of 19.5 cm and a maximum of 25.0 cm;
- a worst regional median of 16.2 cm.

In this diagnostic experiment:

- Four station visits completed and passed ArUco identity verification.
- The fifth visit was incomplete because Gazebo crashed. Instrumentation interference is suspected, not confirmed.
- Motion after a successful final completion remains unresolved.
- The BT client's runtime clock was inferred rather than directly verified.
- Action-acknowledgement latency was not measured.

These results are separate from the successful historical missions in the table above. Methods,
measurements, limitations and an untested improvement backlog are in
[`docs/diagnostics/REPORT.md`](../../docs/diagnostics/REPORT.md).

### Station positioning after the frame correction (Phase 1 validation)

One validation mission on `feat/station-frame-correction`. Only the station goals changed; the Nav2
parameters, map and markers are unchanged. The run used lighter instrumentation than the diagnostic
experiment.

| Station | v1.0.1 diagnostic | Phase 1 position | Phase 1 heading | Marker verified |
|---|---|---|---|---|
| Home Charging Dock | 0.50 m, +9.6° | **0.04 m** | +6.9° | yes |
| Loading Zone | 0.40 m, +9.5° | **0.08 m** | −10.4° | yes |
| Inventory Scan Station | 0.18 m, +8.9° | **0.05 m** | +8.0° | yes |
| Dispatch Zone | 0.32 m, −2.4° | **0.10 m** | −7.0° | yes |
| Return to Home Charging Dock | not reached | **0.10 m** | −7.7° | yes |

Results:

- 5/5 tasks succeeded, exit code 0, no simulator crash.
- The mean position error on the four comparable stations fell from 0.35 m to 0.07 m.
- The mean heading bias fell from +6.4° to −0.6°. The heading magnitude stays at 7–10° because the goal
  checker accepts headings at the edge of its unchanged 8.6° tolerance.
- A residual turning motion settled 1.26 s after the final NavigateToPose success, which came before the
  MISSION COMPLETE report. The robot was then stationary (0.0 mm displacement, no velocity commands) for the
  remainder of the observation window, including 60 s after the mission report.
- The run was not error-free. Spin, Wait and BackUp recoveries still occurred, and each coincided with an
  action-acknowledgement timeout (16 in total), which belongs to a later phase. The detector and RViz
  exited with errors at teardown.

Details: [`docs/validation/phase1_station_frames/REPORT.md`](../../docs/validation/phase1_station_frames/REPORT.md).

### Unknown space after the map-classification correction (Phase 2a validation)

One validation mission on `feat/map-unknown-space`, compared with the Phase 1 mission. The only robot change
is the map's `free_thresh`, from 0.25 to 0.01. The map's unknown cells (60.1% of the image: unobserved or low-confidence) now load as
**unknown** instead of free: 3,772 occupied, 51,911 free and 83,919 unknown cells, verified with the real
`nav2_map_server` in the local and CI environments and regression-tested. The threshold is empirical: the
installed builds do not decode the PGM by the textbook formula, so 0.19 would have changed nothing.

The planner still **may** cross unknown space (`allow_unknown: true`), but now pays heavily for it: crossing
1 m of unknown costs as much as about 4–6 m of free travel. Prohibiting unknown space is deferred to a
re-mapping phase, because the current map leaves the north and south stations connected through observed
space only via a single unknown cell.

| Measure | Phase 1 | Phase 2a |
|---|---|---|
| Mission | 5/5, exit 0 | 5/5, exit 0 |
| Station position error, stations 1–4 mean / all five mean / max | 0.069 / 0.076 / 0.102 m | 0.057 / 0.055 / 0.103 m |
| First plans over unknown cells (unobserved or low-confidence) | 25.9 m | 5.7 m |
| Driven over unknown cells (unobserved or low-confidence) | 25.8 m (41%) | 15.8 m (23%), of which 5.1 m on cells still unknown in the live costmap |
| Return-home leg | 18.6 m | 24.7 m |
| Minimum clearance to true geometry | 0.32 m | 0.28 m (central-block corner) |
| Route changes > 0.5 m during a leg | 0 | 2 (one leg) |

This is a reduction, not a prevention, of unknown-space traversal. The robot also now takes longer, closer
routes through the one observed corridor. Details:
[`docs/validation/phase2a_map_unknown_space/REPORT.md`](../../docs/validation/phase2a_map_unknown_space/REPORT.md).

### Re-mapped warehouse and surveyed goals (Phase 2b validation)

One validation mission on `feat/slam-remap`. Changes:

- A new Cartographer map from a scripted, collision-aware coverage drive (two attempts; the second added
  targeted passes of the west aisle).
- Station goals surveyed in that map's frame.
- `allow_unknown: false`.

Camera, controller, timeouts and goal tolerances are unchanged.

| Measure | Phase 1 | Phase 2a | Phase 2b |
|---|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 |
| Station position error, stations 1–4 mean / max | 0.069 / 0.102 m | 0.057 / 0.103 m | 0.078 / 0.116 m |
| First plans over unknown cells of the map in use | 25.9 m | 5.7 m | 0.08 m |
| Return-home leg | 18.6 m | 24.7 m | 19.6 m |
| Minimum clearance to true geometry | 0.32 m | 0.28 m | 0.36 m |
| Route changes > 0.5 m during a leg | 0 | 2 | 1 (0.77 m) |
| Recoveries (Spin / Wait / BackUp) | 1 / 1 / 1 | 1 / 1 / 1 | 3 / 3 / 3 |

Results:

- Every leg now plans through known space. Two poses clip a thin low-confidence streak, an interpolation artifact.
- The return detour and the narrow central-corner passage are gone.
- Recoveries tripled. All of them followed action-acknowledgement timeouts.
- One new failure mode appeared: a planning failure right after a full costmap clear, which a timeout had
  triggered. *Hypothesis, not established:* the stricter unknown-space policy exposes such transient costmap
  states. It is the next phase's topic.

Details: [`docs/validation/phase2b_slam_remap/REPORT.md`](../../docs/validation/phase2b_slam_remap/REPORT.md).

### Timing and synchronization (Phase 3 validation)

One validation mission on `feat/timing-sync`. Changes:

- The behaviour tree's acknowledgement and service budget (`default_server_timeout`) goes from 20 to 200 ms.
  Measured acknowledgement latency was usually about 4 ms, but up to 86 ms.
- The velocity smoother now runs on simulation time; this had no measurable effect on its output.
- A copy of Nav2's behaviour tree whose global costmap clears keep the static map. This avoids a clear/plan
  race reproduced offline; it was not exercised in the mission.

| Measure | Phase 2b | Phase 3 |
|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 |
| Action-acknowledgement timeouts | 19 | 0 |
| Recoveries (Spin / Wait / BackUp) | 3 / 3 / 3 | 0 / 0 / 0 |
| Orphaned late-accepted goals | 15 | 0 |
| Mission time | 591 s | 509 s |
| Station position error, max | 0.116 m | 0.119 m |
| Minimum clearance to true geometry | 0.36 m | 0.31 m (same south pinch) |

Details: [`docs/validation/phase3_timing_sync/REPORT.md`](../../docs/validation/phase3_timing_sync/REPORT.md).

### Camera performance (Phase 4 validation)

The TurtleBot3 camera model used a 182° `wideanglecamera`. That sensor renders six views per frame and remaps
them, and this machine renders in software, so it delivered about 1 Hz instead of its configured 30 Hz. The
robot now uses a project copy of the model with a 100° pinhole camera. The resolution, rate, topics and
everything else are unchanged. On the bench, the marker was detected in every frame at all four stations, at
44 poses within ±0.12 m and ±11.5° of the goal. A wider sweep of 300 poses within ±0.25 m found no wrong IDs.
Detection failed only when the robot was at least 0.125 m closer to the marker than intended, combined with
lateral and heading offsets. All 25 real stops so far have been short of the goal, never closer to the marker.

| Measure | Phase 3 | Phase 4 |
|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 |
| Camera frames delivered and processed by the detector | 0.69 Hz sim | 4.16 Hz sim (6.0×) |
| Frame age, median | 1.43 s sim | 0.24 s sim |
| Arrival to marker verification, mean | 1.35 s | 0.23 s |
| Real-time factor | 0.71 | 0.76 |
| Memory used, median | 2,427 MiB | 2,175 MiB |
| Timeouts / recoveries | 0 / 0 | 0 / 0 |
| Station position error, max | 0.119 m | 0.111 m |

Details: [`docs/validation/phase4_perception_performance/REPORT.md`](../../docs/validation/phase4_perception_performance/REPORT.md).

### Headless operation (Phase 4b validation)

One mission with `gui:=false use_rviz:=false debug_window:=false`. The camera, Nav2 configuration, map, goals
and mission logic are unchanged.

| Measure | Phase 4 (GUI) | Phase 4b (headless) |
|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 |
| Camera frames delivered | 4.16 Hz sim | 6.83 Hz sim |
| Arrival to marker verification, mean | 0.23 s | 0.09 s |
| Real-time factor | 0.76 | 0.87 |
| Mission time | 476 s | 414 s |
| Memory used, median | 2,175 MiB | 1,581 MiB |
| Timeouts / recoveries | 0 / 0 | 0 / 0 |
| Station position error, max | 0.111 m | 0.107 m |
| Minimum clearance to true geometry | 0.32 m | 0.35 m |

Details: [`docs/validation/phase4b_headless_operation/REPORT.md`](../../docs/validation/phase4b_headless_operation/REPORT.md).

### Motion control (Phase 5 validation)

Two controller changes, each selected through controlled experiments:

- `yaw_goal_tolerance` 0.15 → 0.07 rad;
- DWB `PathAlign`/`GoalAlign` `forward_point_distance` 0.1 → 0.325 m.

Compared with three baseline headless missions:

| Measure | Baseline (3 missions) | Phase 5 |
|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 |
| Total absolute rotation | 2,254–2,366° | 1,948° |
| Angular command change per step, p95 | 0.103 rad/s | 0.051 rad/s |
| Settled heading error, mean / max | 5.9–8.8° / 9.5–10.8° | 4.2° / 5.8° |
| Station position error, max | 0.091–0.107 m | 0.114 m |
| **Footprint clearance at the south pinch** | **0.25–0.27 m** | **0.21 m** |

The clearance criterion was **not fully met**. The Phase 5 validation measured 0.21 m footprint clearance
against 0.25, 0.26 and 0.27 m in the baselines, accepted as a trade-off for the current simulated environment.
The obstacle-directed tracking deviation rises with the alignment distance. However, the Phase 6 mission, with
the identical configuration, measured 0.25 m, so the size and repeatability of the regression are unresolved.
Nothing here certifies the controller as safe.

Details: [`docs/validation/phase5_motion_control/REPORT.md`](../../docs/validation/phase5_motion_control/REPORT.md).

### Mission verification (Phase 6 validation)

Stricter completion rules in the mission node and detector; no Nav2, camera, map or goal change.

| Measure | Phase 5 | Phase 6 |
|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | 5/5, exit 0 / 5 |
| Settling verified after goal success | no | yes, 0.60–1.00 s sim |
| Marker evidence: image captured after settling / age at acceptance | not checked | yes / 44–124 ms |
| Arrival to verification, mean | 0.12 s | 1.10 s |
| Final stationary and Nav2-idle check | no | passed |
| Detector exit at shutdown | 1 (traceback) | 0 (clean) |
| System shutdown | runner SIGKILL after 30 s | project processes clean by 1.6 s; all down in 10.5–11.2 s (launch force-kills the upstream Nav2 container) |

Details: [`docs/validation/phase6_mission_verification/REPORT.md`](../../docs/validation/phase6_mission_verification/REPORT.md).

---
