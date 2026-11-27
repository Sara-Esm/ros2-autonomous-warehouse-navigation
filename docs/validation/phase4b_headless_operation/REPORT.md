# Phase 4b: headless operation

Branch `feat/headless-operation`, from Phase 4 (`1b1f48d`). It adds independent launch options for the Gazebo
client, RViz and the ArUco debug window, with graphical defaults, and one instrumented five-step headless
mission compared with Phase 4.

Nothing in the camera architecture, controller parameters, map, station goals, timing configuration, goal
tolerances or mission logic was changed.

## 1. Result

| Measure (mission window) | Phase 4 (GUI) | **Phase 4b (headless)** |
|---|---|---|
| Mission / exit code / markers verified | 5/5 / 0 / 5 | **5/5 / 0 / 5** |
| Camera frames delivered (`camera_info`) | 4.16 Hz sim (3.16 wall) | **6.83 Hz sim (5.96 wall), +64%** |
| Detector frames processed | 1,501 (4.16 Hz sim) | **2,465 (6.83 Hz sim)** |
| Frame age at receipt p50 / p95 / max | 238 / 305 / 390 ms sim | **145 / 178 / 241 ms sim** |
| Time from arrival to verification, mean (range) | 0.23 s (0.12–0.38) | **0.09 s (0.05–0.11)** wall |
| RTF | 0.759 | **0.873** |
| Mission duration | 476 s wall / 361 s sim | **414 s wall** / 361 s sim |
| CPU busy, median / load average median | 93% / 27.4 | **82% / 12.0** |
| Memory used, median / available, minimum | 2,175 / 1,471 MiB | **1,581 / 2,126 MiB** |
| gzserver / gzclient / RViz / Nav2 container / detector (% of one core) | 292 / 194 / 101 / 68 / 16 | 498 / – / – / 72 / 29 |
| Station position error, max (mean of 5) | 0.111 m (0.070) | 0.107 m (0.085) |
| Heading error, range | −9.3 … +8.3° | −10.9 … +9.9° |
| AMCL residual median / p95 | 7.6 / 16.6 cm | 7.3 / 15.9 cm |
| Minimum clearance to true geometry | 0.32 m | **0.35 m** |
| Executed on static-unknown / live-unknown cells | 9.14 / 3.46 m | 9.28 / 3.38 m |
| Ack / service timeouts, halt errors, orphaned goals, overlap | 0 / 0 / 0 / 0 / 0 s | 0 / 0 / 0 / 0 / 0 s |
| Recoveries, planning failures, BT navigation failures | 0, 0, 0 | 0, 0, 0 |
| Controller missed-rate warnings / BT tick-rate overruns | 12 / 2,244 | **0 / 361** |

Sources: `data/perception_and_timing_vs_phase4.txt`, `phase1_results.txt`, `unknown_traversal.txt`,
`live_unknown_timing.txt`, `arrival_accuracy.txt`, `host_conditions.txt` and `runtime_params.txt`. The scripts
and definitions are the same as for Phases 3 and 4.

**The headless mode completed the mission and improved every performance measure. It met the RTF ≥ 0.8
target (0.873); the 10 Hz camera target was not met (6.83 Hz sim).** It is kept as an optional autonomous
execution mode; the graphical mode remains the default.

## 2. Launch options

| Argument | Default | Effect when `false` |
|---|---|---|
| `gui` | `true` | `gzserver` only, with no Gazebo client (Phase 4) |
| `use_rviz` | `true` | RViz is not started |
| `debug_window` | `true` | The detector runs without its OpenCV window |

```bash
ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py gui:=false use_rviz:=false debug_window:=false
```

- **RViz.** It was started unconditionally inside the vendored `turtlebot3_navigation2/launch/navigation2.launch.py`,
  which stays untouched. `warehouse_navigation.launch.py` now reproduces that file's two actions itself: the
  `nav2_bringup/bringup_launch.py` include with the same map, `use_sim_time` and parameters, and the same
  `rviz2` node and `tb3_navigation2.rviz` config. RViz is now conditional on `use_rviz`.
- **Debug window.** The detector's previously hard-coded `debug_window: True` now follows the argument.
- **Tests.** `test_headless_launch.py` (3 tests) pins the graphical defaults, the wiring and the equivalence with
  the vendored launch.

Everything else still runs headless:

- gzserver physics and camera sensor rendering;
- LiDAR, odometry, `robot_state_publisher` and TF;
- Nav2 and AMCL;
- `/camera/image_raw` transport, ArUco detection and mission supervision;
- the same recording as the Phase 3 and Phase 4 runs.

`runtime_params.txt` confirms: every Nav2 server on sim time, timeout 200 ms, keep-static BT, detector
`debug_window` false. The launch log contains no `gzclient` or `rviz2` process.

## 3. Historical GUI-free Trials 05–06

Both ran at **`00b70b6`** ("experiment: run Gazebo server without GUI"; branch
`experiment/headless-performance`, never merged). That commit is Trial 02's code (`3674bf1`) with one change:
`gazebo` → `gzserver` in the simulation launch. RViz and the debug window were still on. Results: Trial 05
0/5, Trial 06 3/5, RTF about 0.87 (`docs/trial_results.md`).

Failure signatures from the archived launch logs (`data/trials05_06_review.txt`; no bags were recorded):

| | Trial 04 (GUI) | Trial 05 | Trial 06 |
|---|---|---|---|
| Planning failures | 0 | 53 | 24 |
| No valid trajectories / failed to make progress | 0 / 1 | 22 / 14 | 4 / 3 |
| "None of the points of the global plan were in the local costmap" | 0 | 22 | 4 |

- Trial 05's planning failures targeted (−3.68, 9.30), (1.80, 9.30), (1.80, −9.00) and (−3.63, −9.30). Those are
  the **world-frame** station coordinates, sent as map-frame goals before the Phase 1 frame correction; on
  the original map they lay near or in obstacles and unknown space.
- Its first two legs ran for 180 s and 158 s before failing. The last three failed within about 26 s each,
  consistent with the robot being stuck or mislocalized after the early legs.

**Applicability now.** Every condition those trials ran under has since been changed:

- the goal frame (Phase 1);
- unknown space on the original map (Phases 2a and 2b);
- the Nav2 tuning (`3e8e1b7`);
- the readiness gate and supervision (`3de4301` onwards);
- the acknowledgement timeout and node clocks (Phase 3);
- the camera (Phase 4).

With no recordings, what made the 2026-11-24 headless runs fail cannot be established. *Hypothesis, not
established:* the higher RTF made the uncorrected goal-frame and map problems more consequential. Under the
current configuration, the one headless mission shows **no** degradation: 0 planning failures, 0
no-progress events and 0 recoveries, against 53 / 14 / 26 in Trial 05. One run does not establish a
reliability rate.

## 4. Why still below 10 Hz

Removing gzclient and RViz freed about 3 cores, and the load average fell from 27 to 12. gzserver absorbed
most of it (292% → 498%): physics now runs faster (RTF 0.87), and the camera, still rendered in software
(Phase 4 §2), delivers more frames per wall second (5.96 Hz). Measured in sim time, the rate rose 64%. The
remaining limit is software rasterization inside gzserver. Hardware rendering is not available on this host
(Phase 4 `gpu_check.txt`). Further gains would need that host change, a lower resolution or lighter scene
rendering. I stopped here, as the brief asks.

**Clock effect on the controller.** The controller loop ran at the same **wall** rate in both modes (9.38 Hz vs
9.34 Hz), so at the higher RTF it produced fewer commands per **sim** second: 10.74 Hz vs 12.30 Hz. That is
still above its configured 10 Hz, and it produced no missed-rate warnings (12 with the GUI). Positioning and
clearance did not degrade. Any further RTF gain would push the controller toward its configured rate, and that
should be watched.

## 5. Loop timing semantics (Nav2 1.1.18 sources; `data/loop_timing.txt`)

Every one of these loops is **paced by wall time**. `use_sim_time` changes only what `now()` returns:
timestamps, durations and timeouts. It does not change how often a loop runs.

| Component | Pacing mechanism (source) | Uses the ROS clock (sim time) for |
|---|---|---|
| `controller_server` | `rclcpp::WallRate(controller_frequency)`, 10 Hz (`controller_server.cpp:384`) | Command stamps, the `failure_tolerance` check |
| `velocity_smoother` | `create_wall_timer(1000/smoothing_frequency ms)`, 20 Hz (`velocity_smoother.cpp:137`); per-tick limits `accel / smoothing_frequency` | Only `last_command_time_` and the `velocity_timeout` (1 s) check |
| BT engine | `rclcpp::WallRate(bt_loop_duration)`, 100 Hz tick (`behavior_tree_engine.cpp:44`) | — |
| BT replanning (`RateController`, 1 Hz) | `std::chrono::high_resolution_clock`, a steady clock that `use_sim_time` does not affect (`rate_controller.cpp`) | — |
| BT action and service waits (`default_server_timeout`) | `std::chrono::milliseconds`, wall time | — |
| `costmap_2d_ros` update loop | `rclcpp::WallRate(update_frequency)`: global 1 Hz, local 5 Hz (`costmap_2d_ros.cpp:467`) | **Publishing:** only when `last_publish_ + 1/publish_frequency < now()` (`:488–494`) |
| DWB `sim_time` (1.5 s) | A prediction horizon in model time, not a timer | — |

**Measured: sim-clock frequency = wall-clock frequency / RTF.** For every loop in Phases 3, 4 and 4b, the
wall/sim frequency ratio equals the mission's RTF to three decimals:

| Loop | Phase 3 (RTF 0.711) | Phase 4 (RTF 0.759) | Phase 4b (RTF 0.873) |
|---|---|---|---|
| Controller (`/cmd_vel_nav`) | 9.04 Hz wall, 12.71 Hz sim | 9.34 / 12.30 | 9.38 / 10.74 |
| Smoother (`/cmd_vel`) | 18.85 / 26.54 | 18.91 / 24.90 | 18.96 / 21.71 |
| Replanning (`/plan`) | 0.89 / 1.26 | 0.91 / 1.19 | 0.91 / 1.04 |

**Consequence for control.** The controller produces about 9.4 commands per **wall** second whatever the
RTF. Raising the RTF therefore lengthens the controller's update interval in **simulation** time:
79 ms sim in Phase 3, 81 ms in Phase 4, **93 ms** in Phase 4b. Robot motion and DWB's prediction happen in sim
time, so a faster simulation means coarser control per simulated second. At RTF ≥ about 0.94 the sim-time
rate would fall below the configured 10 Hz.

**Global costmap publication interval (1.5 → 1.8 s sim).** The update loop iterates once per wall second;
publication waits until 1 s of **sim** time has passed. With RTF < 1, one iteration covers less than 1 s sim,
so the costmap is published every second iteration: **2.00 s wall** in all three missions. In sim time that is
2 × 1.00 s × RTF = 1.42, 1.52 and 1.75 s predicted, against 1.50, 1.50 and 1.80 s measured (header stamps,
within 0.08 s). The change from 1.5 to 1.8 s is the RTF increase, not a configuration change.

**The Phase 3 smoother null result, explained.** Phase 3 switched the smoother to sim time. In its timer
callback, the ROS clock appears only in the `velocity_timeout` check, which never fires while commands keep
arriving. Its pacing (a wall timer) and its acceleration limit per tick (`accel / 20`) do not depend on the
clock, so its output could not change.

One side effect: the smoother applies those per-tick limits about 18.9 times per **wall** second, so in sim
time the effective acceleration limit is about 0.95 / RTF × configured. That is 1.33× in Phase 3 and 1.08×
in Phase 4b. It matters for Phase 5 motion tuning.

## 6. Clarifications (added at finalization)

**Camera detection envelope.** The Phase 4 envelope (§5 and §5.1 of that report) depends on the robot's
**physical** position, its orientation, and where the marker is:

- Home dock, loading zone and dispatch have their marker about **0.67 m** beyond the goal; inventory scan has
  about **0.97 m**.
- All 30 measured arrivals (25 in Phases 1–4, 5 in this headless mission; dx −0.119 to 0.000 m) approached
  from the corridor side, facing the marker, and **stopped short** of the goal.
- A different approach direction, a larger localization error, or an overshoot could put the robot inside
  the near-side failure region: about 0.125 m or more closer than intended, combined with lateral and heading
  error.
- The fixed-route results therefore do **not** establish detection reliability for arbitrary routes or
  approach directions.

*Note added 2026-11-27 (Phase 5):* a Phase 5 controlled south-route session (forward-point distance 0.20–0.325 m,
yaw tolerance 0.07 rad) produced settled stops up to about **+0.025 m beyond** the intended goal. The
all-stops-short observation above **no longer holds universally**.

- That stop is outside the tested near-side failure region (about +0.125 m or more with lateral and heading
  error). The boundary depends on position, orientation and marker geometry.
- A possible coupling: a tighter yaw tolerance keeps the robot adjusting at the goal, which makes the final
  position more dependent on localization error. It has **not** been shown that the yaw tolerance alone caused
  the sign change.
- Phase 6 station verification must not assume the robot stops short of the goal (Phase 5 report §3.2).

**Residual unknown-space traversal** (`data/live_unknown_residual.txt`). This is the executed path on cells
that were unknown in the latest global-costmap snapshot **and still unknown in the next one**:

| | Phase 3 | Phase 4 | Phase 4b |
|---|---|---|---|
| Residual | 0.38 m | 0.29 m | 0.02 m |
| Range under one-cell (5 cm) alignment shifts | 0.00–0.65 m | 0.00–0.60 m | 0.00–0.61 m |

The residual is neither zero nor a proven upper bound on traversal of genuinely unobserved space. At the
resolution of the evaluation alignment it is a sub-metre quantity, between 0 and about 0.65 m per mission.

**Launch compatibility test.** The original string-based test (`test_navigation_reproduces_vendored_turtlebot3_launch`)
did **not** detect divergence of the vendored launch. In a mutation check (`data/launch_test_mutation_check.txt`) it
passed all four injected changes:

- an extra Nav2 include argument;
- a renamed RViz node;
- an added RViz keyword;
- a newly declared argument.

The added `test_navigation_matches_vendored_launch_structure` compares the include-argument keys, the RViz node
fields and the declared arguments structurally. It passes on the real file and fails on all four changes.

## 7. Shutdown

Headless teardown logs one `process has died` line, the ArUco detector exiting with code 1 on SIGINT. The
`gazebo` (exit 255) and `rviz2` (−6/−11) shutdown errors of the graphical runs do not occur because those
processes are not started. The detector's exit code on SIGINT is a pre-existing minor issue.

## 8. Tests

`colcon test` for the warehouse packages: **85 tests: 82 passed, 0 failed, 3 skipped**. The three skips are
the copyright tests, skipped by design. Phase 4 had 81 tests; `test_headless_launch.py` adds 4.

## Limitations

- **One headless mission.** It shows the configuration *can* complete reliably-looking runs under the current
  stack; it does not measure a reliability rate. Trials 05–06 show headless runs have failed before under
  other conditions.
- **No visual monitoring.** RViz and Gazebo are absent in this mode, by design. The recorded bag and logs are
  the record.
- **Host specificity.** The gains come from freeing CPU on a software-rendering host. On a host with GPU
  rendering, the GUI costs much less and the difference would be smaller.
- The unknown-space and detection-envelope limitations of Phase 4 apply unchanged.

## Reproduction

```bash
colcon build --packages-select warehouse_bringup
./run_phase4b_validation.sh            # (this folder's scripts/) writes diagnostics_phase4b/run (bag, logs, meta)
# analysis: the Phase 3 / Phase 4 scripts (perception_metrics.py, timing_metrics.py, phase1_evaluate.py,
# unknown_traversal_v2.py, live_unknown_timing.py, arrival_accuracy.py) on diagnostics_phase4b/run
```
