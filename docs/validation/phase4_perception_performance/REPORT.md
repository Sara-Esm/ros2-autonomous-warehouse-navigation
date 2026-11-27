# Phase 4: perception and computational performance

Branch `feat/perception-performance` (from Phase 3, `3817e8b`). One change to the robot model, one optional
launch argument, focused bench experiments and one instrumented five-step mission compared with Phase 3.
Nothing in the Phase 2b map, surveyed goals, Phase 3 timing parameters, unknown-space policy, DWB tuning, goal
tolerances or mission supervision was changed.

## 1. Result

| Measure (mission window) | Phase 3 | **Phase 4** |
|---|---|---|
| Mission / markers verified | 5/5, exit 0 / 5 | **5/5, exit 0 / 5** |
| Camera frames delivered (`camera_info`) | 0.69 Hz sim (0.49 wall) | **4.16 Hz sim (3.16 wall), 6.0×** |
| Frame interval p50 / max (header stamps) | 1,434 / 2,123 ms sim | **239 / 394 ms sim** |
| Frame age at receipt p50 / p95 | 1,433 / 1,715 ms sim | **238 / 305 ms sim** |
| Frames processed by the detector | 250 (0.69 Hz sim) | **1,501 (4.16 Hz sim)** |
| Time from arrival to verification, mean (range) | 1.35 s (0.25–2.66) | **0.23 s (0.12–0.38)** wall |
| RTF | 0.711 | **0.759** |
| Mission time | 509 s wall | **476 s wall** (361 vs 362 s sim) |
| CPU busy, median / load average median | 94% / 28.8 | 93% / 27.4 |
| Memory used, median / available, minimum | 2,427 / 1,212 MiB | **2,175 / 1,471 MiB** |
| gzserver / gzclient / RViz / Nav2 container / detector (% of one core) | 324 / 191 / 103 / 72 / 3.5 | 292 / 194 / 101 / 68 / 15.5 |
| Station position error, max (stations 1–4 mean) | 0.119 m (0.085) | **0.111 m (0.063)** |
| Ack / service-call timeouts, halt errors, orphaned goals, overlap | 0 / 0 / 0 / 0 / 0 s | 0 / 0 / 0 / 0 / 0 s |
| Recoveries, planning failures, BT navigation failures | 0, 0, 0 | 0, 0, 0 |
| Controller missed-rate warnings | 51 | 12 |
| Minimum clearance to true geometry | 0.31 m | 0.32 m (same south pinch) |
| Executed path on static-unknown cells | 15.6% of 63.0 m | 14.5% of 63.1 m |

**Summary:**

- The detector processed **6.0× more frames** per second of sim time (0.69 → 4.16 Hz).
- Image age fell from 1.43 to 0.24 s sim.
- Marker verification after arrival fell from 1.35 to 0.23 s.

Phase 4 delivers 6× the camera frames at one sixth of the age. Verification is now immediate, RTF is higher,
memory use is lower, and the Phase 3 timing gains are intact. The runtime parameters are the same: timeout
200 ms, keep-static BT path, every Nav2 server on sim time (`data/mission/runtime_params.txt`). The desirable
target of **≥ 10 Hz was not reached in the mission (4.16 Hz)**. RTF 0.76 approaches the 0.8 target. §4
explains the limit.

## 2. Diagnosis: why the camera delivered about 1 Hz

Bench sessions (`scripts/run_session.sh`) ran the warehouse world with the Gazebo GUI, as in the mission, and
one robot-model variant at the loading-zone approach pose. There was no Nav2 and no RViz. Each session had
three 40 s stages: camera only, plus the detector, plus the detector with its debug window. A probe counted
every published frame (RELIABLE, depth 50) and recorded `/clock`. The rates are stage medians from
`data/bench/rate_sessions.txt`.

| Camera | GUI | Image Hz sim (camera only / + detector) | Image age p50 (sim) | RTF |
|---|---|---|---|---|
| Installed: `wideanglecamera`, 182°, 320×240, 30 Hz configured | yes | **1.14** / 0.93 | 831 ms | 0.80 |
| Same, `visualize` false | yes | 1.11 / 0.92 | 829 ms | 0.81 |
| Pinhole `camera`, 100°, 320×240, 30 Hz | yes | **6.90** / 5.73 | 96 ms | 0.82 |
| Pinhole, 100°, 320×240, **15 Hz** | yes | 6.61 / 5.45 | 102 ms | 0.83 |
| Pinhole, 100°, 30 Hz, GPU path attempted | yes | 6.43 / 5.42 | 110 ms | 0.78 |
| Wide-angle 182°, headless (gzserver only) | no | 1.47 / 1.31 | 635 ms | 0.93 |
| Pinhole, 100°, headless | no | **9.72** / 8.30 | 47 ms | 0.95 |

Findings:

- **Camera type.** The installed burger_cam uses Gazebo's `wideanglecamera` with a custom lens. It renders a
  cube map (six views, `env_texture_size` 512) every frame and remaps it to the 182° image. A standard pinhole
  camera of the same resolution delivered **6.1× more frames** under identical conditions. Wide-angle rendering
  is the dominant cause.
- **The FOV is not the cost by itself.** Wide-angle rendering is what costs, and it is only needed for FOVs
  near or above 180°. 100° is the largest FOV a single pinhole projection renders without severe edge
  stretching.
- **Configured rate is not delivered rate.** 30 Hz and 15 Hz configured gave the same delivered rate: rendering
  throughput caps it. `camera_info` is published once per rendered frame and tracked the image rate in every
  session. Marker messages tracked frames whenever the marker was in view.
- **Rendering is software rasterization.** `~/.bashrc` sets `LIBGL_ALWAYS_SOFTWARE=1`, so Mesa's llvmpipe
  rasterizes every camera frame, the GUI and RViz on the CPU. Without that variable Mesa 23.2.1 still selects
  llvmpipe. Forcing `GALLIUM_DRIVER=d3d12` fails to create a screen, although `/dev/dxg` is present
  (`data/bench/gpu_check.txt`). **Hardware rendering is not available on this host as configured.** That is a
  host limit; I did not change system packages or your shell configuration.
- **The GUI costs about 1.4×.** gzclient renders the scene a second time in software (about 280% of one core).
  Headless, the pinhole camera reached 9.7 Hz and RTF 0.95.
- **The detector is not the bottleneck.** Offline on recorded frames (`data/bench/detector_cost.txt`):
  `imgmsg_to_cv2` 0.26 ms and `detectMarkers` 3.7 ms (p50) per pinhole frame, against 9.2 ms on wide-angle
  frames, where the marker is smaller and distorted. The p99 values were inflated by a concurrent simulation.
  Adding the detector lowered the delivered rate by about 17% (CPU contention, not queueing: frame age stayed
  near one frame interval). The debug window had no measurable cost. Reliable QoS with depth 10 never queued
  at these rates.
- **Instrumentation overhead.** The probe and `top` sampling cost about 5% and 1% of one core. The mission bag
  recorder cost 7.5% in both missions.

Clock note: rates and ages are in **sim** time unless marked wall. Sim time advances at the RTF. Wall-clock
rates are given where they matter to a human observer.

Host-hygiene note: the first baseline session ran with leftovers from earlier offline experiments on
isolated DDS domains 174 and 177 (32 `static_transform_publisher` processes, a frozen-state Cartographer
node) plus a `gz stats` process. Together they cost about 0.15 core. They could not exchange DDS messages
with the default-domain missions, but they were present during the Phase 3 mission and were removed before
Phase 4. Their effect on the baseline was within noise (`data/bench/rate_session_with_leftover_processes.txt`:
1.10 vs 1.14 Hz). The Phase 3 vs Phase 4 comparison is therefore slightly favourable to Phase 4 by that
margin. Earlier leftover checks had used `pgrep -x` with names longer than the kernel's 15-character process
names, so they could never match. The checks now match full command lines.

## 3. Change

- `src/warehouse_gazebo/models/turtlebot3_burger_cam_pinhole/`: a project copy of the installed
  `turtlebot3_burger_cam` model (turtlebot3_gazebo 2.3.8, Apache-2.0). **Only the camera `<sensor>` block
  differs:** type `camera` instead of `wideanglecamera`, `horizontal_fov` 1.745 rad (100°) instead of
  3.183 rad, no `<lens>` block, and `visualize` false. Resolution 320×240, the 30 Hz update rate, pose, topics
  and frame are unchanged. A test asserts that the rest of the model is identical to the installed one.
- `warehouse_simulation_camera.launch.py` spawns that model. A new `gui` argument (default `true`, so default
  behaviour is unchanged) runs `gzserver` alone when `false`. `warehouse_robot_system_camera.launch.py` passes
  it through.
- `test/test_camera_model.py`: 3 tests covering the camera parameters, the difference from the installed model
  (only the sensor), and the launch wiring.

Why this is the smallest effective configuration:

- The sensor type was the one change that removed the bottleneck.
- Resolution was not reduced. At 320×240 and 100° the marker spans 47×47 px at the station (32×27 px with
  the wide-angle camera), and the detector is not the limit.
- Lowering the configured rate had no effect.
- Detector, transport and QoS changes would address costs that measurement showed to be small.
- Headless operation is provided only as an option. The README records that the earlier GUI-free trials were
  less reliable, for reasons that are still unexplained, and this phase validated only the default GUI
  configuration.

## 4. Why the mission reached 4.2 Hz, not 10 Hz

The camera shares a saturated CPU (93% busy, load average about 27 on 8 logical CPUs) with:

- physics;
- gzclient's second software render;
- RViz, which also renders in software;
- Nav2;
- the mission node.

The bench rate without Nav2 and RViz was 5.7 Hz with the detector. The mission adds about 2 cores of load (RViz
101%, Nav2 container 68%, mission node 32%, bag recorder 7.5%), and the rate falls to 4.2 Hz. Two measured routes would go further, both outside this phase's validated
default:

- **Run headless** (`gui:=false`): 8.3 Hz with the detector on the bench, RTF 0.92–0.95. This needs a
  validation of its own because of the earlier GUI-free reliability record.
- **Enable hardware rendering:** a Mesa build with the d3d12 Gallium driver, plus removing
  `LIBGL_ALWAYS_SOFTWARE` (a host decision).

I stopped iterating here, as the brief asks, rather than chase 10 Hz on this host.

## 5. Detection reliability under arrival uncertainty

`scripts/perturb.py` ran on the bench world plus `libgazebo_ros_state`, with the detector at its defaults.
It teleported the pinhole robot to **44 poses**: 4 stations × 11 perturbations of the surveyed approach pose
(ground truth, world frame). The perturbations were:

- nominal;
- ±0.12 m along the heading;
- ±0.12 m across it;
- ±11.5° of heading;
- four corners where the lateral offset and heading error together push the marker furthest off-axis, near and
  far.

At each pose it read back the true pose and recorded 5 s of detector output.

**Result: 1,703 of 1,703 processed frames detected a marker; 0 wrong-ID messages; every pose had 100%
detection** (`data/bench/perturbation_pinhole100.json`). The commanded and read-back poses agreed exactly,
and each station decoded only its own ID (0, 1, 2, 3), confirming each image came from the teleported robot.
The worst case in these bounds puts the marker about 21° off the optical axis, inside the ±50° half-FOV.

A first version of the test deleted and respawned the robot instead of teleporting it. After a few deletions
the image stream stopped following the respawned robot: wrong-station IDs appeared at the home dock. Those
runs are discarded as a harness defect, kept locally only.

### 5.1 Extended envelope: ±0.25 m (`data/bench/envelope_grid25*.{json,txt}`)

One further campaign used the same teleport harness: **300 poses**, all four stations. At each station, a 5×5
position grid covered ±0.25 m in 0.125 m steps along and across the approach heading, each at heading 0 and
±11.5°, with 3 s of detector output per pose. Every read-back pose matched its command (no contact with
geometry).

| | Result |
|---|---|
| Frames processed | 5,811 |
| Correct detections | 4,695 |
| Missed (frames with no marker) | 1,116, all at 57 poses that detected **nothing** |
| Incorrect marker IDs | **0** |
| Poses with partial detection | 0 (every pose was 100% or 0%) |

| Along-heading offset (dx; + = closer to the marker) | Poses failing / tested | Marker range |
|---|---|---|
| −0.25 m | 0 / 60 | 0.84–1.18 m |
| −0.125 m | 0 / 60 | 0.72–1.05 m |
| 0 | 0 / 60 | 0.59–0.93 m |
| +0.125 m | 12 / 60 | 0.47–0.81 m |
| +0.25 m | 45 / 60 | 0.34–0.70 m |

- **Largest tested region with reliable detection:** every pose from 0.25 m short of the goal up to the goal
  distance, across the full ±0.25 m lateral range and ±11.5° of heading: 180 poses, 3,395 frames, all
  detected.
- **Closer than the goal:** detection held at +0.125 m except at the four corners where the lateral offset and
  heading error combine to put the marker ≥ 28° off-axis.
- **Inventory scan:** detected at all 75 poses. Its marker is 0.97 m from the goal, not 0.67 m.
- **Observed failure boundary:** the marker closer than about **0.44 m**, or closer than about 0.54 m when it
  is also ≥ 28° off-axis. That fits the geometry: at close range the marker's top edge leaves the 84°
  vertical field of view, and an off-axis marker is also cropped horizontally.
- This boundary is **directional**: it is a grid in the station approach frame, not a detection radius, and
  says nothing about untested directions or headings beyond ±11.5°.

**Map-frame tolerance vs physical error.** The goal checker accepts when the **AMCL estimate** is within
0.12 m and 0.15 rad of the map-frame goal. The physical error also includes localization error (AMCL residual
p95 16.6 cm in the Phase 4 mission) and survey error. A physical stop beyond 0.12 m is therefore possible.

In practice, all **25 true stops across Phases 1–4** (`data/bench/historical_true_stops.txt`) were **short**
of the goal: dx from −0.119 to 0.000 m, |dy| ≤ 0.091 m, |heading| ≤ 11.5°. The robot approaches facing the
marker and is accepted on entering the tolerance circle. No mission stop has come near the near-side failure
region, which needs an overshoot of at least 0.125 m toward the wall plus offset and heading error. **No
significant detection weakness is established, so the camera configuration is unchanged.

*Note added in Phase 4b:* this conclusion covers the fixed routes measured, all of which approached from the
corridor side and stopped short of the goal. It does not establish detection reliability for other approach
directions, larger localization errors or overshoot; see the Phase 4b report, §6.** If a later phase
changes the approach direction or the goal checker, this boundary should be rechecked.

## 6. Mission validation (`data/mission/`)

The mission used the same launch command, mission command, recorded topics and runner as Phase 3
(`scripts/run_phase4_validation.sh`), with the default GUI and a quiet host. Raw images were not recorded in
either run. The delivered image rate and age are therefore measured on `camera_info`, which the bench showed
is published once per rendered frame. The method is identical in both runs (`scripts/perception_metrics.py`).

**Station positioning** (truth at the ARRIVED line; `phase1_results.txt`):

| Station | Phase 3 | Phase 4 |
|---|---|---|
| Home dock | 0.083 m / +7.3° | 0.051 m / +8.3° |
| Loading zone | 0.088 m / −10.2° | 0.111 m / −8.9° |
| Inventory scan | 0.119 m / −8.5° | 0.084 m / −6.2° |
| Dispatch zone | 0.049 m / −8.3° | 0.008 m / −8.5° |
| Return home | 0.094 m / +5.7° | 0.096 m / −9.3° |
| Max, mean | 0.119 m, 0.087 m | **0.111 m, 0.070 m** |

All errors are within the 0.12 m / 0.15 rad goal tolerance as seen by the estimate (`phase_checks.txt`).

- **Acceptance vs settled** (same definitions as Phase 3, `arrival_accuracy.txt`): stations 1–4 mean
  0.064 m at acceptance and 0.063 m settled. The largest motion after acceptance was 1.3 mm (inventory scan,
  accepted at 72 mm/s).
- **AMCL residual** (median, fixed calibration): 7.6 cm vs 6.3 cm. One run each, so within run-to-run
  variation.
- The station-to-station differences are of the size seen between runs in earlier phases. The camera does not
  feed localization or control.

**Regressions checked, none found:**

- 0 timeouts, halt errors, orphaned goals, overlaps, recoveries, planning failures and BT navigation failures;
- clearance 0.32 m;
- the same routes and unknown-space share (`unknown_traversal.txt`);
- the controller, smoother and planner output rates are unchanged: 9.34 / 18.91 / 0.91 Hz wall vs
  9.04 / 18.85 / 0.89 (the smoother loop is wall-paced, so its sim-time rate falls slightly as RTF rises).

The detector's CPU rose from 3.5% to 15.5% of one core, because it processes 6× the frames. gzserver fell
from 324% to 292%.

The three `process has died` lines at +65–67 s after mission exit are the runner's teardown (SIGINT after the
60 s observation window), as in Phase 3.

## 7. Tests

`colcon test` for the warehouse packages: **81 tests: 78 passed, 0 failed, 3 skipped**. The three skips are
the `test_copyright` tests, one per package, skipped by design. Phase 3 had 78 tests; the 3 new ones are in
`test_camera_model.py`.

## 8. Unknown-space policy: planned vs executed traversal

The configuration is unchanged. This section establishes what `allow_unknown: false` does and does not
guarantee. It is based on the installed Nav2 1.1.18 sources and `data/mission/live_unknown_timing.txt`.

| Component | Setting | Effect |
|---|---|---|
| Global costmap | `track_unknown_space: true`; static + obstacle + inflation; update 1 Hz | Unknown cells (255) exist. The obstacle layer clears them to free along LiDAR rays and under the robot footprint (`footprint_clearing_enabled` defaults to true). No-return beams are dropped (`inf_is_valid` defaults to false), so they clear nothing. |
| NavFn | `allow_unknown: false`, tolerance 0.5 m | `navfn.cpp` `setCostmap` maps 255 to `COST_OBS` (impassable) **when a plan is computed**, against the global costmap at that moment. The BT replans at 1 Hz. |
| Local costmap | `track_unknown_space` not set (default **false**); obstacle + inflation only; rolling 3×3 m | Cells default to FREE (`layered_costmap.cpp`, `obstacle_layer.cpp`). **The local costmap never contains unknown cells.** |
| DWB `BaseObstacle` | — | Rejects trajectory poses on LETHAL, INSCRIBED or NO_INFORMATION cells (`base_obstacle.cpp`), checking each pose's centre cell in the **local** costmap. Because that costmap holds no unknown cells, **the controller cannot see or avoid unknown space.** |

So `allow_unknown: false` is a **planning** rule. It does not guarantee zero executed travel through cells that
are unknown at that instant. The controller follows the plan with normal tracking deviation, and the global
costmap behind both checks lags the robot.

**Planned vs executed** (truth robot centre, evaluation fit to the map frame):

- **Planned:** 0% of plan poses lay on cells unknown in the preceding global-costmap snapshot, in Phase 3 and
  in Phase 4.
- **Executed:** **3.46 m** (Phase 4; 3.48 m in Phase 3) of the robot centre's path lay on cells unknown in the
  latest snapshot. That figure, reported earlier, was measured against the **global** costmap (the local
  costmap was not recorded), pairing each truth sample with the last snapshot received before it (bag wall
  time).
- **Recording frequency:** the global costmap was published every **1.5 s sim** (median; max 1.7 s).
  Re-ordered by sim header stamps, the total is 3.29 m (Phase 3: 3.34 m). The snapshot age at those samples
  was median 1.00 s, max 1.60 s.
- **Temporal uncertainty:** **3.00 m (91%)** of it was already free in the *next* snapshot. 0.29 m stayed
  unknown.
- **Spatial uncertainty:** shifting the robot by one 5 cm cell, the alignment uncertainty of the evaluation
  fit, changes the total to 2.68–4.19 m.

**Interpretation:**

- The pattern is consistent with cells along corridor edges that face the unmapped open centre. No LiDAR
  return clears those cells; footprint clearing frees them only once the robot reaches them.
- It is **not** consistent with plans routed through unknown space.
- The clearance to true geometry stayed ≥ 0.32 m.

**Preserved for Phase 5 review (not changed here):** at the controller level, unknown space has no
representation at all. If execution-time avoidance of unknown space matters, the options are:

- give the local costmap `track_unknown_space` plus a static layer;
- set `inf_is_valid: true` so that no-return beams clear;
- monitor exposure at runtime.

Each would need its own validation.

## Limitations

- **One mission.** The comparison with Phase 3 is one run against one run. Rate and age differences of 6×
  are far outside run-to-run variation; the small differences in positioning and AMCL residual are not.
- **Bench poses are teleported.** The perturbation test checks visibility and decoding at static poses. It
  does not model motion blur, which Gazebo does not render.
- **Simulation detection margin is not real-world evidence.** The markers are rendered without noise, glare
  or defocus.
- **Field of view.** 100° is narrower than 182°. Verification still requires the marker in view after arrival.
  §5 and §5.1 show it is at every pose up to the goal distance within ±0.25 m and ±11.5°. Detection fails
  only when the robot is at least 0.125 m *closer* to the marker than intended, combined with large lateral
  and heading errors. No mission has stopped there. A stop far outside the goal tolerance could now fail
  verification where the 182° camera would have passed. That arguably makes verification more meaningful
  (see README limitation 7), but it is a behaviour change.
- **Headless mode** was not mission-validated in Phase 4 itself; Phase 4b covers it.
- **RTF attribution.** RTF rose from 0.711 to 0.759, but not all of that can be credited to the camera
  architecture. During the Phase 3 mission, leftover processes from earlier experiments (32 static TF
  publishers, a Cartographer node and `gz stats`) consumed about 0.15 CPU core, about 2% of the 8-thread
  host's capacity. They were removed before Phase 4. Run-to-run host variation adds to that. The bench
  comparisons in §2 used a clean host and the same session structure, and they support the camera
  attribution for the **image-rate** gain (6×). The size of the RTF gain is less certain.
- **Simulation shutdown errors.** Each mission ends with three `process has died` lines when the runner sends
  SIGINT after the 60 s observation window:
  - `aruco_station_detector`: exit 1;
  - `gazebo`: exit 255;
  - `rviz2`: −6 here, −11 in Phase 3 (an abort or segfault during shutdown).

  They occur in Phase 3 and Phase 4 alike, after the mission has exited. They don't affect results, but the
  shutdown is not clean.
- **No hardware rendering on this host** (§2).

## Reproduction

The scripts expect the `diagnostics_phase4/` layout: `bench/` for the bench scripts, `models/` generated by
`make_models.py`, and `bench/small_warehouse_state.world` generated by `make_state_world.py`.

```bash
python3 make_models.py ../models                              # camera variants from the installed SDF
python3 make_state_world.py ~/projects/warehouse_ws small_warehouse_state.world
./run_session.sh cam_pin100_320_30hz 1.803 9.3 1.5708 s_out 40   # GUI=false for headless
python3 analyze.py s_out
./run_perturb.sh cam_pin100_320_30hz p_out 5
python3 detector_cost.py s_out/images
run_phase4_validation.sh                                      # mission, then perception_metrics.py
```
