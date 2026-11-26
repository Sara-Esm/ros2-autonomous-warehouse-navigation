# Phase 1 validation: station coordinate-frame correction

*Branch `feat/station-frame-correction`, 2026-11-25. Baseline: v1.0.1 and its diagnostic experiment
([`docs/diagnostics/REPORT.md`](../../diagnostics/REPORT.md)). One validation mission. Only the station
goals changed: Nav2 parameter sha256 `869d998…` and the map are unchanged.*

## The defect and the correction

The SLAM map's orientation is arbitrary; that is not a defect. The defect was that the station approach poses
were measured in Gazebo's **world** frame but supplied to Nav2 as **map**-frame coordinates.

The calibration comes from the diagnostics (fit A: map obstacles against the true world geometry):

```
world = R(θ)·map + t        θ = +2.05°,  t = (−0.142, −0.049) m
map   = R(−θ)·(world − t)
yaw_map = normalize(yaw_world − θ)
```

**Direction check (before implementation).** Applying the forward transform to the v1.0.1 goals reproduces
the measured v1.0.1 stopping positions within 0.03–0.11 m. Applying the inverse instead would miss them by
0.35–1.01 m. This is encoded as a unit test.

**Implementation.**
- `warehouse_bringup/stations.py` holds the world-frame approach poses as unchanged reference data, the
  calibration constants, `world_to_map` / `map_to_world`, and the derived map-frame `WAYPOINTS`.
- The mission node imports `WAYPOINTS` (−29/+2 lines). Its goal interface is unchanged.
- Step 5 reuses the `home_dock` station entry, so it has exactly the same pose as step 1.
- Markers, the map origin, AMCL initialization and all Nav2 parameters are unchanged.

## Goals (pre-launch sanity table)

| Station | Intended world pose | v1.0.1 map goal | Corrected map goal | Corrected goal → world | v1.0.1 goal → world (predicted old stop) |
|---|---|---|---|---|---|
| Home Dock (steps 1, 5) | (−3.678, 9.300, 90.00°) | (−3.678, 9.300, 90.00°) | **(−3.199, 9.470, 87.95°)** | (−3.678, 9.301, 90.00°) | (−4.150, 9.113, 92.05°), 0.508 m off |
| Loading Zone | (1.803, 9.300, 90.00°) | (1.803, 9.300, 90.00°) | **(2.278, 9.273, 87.95°)** | (1.803, 9.300, 90.00°) | (1.327, 9.310, 92.05°), 0.476 m off |
| Inventory Scan | (1.801, −9.000, −90.00°) | (1.801, −9.000, −90.00°) | **(1.622, −9.015, −92.05°)** | (1.801, −9.000, −90.00°) | (1.980, −8.979, −87.95°), 0.180 m off |
| Dispatch Zone | (−3.630, −9.300, −90.00°) | (−3.630, −9.300, −90.00°) | **(−3.817, −9.120, −92.05°)** | (−3.630, −9.300, −90.00°) | (−3.437, −9.473, −87.95°), 0.259 m off |

The corrected goals round-trip to the intended poses within 0.6 mm, which is the rounding to 1 mm.

The corrected goals' clearance to map obstacles is 1.00 / 0.60 / 0.90 / 1.05 m. That matches the
*true-geometry* clearance at the intended world poses (1.00 / 0.65 / 1.00 / 1.00 m) from the static diagnostics.

**AMCL initialization.** The robot spawns at world (0, 0, 0), which is map (0.144, 0.044, −2.05°). AMCL is
seeded at map (0, 0, 0), so the seed is off by 0.15 m and 2.05°. That is within AMCL's initial uncertainty, and
AMCL converged in both runs. The seed was left unchanged in Phase 1 to keep Nav2 parameters fixed.

## Build and tests

- `colcon build`: OK.
- `colcon test` (bringup, perception, gazebo): **70 tests, 0 failures, 3 skipped**. That is the previous 54
  plus 16 new tests in `test/test_stations.py`: direction (forward reproduces measured stops, inverse does
  not), position, orientation and wrap-around, round trip, distance preservation, waypoint/world consistency,
  return-home equality, mission order and marker IDs, and markers in the world file 0.6–1.0 m ahead of and
  facing each approach pose. flake8 and pep257 are clean.
- One earlier full-suite run had 1 failure in an existing wall-clock tolerance test
  (`test_verification_window_uses_simulation_time[2.0]`: 0.95 s measured against a 0.90 s bound). It passed 3/3 in
  isolation and in the next full run. This is a timing flake under load, unrelated to station coordinates.

## Validation mission

Same launch file and mission command as `scripts/run_mission_trial.sh`. Recording was lightweight: rosbag
with hidden action-status topics, no images, `top` every 2 s. There were **no `gz` transport probes** and no
parameter/QoS CLI probes. Ground truth is `/odom`, which equals the Gazebo world pose (verified in the diagnostics).

### Mission outcome

**5/5 tasks succeeded, exit code 0, all five markers verified.** No simulator crash.

### Station positioning (true world pose at the ARRIVED log line)

| Station | v1.0.1 position error | **Phase 1 position error** | v1.0.1 heading error | **Phase 1 heading error** | Arrival → marker confirm |
|---|---|---|---|---|---|
| Home Dock | 0.504 m | **0.042 m** | +9.6° | +6.9° | 2.08 s |
| Loading Zone | 0.404 m | **0.084 m** | +9.5° | −10.4° | 0.51 s |
| Inventory Scan | 0.176 m | **0.051 m** | +8.9° | +8.0° | 2.26 s |
| Dispatch Zone | 0.316 m | **0.098 m** | −2.4° | −7.0° | 0.09 s |
| Return Home | not reached (crash) | **0.102 m** | – | −7.7° | 0.30 s |

- Position error on the four comparable stations: mean **0.350 → 0.069 m**, max **0.504 → 0.098 m**.
  All five stations are ≤ 0.15 m (max 0.102 m).
- The observed residuals (0.04–0.10 m) are of the size the calibration fit predicted (0.03–0.11 m). Those
  were predictions, not guarantees.
- Heading: the mean signed error on the four comparable stations went from **+6.4° to −0.6°**. The systematic
  bias from the 2.05° frame rotation is gone. The magnitude (|7–10°|) did **not** shrink, for the reason below.

**Why heading scatter remains (MEAS, `data/checks.txt`).** At every station the goal checker's estimate (TF
map→odom∘odom→base) was accepted at the edge of the 0.15 rad = 8.59° yaw tolerance: +8.2°, −8.2°, +8.0°,
−7.3°, −6.9°. The robot rotates toward the goal heading and stops as soon as it is inside the tolerance,
from whichever side it approached. Localization contributes only −2.3° to +0.1°. The scatter therefore
comes from the unchanged goal tolerance, not from the coordinate transform. Tolerances were not tightened in
this phase, by design.

### Other measurements, against the v1.0.1 diagnostic run

| Quantity | v1.0.1 diagnostic | Phase 1 | Note |
|---|---|---|---|
| AMCL residual vs truth, fixed calibration (median / p95 / max) | 8.3 / 21.3 / 24.7 cm | 9.0 / 18.0 / 22.2 cm | unchanged, as expected; Phase 1 does not change localization |
| BT `ComputePathToPose`/`FollowPath` FAILURE records (see reconciliation below) | 19 | 17 | Phase 3 topic |
| Ack timeouts during the mission | 19 | 16 (9 compute_path, 7 follow_path) | |
| Recoveries | Spin 1, Wait 1, BackUp 1 | Spin 1, Wait 1, BackUp 1 | each coincided with an ack timeout |
| No valid trajectories / failed to make progress / planner "failed to create plan" | 0 / 0 / 0 | 0 / 0 / 0 | |
| Outer NavigateToPose results | 4 succeeded + 1 halted (crash) | 5 succeeded | |
| RTF (mission, 10 s windows) | 0.66 (0.57–0.74) | 0.66 (0.54–0.72) | |
| CPU busy / load average | 95% / 30 | 94% / 29 | removing the `gz` probes did not reduce load measurably |
| Camera frame rate | 0.60 Hz sim | 0.61 Hz sim | Phase 4 topic |
| Executed path per leg (m) | 11.80 / 9.05 / 18.58 / 5.52 / – | 11.88 / 8.65 / 18.35 / 5.74 / 18.60 | within 1–2% of each leg's first plan |

### Reconciliation: 16 acknowledgement timeouts vs 17 BT failure records (`data/reconciliation.txt`)

The stock behaviour tree contains **two nodes named `ComputePathToPose`**: the action node itself, and the
`RecoveryNode` wrapper around it (`<RecoveryNode … name="ComputePathToPose">`). The same holds for
`FollowPath`. The BT log identifies nodes only by name.

- 15 timeouts each produced exactly one action-node FAILURE record.
- At 146.20 s, one `compute_path_to_pose` timeout produced two records:
  - 146.228 s: the action node failed on the acknowledgement timeout.
  - The wrapper then ran `ClearGlobalCostmap-Context`. That service call also timed out ("Node timed out while
    executing service call to global_costmap/clear_entirely_global_costmap", same 20 ms `server_timeout`).
  - 146.251 s: the wrapper reported FAILURE under the same name. `NavigateWithReplanning` failed and the
    Wait recovery started.
- Result: 17 records = 16 action-node failures (one per timeout) + 1 wrapper failure.
- The v1.0.1 figure of 19 is likewise a count of failure *records*, so the two counts compare records, not
  distinct incidents.
- Outer-pipeline failures (`NavigateWithReplanning`, each followed by a recovery round) occurred 5 times:
  24.8, 106.4, 146.3, 238.6 and 534.3 s.

### Action-status abort counts

`follow_path` had 469 goals: 460 ended ABORTED, 7 SUCCEEDED, 2 CANCELED. These are not 460 failures.
In the installed `nav2_util/simple_action_server.hpp` (lines 345–347), a new goal arriving while one is
active is accepted as a preemption. `accept_pending_goal()` then calls `current_handle_->abort()` on the
previous goal and logs it only at debug level. The BT sends a new path to the controller at every 1 Hz
replan, so almost every superseded path goal ends ABORTED.

Genuine server-side terminations show up as warnings instead: `[follow_path] [ActionServer] Aborting handle`
appeared 4 times (172.8, 308.4, 426.8, 542.8 s). Each one coincides with a `follow_path` acknowledgement
timeout. `compute_path_to_pose` goals complete within one request, so all 475 SUCCEEDED.

### Errors that remain

The system is not error-free. During the mission:

- 16 acknowledgement timeouts and 1 service-call timeout.
- 167 controller missed-rate warnings and 4,157 behaviour-tree tick-rate warnings.
- 3 recovery behaviours.

At teardown, after the observation window, the ArUco detector exited with code 1 and RViz with −11
(segmentation fault), ≈66–68 s after mission exit. The same teardown pattern appears in the historical logs.
The one-off flaky timing test is recorded under *Build and tests*.

### Motion after the final mission report (previously unresolved)

| Window | Duration | True displacement | Max true speed | Nonzero velocity commands | Active Nav2 goals |
|---|---|---|---|---|---|
| Final NavigateToPose success → MISSION COMPLETE | 8.3 s | 0.0 mm | 0.75 mm/s, 0.13 rad/s (settling) | 0 | – |
| MISSION COMPLETE → process exit | 0.6 s | 0.0 mm | 0.01 mm/s | 0 | 0 |
| Process exit → +60 s | 59.9 s | 0.0 mm | 0.01 mm/s | 0 | 0 |

A residual turning motion (peak 0.13 rad/s, translation < 1 mm/s) settled **1.26 s after the final
NavigateToPose success report**. The last sample above 2 mm/s or 0.005 rad/s came at +1.26 s. The robot was then
stationary for the remainder of the observation window: ≈7 s before MISSION COMPLETE, and 60.5 s after it. The success path still does not
*verify* a stop in code; that belongs to Phase 6.

## Real-world applicability

The calibration uses **simulator ground truth** (Gazebo's world geometry and world pose). That suits a
controlled simulation experiment. A real deployment has no world-frame ground truth, and would normally
establish station goals **directly in the robot's map frame**. For example, an operator drives the robot to
each intended approach pose and records its validated localization estimate. The map-frame `WAYPOINTS`
produced by `stations.py` play that role here.

## Limitations

- One validation run, compared with one baseline run; no repeat statistics.
- Instrumentation differed. The baseline had `gz` probes and CLI probes; this run had neither, but the same
  bag recorder plus hidden action topics. CPU load and RTF were nevertheless the same.
- The baseline return-home leg was incomplete, so step 5 has no before value.
- Ground truth is sampled at the ARRIVED log line (bag receipt time). Wall-time alignment is within ≈50 ms,
  and the robot was settling at < 1 mm/s.
- The calibration's own uncertainty is about 1–2 cm in translation and 0.01° in rotation (fit A vs fit B).
  The AMCL residual (median 9 cm) is unchanged and bounds achievable positioning together with the 0.12 m goal tolerance.
- The AMCL seed remains 0.15 m / 2.05° from the true spawn pose (unchanged by design).

## Conclusion

Phase 1 met its targets:

- Every station is within 0.15 m (max 0.102 m, against 0.504 m before).
- Navigation and marker identity verification were preserved (5/5, exit 0).
- The systematic heading bias from the frame mismatch was removed (mean +6.4° → −0.6°).

Heading magnitude is limited by the unchanged yaw tolerance. Recoveries, acknowledgement timeouts, camera
rate and CPU load are unchanged and belong to later phases. The system is ready for Phase 2, starting from
this branch.

## Reproduction

Raw data (54 MB bag) is kept locally in the git-ignored `diagnostics_phase1/`.

```bash
source /opt/ros/humble/setup.bash
V=docs/validation/phase1_station_frames/scripts
python3 $V/phase1_evaluate.py diagnostics_phase1/run diagnostics_20261125/run .
python3 $V/phase1_checks.py diagnostics_phase1/run .
```

`run_phase1_validation.sh` records a new run. Do not reuse its output directory.
