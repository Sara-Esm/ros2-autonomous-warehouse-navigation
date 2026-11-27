# Phase 5: controlled motion optimization

Branch `feat/motion-control`, from Phase 4b (`6730dc7`). Two DWB/goal-checker parameter changes, each selected by
measurement. Every run was headless (Phase 4b mode):

- three baseline missions;
- six short controlled sessions on the north route;
- three matched sessions on the south-pinch route;
- one replay study of local unknown-space tracking;
- one full validation mission.

The map, surveyed goals, camera architecture, global planning policy, Phase 3 timing parameters, mission supervision,
controller (DWB), velocity and acceleration limits, `BaseObstacle.scale` (0.02) and xy tolerance were not changed.

**Status: approved.** The obstacle-clearance trade-off (§5) was explicitly accepted for the current simulated
operating environment. The original clearance criterion is **not fully met**, and nothing here certifies the
controller as safe. Checkpoint of this configuration: `diagnostics_phase5/checkpoint_fpd0325_*`, also copied to
the Windows backup folder.

## 1. Result

**Selected configuration:**

- `yaw_goal_tolerance` **0.07 rad** (was 0.15).
- `PathAlign`/`GoalAlign` `forward_point_distance` **0.325 m** (was 0.1; the Nav2 default).

What it changed, against the three-mission baseline range:

- **Less turning overall:** total absolute rotation per mission fell **14%** (2,254–2,366° → 1,948°) for the same
  route and distance.
- **Smoother commands:** the per-command change in angular velocity (|Δω| p95) halved, 0.103 → **0.051 rad/s**.
  The high-pass RMS of ω while cruising fell 0.046–0.057 → **0.033 rad/s**. The p95 of physical angular
  acceleration fell 2.39–2.65 → **1.81 rad/s²**.
- **Better final heading:** settled heading error mean 5.9–8.8° → **4.2°**, max 9.5–10.8° → **5.8°**. The robot
  is nearly stopped when accepted, and settling takes 0.40 s instead of 0.76–1.14 s.
- **Fewer reversals, not smaller ones:**
  - Corrective angular reversals while cruising fell from 7.15–8.28 to 1.34 per navigation minute. This is a
    full-mission comparison in which **both** parameters changed. The frequency depends on route and
    configuration. The single-parameter, matched comparison is §3.2: 12.15 → 1.71 per minute on the south
    route at a fixed yaw tolerance.
  - The remaining reversals are about as large as before (median peak-to-peak 0.31 against 0.36–0.41 rad/s).
  - The physical straight-line heading weave fell from **0.38–0.43° RMS** to **0.25° RMS**. It was small to
    begin with.
- **Not improved:** peak angular acceleration (11.95 against 11.25–11.71 rad/s²; spikes near the actuator's
  limit remain).

| Measure | Baseline: 3 missions (fpd 0.1, yaw 0.15) | Phase 5 validation (fpd 0.325, yaw 0.07) |
|---|---|---|
| Mission / exit / markers verified | 5/5 / 0 / 5 in all three | 5/5 / 0 / 5 |
| Total absolute rotation | 2,254 – 2,366° | **1,948°** |
| Per-command \|Δω\| p95 / cruise ω high-pass RMS | 0.103 / 0.046 – 0.057 rad/s | **0.051 / 0.033** |
| Truth angular acceleration p95 / max | 2.39 – 2.65 / 11.25 – 11.71 rad/s² | **1.81** / 11.95 |
| Cruise reversals per nav-minute / median peak-to-peak | 7.15 – 8.28 / 0.36 – 0.41 rad/s | 1.34 / 0.31 |
| Straight-line heading weave (truth, RMS) | 0.38 – 0.43° | 0.25° |
| Settled \|heading\| mean / max | 5.9 – 8.8° / 9.5 – 10.8° | **4.2° / 5.8°** |
| Settling time after acceptance | 0.76 – 1.14 s sim | 0.40 s |
| Station position error, max / mean | 0.091 – 0.107 / 0.055 – 0.085 m | 0.114 / 0.078 m (**max slightly above the baseline range**, within 0.15 m) |
| **Collision-footprint clearance, south pinch (window minimum)** | **0.25 – 0.27 m** | **0.21 m** (−4 to −6 cm, 16–22% of the baseline margin; §5) |
| Robot-centre clearance, south pinch | 0.35 – 0.36 m | 0.30 m |
| Executed on static-unknown / live-unknown cells | 7.69 – 9.28 / 3.38 – 3.61 m | 6.19 / 3.26 m |
| Distance / navigation time | 62.99 – 63.25 m / 319 – 320 s sim | 62.72 m / 314 s sim |
| Mission duration / RTF | 412 – 418 s / 0.863 – 0.875 | 408 s / 0.873 |
| Controller / smoother rate | 10.71 – 10.85 / 21.64 – 21.93 Hz sim | 10.73 / 21.70 Hz sim (same RTF, so same effective control interval) |
| Camera / verification after arrival | 6.78 – 6.84 Hz sim / mean 0.09 – 0.17 s | 6.89 Hz / 0.11 s |
| Timeouts, halt errors, orphans, overlap, recoveries, planning failures | 0 in all | 0 |
| Stop offset toward the marker (dx; envelope fails at ≥ +0.125 m) | ≤ −0.005 m | −0.114 … −0.005 m |

These are ranges from three missions and one validation observation, not reliability rates. No energy or wear
quantities were measured, and none are claimed.

## 2. Baseline and control architecture

**Configuration (unchanged except §4):**

- **DWB:** 20 × 40 velocity samples, a 1.5 s horizon, `max_vel_x` 0.22 m/s, `max_vel_theta` 1.0 rad/s,
  `acc_lim_x` 2.5 m/s², `acc_lim_theta` 3.2 rad/s².
- **Critics:** RotateToGoal 32, PathAlign 32, PathDist 32, GoalAlign 24, GoalDist 24, BaseObstacle 0.02,
  Oscillation.
- **Goal and progress checkers:** `SimpleGoalChecker` at 0.12 m, stateful; `SimpleProgressChecker` requiring
  0.5 m within 10 s.
- **Velocity smoother:** Nav2 defaults, open loop.
- **Loop timing:** wall-paced; about 10.7 Hz sim at RTF 0.87.

**Measured behaviour (`data/baseline/`):**

- **Command-to-motion lag:** 60–80 ms sim.
- **Cruise reversals** are unwanted oscillation, not turning:
  - they occur at full speed on straights (mean |ω| about 0.03 rad/s over ±2 s);
  - they originate in DWB's output;
  - they are **not** caused by replanning (onsets within 0.3 s of a new plan at chance level, 30–35% against
    32–34%).
  - Sustained turns of one sign lasting ≥ 1 s are counted separately as legitimate turning.
- **Heading at arrival:** 12 of 15 settled errors fell between 6° and 11°, bounded by the 8.6° yaw tolerance.
- **Localization:** AMCL heading error median about 1°, p95 about 3–4°. That is the floor for heading accuracy.

## 3. Controlled experiments

### 3.1 Single changes, north route (`data/variants/`)

Each session was a fixed five-leg route (home ↔ loading). The variant was applied to the installed parameters
for that session only, then restored.

| Session | Change | Cruise reversals / min | Settled \|heading\| mean / max |
|---|---|---|---|
| A | control | 7.56 (inside the baseline range) | 8.2° / 10.6° |
| B | `forward_point_distance` 0.1 → 0.325 | 1.57 | 9.0° / 9.7° |
| C | angular accel 3.2 → 1.6 (DWB and smoother) | 4.54 | 9.4° / 10.9° |
| D | `yaw_goal_tolerance` 0.15 → 0.07 | 7.32 | 4.7° / 6.8° |
| E | B + D | 2.07 | 4.6° / 5.6° |

These are north-route figures. Compare them only with each other, not with the south-route sessions (§3.2) or
the full missions (§1).

C was not selected: its effect was smaller than B's, and the brief asks for few changes. The first C session
lost its first goal to a DDS discovery race ("Failed to send goal response"); that run is discarded and its log
kept. The route node now waits 3 s after discovery and retries once. *Recorded for Phase 6.*

### 3.2 Forward-point distance, matched south-pinch sessions (`data/fpd_sessions/`)

Each session ran spawn → inventory → dispatch → inventory → dispatch → inventory. That crosses the south pinch
four times (twice westbound through its tightest point) and includes four station approaches. Yaw tolerance
was 0.07 rad and `BaseObstacle.scale` 0.02 throughout; only `forward_point_distance` changed, for both critics.

| fpd | Cruise reversals / min | Heading weave RMS | Total rotation | Pinch: centre clearance (2 passes) | Pinch: footprint window min | Robot off plan, toward obstacle | Settled \|heading\| mean / max | Settled position max | Nav time |
|---|---|---|---|---|---|---|---|---|---|
| 0.10 | 12.15 | 0.41° | 1,582° | 0.32 / 0.38 m | 0.25 / 0.30 m | +0.3 / −0.3 cm | 3.2° / 4.7° | 0.125 m | 178 s |
| 0.20 | 5.48 | 0.38° | 1,443° | 0.36 / 0.32 m | 0.26 / 0.25 m | +2.4 / +4.7 cm | 4.2° / 4.4° | 0.116 m | 175 s |
| 0.325 | **1.71** | **0.29°** | **1,356°** | 0.32 / 0.31 m | 0.25 / 0.21 m | +5.4 / +5.3 cm | 4.3° / 5.1° | 0.122 m | 175 s |

All three sessions: 0 timeouts, recoveries and planning failures; RTF 0.857–0.871.

**Result:**

- **The controller's contribution rises monotonically.** Mean deviation toward the obstacle at the pinch was
  about **0 cm** at 0.10 m, **3.55 cm** at 0.20 m and **5.35 cm** at 0.325 m (two passes each). The sample is
  small; this is an observed ordering, not an established population expectation.
- **The minimum total clearance does not resolve that contribution cleanly.** Global-plan placement varied by
  up to 9 cm between passes, enough to mask a few centimetres of tracking difference. 0.20's worst pass
  (0.32 m centre) equals 0.325's (0.31–0.32 m) and 0.10's (0.32 m).
- **Trade-off behind retaining 0.325 m:** compared with 0.20 m, it gives up about **1.8 cm** of mean tracking
  margin at the pinch. In return, 0.20 m had **3.2× as many** cruise reversals (5.48 against 1.71 per minute),
  a heading weave of 0.38° RMS (0.10 m: 0.41°; 0.325 m: 0.29°), and more rotation (1,443° against 1,356°).
- No 0.25 m candidate was run, and no `BaseObstacle.scale` change was made.

**Camera envelope:**

- Settled offsets toward the marker reached up to **+0.025 m** on this route: the first stops slightly *past*
  the goal seen so far. The earlier "all stops short" observation (Phase 4b §6) no longer holds universally.
- That is outside the tested near-side failure region (+0.125 m or more combined with lateral and heading error).
  The boundary depends on position, orientation and marker geometry.
- **Possible coupling, not established:** with the tighter yaw tolerance the robot keeps adjusting at the goal,
  so its final position depends more on AMCL error. This **could** contribute to stops past the goal. The
  sessions do not prove the yaw tolerance alone caused the sign change.
- The validation mission's stops were all short of the goal.

## 4. Change

`turtlebot3_navigation2/param/humble/burger_cam.yaml`, with in-file justification:

- `general_goal_checker.yaw_goal_tolerance`: 0.15 → **0.07** rad. That is above the AMCL heading-error p95
  (about 3–4°).
- `FollowPath.PathAlign.forward_point_distance` and `GoalAlign.forward_point_distance`: 0.1 → **0.325** m.

`test_motion_config.py` (3 tests) pins these values and everything that must stay unchanged.

## 5. Clearance mechanisms (`data/validation/clearance_mechanisms_missions.txt`, `data/fpd_sessions/`)

**Physical footprint clearance comes first.** The collision footprint's window minimum at the south pinch was:

- **0.25–0.27 m** in the baseline full missions;
- **0.21 m** in the Phase 5 validation mission;
- **0.21 m** minimum in the matched 0.325 m session.

That is a reduction of **4–6 cm, about 16–22% of the measured baseline footprint margin**. The figures come from
a 5 cm raster of the true geometry (±2.5 cm quantization) and the collision outline without the LiDAR housing
overhang. That quantization is a measurement uncertainty, **not a comprehensive safety bound**.

The minimum is mid-route, about 1.4 m into the Dispatch leg at world (0.44–0.54, −8.85 to −8.91), in every run.
Clearance during final station alignment (within 0.30 m of the stop) was 0.85–1.05 m (centre) in every run.
**The tighter yaw tolerance does not affect the pinch.**

At the pinch, the clearance has two components:

1. **Path-tracking deviation, which grows with forward-point distance:**
   - 0.1: −0.8 to +4.2 cm toward the obstacle (5 passes);
   - 0.20: +2.4, +4.7 cm;
   - 0.325: +5.4, +5.3 cm, and **+9.8 cm** in the validation mission.

   The robot crosses 7–18° off the plan direction. This is the corner-cutting effect of the longer
   alignment point, and it is real.
2. **Where the planner puts the path, which varies from run to run:**
   - The plan's own clearance from the true geometry at the pinch was 0.33–0.42 m.
   - The plan is computed in the AMCL-estimated frame, and AMCL position error at the pinch was 3.5–15.7 cm.
   - Localization and planning vary as much as the tracking deviation does, independently of the controller.

**Footprint vs centre:**

- The collision outline (base box behind the axle, wheels at ±0.089 m) is 5–11 cm closer than the centre,
  depending on heading. Footprint figures carry the 5 cm raster's ±2.5 cm quantization.
- The validation mission's window-minimum footprint clearance was 0.21 m (baselines 0.25–0.27 m), and 0.21 m
  also occurred in a 0.325 session.

**Assessment:**

- The validation minimum (0.30 m centre, 0.21 m footprint) combines a large tracking deviation (9.8 cm) with an
  average plan.
- The controller's contribution rises monotonically with forward-point distance (mean about 0, 3.55 and
  5.35 cm). Global-plan placement (0.33–0.42 m) and localization error vary as much, so the total minimum does
  not isolate it cleanly.
- The clearance criterion is **not fully met**. The reduction was explicitly **accepted as a trade-off for the
  current simulated operating environment**.

**Carried forward (not changed in Phase 5 or 6):**

- **Phase 7:** add footprint clearance at the south pinch to the repeatability measurements.
- **Named future improvement: planner-side clearance.** The global plan's own clearance at the pinch was
  0.33–0.42 m. Global inflation cost and NavFn path placement are the candidates to investigate, with
  localization uncertainty as a contributing factor. Implement only if integrated validation shows a
  significant problem.

## 6. Local unknown-space tracking (`data/local_unknown/`): not enabled

Baseline run 2 was replayed into three standalone local costmaps (identical input, separate DDS domains):

| Configuration | Unknown share of window: start / moving (median, p95) | Executed path would be rejected by DWB | No valid arc (blocking) |
|---|---|---|---|
| current (`track_unknown_space` false) | 0% / 0%, 0% | 0% | 0% |
| `track_unknown_space` true | **74%** / **35%, 60%** | **63.9%** | **26.8%** |
| + `inf_is_valid` true | 1% / 5%, 24% | 0% | 0% |

- **Strict tracking blocks DWB.** No-return beams clear nothing, so open space facing the unmapped centre stays
  unknown. The current configuration is retained.
- **The `inf_is_valid` variant** was benign in this open-loop replay but changed nothing on this route. It
  would also let no-return beams from glass or dark surfaces clear cells out to 3 m. It would need closed-loop
  and real-sensor validation before any adoption.

## 7. Actuator and timing consistency (documented, not tuned)

**Wheel vs body acceleration:**

- The Gazebo diff drive (gazebo_plugins 3.9.0) limits each wheel's **rim speed** change to 1.0 m/s²
  (`max_wheel_acceleration`), applied at 30 Hz sim. Rim speeds are v ∓ ω·b/2 with b = 0.16 m.
- The **body** therefore obeys |a_lin| + |α|·b/2 ≤ 1.0 m/s² per wheel: at most 1.0 m/s² linear, up to 12.5 rad/s²
  angular in pure rotation, sharing one wheel budget.
- DWB and the smoother assume 2.5 m/s² linear and 3.2 rad/s² angular. Their **linear** model is more permissive
  than the actuator.

**How often it matters** (`data/validation/actuator_demand.txt`; method of `docs/diagnostics/REPORT.md` §5):

- DWB command changes needing more than 1.0 m/s² at a wheel within one control period: **0.2–0.3%** in the three
  baselines and **0.3%** in the validation mission. The v1.0.1 diagnostic found 0.3%.
- Wheel demand p95 is 0.36–0.37 m/s² in the baselines and 0.25 m/s² in the validation mission.
- The earlier diagnostic found no causal link between the mismatch and the oscillation; the oscillation here was
  resolved through path alignment, not acceleration.

**Smoother timing:** the velocity smoother runs on a **wall** timer (20 Hz) and limits each tick to
`accel / 20`. In sim time its effective limits scale as about 0.95 / RTF × configured: 1.33× in Phase 3, 1.08×
at RTF 0.87 (Phase 4b report §5). The measured |Δω| per command still clips at exactly 0.160 rad/s
(3.2 / 20).

**Why no acceleration tuning now:**

- The mismatch binds for well under 1% of commands, and linear tracking error is 3.5–5 mm/s RMS.
- A controlled test of lower angular acceleration (variant C) gave a smaller improvement than the alignment
  change.
- Matching DWB to the actuator (`acc_lim_x` 1.0) would change acceleration behaviour on every start and stop. It
  belongs to a separate, validated change if it is pursued at all.

## 8. Acceptance

| Criterion | Result |
|---|---|
| Measurable reduction in unwanted oscillation or abrupt angular motion | **Met:** rotation −14%, \|Δω\| p95 halved, angular acceleration p95 −24% or more, reversal frequency −82%, heading weave 0.38–0.43° → 0.25°. The peak is unchanged. |
| Improved settled heading accuracy | **Met:** mean 4.2°, max 5.8° |
| Maximum position error ≤ 0.15 m | **Met:** 0.114 m (slightly above the baseline's 0.091–0.107 m) |
| No significant degradation in obstacle clearance | **Not fully met; trade-off accepted.** Footprint clearance at the pinch 0.25–0.27 → 0.21 m (−16–22%). Part of the loss comes from corner-cutting, part from plan placement and localization (§5). |
| No new navigation or perception regressions | **Met** |
| All five stations verified; 5/5, exit 0 | **Met** |

**Yaw-tolerance trade-offs:**

- The final alignment takes slightly longer: in-place rotation at the goal is more frequent (0.98 against
  0.25–0.76 per minute in the north sessions).
- Accepted positions depend more on AMCL error while the robot adjusts at the goal (position max 0.114–0.139 m
  in sessions).
- The robot is calmer at acceptance and settles faster.

## 9. Tests

`colcon test` for the warehouse packages: **88 tests: 85 passed, 0 failed, 3 skipped** (copyright, by design).

## Limitations

- These are ranges and single observations, not reliability rates. The fpd sessions are one session per value,
  with two pinch passes each.
- Clearance figures carry ±2.5 cm raster quantization. The footprint model is the collision outline, without
  the LiDAR housing overhang. Neither is a safety bound, and the controller is not safety-certified.
- Cruise reversal frequency depends on route and configuration. North-route sessions, south-route sessions and
  full missions are compared only within their own group.
- The variant sessions on the north route lacked hidden action-status topics; acceptance came from logs. The
  recorder was fixed for the south sessions and missions.
- No energy, wear or mechanical-load measurements were made.
- Results hold for the fixed routes and approach directions. Different approaches could change both the
  clearance and the camera envelope (Phase 4b §6).
