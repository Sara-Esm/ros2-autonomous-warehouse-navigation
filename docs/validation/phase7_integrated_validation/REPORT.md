# Phase 7: integrated validation (v1.1)

Branch `feat/integrated-validation`, from Phase 6 (`4cba097`). No code or parameter changes: this phase measures
the final system's repeatability in both operating modes, monitors the open observations, and prepares the
release. Each mode has **three** full missions. Six missions show repeatable completion under the tested
conditions; they do not establish a reliability rate.

## 1. Campaign

| Run | Mode | Instrumentation | Notes |
|---|---|---|---|
| H1 | headless | Phase 6 validation recording | no DWB recorder |
| H2, H3 | headless | same recording + DWB departure recorder | |
| G1, G2, G3 | graphical (preserved defaults: Gazebo client, RViz, detector window) | same as H2/H3 | |

- **Order and conditions:** run order H2, G1, H3, G2, G3, alternating modes. Each run started on a quiet host
  (load < 2) with an empty process group.
- **Configuration:** every run used the same installed configuration. The Nav2 parameter hash is `cb66b92c…`,
  and the mission-node and detector hashes are recorded in each run's `meta.txt`.
- **Teardown:** by tracked process group, with the signal sent to `ros2 launch` only.
- **Nothing was excluded:** no run failed, and every run is reported.
- Evidence per run is in `data/<run>/`; the consolidated table is `data/integrated_table.md`.

## 2. Results per run

| | H1 | H2 | H3 | G1 | G2 | G3 |
|---|---|---|---|---|---|---|
| Mission / exit / verified | 5/5 / 0 / 5 | 5/5 / 0 / 5 | 5/5 / 0 / 5 | 5/5 / 0 / 5 | 5/5 / 0 / 5 | 5/5 / 0 / 5 |
| Final check (stationary, Nav2 idle) | passed | passed | passed | passed | passed | passed |
| Wall time / RTF | 418 s / 0.881 | 418 s / 0.880 | 469 s / 0.774 | 491 s / 0.748 | 560 s / 0.653 | 544 s / 0.676 |
| Position error max / mean | 0.099 / 0.075 m | 0.127 / 0.089 | 0.127 / 0.087 | 0.099 / 0.074 | 0.099 / 0.081 | 0.117 / 0.073 |
| Settled heading mean / max | 3.9° / 5.2° | 3.3° / 3.9° | 3.9° / 5.7° | 3.6° / 4.8° | 4.4° / 6.6° | 4.7° / 6.8° |
| Settling after goal success | 0.6–1.0 s | 0.6–1.1 | 0.6–1.1 | 0.7–0.9 | 0.5–1.1 | 0.6–0.9 |
| Marker evidence age at acceptance | 44–124 ms | 73–143 | 103–193 | 185–253 | 312–377 | 222–395 |
| Camera (sim) | 6.92 Hz | 7.19 | 4.70 | 4.15 | 2.63 | 2.69 |
| Controller / smoother (sim) | 10.5 / 21.3 Hz | 10.6 / 21.4 | 12.0 / 24.3 | 12.3 / 24.9 | 13.8 / 28.4 | 13.5 / 27.6 |
| Ack timeouts / orphaned goals / overlap | 0 / 0 / 0 s | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| Controller aborts / recoveries (Spin/BackUp/Wait) | 1 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| Loading→Inventory: time to turn 30° | **9.3 s** | **4.7** | 1.1 | **2.0** | 0.9 | **2.2** |
| Cruise reversals / nav-min | 0.93 | 1.31 | 2.09 | 1.89 | 2.08 | 2.46 |
| Total rotation | 1,935° | 2,066 | 2,025 | 2,105 | 1,984 | 1,906 |
| Angular accel p95 (LS ±0.1 s) | 1.58 rad/s² | 1.68 | 1.69 | 1.77 | 1.60 | 1.51 |
| South pinch: centre / footprint (window min) | 0.33 / 0.25 m | 0.37 / 0.29 | 0.32 / 0.25 | 0.40 / 0.32 | 0.33 / 0.25 | 0.32 / 0.25 |
| South pinch: plan clearance / robot off plan / AMCL error | 0.36 m / +0.6 cm / 4.5 cm | 0.43 / +1.7 / 10.4 | 0.32 / −1.3 / 4.0 | 0.43 / +1.0 / 6.8 | 0.36 / +1.2 / 10.9 | 0.32 / −0.6 / 12.9 |
| Stop offset toward marker (dx) | −0.097…+0.006 m | −0.127…+0.040 | −0.124…−0.020 | −0.095…+0.053 | −0.099…+0.018 | −0.111…+0.022 |
| Live-unknown path / residual | 4.27 / 0.31 m | 3.47 / 0.15 | 3.62 / 0.40 | 2.37 / 0.00 | 3.17 / 0.30 | 3.53 / 0.46 |
| Shutdown (signal → group empty) | 10.5 s | 10.5 | 10.9 | 11.1 | 10.8 | 11.3 |

Signs: robot off plan "+" = toward the obstacle. Stop offset "+" = past the goal, toward the marker.

**Shutdown exit codes:**

- Every run: detector, `robot_state_publisher`, `spawn_entity` and (headless) gzserver finish cleanly. The
  Nav2 container is force-killed by `ros2 launch` (−9; upstream, Phase 6 §6).
- Graphical runs also have the `gazebo` wrapper exiting 255 in G1 and G2 (cleanly in G3) and `rviz2` exiting
  −11 in all three (upstream GUI teardown).
- No process survived any teardown. The process group was empty every time.

**Host load** (`data/host_conditions.txt`):

- **Headless:** CPU 81–86% busy, load average 12–15.
- **Graphical:** CPU 92–96% busy, load average 27–32. Gazebo's client (about 180%) and RViz (about 105%) take
  CPU from gzserver, which lowers RTF and the camera rate.
- **H3** ran slower than H1 and H2: its Nav2 container used about 100% of a core against about 70%, with no
  other host difference. That is run-to-run variation; the cause is unknown.

## 3. Ranges by mode

| Measure | Headless (H1–H3) | Graphical (G1–G3) |
|---|---|---|
| Completion | 3/3, exit 0, 15/15 stations verified | 3/3, exit 0, 15/15 stations verified |
| Position error, max (per run) | 0.099–0.127 m | 0.099–0.117 m |
| Settled heading, mean / max (per run) | 3.3–3.9° / 3.9–5.7° | 3.6–4.7° / 4.8–6.8° |
| Settling after goal success | 0.6–1.1 s sim | 0.5–1.1 s sim |
| Marker evidence age at acceptance | 44–193 ms | 185–395 ms (limit 1.0 s) |
| Camera / RTF | 4.7–7.2 Hz sim / 0.77–0.88 | 2.6–4.2 Hz sim / 0.65–0.75 |
| Wall time | 418–469 s | 491–560 s |
| Footprint clearance at the south pinch | 0.25–0.29 m | 0.25–0.32 m |
| Timeouts, orphans, overlap, recoveries | 0 | 0 |
| Shutdown | 10.5–10.9 s | 10.8–11.3 s |

## 4. Comparison with the v1.0.1 baseline

The v1.0.1 figures come from the diagnostic mission (`docs/diagnostics/REPORT.md`, 2026-11-25). That was a single
graphical run with the 182° camera, world-frame goals and Nav2's 20 ms acknowledgement budget.

| Measure | v1.0.1 (diagnostic run) | v1.1 headless (3 runs) | v1.1 graphical (3 runs) |
|---|---|---|---|
| Mission completion | final step not observed (simulator crash) | 3/3, exit 0 | 3/3, exit 0 |
| Station position error (true stop vs intended pose) | 0.18–0.50 m (mean 0.35 m, max 0.504 m) | max 0.099–0.127 m | max 0.099–0.117 m |
| Nav2-internal failures from acknowledgement timeouts | 19 (13 planner, 6 controller) | 0 | 0 |
| Recovery behaviours | Spin, Wait, BackUp | 0 | 0 |
| Camera frames | 0.60 Hz sim, about 1.6 s latency | 4.7–7.2 Hz sim | 2.6–4.2 Hz sim |
| Marker evidence | unstamped ID, any time after arrival | image captured after settling, ≤ 0.2 s old | image captured after settling, ≤ 0.4 s old |
| Stationary check before verification and success | none | settled 0.5 s after success; final check | same |
| RTF | about 0.66 | 0.77–0.88 | 0.65–0.75 |
| Shutdown | not measured (crash) | project processes clean; container force-killed by launch; 10.5–10.9 s | same; GUI wrapper and RViz upstream exits; 10.8–11.3 s |

Intermediate phases are documented in their own reports (index: `docs/validation/README.md`).

## 5. Observations monitored

**Departure delay (Loading → Inventory).** This is the only leg start that needs a turn of about 180° with the
station wall behind the robot.

| Runs | Configuration | Time to turn 30° |
|---|---|---|
| Baselines B1–B3 | forward-point 0.1 m, yaw tolerance 0.15 | 0.8–0.9 s |
| Phase 5 validation | 0.325 m, 0.07 | 0.8 s |
| Phase 6–7 runs H1, H2, H3, G1, G2, G3 | 0.325 m, 0.07 | 9.3, 4.7, 1.1, 2.0, 0.9, 2.2 s |

- **H1's** 9.3 s ended in a Nav2 progress-checker abort, with an internal FollowPath restart, no recovery
  behaviour and no mission impact. The others resolved by themselves.
- **DWB decision evidence** (`data/stall_candidates.txt`), recorded only during this leg in H2–G3. In the
  stalled cycles of H2 and G3:
  - all 819 candidate trajectories were valid (419 in two cycles);
  - the chosen command was nearly still (vx 0, |vθ| 0.026 rad/s), with cost 46.40 (H2) and 46.20 (G3);
  - the best right and left turns cost 46.60 and 47.20 (H2), and 46.60 and 46.40 (G3);
  - the critic terms were dominated by `GoalAlign` (about 22) and `GoalDist` (about 19).
- **So this is a near-flat cost landscape:** turning in place barely changes the alignment and goal costs at a
  ~180° departure. The robot left when a full turn tied with or beat standing still, as new plans arrived.
- **Association:** delays of 2 s or more appear in 4 of 7 runs with the Phase 5 configuration and in 0 of 3
  baselines. That is an association with the configuration (plausibly the longer alignment point), **not an
  established cause**; the Phase 5 validation itself departed normally.
- **No parameter was changed.** A standard mitigation would be Nav2's `RotationShimController`, which rotates
  toward the path heading before handing over to DWB. It is future work and needs its own validation.

**Recorder overhead** (`data/departure_and_overhead.txt`):

- The recorder subscribed to `/evaluation` only during the 25 s wall window. DWB in Nav2 1.1.18 builds the
  evaluation every cycle by default (`publish_evaluation` and `publish_trajectories` default to true) but
  publishes only while subscribed.
- The controller held 9.5–10.0 Hz wall inside every window, the same as without the recorder.
- Window RTF was at most about 7% below the run average (H2, G3); in other runs it was not lower.
- The recorder caught only 0.5–1.4 evaluations per second: depth-1 publisher, large messages, Python
  deserialization. Departure timing is therefore taken from odometry.
- **Not a significant change:** these runs stay in the repeatability set.

**Angular-acceleration peaks.** The maxima (LS ±0.1 s) were 12.5–18.1 rad/s², and remain method-dependent
(Phase 6 §7). No peak claim is made. The p95 (1.51–1.77 rad/s²) stays below the baselines' 2.16–2.38 rad/s².

**Footprint clearance at the south pinch:**

- Phase 7 measured 0.25, 0.29 and 0.25 m (headless) and 0.32, 0.25 and 0.25 m (graphical).
- Across Phases 5–7, with the same configuration, the values were 0.21 (Phase 5), 0.25 (H1), then the above;
  the baselines were 0.25, 0.26 and 0.27 m.
- Global-plan clearance at the pinch varied from 0.32 to 0.43 m. The robot tracked the plan within ±1.7 cm in
  every Phase 7 run.
- The Phase 5 value of 0.21 m was the lowest observed. The size of any systematic regression stays unresolved
  next to this run-to-run variation, and the original clearance criterion is not shown to be met.

**Collisions:** none observed in any mission.

- The minimum robot-centre clearance to the true geometry was 0.32 m (all legs, all runs), always at the south
  pinch.
- The collision outline extends at most 0.135 m from the centre (rear corners at −0.102 m, ±0.089 m), so the
  footprint clearance is at least 0.32 − 0.135 = 0.185 m everywhere. The measured footprint clearance at the
  pinch was 0.25 m or more.
- Raster quantization is ±0.025 m. This is a geometric bound from recorded truth poses, not a safety
  certification.

**Stops past the goal:** in 5 of 6 runs, at least one station stop was past the goal, by up to +0.053 m (G1). That
is still well short of the +0.125 m near-side detection failure region (Phase 4 §5.1). Every station was verified
with fresh evidence.

**Unknown space:**

- The live-unknown path was 2.37–4.27 m per mission, measured against global-costmap snapshots 1.8 s apart.
- The residual still unknown in the next snapshot was 0.00–0.46 m.
- The same snapshot-lag and alignment limitations apply (Phase 4 §8, Phase 4b §6).

## 6. Tests and CI

- **Automated tests on the final code:** 108 tests, 105 passed, 0 failed, 3 skipped (`test_copyright` ×3, by
  design). Phase 7 adds no code.
- **CI** (`.github/workflows/warehouse-humble.yml`) runs `warehouse-tests` and `gazebo-smoke` on pull requests
  into `main`. Phase 6 added `ros-humble-vision-msgs` to both jobs.
- **Timing headroom:** the local suite takes about 6.6 min. On GitHub the pre-Phase 6 test step took 186 s
  against a 30 min job limit, and the Gazebo mission 524 s against 50 min.
- `main` currently enforces no required status checks.
- The integration sequence, base branches, expected commits and retargeting are in
  `docs/release/INTEGRATION_PLAN.md`. Nothing has been merged.

## 7. Acceptance

| Criterion | Result |
|---|---|
| Repeatable completion in both modes | **Met:** 3/3 headless and 3/3 graphical, exit 0 |
| Station verification with fresh perception evidence | **Met:** 30/30 visits; evidence captured after settling, 44–395 ms old |
| Explicit stationary-state completion | **Met:** every station settled; final check passed in 6/6 |
| No overlapping or orphaned navigation goals in tested scenarios | **Met:** 0 in all runs and in the failure-path tests |
| No collisions in the tested missions | **Met:** footprint clearance ≥ 0.185 m by geometric bound, ≥ 0.25 m measured at the pinch |
| Positioning within 0.15 m | **Met:** max 0.127 m |
| Transparent reporting of clearance, timing and shutdown limitations | **Met:** §5 and the Phase 5–6 reports |
| Passing automated tests | **Met:** 105 passed, 0 failed, 3 skipped by design |

**Limitations carried into v1.1:**

- The departure delay (§5).
- The south-pinch clearance variation.
- Method-dependent acceleration peaks.
- Nav2's container needing SIGKILL at shutdown, and upstream GUI teardown exits.
- Simulation only, one machine, six missions.
