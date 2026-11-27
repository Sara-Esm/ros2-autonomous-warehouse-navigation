# Phase 3 validation: timing and synchronization

*Branch `feat/timing-sync`, based on the Phase 2b commit `c638b55`. One validation mission, compared with
Phase 2b using the same scripts. Unchanged: map, station goals, camera, DWB tuning, goal tolerances, perception
and mission supervision.*

## Summary

| Measure | Phase 2b | **Phase 3** |
|---|---|---|
| Mission / markers | 5/5, exit 0 / 5 | **5/5, exit 0 / 5** |
| Action-acknowledgement timeouts | 19 (1.93 / wall min) | **0** |
| BT service-call timeouts (costmap clears) | 7 | **0** |
| `halt()` cancel/result errors | 3 | **0** |
| Late-accepted (orphaned) goals after a timeout | 15 | **0** |
| Time with two recovery behaviours executing at once | 17.3 s | **0 s** |
| BT `ComputePathToPose`/`FollowPath` failure records | 16 | **0** |
| Recoveries (Spin / Wait / BackUp) | 3 / 3 / 3 | **0 / 0 / 0** |
| "Failed to create plan" | 1 | **0** |
| Mission wall time / RTF | 591 s / 0.657 | **509 s / 0.711** |
| Station position error, stations 1–4 mean / all five mean / max | 0.078 / 0.076 / 0.116 m | 0.085 / 0.087 / 0.119 m |
| Minimum clearance to true geometry | 0.36 m | 0.31 m (south pinch, same place) |
| First plans on static-unknown / live-unknown cells | 0.08 m / 0 | 1.52 m / 0 (see §5) |
| Route changes > 0.5 m | 1 (0.77 m) | 1 (0.76 m) |

## 1. Timing semantics (installed Nav2 1.1.18 code, runtime parameters, existing evidence)

- **`default_server_timeout`** was 20 ms (runtime value now 200, `data/run/runtime_params.txt`). In
  `bt_action_node.hpp` and `bt_service_node.hpp` this one value is the budget for:
  1. every BT action goal **acknowledgement** (planner, controller, Spin, BackUp, Wait);
  2. every BT **service** call (costmap clears);
  3. the cancel and result waits in `halt()`.
- **Elapsed time** is measured on the BT client node's clock (`node_->now()`). The blocking waits use the
  executor's steady clock, at most `bt_loop_duration`/2 = 5 ms per tick.
- **Which clock the BT client node uses.** `bt_navigator_navigate_to_pose_rclcpp_node` receives no
  `use_sim_time` (inferred from code, §4.2 of the diagnostics report), so it is on wall time. A statistical
  test on the timeout phases supports this. Its parameter service does not answer: `ros2 param get` hung
  indefinitely and stalled the first run (`data/run/stalled_first_attempt.md`), so it cannot be read back
  directly. It was left on wall time **deliberately**: acknowledgement latency is a communication effect, and
  the measured latencies are wall-clock.
- **On acknowledgement timeout**, `BtActionNode` resets the future and returns FAILURE **without cancelling**.
  `goal_handle_` is never set, so a later `halt()` cannot cancel either, and the late-accepted goal runs
  orphaned. Confirmed in code and in Phase 2b data: 15 of 19 timed-out goals were later accepted, giving
  17.3 s of overlapping behaviours. Example: at 93 s a BackUp "failed" in the BT but completed on the server,
  while the BT had moved on to Spin.
- **On service timeout**, the BT reports FAILURE, but the server still performs the call (the clear happens).
- **Velocity smoother**: `use_sim_time` was false (no YAML section), on wall time. See §3.
- **Costmap clearing and replanning** (`data/offline/post_clear_*.txt`, real `planner_server`, no Gazebo):
  - **Sequential** "clear entire global costmap, then plan" never failed (15/15 at delays 0–2 s). The costmap
    published 0.03 s after a clear already contained the full static map, so there is no stable "all unknown"
    window.
  - **Concurrent** (plan request, then a full clear while it is being computed): **4/120** plans failed with
    `allow_unknown: false`, **0/100** with `allow_unknown: true`, and **0/160** when the clear was
    `ClearCostmapExceptRegion`, which leaves the static layer alone.
  - So the Phase 2b planner failure (planning started 2 ms after a timeout-triggered full clear) is consistent
    with a **clear/plan race** that bites under the strict policy. The counts are small: the race is
    reproduced, but the policy comparison alone (3/100 vs 0/100) is not statistically strong.

## 2. Acknowledgement latency and the chosen timeout (`data/offline/ack_latency_baseline.txt`)

Latency estimate: (wall time a new `compute_path_to_pose` goal first appears in the action status) − (BT event
time of `ComputePathToPose` IDLE→RUNNING). The status transport adds a small positive bias.

| Run | Goals | p50 | p99 | p99.9 | max | > 20 ms |
|---|---|---|---|---|---|---|
| Phase 1 | 475 | 4.4 ms | 25.5 ms | 67.3 ms | 86.3 ms | 2.7% |
| Phase 2a | 497 | 4.3 ms | 24.8 ms | 59.9 ms | 68.1 ms | 2.2% |
| Phase 2b | 460 | 4.3 ms | 31.1 ms | 50.4 ms | 57.0 ms | 3.9% |
| **Phase 3** | 429 | 3.2 ms | 25.5 ms | 57.1 ms | **74.7 ms** | 1.9% (no timeouts at 200 ms) |

Pooled over all **1,861** sampled planner goals (Phases 1, 2a, 2b and 3; `data/run/phase3_evidence.txt`): p50 4.0 ms,
p90 13.0, p95 16.3, p99 28.7, p99.9 69.0, **max 86.3 ms**. 50 goals (2.7%) exceeded 20 ms, 5 exceeded 50 ms, none
exceeded 100 ms. **Eight Phase 3 estimates exceeded the original 20 ms deadline**; those would plausibly have timed
out under the old budget, and none did at 200 ms.

These are **estimates**: the time a goal first appears in the status topic minus the BT's send event. They include
status-topic transport. They are not direct measurements of the acknowledgement response inside `BtActionNode`.

**200 ms** is 2.3× the largest latency seen over 1,432 baseline goals. The cost is slower detection of a truly
unresponsive server (+0.18 s), negligible against 1 Hz replanning. Only `compute_path_to_pose` latency is
directly measurable from the recordings; the other actions' timeouts were proportionally similar. It was not
the case that every timeout was raised: `bt_loop_duration`, `wait_for_service_timeout` and all controller,
planner and costmap rates are unchanged.

## 3. Clock configuration

| Node | Before | After (runtime) |
|---|---|---|
| bt_navigator, controller, planner, behavior server, AMCL, local costmap | sim | sim |
| `velocity_smoother` | **wall** | **sim** |
| `smoother_server` (not used by the BT) | **wall** | **sim** |
| BT client node (acknowledgement timer) | wall | wall (deliberate, §1) |

Fix: explicit `velocity_smoother` and `smoother_server` sections, so the bringup's `use_sim_time` rewrite
reaches them. Other parameters keep their Humble defaults.

**Evaluated separately** (`data/run/smoother_effect.txt`, recovery periods excluded):

| | DWB→smoother lag | Correlation (v) | RMS smoother − DWB (v) | Smoother output rate |
|---|---|---|---|---|
| Phase 2b (wall clock) | 50 ms | 0.996 | 0.0044 m/s | 19.0 Hz wall |
| Phase 3 (sim clock) | 50 ms | 0.996 | 0.0041 m/s | 18.6 Hz wall |

The smoother's output loop stays wall-paced (about 19 Hz wall in both runs). **The clock change has no
measurable effect on command shaping.** It is a consistency correction. A first, uncorrected comparison
suggested a large improvement; that turned out to be confounded by Phase 2b's recoveries, which publish
`/cmd_vel` directly.

## 4. Planning against an updated costmap: minimal BT change

`behavior_trees/navigate_to_pose_w_replanning_and_recovery_keep_static.xml` is Nav2's stock tree with exactly
one change, test-enforced: the two **global** `ClearEntireCostmap` nodes become `ClearCostmapExceptRegion`
(`reset_distance 1.0`). These still clear stale obstacle-layer marks but never reset the static layer. Local
clears are unchanged. `warehouse_navigation.launch.py` sets `default_nav_to_pose_bt_xml` through
`RewrittenYaml`; the runtime value confirms the tree was loaded.

**Exercise in the mission:** none. No BT failures occurred, so **no costmap clear happened** in the Phase 3
mission. The change is supported by the controlled race test (0/160 failures against 4/120), not by the full
mission.

## 5. Validation mission (same instrumentation as Phase 2b, plus a runtime parameter snapshot)

- **Outcome: 5/5, exit code 0, all five markers verified.** Mission 509 s wall (Phase 2b: 591 s).
- **No acknowledgement, service or halt timeouts; no BT failures; no recoveries; no orphaned or overlapping
  actions.**
- **One `[follow_path] Aborting handle` warning at 261.9 s, benign.** A new path goal arrived 31 ms later, and
  no BT failure followed. It matches `simple_action_server.hpp` terminating a superseded *pending* goal when a
  newer one arrives (lines 185–189), so it is not a controller failure (inferred from code and timing).
  Earlier runs had 2–4 each.
- **Update rates:** controller (`/cmd_vel_nav`) 12.7 Hz sim / 9.0 Hz wall (Phase 2b: 13.5 / 8.8); planner
  (`/plan`) 1.26 Hz sim (1.25); smoother 18.9 Hz wall (19.1). Controller missed-rate warnings fell from 172 to
  51, planner missed-rate warnings from 44 to 32, BT tick warnings from 3,942 to 2,521. RTF was 0.711 against
  0.657, with similar host load during both missions (load average about 25–26). The RTF difference is not
  attributed.
- **Positioning:** 0.083 / 0.088 / 0.119 / 0.049 / 0.094 m, all ≤ 0.15 m. Stations 1–4 mean 0.085 m (Phase 2b:
  0.078 m). Headings again sit at the 8.6° tolerance edge. The difference is within what single runs cannot
  resolve; it is not attributed to the timing changes.
- **AMCL residual:** median 6.3 cm, p95 17.4 cm, max 23.4 cm (Phase 2b: 7.8 / 18.3 / 28.6 cm).
- **Unknown-space policy:** live-unknown share of all plans 0%.
  - Plans used 1.52 m of static-unknown cells (return leg: mean 15% of plan poses). No clears happened, so the
    live costmap kept corridor cells the LiDAR had observed on earlier legs, and the planner used them. Under
    `allow_unknown: false` that is the intended behaviour: observed free is free.
  - Executed exposure: 9.8 m static / 3.5 m live-unknown (Phase 2b: 7.0 / 2.9 m), near corridor edges as before.
- **Clearance:** minimum 0.31 m at the south pinch (Phase 2b: 0.36 m), with the same corridor and the same
  limiting geometry, about 5 cm closer this run. Route changes: 1 (0.76 m, return leg), as in Phase 2b.
- **Final arrival:**
  - DWB's last nonzero command came 0.07 s before the final NavigateToPose success, at v = 0.069 m/s. The goal
    checker accepted while the robot was still translating inside the 0.12 m tolerance; Phase 2b accepted at
    the end of an in-place rotation.
  - Commands were zero from then on. The robot stopped within 0.2–0.5 s (last motion above 2 mm/s at +2.2 s,
    a small yaw settle) and stayed stationary through MISSION COMPLETE (10.4 s later) and the 60 s observation
    window.
- **Teardown:** detector exit 1, RViz −11 and Gazebo 255 at about 66–68 s after mission exit, as before.

## 6. Acceptance

| Criterion | Result |
|---|---|
| Substantial reduction in unnecessary acknowledgement timeouts and recoveries | **Met:** 19 → 0 timeouts, 9 → 0 recoveries, 15 → 0 orphaned goals |
| Consistent, justified clock configuration | **Met:** smoother nodes on sim time; BT client node on wall time by design (documented, not read back at runtime) |
| No new action-sequencing or cancellation defects | **Met:** no halt errors, overlaps or orphans; the one pending-goal abort is a known benign pattern |
| 5/5, exit 0; all markers verified | **Met** |
| Positioning ≤ 0.15 m | **Met** (max 0.119 m) |
| No degradation of obstacle clearance or unknown-space policy | **Policy: met** (0% live-unknown in plans). **Clearance: 0.31 m against 0.36 m** at the same south pinch the route must use. Within the run-to-run range seen across phases (0.28–0.36 m) and not attributed to timing changes, but it is a lower number and is reported as such. |

**Phase 3 verdict: accepted.** The 200 ms acknowledgement timeout removed every timeout-driven failure in the
mission, and with them all recoveries, orphaned actions and the post-clear planning failure (19 → 0, 9 → 0).
The clock correction is consistency-only, with no measurable behavioural effect. The keep-static global clear
is supported by the controlled race test but was not exercised in the mission, because nothing failed. One run
per phase: these are measured improvements on a single validation run, not reliability rates.

## 7. Finalization evidence (added before commit)

### 7.1 Attribution and host conditions

The reduction from 19 to 0 acknowledgement timeouts is an **observed result of the combined intervention** in one
run: the timeout change, the clock sections and the BT change, under the host conditions of that run. It is not
attributed entirely to either the timeout or the host.

**Timeout rates with matching action categories** (`data/run/phase3_evidence.txt`):

| Run | `compute_path_to_pose` timeouts / goals | `follow_path` timeouts / goals | Per wall min (both) |
|---|---|---|---|
| Phase 1 | 9 / 475 (0.019) | 7 / 469 (0.015) | 1.67 |
| Phase 2a | 9 / 497 (0.018) | 6 / 491 (0.012) | 1.50 |
| Phase 2b | 10 / 460 (0.022) | 6 / 454 (0.013) | 1.62 (+3 spin/backup) |
| **Phase 3** | **0 / 429** | **0 / 428** | **0** |

**Host conditions during the mission** (top samples every ~2 s):

| | Phase 2b | Phase 3 |
|---|---|---|
| CPU busy (median) | 95% | 94% |
| Load average, 1-min median | 30.3 | 28.8 |
| Memory used, median | 2,445 MiB | 2,427 MiB |
| Memory available, minimum | 1,128 MiB | 1,212 MiB |
| RTF | 0.657 | 0.711 |
| gzserver / gzclient / Nav2 container / RViz (% of one core) | 306 / 191 / 91 / 68 | 324 / 191 / 72 / 103 |

The conditions were similar. The 8 Phase 3 latency estimates above 20 ms show that the load still produced
deadline-exceeding acknowledgements; the 200 ms budget absorbed them.

**Remaining vulnerability.** Nav2's `BtActionNode` still neither cancels nor tracks a goal whose acknowledgement
exceeds the budget. An acknowledgement slower than 200 ms (none observed so far) would still produce an orphaned goal.

### 7.2 Arrival accuracy at acceptance and after settling (`data/run/arrival_accuracy.txt`)

Definitions, identical for every phase:

- **Acceptance:** NavigateToPose status SUCCEEDED.
- **Settled:** the first time from which |v| < 2 mm/s and |ω| < 0.005 rad/s hold for 1.0 s of sim time.
- **Errors:** measured against the intended physical approach poses.

Nav2's `SimpleGoalChecker` (the one configured) checks pose tolerances only; checking velocity is what the
separate `StoppedGoalChecker` adds. Acceptance while moving is therefore by design.

| Run | Stations 1–4 mean: acceptance / settled | Max (all five): acceptance / settled | Settling time | Largest travel after acceptance |
|---|---|---|---|---|
| Phase 1 | 0.069 / 0.069 m | 0.102 / 0.102 m | 0.7–1.5 s sim | 0.4 mm |
| Phase 2a | 0.057 / 0.057 m | 0.103 / 0.103 m | 0–0.8 s | 0.1 mm |
| Phase 2b | 0.078 / 0.078 m | 0.116 / 0.116 m | 0.7–1.3 s | 0.3 mm |
| Phase 3 | 0.085 / 0.085 m | 0.119 / 0.119 m | 0.5–1.2 s | 2.2 mm (return home, accepted at 83 mm/s) |

Positions at acceptance and after settling agree within millimetres. Headings changed by up to 2.8° during the
residual rotation (Phase 1 dock: +6.9° → +4.1°).

### 7.3 Return-home plans against the live global costmap (`data/run/plan_vs_live_costmap.txt`)

The costmap origin and size equal the map's, (−7.28, −10.7) and 258×428, so no alignment fit is needed. Each of
the 123 return-leg plans was compared with the preceding recorded costmap (age median 1.02 s, max 2.72 s wall):

- **6,587** plan poses lay on static-unknown cells. In the live costmap **6,555 (99.5%) were free**, cleared by
  LiDAR observation, and 32 were still unknown in the snapshot. That is consistent with the snapshot lag; with
  `allow_unknown: false` the planner's own costmap must have marked them traversable when it planned.
- **No plan pose was on an inscribed or lethal cell.**

### 7.4 Bounded failure-path tests (`data/failure_tests/`)

The successful mission did not exercise the modified recovery behaviour or any failure-triggered halt path. Four
bounded experiments did, without a five-station mission:

| Test | What it exercised | Result |
|---|---|---|
| F1, offline real `planner_server` | Injected **stale** obstacles (synthetic scans, then a sensor stream that no longer sees them): A 0.35 m from the robot, B a wall ~1.2 m away | Implemented `ClearCostmapExceptRegion(reset_distance 1.0)` cleared B and **kept A**; the static layer was intact. Stock `ClearEntireCostmap` cleared both. |
| F2, live | Cancel during normal navigation; 25 s injected phantom obstacle ahead of the robot | Cancel response 3 ms, CANCELED in 35 ms, robot stopped in 0.55 s. The phantom slowed the robot (0.20 → 0.085 m/s) but caused no BT failure: real scans kept clearing it. Goal succeeded. |
| F3, live | Unreachable goal (inside the central block; injected fault) → planning failures and recoveries | 28 planning failures, each followed by a keep-static global clear; Spin, Wait and BackUp in round-robin; clean ABORT after retries (38.7 s). 0 ack, service or halt errors; 0 orphaned or overlapping behaviours. |
| F4, live | Cancel **during** Spin and **during** Wait | Navigation and behaviour goals both CANCELED within about 0.10 s; robot stopped (1.09 s after a spin, 0.13 s during a wait); 0 behaviour goals active 5 s later; 0 halt errors. |

**No release-blocking defect was found.** Residual risk from F1: a stale obstacle *inside* the preserved 1 m
region survives the global clear. In live operation it is normally erased by real LiDAR rays, but not along
no-return beams. That is a trade-off against the clear/plan race the change avoids; it is documented, not changed.

### 7.5 Tests and runner

- Automated tests: **78 collected: 75 passed, 0 failed, 3 skipped** (the copyright tests, skipped by design).
- **Stalled first validation runner:**
  - It hung on `ros2 param get` for the BT client node, whose parameter service does not answer.
  - `timeout` lacked `-k`, so it never escalated to SIGKILL.
  - The host then suspended.
  - Correction: hard-kill timeouts (`timeout -k 3 15`), and that node is no longer queried. The stalled files are
    kept locally (`data/run/stalled_first_attempt.md`).

## Limitations

- One validation run. The first attempt stalled (a hung `ros2 param get`, then host suspension) and was
  replaced by a fresh run; its files are preserved locally.
- Acknowledgement latency is measured only for `compute_path_to_pose`. The other actions are inferred from
  timeout counts.
- The BT client node's clock is inferred (code plus a statistical test), not read back.
- The keep-static BT change was not exercised by the validation mission. It was exercised in the bounded tests
  F1, F3 and F4 (§7.4).
- The late-goal cancellation weakness in `BtActionNode` remains possible for acknowledgements slower than 200 ms.

## Reproduction

Raw data is kept locally in the git-ignored `diagnostics_phase3/`. Scripts:

- `ack_latency.py <label>=<run>`;
- `timing_metrics.py <label>=<run> ...`;
- `smoother_effect.py <label>=<run> ...`;
- `post_clear_experiment.py <ws> <out>` (env: `RACE_ONLY=<N>`, `ALLOW_UNKNOWN=true|false`);
- `run_phase3_validation.sh`;
- the Phase 2b evaluation scripts via `eval_wrapper.py`.
