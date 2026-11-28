# Phase 6: mission verification and system reliability

Branch `feat/mission-verification`, from Phase 5 (`fe93537`). The phase strengthens how mission completion is
established, makes perception evidence time-aware, fixes signal handling, and makes shutdown orderly. The map,
localization, surveyed goals, camera, controller, costmaps, Phase 3 timing and the validated Phase 5 parameters
(installed parameter hash identical to the Phase 5 validation, `cb66b92c…`) are unchanged.

**Status: not committed; awaiting review.**

## 1. Result

| Measure | Phase 5 validation | **Phase 6 validation** |
|---|---|---|
| Mission / exit / stations verified | 5/5 / 0 / 5 | **5/5 / 0 / 5** |
| Speed at NavigateToPose acceptance | ≤ 1 mm/s, ≤ 0.078 rad/s | ≤ 2 mm/s, ≤ 0.020 rad/s |
| Stationarity established after goal success | not required | **0.60–1.00 s sim** (hold 0.5 s counted after success) |
| Marker evidence | any unstamped `Int32` after arrival | **image captured 0.03–0.12 s after settling, age 44–124 ms at acceptance** |
| Arrival → station verified, mean | 0.12 s wall | **1.10 s wall** (+1.0 s per station) |
| Final check before success | none | **passed:** robot stationary, Nav2 idle |
| Station position error, max / mean | 0.114 / 0.078 m | 0.099 / 0.075 m |
| Settled heading error, mean / max | 4.2° / 5.8° | 3.9° / 5.2° |
| Footprint clearance at the south pinch (window minimum) | 0.21 m | 0.25 m (an observation with identical configuration, not a fix; §7) |
| Camera / controller (sim) | 6.89 Hz / 10.73 Hz | 6.92 Hz / 10.54 Hz |
| RTF / mission duration | 0.873 / 408 s wall (356 s sim) | 0.881 / 418 s wall (368 s sim) |
| Ack timeouts, orphaned goals, overlap, recoveries | 0 | 0 |
| Controller aborts (progress checker) / BT-internal FollowPath retries / recovery behaviours / mission-level navigation failures | 0 / 0 / 0 / 0 | **1 / 1 / 0 / 0** (§7) |
| Live-unknown executed path (original pairing; sim-stamp ordering) | 3.26 m; 3.13 m | 4.41 m; 4.27 m. Residual still unknown in the next snapshot: 0.30 m against 0.31 m (§7) |
| Shutdown: detector / Nav2 container / total | exit 1 / killed by runner, unlogged / > 30 s | **exit 0** / killed by `ros2 launch` escalation (upstream) / **10.5 s** |
| Automated tests | 88 (85 passed, 3 skipped) | **108 (105 passed, 0 failed, 3 skipped)** |

**Overhead:** about 1.0 s per station, about 5 s per mission. Settling takes 0.6–1.0 s sim, then the first fresh
image arrives. The 12 s sim longer mission also contains a 9 s leg-start stall (§7) unrelated to these
changes, so the duration difference overstates the overhead.

## 2. Stationary-state verification (`warehouse_autonomy_node.py`)

Before Phase 6, NavigateToPose SUCCEEDED led straight to marker verification, and mission success was
reported without any stop check. `SimpleGoalChecker` accepts a moving robot; the Phase 5 recordings show
acceptance at up to 80 mm/s (Phase 3: 83 mm/s).

**Now:**

- **ARRIVED → SETTLING.** The station is verified only after fresh, finite odometry shows
  |v| < 0.005 m/s and |ω| < 0.01 rad/s held continuously for **0.5 s sim counted from goal success**.
  - The `settle_motion` tracker reuses `MotionTracker` from `nav2_supervision.py`, with stricter thresholds,
    fed by the same odometry callback.
  - **Stale data is not proof.** Odometry stamped more than `odom_max_age` away from the clock is rejected.
    Freshness is also judged by wall-clock arrival, so a stopped stream cannot keep a hold alive. The hold only
    counts stillness backed by a message stamped after success.
- **Timeouts:**
  - `settle_timeout` 10 s sim; a per-station deadline (settle + verify) bounds re-settling.
  - Failure reasons are explicit: "robot did not remain stationary", "no recent odometry" or "odometry not
    updating".
  - A stalled simulation clock halts the mission (existing watchdog, now also covering SETTLING).
- **Motion during verification** returns to SETTLING. Evidence from that window no longer describes the stop.
- **FINAL_CHECK before success.** The robot must be settled on odometry received after the check began, and no
  Nav2 goal may be active, within 10 s sim. Otherwise the mission reports "NOT CONFIRMED COMPLETE" and exits 1.
  The task counts are left truthful.

**Thresholds from Phase 5 recordings** (`data/validation/stationary_thresholds.txt`):

- At rest (inside the 3 s task hold, 20 arrivals): |v| ≤ 1.0 mm/s and |ω| ≤ 0.35 mrad/s. The thresholds sit
  5× and about 30× above that floor.
- With these thresholds, all 35 recorded arrivals settled: median 0.84 s, p95 1.25 s, max 1.62 s after
  acceptance. The 10 s timeout is about 6× the worst case.

**A defect found and fixed during testing.** The first version credited stillness observed **before** the goal
(at the readiness gate) when odometry stopped during navigation, and "settled" 0.00–0.06 s after success with no
post-arrival evidence. The hold now starts at the later of stationarity onset and goal success.

## 3. Marker freshness (`aruco_station_detector.py`, mission node)

**Before**, the mission accepted any `std_msgs/Int32` on `/warehouse/marker_id` that arrived while it was
VERIFYING. The detector discarded the image timestamp, so the mission could accept:

- an ID from an image captured while still moving;
- a queued or delayed observation of an earlier image;
- a correct ID while the robot was still moving.

The mission did not accept an ID received before arrival, or a cached ID from an earlier station (it reset
on entry and the IDs differ).

**Now:**

- **The detector** also publishes `/warehouse/marker_detections` (`vision_msgs/Detection2DArray`), one message
  per processed image. Its header is the **source image's header**, so it carries the acquisition stamp and
  camera frame. It lists every decoded ID and bounding box, and is empty when no marker was found.
  `/warehouse/marker_id` and `/warehouse/marker_detected` are kept for compatibility. The mission no longer
  uses them.
- **The mission** accepts only an observation that meets all of these:
  - its image was captured at or after the verification window opened (settling complete);
  - it is no older than `marker_max_age` (1.0 s sim; measured image age p95 0.18 s headless, max 0.39 s with
    the GUI);
  - it contains the expected ID;
  - it arrives while the robot is still settled.
- Every rejected observation is counted by reason: captured before the window, too old, wrong ID, no marker,
  robot not stationary, unstamped. The timeout log reports the counts.
- Camera, marker geometry and station goals are unchanged.

## 4. Action startup and acknowledgement

**The production mission node already protects against the Phase 5 harness failure.** The existing tests
cover each point:

- **No goal right after discovery:** the first goal follows `server_is_ready()`, `bt_navigator` ACTIVE, and a
  10 s sim start delay.
- **No resend:** a lost response halts after `goal_response_timeout` ("goal state unknown"). The test asserts
  exactly one goal was sent.
- **Late acceptance is cancelled:** a goal accepted after the timeout is cancelled; the test asserts the
  cancel.
- **Unresolved outcomes are reported:** "ROBOT STOP NOT VERIFIED … navigation goal outcome unresolved".

**The defect was the Phase 5 experiment harness.** It sent its goal immediately after discovery, and its
workaround resent an unacknowledged goal: exactly the blind resend to avoid. The resend is removed; an
unacknowledged goal now fails the session.

## 5. Signal handling (a production defect, fixed)

The full test suite reproduced an exit 1 after SIGINT in the idle phase: `RuntimeError: Unable to convert call
argument`, raised inside rclpy's C-level message take. The mission's handler raised an exception wherever the
main thread was, so it could land inside the executor. The process then ended **without cancel or stop
observation**.

This is the same weakness the README had listed as a next step ("ROBOT STOP NOT VERIFIED" in 2 of about 17
timed runs). The handler now only records the signal (`_StopRequest`). The main loop acts on it between
`spin_once` calls (≤ 0.1 s) and runs the same bounded cleanup, exiting 130. If the mission finishes at the same
moment, its own exit code is kept.

## 6. Graceful shutdown (`data/shutdown/`)

| Item | Cause | Owner | Result |
|---|---|---|---|
| Detector exit 1 with traceback | `rclpy.spin` raises `ExternalShutdownException` on SIGINT; `main()` caught only `KeyboardInterrupt` | **project** | **Fixed:** "finished cleanly" in every later run. The window is destroyed only if opened; `try_shutdown` used |
| `ros2 launch` ignored SIGINT | A non-interactive bash starts background jobs with SIGINT **ignored** (`SigIgn` 0x6, verified), so `ros2 launch` never ran its orderly shutdown | **project tooling** (all runners, including `scripts/run_mission_trial.sh` used by CI) | **Fixed:** `scripts/sigdfl_exec.py` restores the default SIGINT, SIGTERM and SIGPIPE before `exec` |
| Group-wide SIGINT from runners | Every process got SIGINT at once, racing launch's own orchestration | project tooling | **Fixed:** SIGINT to `ros2 launch` only; group SIGTERM/SIGKILL kept only as a recorded last resort |
| Nav2 `component_container_isolated` survives the forwarded SIGINT and SIGTERM; launch sends SIGTERM 5 s and SIGKILL 10 s after the forwarded SIGINT (exit −9) | Nav2 Humble composed bringup: the lifecycle manager reports "CRITICAL FAILURE … IS DOWN" and failed transitions while servers tear down | **upstream** | Not changed. Forced termination is now **launch's** bounded last resort; group empty in 10.5–10.8 s (3 runs) |
| gazebo exit 255, RViz −6/−11 | GUI-mode wrapper and RViz teardown | upstream | GUI mode only; not re-tested in this headless phase; documented |

**Verified timeline** (one wall clock; launch log lines timestamped on receipt; SIGINT to `ros2 launch` at t = 0):

| t | Event |
|---|---|
| +0.01 s | launch registers SIGINT |
| +1.03 s | launch forwards SIGINT to all four processes |
| +1.20 / +1.32 / +1.55 s | `robot_state_publisher`, detector, gzserver **finish cleanly** |
| +6.03 s | container still alive: SIGTERM |
| +11.03 s | SIGKILL; container exit −9 at +11.05 s |
| +11.23 s | process group empty |

**Wording versus timing.** Launch 1.0.10 starts both escalation timers at the forwarded SIGINT: SIGTERM after
`sigterm_timeout` (5 s), and SIGKILL after `sigterm_timeout + sigkill_timeout` (10 s). Its second message, "failed
to terminate '10.0' seconds after receiving 'SIGTERM'", reuses that 10 s period while naming SIGTERM, so it is
misleading. The SIGKILL comes 5 s after SIGTERM, not 10. The earlier 10.5–10.8 s measurements are consistent
with this timeline.

The probe's own per-PID summary (`timeline_probe_raw.txt`) has two artifacts:

- `ros2 launch`'s exit was not stamped, because of the polling-loop order;
- a spurious entry "0".

The stamped launch log is authoritative. All timestamps above come from one clock.

**Shutdown experiment** (headless system up, no mission):

| Launcher / signal | Before the fix (SIGINT ignored) | After the fix |
|---|---|---|
| SIGINT to the group | container hung more than 60 s, runner SIGKILL | group empty in 10.8 s; container killed by launch |
| SIGINT to `ros2 launch` | **nothing happened for 60 s** | group empty in 10.8 s; container killed by launch |
| Validation mission | — | 10.5 s, no runner escalation, group empty |
| Timestamped shutdown-only run (`data/shutdown/timeline_verified.txt`) | — | 11.2 s; per-process times below |

The mission node's shutdown is tested in each phase:

- idle, settling and verifying: exit 130, no new goal;
- an honest stop report: "NOT VERIFIED" while the mock robot is still moving;
- five arbitrary interrupt points;
- the existing SIGINT and SIGTERM tests during executing and pending goals.

Process cleanup in every Phase 6 script uses tracked process groups. One more ambiguous name match was found
and fixed: the runner's "simulation already running" guard (`pgrep -f warehouse_autonomy`) matched the
invoking shell's own command line. It is now anchored.

## 7. Observations from the validation mission

**Departure stall and FollowPath progress failure.** Four separate levels are involved:

| Level | What happened |
|---|---|
| Controller (DWB) | On departure from the Loading Zone the robot stayed nearly still for about 9 s sim; DWB produced only 4 commands with \|ω\| ≥ 0.3 rad/s |
| Controller abort | Nav2's progress checker (0.5 m in 10 s) **aborted FollowPath** once, 10.1 s into the leg ("Failed to make progress") |
| BT-internal retry | The navigate-to-pose behaviour tree cleared the local costmap and **re-issued FollowPath** within the same NavigateToPose goal. This is the stock tree's internal controller retry. |
| Recovery behaviours | **None:** no Spin, Wait or BackUp ran |
| Mission level | **No failure:** the mission's NavigateToPose goal SUCCEEDED and the station was verified |

- The earlier missions, including the Phase 5 validation with the identical controller, turned 30° within
  0.8–0.9 s at this departure.
- Tested and refuted: turn-direction dithering (no sign flips), and a more symmetric 180° departure (23.9° from
  symmetric, against 9–17° in runs that turned immediately).
- Nav2 logged a path within 1.1 s and a new path every second. DWB trajectory details were not recorded.
- **The cause is not established.** Phase 6 changes nothing in Nav2 or its configuration. Phase 7 monitors this
  departure (`data/validation/progress_event.txt`).

**Angular-acceleration spike: a measurement limitation** (`data/validation/angular_accel_methods.txt`):

- The first report gave 21 rad/s² at the Dispatch departure, from the Phase 5 method (10 ms interpolation, 0.1 s
  box filter, difference).
- **Odometry timing is clean:** Gazebo's `/odom` header stamps are on an exact 34 ms grid in every mission, with
  no repeated or out-of-order stamps.
- **The raw data contains transients the actuator cannot produce:** ω changes by up to 0.53–0.93 rad/s between
  consecutive 34 ms samples in every mission, baselines included. The wheel-rim limit (1.0 m/s², b = 0.16 m)
  allows at most 12.5 rad/s² in pure rotation, that is 0.425 rad/s per sample. The diff drive reports Gazebo's
  rigid-body twist (`odometry_source: WORLD`), which includes contact transients. The **commanded** reversal
  was limited by the smoother to 0.16 rad/s per tick.
- **Peaks depend on the method:** the maximum ranges 10.2–27.4 rad/s² across four methods. With a least-squares
  slope over ±0.1 s of raw samples it is 10.2–10.8 in the baselines and Phase 5, and **18.1** in Phase 6.
- **Peak-acceleration claims are withdrawn** from comparisons. The **p95** is robust in direction:
  - baselines 2.16–2.38 rad/s², Phase 5 1.63, Phase 6 1.58 (LS ±0.1 s);
  - Phase 5 and 6 are 21–28% below the baselines under every method tested.

  The original figures are kept as historical evidence.
- The Phase 6 maximum remains higher than the others under every method. One occurrence at a turn reversal,
  recorded and not acted on.

**Live-unknown traversal increase** (`data/validation/live_unknown_timing.txt`):

- Phase 6 measured 4.27 m (sim-stamp ordering) against 3.13 m in Phase 5. **92% of it (3.91 m) was free in the
  next global-costmap snapshot**, published every 1.8 s sim. That is the snapshot-lag component.
- The **residual**, still unknown in the next snapshot, is unchanged: 0.31 m against 0.30 m. Its one-cell
  alignment range is 0.04–0.62 m against 0.00–0.53 m.
- The increase therefore lies in the component dominated by measurement timing, in one run. The unknown-space
  policy and route are unchanged. Phase 7 keeps measuring it.

**Footprint clearance 0.25 m:** an observation, not a correction. The configuration is identical to Phase 5
(0.21 m), so the difference shows run-to-run variation (Phase 5 report §5). Phase 6 did not change anything
that affects clearance.

## 8. Tests

`colcon test`: **108 tests: 105 passed, 0 failed, 3 skipped** (`test_copyright` ×3, by design). Phase 5 had 88.

`test_mission_verification.py` adds 20, covering:

- a correct fresh observation after settling;
- navigation succeeding while the robot still moves;
- a robot that never becomes stationary;
- evidence captured before the window, delayed beyond the age limit, the wrong ID, or the correct ID only on
  the legacy topic;
- stale odometry and odometry lost after arrival;
- a clock stall while settling;
- a final check failing on an active Nav2 goal, and on a moving robot;
- SIGINT while idle, settling and verifying;
- SIGINT at five arbitrary points.

Cancellation during navigation and recovery, and unacknowledged and late-accepted goals, remain covered by
`test_mission_manager.py`.

**Test-reliability fixes made along the way:**

- A race in the lost-odometry test (now cut deterministically at goal receipt).
- The settle-time assertions now compare against the mock's actual stop time.
- The idle-phase interrupt now triggers on a readiness signal instead of a fixed delay.
- The existing sim-time window test now measures on the node's own steady clock. Observing from the test thread
  lagged under load. rclpy's steady clock does not share `time.monotonic`'s base, a mistake caught by the first
  attempt.

## 9. Acceptance

| Criterion | Result |
|---|---|
| No station verified on stale or incorrect evidence | **Met:** tested for images captured before the window, delayed, wrong-ID, legacy-only and moving-robot cases; the mission accepted only post-settling images (age 44–124 ms) |
| Explicit stationary state before verification and success | **Met:** settling per station (0.6–1.0 s) and the final check |
| Bounded handling of missing data and failures | **Met:** settle, station, final-check and existing watchdogs; tested for lost or stale odometry and a stalled clock |
| No new duplicate or orphaned goals in tested scenarios | **Met:** 0 in tests and the mission; the harness resend removed |
| Improved project-owned shutdown | **Met:** every project-owned process exits cleanly within 1.6 s of the signal; the upstream container is force-killed by launch, and the system is down in 10.5–11.2 s; the signal defect fixed |
| 5/5, exit 0 | **Met** |
| No meaningful regression of Phase 5 motion or perception | **Met, with observations:** positioning, heading, smoothness, camera and clearance are comparable. One leg-start stall with an unexplained cause (§7) |

**Remaining upstream limitations:** Nav2's container needs SIGKILL from launch at shutdown (Humble). Gazebo and
RViz GUI-mode exit codes are unchanged.

## Limitations

- One validation mission; tests use a mock Nav2 and are not physical evidence.
- Stationarity is judged from Gazebo odometry, the same source the controller uses, not an independent sensor.
- Marker evidence remains a presence check of the expected ID, not marker pose or range.
- The camera-envelope caveats of Phases 4b and 5 apply. The verification design does not assume the robot stops
  short of the goal; it only requires a settled robot and a fresh image.
