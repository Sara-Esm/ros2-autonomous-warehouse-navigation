# v1.0 history: trials, supervision story and original results

*Moved verbatim from the README at v1.1 (Phase 7). Kept as the historical baseline; numbers reflect the code at the time of each trial. Current results: [`../README.md`](../README.md) and [`validation/phase7_integrated_validation/REPORT.md`](validation/phase7_integrated_validation/REPORT.md).*

## The story: from Trial 08 to Trial 12

**Trial 08 exposed the problem.** On the final return home, the outer `NavigateToPose`
aborted. Nav2's spin, backup and wait behaviors kept running for about 10 s afterwards, and
a `follow_path` controller goal stayed active for about 64 s. The mission manager of that
time treated the abort as "Nav2 is idle". In a separate fault-injection run with the
controller deactivated, the same manager sent the next goal during Nav2's lifecycle restart
and then waited forever for a response.

**The fix** was to check readiness before each goal and bound the wait. Reported activity
on a monitored action or fresh odometry showing motion keeps the gate closed.

**Trial 12 tested it under stress.** The host was heavily loaded (load average 21) and the
simulation ran at 0.31× real time. At step 1, `bt_navigator` timed out waiting for its own
servers to acknowledge goals, and the outer goal aborted. The orphaned `follow_path` goal
kept driving the robot toward the dock. The gate reported
`Nav2 goals still active: follow_path(1); robot not stationary` and held the retry for
about 116 s (wall clock) until the controller finished and the robot stopped. The retry
then succeeded at once, every later station succeeded first time, and the mission ended
5/5 with exit code 0. **No overlapping goal was sent.**

---

## Testing (as of v1.0.1 and the phases after it)

**Automated tests** (`colcon test`, no Gazebo): 54 tests, 51 passed, 0 failures, 3 skipped (ament
template copyright checks, which skip themselves because the files have no copyright header).

- **37 mission-manager tests** run the real node against a scriptable mock Nav2 (action
  server, lifecycle service, `follow_path` status, odometry, marker stream) and a fake
  `/clock`, each on its own DDS domain. They cover rejection, aborts with and without retry,
  wrong or missing markers, sim rates of 0.5× and 2×, Nav2 unavailable or inactive,
  lifecycle failure and recovery, lost and late goal responses, result timeouts, a stalled
  clock, retries held while `follow_path` is active or the robot is moving, stop reports,
  and exit codes 0, 1, 2 and 130 (SIGINT, and SIGTERM to the installed executable).
- **3 tracker unit tests** for the Nav2 activity and odometry motion trackers.
- **Perception and assets:** detection of IDs 0–3 and rejection of a DICT_4X4 marker
  (OpenCV 4.5.4 and 4.11); the committed marker textures match the generator pixel for
  pixel.
- **Lint:** `ament_flake8` and `ament_pep257` for all three packages.

The mock tests verify mission logic and supervision only, not physical motion.

**End-to-end trials** use `scripts/run_mission_trial.sh`: a fresh simulation, readiness
checks (`bt_navigator` and `amcl` ACTIVE, `/amcl_pose`, the `map → odom → base_footprint`
TF chain), a real-time-factor measurement, the mission, and saved logs, metadata and
metrics under `audit_logs/`.

**CI** ([`.github/workflows/warehouse-humble.yml`](../.github/workflows/warehouse-humble.yml))
runs the test suite and then the same trial script on every pull request to `main`, on an
Xvfb virtual display with software rendering and the Gazebo client and RViz enabled. CI does not
exercise the headless mode; that mode has one local validation mission (Phase 4b).

---
