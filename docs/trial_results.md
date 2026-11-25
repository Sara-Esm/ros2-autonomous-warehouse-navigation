# Trial results

Generated from the saved logs in `audit_logs/` with `scripts/trial_metrics.py --markdown --all`.
Every value is counted from the logs; `-` means the log or field does not exist. All trials
ran on one WSL2 machine (8 cores, 3.8 GiB RAM) from a fresh simulation launch. RTF is the
real-time factor measured just before the mission. Warning counts are raw log-line counts,
not distinct failures. "Ctrl rate missed" counts controller warnings that it missed its desired
control rate.

| Trial | Kind | GUI | Commit | Tasks OK | Failed | Nav failures | Retries | Step 1 to summary (wall s) | RTF | Ctrl rate missed | No valid traj | No progress | follow_path aborts | Plan failures | Spin+backup | Init-pose warnings |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 01 | pilot | on | - | 5 | 0 | 0 | 0 | 497 | - | - | - | - | - | - | - | - |
| 02 | repeatability-baseline | on | 3674bf1 | 5 | 0 | 0 | 0 | 597 | 0.6986 | 204 | 24 | 1 | 6 | 0 | 6 | 3 |
| 03 | repeatability-baseline | on | 3674bf1 | 5 | 0 | 0 | 0 | 558 | 0.7351 | 80 | 2 | 0 | 1 | 0 | 2 | 2 |
| 04 | repeatability-baseline | on | 3674bf1 | 5 | 0 | 0 | 0 | 540 | 0.6538 | 98 | 0 | 1 | 2 | 0 | 1 | 3 |
| 05 | gui-off-experiment | off | 00b70b6 | 0 | 5 | 5 | 0 | 427 | 0.8807 | 16 | 22 | 14 | 20 | 53 | 26 | 3 |
| 06 | gui-off-experiment | off | 00b70b6 | 3 | 2 | 2 | 0 | 525 | 0.8678 | 7 | 4 | 3 | 17 | 24 | 8 | 3 |
| 08 | reliability-hardening-validation | on | 3de4301 | 4 | 1 | 1 | 0 | 503 | 0.6348 | 122 | 24 | 1 | 4 | 0 | 4 | 2 |
| 10 | nav-tuning-validation | on | 3e8e1b7 | 5 | 0 | 0 | 0 | 524 | 0.6749 | 75 | 0 | 0 | 2 | 0 | 1 | 0 |
| 11 | mission-trial | on | dcce7f7 | - | - | 0 | 0 | - | 0.6786 | 119 | 0 | 0 | 2 | 0 | 2 | 0 |
| 12 | mission-trial | on | 0998813 | 5 | 0 | 1 | 1 | 677 | 0.3125 | 724 | 0 | 3 | 5 | 0 | 15 | 0 |

## Notes per trial

- **01** Pilot run; launched about 80 min before the mission, so it is not comparable with the cohort.
- **02–04** Controlled repeatability baseline at `3674bf1` (protocol: `docs/trial_baseline_protocol_v1.md`).
- **05–06** Headless experiment (gzclient off). RTF rose to about 0.87 but navigation degraded badly
  (53 and 24 planner failures). Not adopted. Trial 05's old mission manager printed
  "WAREHOUSE MISSION COMPLETE" with 0/5 tasks succeeded, which led to the honest-reporting fix in `3de4301`.
- **07, 09** No logs, metadata or commits exist under these numbers in `audit_logs/` or the Git
  history, and why the numbers were skipped was not written down at the time. Unsuccessful attempts that
  did produce logs are kept, for example the [Trial 03 readiness failure](../audit_logs/meta_trial03_failed_readiness_20261123_153649.txt)
  and the interrupted Trial 11. The reason for skipping 07 and 09 is unknown.
- **08** Outer `NavigateToPose` aborted once on the final return home after Dispatch succeeded (4/5). Afterwards Nav2's spin/backup/wait
  ran on for about 10 s and a `follow_path` goal stayed active for about 64 s. This motivated the readiness gate.
- **10** First run of the tuned Nav2 configuration (`3e8e1b7`): 5/5, no navigation failures.
- **11** **Interrupted, not a result.** Steps 1–3 arrived and were marker-verified (IDs 0, 1, 2) and step 4's goal
  was accepted. Then the runner was terminated externally (the controlling terminal session closed) at 19:45 MST.
  No exit code was recorded. Commits landed while it ran, so it is kept only as evidence.
- **12** Last local Gazebo validation, on `0998813`, which is older than the current code on `main` (supervised mission manager plus the Nav2 configuration of Trial 10,
  identical parameter-file hash). **5/5, exit code 0.** RTF was 0.31, half that of the earlier trials (load
  average 21 after about 2 h uptime), alongside a higher controller missed-rate count.
  Step 1's first attempt aborted: `bt_navigator` timed out waiting for `follow_path`, `spin` and `backup`
  goal acknowledgements (Nav2's default 20 ms `default_server_timeout`). The orphaned `follow_path`
  goal kept driving and reached the dock. The mission manager's gate held the retry while it
  reported `follow_path(1)` active and the robot moving (about 116 s from the abort to the retry, wall clock). It then sent the retry, which succeeded immediately
  from (-3.65, 9.20). Every other step succeeded on its first attempt.

## How to read the trials

- Trials 02–04 are the original baseline: 15/15 station visits, plus 5/5 in pilot Trial 01, for 20/20.
  This counts station visits in missions that started. It excludes a separate startup failure: an
  earlier attempt at Trial 03 stopped before the mission because `bt_navigator` never became ACTIVE
  ([`meta_trial03_failed_readiness_20261123_153649.txt`](../audit_logs/meta_trial03_failed_readiness_20261123_153649.txt)).
- The old mission manager reported "WAREHOUSE MISSION COMPLETE" even when tasks had failed (Trial 05's
  log says so with 0/5 succeeded); `3de4301` fixed that.
- Trials 05–06 tested running without the GUI to raise the real-time factor. RTF went up, but
  navigation became much less reliable, so the change was not kept.
- Trials 10–12 use the current Nav2 configuration (identical parameter-file hash).
- ArUco verification succeeded at every station the robot reached in every trial from 08 on. No trial
  from 10 on logged an AMCL "Please set the initial pose" warning. The goal-response watchdog (present
  from Trial 11 on) never fired in a trial.

**Three levels of failure.** Mission results count only the last of these:

| Level | What it means | Trial 10 | Trial 12 |
|---|---|---|---|
| Nav2-internal `follow_path` abort | The controller gives up on one path; the behavior tree replans or runs a recovery itself | 2 | 5 |
| Outer `NavigateToPose` abort | The behavior tree gives up; the mission manager sees a failure and may retry | 0 | 1 (retried) |
| Mission task failure | A station is not reached or its marker is not confirmed | 0 | 0 |

## CI Gazebo smoke run (current code)

Stored unchanged in [`audit_logs/ci_run_45570293516/`](../audit_logs/ci_run_45570293516/PROVENANCE.md).
It is not part of the numbered local trials above: different machine, headless, and the workflow
always uses trial number 99.

| Run | Code | Environment | Tasks OK | Nav failures / retries | Step 1 to summary (wall s) | RTF | Ctrl rate missed | follow_path aborts |
|---|---|---|---|---|---|---|---|---|
| [45570293516](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45570293516) `847f31c` (tested as merge commit `b2037ca`, identical tree) | GitHub `ubuntu-22.04`, 4 cores, Xvfb, software GL | 5/5, exit 0 | 0 / 0 | 429 | 0.84 | 0 | 1 |

Same Nav2 parameter-file hash as Trials 10–12 (`869d998…`). All five markers were verified on
the first attempt. The abort-and-retry path was **not** exercised. Non-zero exits from the
detector, Gazebo, RViz and the Nav2 container appear only during teardown, after the mission summary.

A second run, [45579301270](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270)
on `d34a0de` (the tree merged to `main`), also passed the Gazebo smoke step. The trial script exits with
the mission's exit code, so a pass means exit 0 with all five tasks succeeded. Its logs exist only as the
run's GitHub artifact (retained until 2027-02-23) and are not copied here, so no further metrics are claimed.

## Fault-injection runs (not mission trials)

- `launch_injection01.log` / `mission_injection01.log` / `motion_injection01.log`: controller deactivated
  during a mission with the earlier mission manager. The next goal was sent during Nav2's lifecycle restart,
  and the log ends with that goal never answered (no acceptance, rejection or result). This led to the goal-response watchdog.
- `launch_fault_halt01.log`: launch log only, with no mission output recorded. It contains 81 AMCL
  "Please set the initial pose" warnings from before AMCL `set_initial_pose` was enabled, and no halt evidence.
  Kept for completeness. The halt and stop-verification paths are covered by the automated tests.
