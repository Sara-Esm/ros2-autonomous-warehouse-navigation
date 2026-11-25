# Trial results

Generated from the saved logs in `audit_logs/` with `scripts/trial_metrics.py --markdown --all`.
Every value is counted from the logs; `-` means the log or field does not exist. All trials
ran on one WSL2 machine (8 cores, 3.8 GiB RAM) from a fresh simulation launch. RTF is the
real-time factor measured just before the mission. Warning counts are raw log-line counts,
not distinct failures.

| Trial | Kind | GUI | Commit | Tasks OK | Failed | Nav failures | Retries | Mission s | RTF | Ctrl rate missed | No valid traj | No progress | follow_path aborts | Plan failures | Spin+backup | Init-pose warnings |
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
- **07, 09** Not recorded as trials (numbers skipped).
- **08** Outer `NavigateToPose` aborted once at the Dispatch Zone (4/5). Afterwards Nav2's spin/backup/wait
  ran on for about 10 s and a `follow_path` goal stayed active for about 64 s. This motivated the readiness gate.
- **10** First run of the tuned Nav2 configuration (`3e8e1b7`): 5/5, no navigation failures.
- **11** **Interrupted, not a result.** Steps 1–3 arrived and were marker-verified (IDs 0, 1, 2) and step 4's goal
  was accepted. Then the runner was terminated externally (the controlling terminal session closed) at 19:45 MST.
  No exit code was recorded. Commits landed while it ran, so it is kept only as evidence.
- **12** Final validation on `0998813` (supervised mission manager plus the Nav2 configuration of Trial 10,
  identical parameter-file hash). **5/5, exit code 0.** RTF was 0.31, half that of the earlier trials (load
  average 21 after about 2 h uptime), which explains the higher controller missed-rate count.
  Step 1's first attempt aborted: `bt_navigator` timed out waiting for `follow_path`, `spin` and `backup`
  goal acknowledgements (Nav2's default 20 ms `default_server_timeout`). The orphaned `follow_path`
  goal kept driving and reached the dock. The mission manager's gate held the retry while it
  reported `follow_path(1)` active and the robot moving (about 116 s from the abort to the retry, wall clock). It then sent the retry, which succeeded immediately
  from (-3.65, 9.20). Every other step succeeded on its first attempt.

## Fault-injection runs (not mission trials)

- `launch_injection01.log` / `mission_injection01.log` / `motion_injection01.log`: controller deactivated
  during a mission with the earlier mission manager. The next goal was sent during Nav2's lifecycle restart,
  and the log ends with that goal never answered (no acceptance, rejection or result). This led to the goal-response watchdog.
- `launch_fault_halt01.log`: launch log only, with no mission output recorded. It contains 81 AMCL
  "Please set the initial pose" warnings from before AMCL `set_initial_pose` was enabled, and no halt evidence.
  Kept for completeness. The halt and stop-verification paths are covered by the automated tests.
