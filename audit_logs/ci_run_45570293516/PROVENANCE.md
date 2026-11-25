# CI Gazebo smoke run 45570293516: provenance

These five files are the unmodified contents of the `gazebo-smoke-logs` artifact.

| Field | Value |
|---|---|
| Workflow run | https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45570293516 |
| Event | `pull_request` (PR #3, branch `fix/mission-goal-cleanup`) |
| PR head tested | `847f31cb74208361d776bf8a6150fa45e073afc2` |
| Commit in `meta_trial99.txt` | `b2037ca1b83e79ea82b586c273c518e1727c79f1`, GitHub's merge of `847f31c` into `main` (`d1c6e3f`). Its source tree is identical to `847f31c`. |
| Jobs | `warehouse-tests` success, `gazebo-smoke` success |
| Artifact | `gazebo-smoke-logs`, 23,539 bytes, zip SHA-256 `c7f4eaefa33501c21727c844301fd0450855b4f028b1b36cbe645bc99ae6cdd1` |
| Environment | GitHub-hosted `ubuntu-22.04`, container `ros:humble-ros-base-jammy`, 4 cores, Xvfb with software rendering |

The trial number 99 is fixed in the workflow, so each CI run is stored in its own
`ci_run_<run id>/` folder instead of beside the numbered local trials.

## What the logs show

- 5/5 stations reached and ArUco-verified (IDs 0, 1, 2, 3, 0). `mission_process_exit_code: 0`.
- 0 outer `NavigateToPose` failures and 0 mission retries. 1 internal `follow_path` abort
  was handled inside Nav2, and there was 1 behavior-tree acknowledgement timeout.
- Pre-mission RTF 0.835. No controller missed-rate warnings. AMCL initial pose set, with
  no "Please set the initial pose" warnings.
- 429 s wall clock from step 1 to the summary (446 s mission process).

This run did **not** exercise the abort-and-retry path. That behavior is covered by the
mock-Nav2 tests and was observed in Gazebo only in Trial 12, on the older commit `0998813`.

After the mission summary, the runner stopped the simulation. During that teardown the ArUco
detector raised `rclpy.executors.ExternalShutdownException`, and Gazebo (255), RViz (-11)
and the Nav2 container (-6) exited non-zero. These happen after the mission ended and do
not affect its result.
