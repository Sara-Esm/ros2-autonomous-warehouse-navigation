# CI Gazebo smoke run 45579301270

These five files are the original contents of the `gazebo-smoke-logs` artifact from
[GitHub Actions run 45579301270](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270).

| Field | Recorded value |
|---|---|
| Event | Pull request [#3](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/pull/3), branch `fix/mission-goal-cleanup` |
| PR head | `d34a0dedcf03cdb2a7f1c07bbd8eefd93ce02337` |
| CI checkout | `5365ca5ac53b5a3d99521a5691994816fd9bca2c`, GitHub's test merge into `d1c6e3f`. Its Git tree matches the PR head. |
| Jobs | [warehouse-tests](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270/job/139084988652) and [gazebo-smoke](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270/job/139086856493) both passed |
| Environment | GitHub-hosted Ubuntu 22.04, four cores, `ros:humble-ros-base-jammy`, Xvfb and software rendering |
| Artifact | ID `15006193518`, 24,070 bytes |
| ZIP SHA-256 | `f4c3a19812b6b123106bf647bde13b21ce5402b8e9319b405ced634f19aac238`, verified against GitHub's artifact digest |

## What the logs show

- The camera verified marker IDs **0, 1, 2, 3, 0** in order. The mission recorded five
  successful visits, zero failed and zero unattempted. [Mission log](mission_trial99.log)
- The mission recorded **zero navigation failures and zero retries**. The launch log
  contains zero `follow_path` abort messages and zero controller missed-rate warnings.
  This run exercised the normal mission path. [Metrics summary](summary_trial99.txt) · [Launch log](launch_trial99.log)
- The pre-mission real-time factor was **0.8370**. Step 1 to the final summary took
  **432 s** of wall time. The mission process ran for **447 s** and exited with code **0**.
  [Run metadata](meta_trial99.txt) · [Metrics summary](summary_trial99.txt)
- AMCL was active before the mission and the runner recorded an initial pose and a
  successful map-to-base transform check. [Initial pose](amcl_initial_trial99.txt) · [Run metadata](meta_trial99.txt)

During teardown, the launch log records non-zero exits from the detector, Gazebo, RViz and
the Nav2 container. The mission process had already returned exit code 0. [Launch log](launch_trial99.log) · [Run metadata](meta_trial99.txt)
