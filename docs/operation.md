# Operating reference

Details moved out of the README: package layout, the interfaces and parameters the mission
manager uses, and how to stop a mission cleanly.

## Packages and key files

| Package / path | Purpose |
|---|---|
| `src/warehouse_bringup/` | Mission manager, launch files, saved map |
| `  warehouse_bringup/warehouse_autonomy_node.py` | Mission state machine (`ros2 run warehouse_bringup warehouse_autonomy`) |
| `  warehouse_bringup/nav2_supervision.py` | Passive helpers: Nav2 action-activity tracker, odometry stop tracker |
| `  launch/warehouse_robot_system_camera.launch.py` | Full system: Gazebo, robot, Nav2 + RViz (t=40 s), detector (t=55 s) |
| `  launch/warehouse_simulation_camera.launch.py` | Gazebo, robot_state_publisher, spawn `turtlebot3_burger_cam` at (0, 0) |
| `  launch/warehouse_navigation.launch.py` | Nav2 with the warehouse map (`turtlebot3_navigation2`) |
| `  maps/warehouse_map.{pgm,yaml}` | 318 × 439 cells at 0.05 m, origin (-7.56, -10.8) |
| `  test/` | Mission-manager tests against a mock Nav2, tracker unit tests, lint tests |
| `src/warehouse_perception/` | `aruco_station_detector`: DICT_5X5_50, OpenCV 4.5.4 and ≥ 4.7 APIs |
| `src/warehouse_gazebo/` | Warehouse world, AWS RoboMaker models, ArUco marker models and generator |
| `src/turtlebot3/` | Vendored ROBOTIS TurtleBot3 packages; overrides apt. Holds the tuned Nav2 parameters |
| `  turtlebot3_navigation2/param/humble/burger_cam.yaml` | AMCL, costmap, NavFn, DWB, behavior-server parameters used at runtime |
| `scripts/run_mission_trial.sh` | Controlled end-to-end trial: fresh launch, readiness checks, mission, metrics |
| `scripts/trial_metrics.py` | Counts comparable metrics from saved trial logs |
| `scripts/legacy/` | Earlier trial runners (Trials 02–10), kept so the historical logs stay reproducible |
| `docs/trial_results.md` | Metrics table and notes for every recorded trial |
| `audit_logs/` | Raw launch, mission and metadata logs from every trial |

`src/aws-robomaker-small-warehouse-world/` is an upstream copy kept for reference; it is
not built (`COLCON_IGNORE`). `warehouse_gazebo/launch/turtlebot3_*warehouse.launch.py` are
early bring-up files that the system launch does not use.

## Interfaces used by the mission manager

| Interface | Type | Role |
|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Station goals, cancellation |
| `/<server>/_action/status` | `action_msgs/msg/GoalStatusArray` | Is any Nav2 server (navigate_to_pose, follow_path, compute_path_to_pose, spin, backup, wait, drive_on_heading, assisted_teleop) still executing? |
| `/bt_navigator/get_state` | `lifecycle_msgs/srv/GetState` | Nav2 must be ACTIVE before each goal |
| `/odom` | `nav_msgs/msg/Odometry` | Stationary before each goal; stop report after a halt |
| `/warehouse/marker_id` | `std_msgs/msg/Int32` | Detected marker ID |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time; stall detection |

## Key parameters

| Mission manager (ROS parameter) | Default | Clock |
|---|---|---|
| `verify_timeout` / `task_hold` / `inter_goal_pause` / `start_delay` | 15 / 3 / 2 / 10 s | sim |
| `nav_retries` / `retry_pause` | 1 / 3 s | sim |
| `nav_timeout` (per navigation attempt) | 300 s | sim |
| `stop_hold` (stationary before a goal) | 1 s | sim |
| `startup_timeout` / `gate_timeout` | 180 / 120 s | wall |
| `goal_response_timeout` / `cancel_timeout` | 20 / 10 s | wall |
| `clock_stall_timeout` / `halt_observe_timeout` | 15 / 15 s | wall |
| `retry_evidence_grace` | 5 s | wall |

| Nav2 (`burger_cam.yaml`) | Value | Why |
|---|---|---|
| AMCL `set_initial_pose` | true, (0, 0, yaw 0) | Matches the Gazebo spawn; removes a startup race |
| Goal tolerance xy / yaw | 0.12 m / 0.15 rad | 0.05 m caused goal oscillation under sim load; marker verification does not need a precise stop (the 100° camera detected the marker at every pose within ±0.12 m / ±11.5°, Phase 4) |
| `robot_radius` | 0.14 m | Burger collision box + wheels ≈ 0.135 m circumscribed (was 0.10) |
| Inflation radius (global / local) | 0.5 / 0.35 m | 0.2 m let NavFn route into gaps about 0.1 m wider than the robot |
| Costmap layers | static, obstacle, inflation | The duplicate voxel layer re-processed the same 2D scan |

## Stopping the mission

The mission process handles SIGINT and SIGTERM the same way: bounded goal cleanup (cancel
any active or pending goal, observe the stop), then exit code 130. How the signal reaches
it matters:

- **Ctrl+C in the terminal running `ros2 run`** works. The terminal signals the whole
  foreground process group, so the mission child receives SIGINT directly, and `ros2 run`
  passes through exit code 130.
- **A signal sent to the `ros2 run` PID** (for example `kill <pid>`) is *not* forwarded.
  Humble's `ros2 run` starts the mission as a child process and does not relay signals,
  so the mission keeps running and no cleanup happens.
- **To stop it from a script or supervisor,** start the installed executable directly, so
  the PID you hold is the mission process, and signal that PID:

  ```bash
  install/warehouse_bringup/lib/warehouse_bringup/warehouse_autonomy \
    --ros-args -p use_sim_time:=true &
  MISSION_PID=$!
  kill -TERM "$MISSION_PID"     # bounded cleanup, then exit code 130
  wait "$MISSION_PID"; echo "exit code: $?"
  ```

Tests cover SIGINT to the mission process (during an executing goal and during pending
acceptance) and SIGTERM to the installed executable during an executing goal.

**Signals are handled as a flag (Phase 6).** Until Phase 6 the handler raised an exception
wherever the main thread happened to be. That once landed inside rclpy's message take and
ended the process with exit code 1 and no cleanup; earlier, a signal at the moment of goal
acceptance sometimes left the stop unverified. The handler now only records the signal; the
main loop notices it within 0.1 s and runs the same bounded cleanup. Tests interrupt at five
different points of a running mission, and in the idle, settling and verifying phases.

**Stopping the whole simulation from a script.** Start `ros2 launch` with default signal
handling (`scripts/sigdfl_exec.py`; a non-interactive bash starts background jobs with SIGINT
ignored) and send SIGINT to the `ros2 launch` process only. Launch then stops every process
and escalates to SIGTERM and SIGKILL on its own timeouts. In Humble, Nav2's composed
container does not exit on SIGINT or SIGTERM and is force-killed. Verified timeline (one wall clock):
launch forwards SIGINT after about 1.0 s; every project-owned process exits cleanly by +1.6 s; launch
sends the container SIGTERM at +6.0 s and SIGKILL at +11.0 s; the process group is empty at +11.2 s.
Across four runs the system was down 10.5–11.2 s after the signal. The container behaviour is upstream. `scripts/run_mission_trial.sh` does this.

## Engineering challenges and solutions

| Challenge | Evidence | Resolution |
|---|---|---|
| Localization start-up race | A one-shot `/initialpose` published at t = 50 s could arrive before AMCL was active; logs show "Please set the initial pose" warnings and a readiness failure | AMCL `set_initial_pose` from parameters on activation; the trial runner checks AMCL ACTIVE, `/amcl_pose` and TF |
| Simulation vs wall time | WSL2 runs at 0.31–0.88× real time, so wall-clock waits were too short in sim terms | Mission waits on the node clock (`use_sim_time`), watchdogs on the steady clock; tested at 0.5× and 2× |
| Blocking mission sequencing | Original node slept and nested `spin_once` inside callbacks | Single timer-driven state machine; callbacks never block |
| Action-server behavior | Outer goal failure while follow_path/spin continued; goal response never arrived during a lifecycle restart | Readiness gate over all Nav2 action statuses and lifecycle state; numbered goals; cancel late or abandoned goals |
| Navigation aborts | Robot routed into a gap ~0.1 m from obstacles near box stacks; "No valid trajectories" then abort | Correct footprint radius, wider inflation, looser goal tolerance; verified offline that all stations remain reachable |
| CPU limits | 75–724 "controller missed its desired rate" warnings per trial, depending on host load | Removed the duplicate voxel layer and DWB debug publishing; remaining warnings documented |
| Asset consistency | Marker generator produced DICT_4X4 while the world and detector use DICT_5X5 | Generator reproduces the committed textures exactly, plus a regression test |
