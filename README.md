# Autonomous Warehouse Inspection Robot

*ROS 2 Humble · Nav2 · Gazebo Classic 11 · OpenCV ArUco*

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange)
![Nav2](https://img.shields.io/badge/Nav2-AMCL_%7C_NavFn_%7C_DWB-green)
![License](https://img.shields.io/badge/License-MIT-brightgreen)

A simulated TurtleBot3 visits four warehouse stations, confirms each one by reading its
ArUco marker, and returns to its charging dock. A mission manager supervises the run. It
will not send a new goal while Nav2 is still moving the robot.

- **Supervised retries.** A failed `NavigateToPose` does not mean Nav2 is idle. In one
  trial, an orphaned `follow_path` goal kept driving the robot for about 64 s after the outer
  goal aborted. The manager watches every Nav2 action server and the odometry, and retries
  only after the controller has finished and the robot has stopped.
- **Sim-time aware.** Mission timing runs on simulation time and watchdogs on wall time, so
  a stalled `/clock` is detected instead of hiding a hung server.
- **Clear reporting.** Every run ends with counts of succeeded, failed and unattempted
  tasks, and a distinct exit code.
- **Tested.** 50 automated tests, 33 of them running the real mission node against a mock
  Nav2. CI runs a full headless Gazebo mission on every pull request.

**Results:** Local Trials 10 and 12 each verified all five visits. Trial 12 recovered from
a navigation abort after a 116-second wait before retrying. Two linked headless CI runs
([45570293516](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45570293516), [45579301270](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270)) also verified all five
visits. [Full trial log →](docs/trial_results.md)

*Simulation only. See [Limitations](#limitations).*

---

## Demo

<img width="1914" height="1030" alt="Gazebo warehouse, RViz with Nav2 costmaps and plan, and the ArUco detector window" src="https://github.com/user-attachments/assets/dab35b3e-c798-4f6f-ab84-d8b0b9bfa21b" />

https://github.com/user-attachments/assets/dd67f2da-6fb0-4903-81f8-b93f97a8f34c

The screenshot and video were recorded before the supervised mission manager and the Nav2
tuning. The world, robot, stations and markers are the same. The current mission ends like
this (Trial 12, exit code 0):

```
WAREHOUSE MISSION COMPLETE: ALL TASKS SUCCEEDED
Total mission steps: 5
Successful tasks: 5
Failed tasks: 0
Unattempted tasks: 0
```

---

## Quickstart

Tested on Ubuntu 22.04 (WSL2) with ROS 2 Humble and Gazebo Classic 11.

```bash
# ROS 2 Humble desktop, then:
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs \
  ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-msgs ros-humble-dynamixel-sdk \
  ros-humble-xacro ros-humble-cv-bridge python3-opencv python3-pytest

git clone https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation.git ~/projects/warehouse_ws
cd ~/projects/warehouse_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y   # optional: verifies dependencies
colcon build
```

The vendored `src/turtlebot3` builds ROBOTIS's packages, including the real-robot driver
(hence `dynamixel-sdk`).

**Run the mission (two terminals):**

```bash
# Terminal 1: Gazebo + Nav2 + RViz + ArUco detector (Nav2 is up after about 60 s)
cd ~/projects/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py

# Terminal 2: the mission waits for Nav2 by itself, then exits with its status code
cd ~/projects/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run warehouse_bringup warehouse_autonomy --ros-args -p use_sim_time:=true
echo "exit code: $?"
```

Ctrl+C in Terminal 2 cancels the active goal and exits with code 130. To stop it from a
script, see [Stopping the mission](docs/operation.md#stopping-the-mission).

**Run the automated tests** (no Gazebo needed, about 3 minutes):

```bash
colcon test --packages-select warehouse_bringup warehouse_perception warehouse_gazebo
colcon test-result --verbose
```

**Run a recorded trial** (starts and stops the simulation itself; the tracked tree must be clean):

```bash
scripts/run_mission_trial.sh 13          # next unused number; writes audit_logs/*_trial13.*
scripts/trial_metrics.py --all           # compare with earlier trials
```

<details>
<summary>Rebuild the map (optional; the map is included)</summary>

Needs `ros-humble-cartographer-ros`.

```bash
ros2 launch warehouse_bringup warehouse_simulation_camera.launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
ros2 run turtlebot3_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f src/warehouse_bringup/maps/warehouse_map
```

</details>

---

## How it works

| Step | Station | Goal (map frame) | Verified by |
|---|---|---|---|
| 1 | Home Charging Dock | (-3.68, 9.30), facing +y | ArUco ID 0 |
| 2 | Loading Zone | (1.80, 9.30), facing +y | ArUco ID 1 |
| 3 | Inventory Scan Station | (1.80, -9.00), facing -y | ArUco ID 2 |
| 4 | Dispatch Zone | (-3.63, -9.30), facing -y | ArUco ID 3 |
| 5 | Return to Home Charging Dock | (-3.68, 9.30), facing +y | ArUco ID 0 |

The robot spawns at the origin. For each step the mission manager sends a `NavigateToPose`
goal. On arrival it waits up to 15 s (simulation time) for the expected marker ID, and only
detections received after arrival count. A failed marker check fails that task and the
mission continues. An aborted navigation is retried once, after the supervisor sees the
controller finish. A cancellation, or a failed retry, halts the mission. The markers are
0.28 m DICT_5X5_50 tags on the warehouse walls, about 0.67–0.97 m in front of the goal poses.

### System architecture

The diagram shows the data flow: Gazebo publishes LiDAR, odometry, camera images and the
simulation clock; AMCL and Nav2 navigate from the LiDAR and the saved map; the ArUco detector
publishes marker IDs; the mission manager sends goals and passively watches Nav2 and odometry.

```mermaid
flowchart LR
  subgraph SIM["Gazebo Classic 11 (small_warehouse.world)"]
    TB3["TurtleBot3 Burger + camera<br/>diff-drive plugin"]
  end

  TB3 -- "/scan (LiDAR)" --> AMCL
  TB3 -- "/scan" --> COSTMAPS
  TB3 -- "/odom, TF odom→base_footprint" --> AMCL
  TB3 -- "/camera/image_raw" --> DET
  TB3 -- "/clock" --> ALL(("all nodes<br/>use_sim_time"))

  MAP["map_server<br/>warehouse_map.yaml"] -- "/map" --> AMCL
  MAP -- "/map" --> COSTMAPS
  AMCL["AMCL<br/>particle filter"] -- "TF map→odom, /amcl_pose" --> BT

  subgraph NAV2["Nav2 (lifecycle-managed)"]
    BT["bt_navigator<br/>NavigateToPose"] -- compute_path_to_pose --> PLAN["planner_server<br/>NavFn"]
    BT -- follow_path --> CTRL["controller_server<br/>DWB"]
    BT -- "spin / backup / wait" --> BEH["behavior_server"]
    COSTMAPS["global + local costmaps<br/>static, obstacle, inflation"] --- PLAN
    COSTMAPS --- CTRL
  end

  CTRL -- "/cmd_vel" --> TB3
  DET["aruco_station_detector<br/>OpenCV ArUco"] -- "/warehouse/marker_id" --> MM

  MM["warehouse_autonomy<br/>mission manager"] -- "NavigateToPose goal / cancel" --> BT
  BT -. "action status (all Nav2 servers)" .-> MM
  BT -. "lifecycle get_state" .-> MM
  TB3 -. "/odom (stop check)" .-> MM
```

Dotted lines are the mission manager's passive supervision inputs. It never publishes
velocity commands.

### Mission state machine

Each goal passes through a readiness gate, a bounded wait for Nav2's response, navigation
and marker verification. Every failure path ends in a halt with a stop check, and every
ending sets an exit code.

```mermaid
stateDiagram-v2
  [*] --> STARTUP
  STARTUP --> START_DELAY: /clock, NavigateToPose server, bt_navigator ACTIVE
  STARTUP --> NOT_STARTED: startup_timeout (wall), exit 3
  START_DELAY --> GATE
  GATE --> AWAIT_RESPONSE: Nav2 ACTIVE + idle, robot stationary
  GATE --> HALT: gate_timeout (wall)
  AWAIT_RESPONSE --> NAVIGATING: accepted
  AWAIT_RESPONSE --> HALT: rejected / no response (wall)
  NAVIGATING --> VERIFYING: succeeded
  NAVIGATING --> RETRY_PAUSE: aborted, retries left
  NAVIGATING --> HALT: aborted, no retries left
  NAVIGATING --> CANCELING: nav_timeout (sim)
  CANCELING --> HALT: canceled / cancel unconfirmed (wall)
  RETRY_PAUSE --> GATE
  VERIFYING --> TASK_HOLD: expected marker after arrival
  VERIFYING --> PAUSE: verify_timeout (sim), task failed
  TASK_HOLD --> PAUSE
  PAUSE --> GATE: next station
  PAUSE --> DONE: last station, exit 0 or 1
  HALT --> HALT_OBSERVE: cancel active goal if any
  HALT_OBSERVE --> DONE: report stop verified / not verified, exit 2
```

A clock-stall watchdog can halt the mission from any active state.

### What the supervisor checks

- **Before every goal:** action-server discovery, a fresh ACTIVE response from
  `bt_navigator`, the identity of the monitored status publishers, no executing goal on any
  Nav2 action server, and a stationary hold across odometry samples. Before retrying an
  abort, it also requires evidence that `follow_path` was active during this attempt and has
  since cleared.
- **Every wait is bounded.** Mission waits run on simulation time. Communication watchdogs
  (goal response, cancel, `/clock` stall) run on a steady wall clock.
- **Unresolved goals are tracked.** A late acceptance after a response timeout is
  cancelled, and the manager waits a bounded time for its result. The same cleanup runs on
  Ctrl+C or SIGTERM.
- **Exit codes:** 0 all succeeded, 1 finished with failed tasks, 2 halted, 3 never started,
  130 interrupted. After a halt the node prints `ROBOT STOP VERIFIED (odometry)` or
  `ROBOT STOP NOT VERIFIED` with the reason.

These are observations from the same software stack, not an independent safety channel.
Interfaces, parameters and design notes are in [`docs/operation.md`](docs/operation.md).

---

## The story: from Trial 08 to Trial 12

**Trial 08 exposed the problem.** On the final return home, the outer `NavigateToPose`
aborted. Nav2's spin, backup and wait behaviors kept running for about 10 s afterwards, and
a `follow_path` controller goal stayed active for about 64 s. The mission manager of that
time treated the abort as "Nav2 is idle". In a separate fault-injection run with the
controller deactivated, the same manager sent the next goal during Nav2's lifecycle restart
and then waited forever for a response.

**The fix** was the readiness gate and bounded waits described above: never send a goal
while any Nav2 action server is still executing or the robot is still moving.

**Trial 12 tested it under stress.** The host was heavily loaded (load average 21) and the
simulation ran at 0.31× real time. At step 1, `bt_navigator` timed out waiting for its own
servers to acknowledge goals, and the outer goal aborted. The orphaned `follow_path` goal
kept driving the robot toward the dock. The gate reported
`Nav2 goals still active: follow_path(1); robot not stationary` and held the retry for
about 116 s (wall clock) until the controller finished and the robot stopped. The retry
then succeeded at once, every later station succeeded first time, and the mission ended
5/5 with exit code 0. **No overlapping goal was sent.**

---

## Testing

**Automated tests** (`colcon test`, no Gazebo): 50 tests, 0 failures, 3 skipped (ament
template copyright checks, which skip themselves because the files have no copyright header).

- **33 mission-manager tests** run the real node against a scriptable mock Nav2 (action
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

**CI** ([`.github/workflows/warehouse-humble.yml`](.github/workflows/warehouse-humble.yml))
runs the test suite and then the same trial script headless under Xvfb on every pull request
to `main`.

---

## Results

Counted from the saved logs by `scripts/trial_metrics.py`.

| Run | Where | Code | Stations | Nav aborts / retries | Real-time factor |
|---|---|---|---|---|---|
| Trial 10 | Local, WSL2 | `3e8e1b7` (Nav2 tuning) | 5/5 | 0 / 0 | 0.67 |
| Trial 12 | Local, WSL2, heavy load | `0998813` (supervised manager) | 5/5, exit 0 | 1 / 1 (recovered) | 0.31 |
| [CI run 45570293516](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45570293516) | GitHub runner, headless | `847f31c` | 5/5, exit 0 | 0 / 0 | 0.84 |
| [CI run 45579301270](https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation/actions/runs/45579301270) | GitHub runner, headless | `d34a0de` | 5/5, exit 0 | - | - |

All four use the same Nav2 parameter file. Both CI runs tested the current mission code;
`d34a0de` is the tree merged to `main`, and `847f31c` differs from it only in tests and
documentation. The trial script exits with the mission's exit code, and 0 means all five
tasks succeeded. The first CI run's logs are kept in
[`audit_logs/ci_run_45570293516/`](audit_logs/ci_run_45570293516/PROVENANCE.md). The second
run's logs are only in its GitHub artifact, so its abort and RTF columns are not filled in.

Earlier trials (01–08) record the baseline, a rejected headless experiment, and the failure
that led to the supervision work. Every trial, including unsuccessful and interrupted ones,
is listed with notes in [`docs/trial_results.md`](docs/trial_results.md).

---

## Limitations

1. **Simulation only; reliability rate unmeasured.** Everything ran in Gazebo, on one WSL2
   machine and GitHub-hosted runners. The runs above demonstrate completion under the tested
   conditions; they do not measure a reliability rate. Nothing was tested across worlds or
   start poses.
2. **No independent safety layer.** Stop verification reads the same odometry the
   controller uses. A real robot would need a safety-rated stop path.
3. **Map quality.** Large parts of the saved occupancy grid are unknown, and NavFn may plan
   through unknown space. Some obstacles are only partly visible to a 2D LiDAR.
4. **Simulation performance.** The simulation runs at 0.31–0.74× real time on the
   development machine, and the controller logs warnings that it missed its desired control
   rate. Earlier headless runs were faster but less reliable, for reasons not yet understood.
5. **Fixed launch delays.** Nav2 starts at t = 40 s and the detector at t = 55 s by timer.
   The mission and trial runner wait on real readiness, but the launch is not event-driven.
6. **Fixed mission.** Waypoints are hard-coded and there is one robot. "Docking" is a
   navigation goal within 0.12 m / 0.15 rad plus a marker check, not precision docking.
7. **Marker presence, not pose.** A station counts as verified when the expected marker ID
   is detected after arrival. There is no marker pose estimation or camera calibration.

## Next steps

- **Raise Nav2's `default_server_timeout`.** `bt_navigator` gives each server 20 ms (the
  Nav2 default) to acknowledge a goal. Every trial logs such timeouts (3–41 per run), and in
  Trial 12 they caused the first abort. A larger value should remove them, but needs its own
  validation trials.
- **Handle signals as a flag.** The signal handler currently raises an exception wherever
  the main thread is. If Ctrl+C or SIGTERM lands exactly as Nav2 accepts a goal, cleanup
  still cancels and exits 130, but reported `ROBOT STOP NOT VERIFIED` in 2 of about 17
  deliberately timed runs. Setting a flag for the main loop to check would close that window.

---

## Documentation

- [`docs/trial_results.md`](docs/trial_results.md): every trial, its metrics and notes
- [`docs/operation.md`](docs/operation.md): package layout, interfaces, parameters,
  stopping the mission, engineering challenges
- [`docs/trial_baseline_protocol_v1.md`](docs/trial_baseline_protocol_v1.md): the original
  repeatability protocol
- [`audit_logs/`](audit_logs/): raw launch, mission and metadata logs;
  [`scripts/legacy/`](scripts/legacy/): earlier trial runners

## Credits and upstream dependencies

- [ROS 2 Humble](https://docs.ros.org/en/humble/), [Nav2](https://docs.nav2.org/) (AMCL,
  NavFn, DWB, behavior tree navigator, lifecycle manager) and
  [Gazebo Classic 11](https://classic.gazebosim.org/) with `gazebo_ros_pkgs`.
- [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3), vendored in `src/turtlebot3`
  (Apache 2.0). The `burger_cam` Nav2 parameters in it were tuned for this project.
- [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
  (MIT-0): warehouse models and world.
- [OpenCV](https://opencv.org/) ArUco module (`cv2.aruco`) and `cv_bridge` for marker detection.

## Author

**Sara Esmaeili**: Robotics Software Engineer. GitHub: [@Sara-Esm](https://github.com/Sara-Esm)

## License

MIT; see [LICENSE](LICENSE). Bundled third-party content (TurtleBot3, AWS RoboMaker
warehouse models) keeps its own licenses.
