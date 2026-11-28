# Recording the v1.1 graphical demonstration

Goal: one continuous screen recording showing three things together:

- the Gazebo warehouse with the robot driving;
- RViz with the map, costmaps, global plan and robot pose;
- the mission terminal ending with `FINAL CHECK` and `WAREHOUSE MISSION COMPLETE`.

A complete mission takes about 8–10 minutes of wall time in graphical mode on the development machine.

## 1. Prepare

```bash
cd ~/projects/warehouse_ws
git status --short            # the tracked tree should be clean (record which commit you show)
source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
```

- Close other heavy applications. Software rendering is CPU-bound, and a quieter host gives a higher real-time
  factor.
- Arrange the screen: Gazebo top left, RViz on the right, the ArUco detector window small, and the mission
  terminal bottom left with a large font.
- Windows: the Xbox Game Bar (`Win+Alt+R`) or OBS Studio can record the WSLg windows. Record at 1080p, 30 fps.

## 2. Launch the simulation (Terminal 1)

```bash
export TURTLEBOT3_MODEL=burger_cam
ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py
```

Wait until RViz shows the map and the robot's particle cloud, Nav2 is active (about 60 s), and the detector
window has opened. Set RViz to show `Map`, `Global Costmap`, `Local Costmap`, `Global Plan` and `RobotModel`;
the default TurtleBot3 Nav2 config already contains them.

## 3. Start recording, then start the mission (Terminal 2)

```bash
ros2 run warehouse_bringup warehouse_autonomy --ros-args -p use_sim_time:=true; echo "exit code: $?"
```

Leave it running until the terminal shows:

```
SETTLED: ... s after goal success              (at each station)
Correct marker detected: expected=N, detected=N (image age ... ms, captured ... s after settling ...)
...
FINAL CHECK: robot stationary and Nav2 idle.
WAREHOUSE MISSION COMPLETE: ALL TASKS SUCCEEDED
exit code: 0
```

Stop the recording after the exit code is visible.

## 4. Stop the simulation

Press Ctrl+C in Terminal 1. In Humble, Nav2's container is force-killed by `ros2 launch` after about 11 s; the
error lines this prints are expected (upstream behaviour; see the README, Limitations).

## 5. Publishing the recording

- Trim the start (launch) and the shutdown. Show the four station arrivals and the final summary unedited.
- Add a caption: "Simulation (Gazebo Classic 11, ROS 2 Humble). Not real hardware."
- Upload the video to the GitHub release or a README asset, and replace the v1.0 media in the README's Demo
  section, noting the commit shown.
- If the mission does not complete, keep the recording and logs and report it. Do not re-record selectively to
  hide a failure.
