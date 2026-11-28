# v1.1 release notes (draft, not yet tagged)

v1.1 turns v1.0.1's mission, which completed but stopped 0.18–0.50 m from its stations, into a measured,
supervised and documented system. All changes are in simulation (ROS 2 Humble, Nav2, Gazebo Classic 11). The
release has not been validated on hardware and is not safety-certified.

## Highlights

- **Accurate stops:** station position error at most 0.099–0.127 m (v1.0.1: 0.18–0.50 m), and settled heading
  error about 4°, over six validation missions.
- **No acknowledgement-timeout failures:** Nav2's goal-acknowledgement budget raised from 20 to 200 ms. In
  v1.0.1's diagnostic run 19 failures and Spin/Wait/BackUp recoveries came from it; v1.1 has 0 in six missions.
- **Verified completion:** a station counts only when the robot has settled on fresh odometry and the expected
  marker appears in a camera image captured after settling. Mission success needs a final
  stationary-and-idle check.
- **Faster perception:** a 100° pinhole camera replaces the 182° wide-angle sensor. Frames arrive 6× faster;
  verification follows arrival within 0.1–0.4 s.
- **Headless mode:** `gui:=false use_rviz:=false debug_window:=false`.
- **Orderly shutdown:** project processes exit cleanly. Scripts stop `ros2 launch` correctly.

## Changes by phase

| Phase | Change | Report |
|---|---|---|
| 1 | Station goals converted from the Gazebo world frame to the map frame | `docs/validation/phase1_station_frames/REPORT.md` |
| 2a | Never-observed map cells loaded as unknown (`free_thresh` 0.01); integration test with `nav2_map_server` | `phase2a_map_unknown_space` |
| 2b | New Cartographer map `warehouse_map_v2` from a scripted coverage drive; goals surveyed in the map frame; `allow_unknown: false` | `phase2b_slam_remap` |
| 3 | `default_server_timeout` 200 ms; behaviour-tree global clears keep the static layer; node clock sections | `phase3_timing_sync` |
| 4 | Project camera model `turtlebot3_burger_cam_pinhole` (100° pinhole, 320×240); optional `gui` launch argument | `phase4_perception_performance` |
| 4b | Independent `gui`, `use_rviz` and `debug_window` launch options; structural test against the vendored Nav2 launch | `phase4b_headless_operation` |
| 5 | DWB `PathAlign`/`GoalAlign` `forward_point_distance` 0.325 m; `yaw_goal_tolerance` 0.07 rad | `phase5_motion_control` |
| 6 | Settling before verification, stamped marker evidence (`/warehouse/marker_detections`), final check; signal flag; detector shutdown fix; `scripts/sigdfl_exec.py` | `phase6_mission_verification` |
| 7 | Integrated validation (3 headless + 3 graphical missions); documentation and release preparation | `phase7_integrated_validation` |

## Interface changes

- **New topic:** `/warehouse/marker_detections` (`vision_msgs/Detection2DArray`), one message per processed image,
  carrying the source image's header.
- **Legacy topics kept:** `/warehouse/marker_id` and `/warehouse/marker_detected` still publish, but the mission no
  longer uses them.
- **New dependency:** `ros-humble-vision-msgs` (also installed in CI).
- **New mission-node parameters:** `settle_linear_tolerance` (0.005), `settle_angular_tolerance` (0.01),
  `settle_hold` (0.5), `settle_timeout` (10.0), `marker_max_age` (1.0) and `final_check_timeout` (10.0).
- **Exit code 1** now also means "all tasks succeeded but completion not confirmed" (final check failed).
- **Launch arguments:** `gui`, `use_rviz` and `debug_window` on `warehouse_robot_system_camera.launch.py`, all
  defaulting to true.
- **Robot model:** the simulation now spawns `warehouse_gazebo/models/turtlebot3_burger_cam_pinhole`.

## Validation

- **Missions:** six full missions on the final code, three headless and three graphical: 6/6 completed with exit
  0, 30/30 stations verified, final check passed in all. Details:
  `docs/validation/phase7_integrated_validation/REPORT.md`.
- **Tests:** 108 automated tests, 105 passed, 0 failed, 3 skipped (copyright, by design).

## Known limitations

See the README (Limitations) and the Phase 5–7 reports:

- Clearance at the south pinch varies run to run (footprint 0.21–0.32 m across Phases 5–7). A
  controller-related reduction is measurable, but its total size is unresolved, and the original clearance
  criterion is not shown to be met.
- An occasional DWB departure delay at the Loading Zone (2–9 s; cause not established).
- No execution-time unknown-space awareness in the controller.
- Software rendering limits RTF and camera rate on the development host.
- Nav2's composed container is force-killed at shutdown (upstream).

## Before tagging

1. Merge the stacked pull requests in order (`docs/release/INTEGRATION_PLAN.md`), with `warehouse-tests` and
   `gazebo-smoke` green on each.
2. Verify CI on the final `main`, then tag `v1.1` on that merge commit and publish this file as the release text.
3. Copy the Windows backups (`C:\Users\user\warehouse_ws_backups\`) to **independent external storage**. They
   currently exist only on the development laptop.
