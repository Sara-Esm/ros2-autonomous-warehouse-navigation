# Stalled Phase 3 validation run (2026-11-25 23:34 start)

- The mission never started (no mission.log).
- The runner blocked on `timeout 15 ros2 param get /bt_navigator_navigate_to_pose_rclcpp_node use_sim_time`.
  SIGTERM from `timeout` did not end the `ros2` process, and without `-k` `timeout` never escalated to SIGKILL.
- Likely cause (inferred, not verified): the BT client node's parameter service is not served, because the node
  is spun only for its BT callback groups.
- The WSL2 VM was suspended for most of the following day: uptime 9 h 16 min against 18.7 h of wall time.
- The simulation stopped publishing /clock; the bag stopped growing at 23:54.
- The run was stopped on 2026-11-26 18:22 and replaced by a fresh run with a corrected snapshot.
