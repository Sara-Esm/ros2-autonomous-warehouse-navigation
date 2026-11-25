# Warehouse Robot — Repeatability Baseline Protocol v1

Trial 01 is a **pilot**, not part of the tightly comparable trial cohort: launch logs indicate the system was started around epoch 1795421317 and the mission started around 1795426208 (~4,891 s later). This does not establish that CPU load was elevated continuously for 81 minutes; `uptime` load averages cover the preceding 1, 5 and 15 minutes, and Linux `free` should distinguish free memory from **available** memory.

## Controlled cohort: Trials 02, 03, 04
- Use Git commit `3674bf1d822b0daba77b575753a16b2fcdfde38b`, without tracked changes, identical GUI/RViz/world/camera configuration, and a **full simulation restart** between runs.
- Terminal 1: `ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py 2>&1 | tee audit_logs/launch_trial02.log` (replace 02 with 03 or 04 on later trials). From workspace root after sourcing ROS 2 and install setup.
- Terminal 2: `./run_trial.sh 02` (same trial ID). The script validates commit and launch log; waits for `/bt_navigator` lifecycle `active` and a real `/amcl_pose` message; stores starting pose and resource snapshot; samples `/clock` with BEST_EFFORT for 8s; samples camera rate for 15s; launches the unchanged mission and captures its output.
- After the mission prints `Failed tasks:` and `WAREHOUSE MISSION COMPLETE`, interrupt mission with Ctrl+C. Then stop the launch and restart it fresh for the next trial.
- Finally run `./summarize_trials.sh` for side-by-side results. Check initial AMCL poses and missing logs. Compare **02–04 with each other**; Trial 01 only for qualitative/pilot context.

## Caveats
- `/amcl_pose` plus active Nav2 is a readiness gate, not proof of identical localization quality. Inspect initial pose, simulation freshness, costmaps and any start-up problems if a trial differs.
- A short pre-mission diagnostic and logger can add overhead, but the same sequence is used on all three trials. The script samples **before** running the mission rather than during it.
- The script only samples start-of-trial RTF and rate, not per-step or during-mission performance. It counts log messages; repeated warnings need not imply distinct failures.
- A successful 3/3 controlled cohort (plus the pilot) is evidence of repeatability under tested conditions, not proof of high reliability.
- Do not overwrite logs or modify code between trials. `run_trial.sh` refuses to overwrite its mission/meta files.
- The script will not change launch behavior or the ROS source. It assumes the baseline simulation is already running in Terminal 1.
