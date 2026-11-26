#!/usr/bin/env bash
# Phase 2b station survey: full system on the NEW map (Nav2 idle, AMCL localizing), survey node drives
# to the intended approach poses and records map-frame localization estimates.
set -Eeo pipefail
WS="$HOME/projects/warehouse_ws"; OUT="$WS/diagnostics_phase2b/survey/session"; cd "$WS"
[[ -e "$OUT/launch.log" ]] && { echo "survey output exists" >&2; exit 1; }
if pgrep -x gzserver >/dev/null; then echo "sim running" >&2; exit 1; fi
mkdir -p "$OUT"
source /opt/ros/humble/setup.bash; source install/setup.bash
export TURTLEBOT3_MODEL=burger_cam
ts() { date +%s.%N; }; meta() { echo "$*" >> "$OUT/meta.txt"; }
{ echo "branch: $(git branch --show-current) head: $(git rev-parse HEAD)"
  echo "installed_map_yaml: $(grep -o "warehouse_map[_v0-9]*.yaml" install/warehouse_bringup/share/warehouse_bringup/launch/warehouse_navigation.launch.py)"
  echo "nav2_params_sha256: $(sha256sum install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml | cut -d' ' -f1)"
  echo "start_wall: $(ts)"; } > "$OUT/meta.txt"
setsid ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py > "$OUT/launch.log" 2>&1 & LP=$!
cleanup() {
  [[ -n "${BAG:-}" ]] && kill -INT "$BAG" 2>/dev/null && sleep 5
  kill -INT -- "-$LP" 2>/dev/null || true; sleep 10; kill -KILL -- "-$LP" 2>/dev/null || true
  pkill -x gzserver 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true
  meta "teardown_wall: $(ts)"
}
trap cleanup EXIT
wait_active() { local n="$1" d=$(( $(date +%s) + $2 )); while (( $(date +%s) < d )); do
  timeout 8s ros2 lifecycle get "$n" 2>/dev/null | grep -q '^active' && return 0; sleep 3; done; return 1; }
wait_active /bt_navigator 300 || { meta "bt_navigator not active"; exit 1; }
wait_active /amcl 30 || { meta "amcl not active"; exit 1; }
meta "nav2_active_wall: $(ts)"
ros2 bag record -o "$OUT/bag" /tf /tf_static /odom /amcl_pose /scan /clock /cmd_vel > "$OUT/bag.log" 2>&1 & BAG=$!
sleep 5
set +e
python3 diagnostics_phase2b/survey/survey_drive.py "$OUT/survey.json" "$OUT/survey.log" > "$OUT/survey.stdout" 2>&1
RC=$?
set -e
meta "survey_exit_code: $RC"; meta "survey_end_wall: $(ts)"
exit "$RC"
