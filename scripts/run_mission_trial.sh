#!/usr/bin/env bash
# Controlled end-to-end mission trial (ROS 2 Humble, Gazebo Classic, GUI on).
#
# Usage: scripts/run_mission_trial.sh NN      (NN = new two-digit trial number, >= 11)
#
# Starts a FRESH simulation (Gazebo + Nav2 + RViz + ArUco detector), checks
# startup readiness (bt_navigator and amcl ACTIVE, /amcl_pose, map->base_footprint
# TF), measures the real-time factor, runs the mission with use_sim_time:=true,
# records its exit code, extracts metrics, and shuts the simulation down.
#
# Writes only new files: audit_logs/{launch,mission,meta,summary}_trialNN.*
# Refuses to run with tracked changes, so the recorded commit is what ran.
set -Eeuo pipefail

TRIAL="${1:-}"
if [[ ! "$TRIAL" =~ ^[0-9]{2}$ ]] || (( 10#$TRIAL < 11 )); then
  echo "Usage: $0 NN   (two-digit trial number >= 11)" >&2
  exit 2
fi

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export TURTLEBOT3_MODEL=burger_cam
mkdir -p audit_logs

LAUNCH="audit_logs/launch_trial${TRIAL}.log"
MISSION="audit_logs/mission_trial${TRIAL}.log"
META="audit_logs/meta_trial${TRIAL}.txt"
SUMMARY="audit_logs/summary_trial${TRIAL}.txt"
for f in "$LAUNCH" "$MISSION" "$META" "$SUMMARY" "audit_logs/amcl_initial_trial${TRIAL}.txt"; do
  if [[ -e "$f" ]]; then echo "Refusing to overwrite $f" >&2; exit 1; fi
done
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "Tracked changes present; commit first so the trial maps to a commit." >&2
  exit 1
fi
if pgrep -x gzserver >/dev/null; then
  echo "gzserver is already running; stop it so this trial starts fresh." >&2
  exit 1
fi

meta() { echo "$*" >> "$META"; }
now_utc() { date -u +%Y-%m-%dT%H:%M:%SZ; }

{
  echo "trial: $TRIAL"
  echo "kind: mission-trial"
  echo "gui: on"
  echo "workspace: $WS"
  echo "commit: $(git rev-parse HEAD)"
  echo "branch: $(git branch --show-current)"
  echo "nav2_params_sha256: $(sha256sum install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml | cut -d' ' -f1)"
  echo "launch_log: $LAUNCH"
  echo "mission_log: $MISSION"
  echo "cpu_cores: $(nproc)"
  echo "launch_start_utc: $(now_utc)"
} > "$META"

LAUNCH_START=$(date +%s)
# A non-interactive bash starts background jobs with SIGINT ignored, and `ros2 launch` would then
# never run its own shutdown. sigdfl_exec.py restores the default dispositions before exec.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
setsid python3 "$SCRIPT_DIR/sigdfl_exec.py" \
  ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py > "$LAUNCH" 2>&1 &
LAUNCH_PGID=$!   # setsid leader = ros2 launch after exec; also the process group ID

cleanup() {
  if pgrep -g "$LAUNCH_PGID" >/dev/null; then
    echo "[trial $TRIAL] Stopping simulation..."
    local t0; t0=$(date +%s)
    # SIGINT to `ros2 launch` only: it forwards SIGINT to every process and escalates to
    # SIGTERM / SIGKILL on its own timeouts (the Nav2 container needs that in Humble).
    kill -INT "$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 300); do pgrep -g "$LAUNCH_PGID" >/dev/null || break; sleep 0.1; done
    if pgrep -g "$LAUNCH_PGID" >/dev/null; then   # last resort, recorded
      meta "shutdown_forced_survivors: $(pgrep -g "$LAUNCH_PGID" | tr '\n' ' ')"
      kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true
      sleep 3
      kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
    fi
    meta "shutdown_seconds: $(( $(date +%s) - t0 ))"
  fi
}
trap cleanup EXIT

fail() { meta "readiness_error: $*"; echo "ERROR: $*" >&2; exit 1; }

wait_active() {  # node, timeout_s
  local node="$1" deadline=$(( $(date +%s) + $2 ))
  while (( $(date +%s) < deadline )); do
    if timeout -k 3 8s ros2 lifecycle get "$node" 2>/dev/null | grep -q '^active'; then return 0; fi
    sleep 3
  done
  return 1
}

echo "[trial $TRIAL] Waiting for bt_navigator ACTIVE (up to 300 s)..."
wait_active /bt_navigator 300 || fail "bt_navigator not ACTIVE"
meta "bt_navigator_active_after_launch_s: $(( $(date +%s) - LAUNCH_START ))"
wait_active /amcl 30 || fail "amcl not ACTIVE"
meta "amcl_active: yes"

echo "[trial $TRIAL] Checking AMCL pose and TF..."
timeout 60s ros2 topic echo --once /amcl_pose --field pose.pose \
  > "audit_logs/amcl_initial_trial${TRIAL}.txt" || fail "no /amcl_pose"
meta "amcl_initial_pose: audit_logs/amcl_initial_trial${TRIAL}.txt"
meta "amcl_set_initial_pose_logged: $(grep -c 'initialPoseReceived' "$LAUNCH")"
meta "init_pose_warnings_before_mission: $(grep -c 'Please set the initial pose' "$LAUNCH")"

python3 - >> "$META" 2>&1 <<'PY' || fail "TF map->base_footprint unavailable"
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
rclpy.init()
node = Node('trial_tf_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
buf = Buffer()
TransformListener(buf, node)
deadline = time.monotonic() + 30.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    for parent, child in (('map', 'odom'), ('odom', 'base_footprint'), ('map', 'base_footprint')):
        if not buf.can_transform(parent, child, rclpy.time.Time()):
            break
    else:
        t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
        print(f'tf_map_to_base_footprint: ok x={t.x:.3f} y={t.y:.3f}')
        sys.exit(0)
print('tf_map_to_base_footprint: unavailable')
sys.exit(1)
PY

echo "[trial $TRIAL] Measuring real-time factor (8 s)..."
python3 - >> "$META" 2>&1 <<'PY'
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
rclpy.init()
node = rclpy.create_node('trial_clock_probe')
samples = []
node.create_subscription(Clock, '/clock',
                         lambda m: samples.append((time.monotonic(), m.clock.sec + 1e-9 * m.clock.nanosec)),
                         qos_profile_sensor_data)
start = time.monotonic()
while time.monotonic() - start < 8.0:
    rclpy.spin_once(node, timeout_sec=0.5)
if len(samples) >= 2 and samples[-1][0] > samples[0][0]:
    rtf = (samples[-1][1] - samples[0][1]) / (samples[-1][0] - samples[0][0])
    print(f'rtf_pre_mission: {rtf:.4f}')
else:
    print('rtf_pre_mission: NA')
PY
{
  echo "--- pre_mission_load ---"
  uptime
  free -h
  echo "mission_start_utc: $(now_utc)"
} >> "$META"

echo "[trial $TRIAL] Running mission; log: $MISSION"
MISSION_START=$(date +%s)
set +e
ros2 run warehouse_bringup warehouse_autonomy --ros-args -p use_sim_time:=true 2>&1 | tee "$MISSION"
RC=${PIPESTATUS[0]}
set -e
{
  echo "mission_process_exit_code: $RC"
  echo "mission_wall_seconds: $(( $(date +%s) - MISSION_START ))"
  echo "mission_end_utc: $(now_utc)"
} >> "$META"

python3 scripts/trial_metrics.py "$TRIAL" > "$SUMMARY"
cat "$SUMMARY"
echo "[trial $TRIAL] Exit code $RC. Files: $LAUNCH $MISSION $META $SUMMARY"
exit "$RC"
