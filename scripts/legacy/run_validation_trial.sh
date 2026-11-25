#!/usr/bin/env bash
# Reliability-hardening validation mission trial runner (ROS 2 Humble, warehouse_ws).
# Usage: ./run_trial.sh 02    (after starting launch_trial02.log in another terminal)
set -Eeuo pipefail

TRIAL="${1:-}"
if [[ ! "$TRIAL" =~ ^0[89]$ ]]; then
  echo "Usage: $0 08|09" >&2
  exit 2
fi

WS="$HOME/projects/warehouse_ws"
VALIDATION_COMMIT="3de4301a4e52e296a00850f8adae2a7c1195ce14"
cd "$WS" || exit 1
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
mkdir -p audit_logs

META="audit_logs/meta_trial${TRIAL}.txt"
MISSION="audit_logs/mission_validation_trial${TRIAL}.log"
LAUNCH="audit_logs/launch_trial${TRIAL}.log"

for item in "$META" "$MISSION"; do
  if [[ -e "$item" ]]; then
    echo "Refusing to overwrite $item. Inspect existing results first." >&2
    exit 1
  fi
done
if [[ ! -s "$LAUNCH" ]]; then
  echo "Missing/empty $LAUNCH. Start and log a FRESH Gazebo launch in Terminal 1 first." >&2
  exit 1
fi
if [[ "$(git rev-parse HEAD)" != "$VALIDATION_COMMIT" ]]; then
  echo "HEAD is not baseline commit $VALIDATION_COMMIT; refusing validation trial." >&2
  exit 1
fi
if ! git diff --quiet || ! git diff --cached --quiet; then
  echo "Tracked changes detected. Do not change source between validation trials." >&2
  exit 1
fi

START_EPOCH=$(date +%s)
{
  echo "trial: $TRIAL"
  echo "kind: reliability-hardening-validation"
  echo "workspace: $WS"
  echo "commit: $(git rev-parse HEAD)"
  echo "launch_log: $LAUNCH"
  echo "mission_log: $MISSION"
  echo "script_start_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "cpu_cores: $(nproc)"
} > "$META"

# Use runtime readiness instead of an arbitrary launch delay.
echo "[trial $TRIAL] Waiting for bt_navigator ACTIVE (up to 240 s)..."
READY=0
for (( i=0; i<48; i++ )); do
  if timeout 8s ros2 lifecycle get /bt_navigator 2>/dev/null | grep -Eq '^active'; then
    READY=1
    break
  fi
  sleep 5
done
if [[ "$READY" -ne 1 ]]; then
  echo "ERROR: bt_navigator did not become ACTIVE." | tee -a "$META" >&2
  exit 1
fi

echo "[trial $TRIAL] Waiting for an actual AMCL pose (up to 180 s)..."
if ! timeout 180s ros2 topic echo --once /amcl_pose --field pose.pose.position \
   > "audit_logs/amcl_initial_trial${TRIAL}.txt"; then
  echo "ERROR: no AMCL pose received; not starting mission." | tee -a "$META" >&2
  exit 1
fi

{
  echo "ready_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "ready_wait_wall_seconds: $(($(date +%s) - START_EPOCH))"
  echo "initial_amcl_pose: audit_logs/amcl_initial_trial${TRIAL}.txt"
  echo "--- initial_pose ---"
  cat "audit_logs/amcl_initial_trial${TRIAL}.txt"
  echo "--- environment_snapshot ---"
  uptime
  free -h
  ps -eo pid,comm,%cpu,%mem --sort=-%cpu | head -12
} >> "$META"

# Same pre-mission measurement procedure as Trials 02-04.
echo "[trial $TRIAL] Measuring /clock (8 s), then camera rate (15 s)..."
python3 - <<'PY' >> "$META" 2>&1
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
rclpy.init()
node = Node('baseline_clock_probe')
values = []
def cb(msg):
    values.append((time.monotonic(), msg.clock.sec + 1e-9 * msg.clock.nanosec))
sub = node.create_subscription(Clock, '/clock', cb, qos_profile_sensor_data)
start = time.monotonic()
while time.monotonic() - start < 8.0:
    rclpy.spin_once(node, timeout_sec=0.5)
if len(values) >= 2:
    wall = values[-1][0] - values[0][0]
    sim = values[-1][1] - values[0][1]
    print(f'clock_samples: {len(values)}')
    print(f'rtf_pre_mission: {sim / wall:.4f}' if wall > 0 else 'rtf_pre_mission: NA')
else:
    print('rtf_pre_mission: NA (insufficient /clock data)')
node.destroy_node()
rclpy.shutdown()
PY

# timeout exits with 124 when its duration expires, which is expected here.
{
  echo "--- camera_rate_15s ---"
  timeout -s INT 15s ros2 topic hz /camera/image_raw 2>&1 || true
  echo "--- pre_mission_load ---"
  uptime
  echo "mission_start_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
} >> "$META"

echo "[trial $TRIAL] Launching validation mission; log: $MISSION"
echo "[trial $TRIAL] After the FINAL mission summary, press Ctrl+C once."
set +e
ros2 run warehouse_bringup warehouse_autonomy --ros-args -p use_sim_time:=true 2>&1 | tee "$MISSION"
RC=${PIPESTATUS[0]}
set -e
{
  echo "mission_process_exit_code: $RC"
  echo "script_end_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
} >> "$META"
echo "[trial $TRIAL] Saved $MISSION and $META. Stop Terminal 1 launch and restart fresh for the next trial."
