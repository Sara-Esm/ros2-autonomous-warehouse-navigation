#!/usr/bin/env python3
"""Detection reliability under arrival uncertainty: teleport the robot to perturbed station poses.

Usage: perturb.py <out_json> [seconds_per_pose] [pose_set: original44 (default) | grid25]
Needs the bench world with libgazebo_ros_state (/set_entity_state, /get_entity_state) and the robot
entity 'robot'. For each station (world-frame approach pose) and robot-frame perturbation (dx along
heading, dy lateral, dyaw): set the pose (zero twist), settle 2 s wall, read back the true pose, then count
/warehouse/marker_detected frames and IDs decoded on /warehouse/marker_id.
Expected ID: home 0, loading 1, inventory 2, dispatch 3. (Delete/respawn was abandoned: after deletions the
image stream did not follow the respawned robot.)
"""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gazebo_msgs.srv import SetEntityState, GetEntityState
from std_msgs.msg import Bool, Int32
out = sys.argv[1]; dwell = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
POSE_SET = sys.argv[3] if len(sys.argv) > 3 else 'original44'
ST = {'home_dock': ((-3.678, 9.3, 90.0), 0), 'loading_zone': ((1.803, 9.3, 90.0), 1),
      'inventory_scan': ((1.801, -9.0, -90.0), 2), 'dispatch_zone': ((-3.630, -9.3, -90.0), 3)}
E, A = 0.12, 11.5
MARKER = {'home_dock': (-3.678265, 9.97), 'loading_zone': (1.802768, 9.97),
          'inventory_scan': (1.800570, -9.97), 'dispatch_zone': (-3.629830, -9.97)}   # world file, centre 0.40 m high
PERT = [(0, 0, 0), (0, E, 0), (0, -E, 0), (E, 0, 0), (-E, 0, 0), (0, 0, A), (0, 0, -A),
        (0, E, A), (0, -E, -A), (-E, E, A), (-E, -E, -A)]
if POSE_SET == 'grid25':
    # extended envelope: 5x5 position grid (+-0.25 m, step 0.125, along / across the heading) x heading {0, +-11.5}
    G = (-0.25, -0.125, 0.0, 0.125, 0.25)
    PERT = [(dx, dy, da) for da in (0.0, A, -A) for dx in G for dy in G]
rclpy.init(); n = Node('perturb_probe', parameter_overrides=[Parameter('use_sim_time', value=True)])
cnt = {'f': 0, 't': 0, 'ids': {}}
def on_det(m):
    cnt['f'] += 1; cnt['t'] += bool(m.data)
def on_id(m):
    cnt['ids'][m.data] = cnt['ids'].get(m.data, 0) + 1
n.create_subscription(Bool, '/warehouse/marker_detected', on_det, 50)
n.create_subscription(Int32, '/warehouse/marker_id', on_id, 50)
se = n.create_client(SetEntityState, '/set_entity_state'); ge = n.create_client(GetEntityState, '/get_entity_state')
assert se.wait_for_service(timeout_sec=60) and ge.wait_for_service(timeout_sec=60)
def call(cli, req, t=20):
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=t); return f.result()
def spin_for(s):
    end = time.monotonic() + s
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)
res = []
for name, ((x, y, yaw), want) in ST.items():
    for dx, dy, da in PERT:
        th = math.radians(yaw)
        px = x + dx * math.cos(th) - dy * math.sin(th); py = y + dx * math.sin(th) + dy * math.cos(th); pa = math.radians(yaw + da)
        q = SetEntityState.Request(); q.state.name = 'robot'; q.state.reference_frame = 'world'
        q.state.pose.position.x, q.state.pose.position.y, q.state.pose.position.z = px, py, 0.0
        q.state.pose.orientation.z, q.state.pose.orientation.w = math.sin(pa / 2), math.cos(pa / 2)
        ok = call(se, q); spin_for(2.0)
        g = GetEntityState.Request(); g.name = 'robot'; g.reference_frame = 'world'; r = call(ge, g)
        tp = r.state.pose; tyaw = math.degrees(2 * math.atan2(tp.orientation.z, tp.orientation.w))
        cnt.update(f=0, t=0, ids={}); spin_for(dwell)
        # true geometry: marker centre relative to the camera (camera 0.076 m ahead of base, 0.103 m up)
        mx, my = MARKER[name]; ty = math.radians(tyaw)
        cx, cy = tp.position.x + 0.076 * math.cos(ty), tp.position.y + 0.076 * math.sin(ty)
        rng = math.hypot(mx - cx, my - cy)
        bearing = math.degrees(math.atan2(my - cy, mx - cx) - ty); bearing = (bearing + 180) % 360 - 180
        row = dict(station=name, want=want, marker_range_m=round(rng, 3), marker_bearing_deg=round(bearing, 1),
                   true_offset_from_goal_m=round(math.hypot(tp.position.x - x, tp.position.y - y), 3), dx=dx, dy=dy, dyaw=da, set_ok=bool(ok and ok.success),
                   true_pose=[round(tp.position.x, 3), round(tp.position.y, 3), round(tyaw, 1)],
                   pose_err_m=round(math.hypot(tp.position.x - px, tp.position.y - py), 3),
                   frames=cnt['f'], detected=cnt['t'], ids={str(k): v for k, v in cnt['ids'].items()},
                   correct=cnt['ids'].get(want, 0), wrong=sum(v for k, v in cnt['ids'].items() if k != want))
        res.append(row); print(json.dumps(row), flush=True)
json.dump(res, open(out, 'w'), indent=1); n.destroy_node(); rclpy.shutdown()
