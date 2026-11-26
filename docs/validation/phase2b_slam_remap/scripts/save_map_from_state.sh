#!/usr/bin/env bash
# Re-publish the saved Cartographer state (frozen, no sensor input) and save the occupancy map with
# Nav2 map_saver_cli. Same nodes and thresholds the live session would have used.

WSDIR="$HOME/projects/warehouse_ws"; MAPDIR="$WSDIR/diagnostics_phase2b/mapping"
CFGDIR="$WSDIR/install/turtlebot3_cartographer/share/turtlebot3_cartographer/config"
source /opt/ros/humble/setup.bash; source "$WSDIR/install/setup.bash"
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
setsid ros2 run cartographer_ros cartographer_node -configuration_directory "$CFGDIR" \
  -configuration_basename turtlebot3_lds_2d.lua -load_state_filename "$MAPDIR/warehouse_map_v2.pbstream" \
  -load_frozen_state true > "$MAPDIR/reload_cartographer.log" 2>&1 & CART=$!
setsid ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0 \
  > "$MAPDIR/reload_grid.log" 2>&1 & GRID=$!
sleep 15
timeout 120 ros2 run nav2_map_server map_saver_cli -f "$MAPDIR/warehouse_map_v2" --ros-args -p save_map_timeout:=30.0 \
  > "$MAPDIR/map_saver_reload.txt" 2>&1
RC=$?
kill -INT -- "-$CART" "-$GRID" 2>/dev/null; sleep 3; kill -KILL -- "-$CART" "-$GRID" 2>/dev/null
echo "map_saver exit $RC"
exit $RC
