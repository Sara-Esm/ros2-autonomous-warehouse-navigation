#!/usr/bin/env bash
WSDIR="$HOME/projects/warehouse_ws"; MAPDIR="$WSDIR/diagnostics_phase2b/mapping"
CFGDIR="$WSDIR/install/turtlebot3_cartographer/share/turtlebot3_cartographer/config"
source /opt/ros/humble/setup.bash; source "$WSDIR/install/setup.bash"
export ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1
setsid ros2 run cartographer_ros cartographer_node -configuration_directory "$CFGDIR" -configuration_basename turtlebot3_lds_2d.lua \
  -load_state_filename "$MAPDIR/warehouse_map_v2.pbstream" -load_frozen_state true > /dev/null 2>&1 & CART=$!
setsid ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0 > /dev/null 2>&1 & GRID=$!
sleep 12
python3 "$WSDIR/diagnostics_phase2b/grid_values_probe.py" "$MAPDIR/cartographer_raw_grid.npz"
kill -INT -- "-$CART" "-$GRID" 2>/dev/null; sleep 3; kill -KILL -- "-$CART" "-$GRID" 2>/dev/null
