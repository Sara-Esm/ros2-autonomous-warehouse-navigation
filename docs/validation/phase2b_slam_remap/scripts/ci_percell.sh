#!/usr/bin/env bash
apt-get update -qq >/dev/null; apt-get install -y -qq --no-install-recommends ros-humble-nav2-map-server python3-numpy python3-yaml >/dev/null 2>&1
source /opt/ros/humble/setup.bash
echo "== CI image nav2-map-server $(dpkg -l ros-humble-nav2-map-server | awk '/^ii/{print $3}')"
for y in /maps/warehouse_map_v2.yaml /maps/warehouse_map.yaml; do python3 /probe/percell_probe.py $y 2>&1 | grep -v "^\["; done
