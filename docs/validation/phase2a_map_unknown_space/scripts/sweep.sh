#!/usr/bin/env bash
set -e
apt-get update -qq >/dev/null
apt-get install -y -qq --no-install-recommends ros-humble-nav2-map-server python3-numpy >/dev/null 2>&1
source /opt/ros/humble/setup.bash
echo "container: $(. /etc/os-release; echo $PRETTY_NAME)"
dpkg -l | grep -E "ros-humble-nav2-map-server |libgraphicsmagick\+\+-q16|libgraphicsmagick-q16" | awk '{print "  "$2" "$3}'
for ft in 0.25 0.19 0.05 0.03 0.025 0.02 0.01 0.002 0.001 0.0005; do
  printf "image: /data/warehouse_map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-7.56, -10.8, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: %s\n" $ft > /tmp/map_ft$ft.yaml
  python3 /probe/loaded_map_probe.py /tmp/map_ft$ft.yaml 2>&1 | grep -v "^\["
done
