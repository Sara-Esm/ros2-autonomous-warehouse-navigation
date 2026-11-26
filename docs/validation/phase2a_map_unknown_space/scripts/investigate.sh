#!/usr/bin/env bash
apt-get update -qq >/dev/null
apt-get install -y -qq --no-install-recommends ros-humble-nav2-map-server python3-numpy >/dev/null 2>&1
source /opt/ros/humble/setup.bash
for ft in 0.15 0.12 0.10 0.08 0.06 0.0002; do
  printf "image: /data/warehouse_map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-7.56, -10.8, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: %s\n" $ft > /tmp/m$ft.yaml
  python3 /probe/loaded_map_probe.py /tmp/m$ft.yaml 2>&1 | grep -v "^\[" | sed "s/^m/map_ft/"
done
echo "--- changelog (1.1.18 -> 1.1.20) ---"
zcat /usr/share/doc/ros-humble-nav2-map-server/changelog.Debian.gz 2>/dev/null | head -40
find /opt/ros/humble/share/nav2_map_server -iname "*CHANGELOG*" -exec head -60 {} \;
