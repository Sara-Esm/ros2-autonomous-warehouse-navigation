#!/usr/bin/env bash
# Reproduction of .github/workflows/warehouse-humble.yml job "warehouse-tests" (with this branch's deps)
set -e
cp -r /src_ro /ws && cd /ws
apt-get update -q >/dev/null
apt-get install -y --no-install-recommends python3-colcon-common-extensions python3-pytest python3-opencv ros-humble-nav2-msgs ros-humble-nav2-map-server ros-humble-cv-bridge ros-humble-ament-copyright ros-humble-ament-flake8 ros-humble-ament-pep257 >/dev/null 2>&1
dpkg -l | grep -E "ros-humble-nav2-map-server |libgraphicsmagick\+\+-q16" | awk '{print $2" "$3}'
source /opt/ros/humble/setup.bash
colcon build --packages-up-to warehouse_bringup warehouse_gazebo warehouse_perception 2>&1 | tail -2
source install/setup.bash
colcon test --packages-select warehouse_bringup warehouse_gazebo warehouse_perception --executor sequential 2>&1 | tail -2
colcon test-result --verbose 2>&1 | tail -15
python3 - <<'PY'
import xml.etree.ElementTree as ET
r = ET.parse('build/warehouse_bringup/pytest.xml').getroot()
for tc in r.iter('testcase'):
    if 'map_classification' in tc.get('classname', ''):
        st = 'SKIPPED' if tc.find('skipped') is not None else 'FAILED' if tc.find('failure') is not None else 'passed'
        print(f"{tc.get('name')}: {st} ({tc.get('time')} s)")
PY
