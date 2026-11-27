#!/usr/bin/env python3
"""Generate camera-model variants from the installed TurtleBot3 burger_cam SDF (only the sensor block changes).

Usage: make_models.py <out_models_dir>
"""
import os, re, sys
SRC = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf'
out = sys.argv[1]
sdf = open(SRC).read()
block = re.search(r'<sensor name="camera" type="wideanglecamera">.*?</sensor>', sdf, re.S).group(0)
plugin = re.search(r'<plugin name="camera_driver".*?</plugin>', block, re.S).group(0)
def pinhole(hfov, w, h, rate):
    return f'''<sensor name="camera" type="camera">
        <always_on>true</always_on>
        <visualize>false</visualize>
        <update_rate>{rate}</update_rate>
        <camera name="picam">
          <horizontal_fov>{hfov}</horizontal_fov>
          <image>
            <width>{w}</width>
            <height>{h}</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.03</near>
            <far>100</far>
          </clip>
        </camera>
        {plugin.replace('<updateRate>30.0</updateRate>', f'<updateRate>{float(rate)}</updateRate>')}
      </sensor>'''
variants = {
    'cam_wide182_30hz': block,                                        # baseline (installed model, copied unchanged)
    'cam_wide182_30hz_novis': block.replace('<visualize>true</visualize>', '<visualize>false</visualize>'),
    'cam_pin100_320_30hz': pinhole(1.745, 320, 240, 30),
    'cam_pin100_320_15hz': pinhole(1.745, 320, 240, 15),
}
for name, sensor in variants.items():
    d = os.path.join(out, 'turtlebot3_' + name); os.makedirs(d, exist_ok=True)
    s = sdf.replace(block, sensor).replace('<model name="turtlebot3_burger_cam">', f'<model name="turtlebot3_{name}">')
    open(os.path.join(d, 'model.sdf'), 'w').write(s)
    open(os.path.join(d, 'model.config'), 'w').write(f'<?xml version="1.0"?>\n<model><name>turtlebot3_{name}</name><version>1.0</version><sdf version="1.6">model.sdf</sdf></model>\n')
    print(name, 'written')
