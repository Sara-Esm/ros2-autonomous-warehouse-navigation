"""Phase 4 camera configuration tests: pinhole robot model and simulation launch wiring."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parents[0]
MODEL = SRC / 'warehouse_gazebo' / 'models' / 'turtlebot3_burger_cam_pinhole' / 'model.sdf'
SIM_LAUNCH = PKG / 'launch' / 'warehouse_simulation_camera.launch.py'
SYSTEM_LAUNCH = PKG / 'launch' / 'warehouse_robot_system_camera.launch.py'
INSTALLED = Path('/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger_cam/model.sdf')


def camera(path):
    model = ET.parse(path).getroot().find('model')
    sensors = [s for s in model.iter('sensor') if s.get('name') == 'camera']
    assert len(sensors) == 1
    return model, sensors[0]


def test_pinhole_camera_parameters():
    _, s = camera(MODEL)
    assert s.get('type') == 'camera'
    cam = s.find('camera')
    assert float(cam.findtext('horizontal_fov')) == pytest.approx(1.745)
    assert (cam.findtext('image/width'), cam.findtext('image/height')) == ('320', '240')
    assert cam.find('lens') is None
    plugin = s.find('plugin')
    assert plugin.get('filename') == 'libgazebo_ros_camera.so'
    assert plugin.findtext('imageTopicName') == 'image_raw'
    assert plugin.findtext('cameraInfoTopicName') == 'camera_info'
    assert plugin.findtext('frameName') == 'camera_link'


def without_camera(path):
    model, s = camera(path)
    for link in model.iter('link'):
        if s in list(link):
            link.remove(s)
    model.set('name', 'm')
    return [(e.tag, (e.text or '').strip(), sorted(e.attrib.items())) for e in model.iter()]


@pytest.mark.skipif(not INSTALLED.exists(), reason='turtlebot3_gazebo not installed')
def test_model_differs_from_installed_only_in_camera_sensor():
    assert without_camera(MODEL) == without_camera(INSTALLED)
    _, ours = camera(MODEL)
    _, theirs = camera(INSTALLED)
    assert theirs.get('type') == 'wideanglecamera'
    for key in ('update_rate', 'camera/image/width', 'camera/image/height', 'camera/clip/near'):
        assert ours.findtext(key) == theirs.findtext(key)


def strings(path):
    return {n.value for n in ast.walk(ast.parse(path.read_text()))
            if isinstance(n, ast.Constant) and isinstance(n.value, str)}


def test_simulation_launch_spawns_pinhole_model_and_gui_is_optional():
    s = strings(SIM_LAUNCH)
    assert 'turtlebot3_burger_cam_pinhole' in s
    assert 'turtlebot3_burger_cam' not in s
    assert {'gui', 'gazebo', 'gzserver'} <= s
    assert 'gui' in strings(SYSTEM_LAUNCH)
