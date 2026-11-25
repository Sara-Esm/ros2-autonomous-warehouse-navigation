"""Marker assets must match the generator and decode with the detector's dictionary."""

import importlib.util
import os

import cv2

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_generator():
    path = os.path.join(PKG, 'markers', 'generate_markers.py')
    spec = importlib.util.spec_from_file_location('generate_markers', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def detect_ids(gray):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        _, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    return [] if ids is None else sorted(int(i) for i in ids.ravel())


def test_committed_textures_match_generator():
    assert load_generator().check()


def test_world_marker_textures_decode_to_station_ids():
    gen = load_generator()
    for marker_id, name in gen.MARKERS.items():
        path = os.path.join(PKG, 'models', f'aruco_marker_{marker_id}', 'materials',
                            'textures', f'{name}.png')
        assert detect_ids(cv2.imread(path, cv2.IMREAD_GRAYSCALE)) == [marker_id], path
