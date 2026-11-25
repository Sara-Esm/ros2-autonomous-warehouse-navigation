"""The detector finds DICT_5X5_50 station markers with the installed OpenCV."""

import cv2
import numpy as np

from warehouse_perception.aruco_station_detector import make_detector


def marker(marker_id, size=200, border=60):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, size)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, size)
    return cv2.copyMakeBorder(img, border, border, border, border,
                              cv2.BORDER_CONSTANT, value=255)


def test_detects_each_station_id():
    detect = make_detector()
    for marker_id in range(4):
        _, ids, _ = detect(marker(marker_id))
        assert ids is not None and ids.ravel().tolist() == [marker_id]


def test_blank_image_has_no_detection():
    _, ids, _ = make_detector()(np.full((300, 300), 255, np.uint8))
    assert ids is None or len(ids) == 0


def test_4x4_marker_is_not_reported_as_station():
    d4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        img = cv2.aruco.generateImageMarker(d4, 1, 200)
    else:
        img = cv2.aruco.drawMarker(d4, 1, 200)
    img = cv2.copyMakeBorder(img, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
    _, ids, _ = make_detector()(img)
    assert ids is None or 1 not in ids.ravel().tolist()
