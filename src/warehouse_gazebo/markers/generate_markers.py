#!/usr/bin/env python3
"""
Generate the station ArUco marker images (DICT_5X5_50, IDs 0-3).

The committed textures in markers/generated_clean/ and
models/aruco_marker_N/materials/textures/ are exactly what this script
produces: a 500 px marker inside a 100 px white border (700 x 700 px).
The detector (warehouse_perception) uses the same dictionary.

    python3 generate_markers.py            # write to markers/regenerated/ (never overwrites)
    python3 generate_markers.py --check    # verify the committed textures match, pixel for pixel

markers/generated/ holds older DICT_4X4_50 images kept as history; they
are not used by the world and are not detectable by the detector.
"""

import argparse
import os
import sys

import cv2
import numpy as np

DICTIONARY = cv2.aruco.DICT_5X5_50
MARKER_PX = 500
BORDER_PX = 100
MARKERS = {
    0: 'charging_dock',
    1: 'loading_zone',
    2: 'inventory_station_a',
    3: 'inventory_station_b',
}

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def marker_image(marker_id):
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    if hasattr(cv2.aruco, 'generateImageMarker'):  # OpenCV >= 4.7
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, MARKER_PX)
    return cv2.copyMakeBorder(img, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
                              cv2.BORDER_CONSTANT, value=255)


def committed_paths(marker_id, name):
    return [
        os.path.join(HERE, 'generated_clean', f'{name}.png'),
        os.path.join(PKG, 'models', f'aruco_marker_{marker_id}', 'materials', 'textures',
                     f'{name}.png'),
    ]


def check():
    ok = True
    for marker_id, name in MARKERS.items():
        expected = marker_image(marker_id)
        for path in committed_paths(marker_id, name):
            actual = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            match = actual is not None and np.array_equal(actual, expected)
            ok &= match
            print(f"{'OK  ' if match else 'DIFF'} {os.path.relpath(path, PKG)}")
    return ok


def generate(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    for marker_id, name in MARKERS.items():
        path = os.path.join(output_dir, f'{name}.png')
        if os.path.exists(path):
            sys.exit(f'Refusing to overwrite {path}')
        cv2.imwrite(path, marker_image(marker_id))
        print(f'Generated ID {marker_id}: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--check', action='store_true',
                        help='verify committed textures instead of writing files')
    parser.add_argument('--output-dir', default=os.path.join(HERE, 'regenerated'))
    args = parser.parse_args()
    if args.check:
        return 0 if check() else 1
    generate(args.output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
