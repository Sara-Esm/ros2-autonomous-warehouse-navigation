#!/usr/bin/env python3
"""
ArUco station detector: publishes the ID of the station marker in view.

Works with OpenCV >= 4.7 (cv2.aruco.ArucoDetector) and with the older
aruco API shipped by Ubuntu 22.04's python3-opencv (4.5.4).
"""

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool


def make_detector(dictionary_id=cv2.aruco.DICT_5X5_50):
    """Return detect(gray) -> (corners, ids, rejected) for this OpenCV version."""
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, 'DetectorParameters_create'):  # OpenCV < 4.7
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.minMarkerPerimeterRate = 0.005
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.01
    parameters.minDistanceToBorder = 1
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers

    def detect(gray):
        return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    return detect


class ArucoStationDetector(Node):

    def __init__(self):
        super().__init__('aruco_station_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('debug_window', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.debug_window = self.get_parameter('debug_window').value

        self.bridge = CvBridge()

        self.marker_pub = self.create_publisher(Int32, '/warehouse/marker_id', 10)
        self.detected_pub = self.create_publisher(Bool, '/warehouse/marker_detected', 10)

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.detect = make_detector(cv2.aruco.DICT_5X5_50)

        self.get_logger().info('ArUco station detector started.')
        self.get_logger().info(f'Subscribing to image topic: {self.image_topic}')
        self.get_logger().info(f'Using dictionary: DICT_5X5_50 (OpenCV {cv2.__version__})')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detect(gray)

        detected_msg = Bool()
        marker_msg = Int32()

        if ids is not None and len(ids) > 0:
            marker_id = int(ids[0][0])

            detected_msg.data = True
            marker_msg.data = marker_id

            self.marker_pub.publish(marker_msg)
            self.detected_pub.publish(detected_msg)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            self.get_logger().info(
                f'ArUco marker detected: ID={marker_id}',
                throttle_duration_sec=1.0
            )

        else:
            detected_msg.data = False
            self.detected_pub.publish(detected_msg)

            self.get_logger().info(
                f'No marker detected. Rejected candidates: {len(rejected)}',
                throttle_duration_sec=2.0
            )

        if self.debug_window:
            cv2.imshow('Warehouse ArUco Station Detector', frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoStationDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
