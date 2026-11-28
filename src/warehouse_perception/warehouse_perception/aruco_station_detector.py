#!/usr/bin/env python3
"""
ArUco station detector: publishes the station markers seen in each camera image.

Topics:
* /warehouse/marker_detections (vision_msgs/Detection2DArray): one message per
  processed image. header = the SOURCE IMAGE's header (acquisition stamp and
  camera frame), so consumers can judge freshness; an empty detections list
  means the image was processed and no marker was found.
* /warehouse/marker_id (std_msgs/Int32) and /warehouse/marker_detected
  (std_msgs/Bool): unstamped, kept for compatibility. Do not use them to decide
  whether an observation is current.

Works with OpenCV >= 4.7 (cv2.aruco.ArucoDetector) and with the older
aruco API shipped by Ubuntu 22.04's python3-opencv (4.5.4).
"""

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


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
        # Off by default so the node runs without a display; the graphical
        # launch turns it on explicitly.
        self.declare_parameter('debug_window', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.debug_window = self.get_parameter('debug_window').value

        self.bridge = CvBridge()

        self.marker_pub = self.create_publisher(Int32, '/warehouse/marker_id', 10)
        self.detected_pub = self.create_publisher(Bool, '/warehouse/marker_detected', 10)
        self.detections_pub = self.create_publisher(
            Detection2DArray, '/warehouse/marker_detections', 10)

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

    @staticmethod
    def detections_msg(image_header, corners, ids):
        """Build the stamped per-image observation (empty when nothing was found)."""
        out = Detection2DArray()
        out.header = image_header  # acquisition stamp and camera frame of the source image
        if ids is None:
            return out
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            pts = marker_corners.reshape(-1, 2)
            det = Detection2D()
            det.header = image_header
            det.id = str(int(marker_id))
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(marker_id))
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)
            det.bbox.center.position.x = float(pts[:, 0].mean())
            det.bbox.center.position.y = float(pts[:, 1].mean())
            det.bbox.size_x = float(pts[:, 0].max() - pts[:, 0].min())
            det.bbox.size_y = float(pts[:, 1].max() - pts[:, 1].min())
            out.detections.append(det)
        return out

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detect(gray)
        self.detections_pub.publish(self.detections_msg(msg.header, corners, ids))

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
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT/SIGTERM: rclpy's signal handler shuts the context down, which
        # makes spin() raise ExternalShutdownException. That is a normal stop.
        pass
    finally:
        if node.debug_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
