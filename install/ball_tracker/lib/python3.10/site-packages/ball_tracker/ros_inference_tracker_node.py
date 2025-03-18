#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco

class ArucoMarkerNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_node')

        self.bridge = CvBridge()
        self.latest_color = None

        # Subscribe to the color image
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)

        # Publisher for the annotated image
        self.annotated_pub = self.create_publisher(Image, '/aruco/annotated_image', 10)

        # Timer to process images at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Create an ArUco dictionary & parameters
        self.marker_id = 1
        self.marker_length = 0.05  # 5 cm for pose estimation
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

        # If your OpenCV version supports cornerRefinementMethod:
        if hasattr(self.aruco_params, 'cornerRefinementMethod'):
            self.aruco_params.cornerRefinementMethod = getattr(aruco, 'CORNER_REFINE_SUBPIX', 1)

        # Example camera intrinsics (replace with your real values)
        self.fx = 644.0
        self.fy = 643.0
        self.cx = 640.0
        self.cy = 366.0
        self.dist_coeffs = np.zeros((5,), dtype=np.float32)

        self.get_logger().info("Aruco marker node initialized.")

    def color_callback(self, msg: Image):
        try:
            color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color = color_img
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")

    def timer_callback(self):
        if self.latest_color is None:
            self.get_logger().warn("No color image yet.")
            return

        color_img = self.latest_color.copy()

        # 1) Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(
            color_img, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # 2) For each marker found, draw corners and ID
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.marker_id:
                    # Draw marker corners in yellow
                    cv2.polylines(color_img, [np.int32(corners[i])], True, (0,255,255), 2)

                    # Put text near the first corner
                    corner_xy = corners[i][0][0].astype(int)
                    cv2.putText(color_img, f"Aruco ID={self.marker_id}",
                                tuple(corner_xy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

                    # 3) (Optional) Estimate pose if you have camera intrinsics
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], self.marker_length, self.get_camera_matrix(), self.dist_coeffs)
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]

                    # Draw a small axis on the marker (0.03 m long)
                    aruco.drawAxis(color_img, self.get_camera_matrix(), self.dist_coeffs,
                                   rvec, tvec, 0.03)

        # Publish the annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
        self.annotated_pub.publish(annotated_msg)

    def get_camera_matrix(self):
        return np.array([
            [self.fx, 0,      self.cx],
            [0,      self.fy, self.cy],
            [0,      0,       1     ]
        ], dtype=np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
