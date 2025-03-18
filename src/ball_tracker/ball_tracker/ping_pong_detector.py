#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class PingPongDetector(Node):
    def __init__(self):
        super().__init__('ping_pong_detector')

        self.bridge = CvBridge()

        # Latest images
        self.latest_color = None
        self.latest_depth = None

        # Subscribe to color and depth
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 1280x720
            self.color_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # 848x480
            self.depth_callback,
            10
        )

        # Publishers
        self.point_pub = self.create_publisher(PointStamped, '/ping_pong_ball', 10)
        self.annotated_pub = self.create_publisher(Image, '/ball_tracker/annotated_image', 10)

        # Timer for continuous processing (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Load YOLO model
        self.model = YOLO("best.pt")
        self.get_logger().info("YOLO model loaded from best.pt.")

        # -- Color camera intrinsics (from your color camera_info) --
        #   1280x720 resolution
        self.color_width = 1280
        self.color_height = 720
        self.color_fx = 644.06097
        self.color_fy = 643.20557
        self.color_cx = 640.95575
        self.color_cy = 366.67944

        # -- Depth camera intrinsics (from your depth camera_info) --
        #   848x480 resolution
        self.depth_width = 848
        self.depth_height = 480
        self.depth_fx = 429.73431
        self.depth_fy = 429.73431
        self.depth_cx = 422.43372
        self.depth_cy = 238.09940

        # Confidence threshold
        self.conf_thresh = 0.3

        # Tracking state
        self.tracking = False
        self.tracker = None
        self.bbox = (0, 0, 0, 0)  # (x, y, w, h)

    def color_callback(self, msg: Image):
        """Store the latest color frame. (1280x720)"""
        try:
            # We assume the raw image is already 1280x720. No resizing.
            color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color = color_img
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")

    def depth_callback(self, msg: Image):
        """Store the latest depth frame. (848x480) in mm."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # depth_img is 16UC1 in mm (0 = invalid).
            self.latest_depth = depth_img
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def timer_callback(self):
        """Continuously run YOLO detection, then fallback to tracker if no detection."""
        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for color and depth frames...")
            return

        color_img = self.latest_color.copy()
        depth_img = self.latest_depth.copy()

        # 1) Always run YOLO detection on color_img
        results = self.model(color_img)
        detection_found = False

        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            confidences = boxes.conf.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()  # [x1, y1, x2, y2]

            # Find best detection above threshold
            best_idx = np.argmax(confidences)
            best_conf = confidences[best_idx]

            if best_conf >= self.conf_thresh:
                x1, y1, x2, y2 = xyxy[best_idx].astype(int)
                w = x2 - x1
                h = y2 - y1

                # Re-init the tracker with the new detection
                self.bbox = (x1, y1, w, h)
                self.tracker = cv2.TrackerCSRT_create()
                self.tracker.init(color_img, self.bbox)
                self.tracking = True
                detection_found = True

                # Draw the YOLO detection box in green
                cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            else:
                self.get_logger().info("Detected ball but confidence below threshold.")

        # 2) If no detection was found, try the tracker
        if not detection_found:
            if self.tracking and self.tracker is not None:
                success, tracked_bbox = self.tracker.update(color_img)
                if success:
                    self.bbox = tracked_bbox
                    x, y, w, h = map(int, self.bbox)
                    # Draw the tracking box in blue
                    cv2.rectangle(color_img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                else:
                    self.get_logger().info("Tracker lost the ball; no new detection found.")
                    self.tracking = False
                    self.tracker = None
                    self.publish_annotated_image(color_img)
                    return
            else:
                self.get_logger().info("No detection and no tracker available.")
                self.publish_annotated_image(color_img)
                return

        # 3) We have a bbox in color coordinates
        x_col, y_col, w_col, h_col = map(int, self.bbox)
        center_x_col = x_col + w_col // 2
        center_y_col = y_col + h_col // 2

        # Check color bounding box center is within the color image
        if not (0 <= center_x_col < self.color_width and 0 <= center_y_col < self.color_height):
            self.get_logger().info("Color bounding box center is out of color image bounds.")
            self.publish_annotated_image(color_img)
            return

        # 4) Map the color bounding box center to the depth image
        #    (Naive approach: scale from color resolution to depth resolution)
        depth_x = int(round(center_x_col * (self.depth_width / self.color_width)))
        depth_y = int(round(center_y_col * (self.depth_height / self.color_height)))

        # Check if mapped coordinates are within the depth image
        if not (0 <= depth_x < self.depth_width and 0 <= depth_y < self.depth_height):
            self.get_logger().info("Mapped depth pixel is out of depth image bounds.")
            self.publish_annotated_image(color_img)
            return

        depth_value = depth_img[depth_y, depth_x]
        if depth_value == 0:
            self.get_logger().info("Invalid depth (0) at mapped pixel.")
            self.publish_annotated_image(color_img)
            return

        # Depth is in mm, convert to meters
        Z = depth_value / 1000.0

        # 5) Use depth camera intrinsics to compute 3D (in the depth camera frame)
        # X_d = (u_d - cx_d) * Z / fx_d
        # Y_d = (v_d - cy_d) * Z / fy_d
        # Z_d = Z
        X_d = (depth_x - self.depth_cx) * Z / self.depth_fx
        Y_d = (depth_y - self.depth_cy) * Z / self.depth_fy
        # Z_d = Z

        # Publish the 3D position (assuming same frame_id or you might rename it)
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera_depth_optical_frame"  # or your desired frame
        point_msg.point.x = float(X_d)
        point_msg.point.y = float(Y_d)
        point_msg.point.z = float(Z)
        self.point_pub.publish(point_msg)

        self.get_logger().info(
            f"Ball center (color) = ({center_x_col}, {center_y_col}), mapped depth px = ({depth_x}, {depth_y}), "
            f"Z={Z:.3f} m => 3D=({X_d:.3f}, {Y_d:.3f}, {Z:.3f})"
        )

        # 6) Annotate the color image for visualization
        cv2.circle(color_img, (center_x_col, center_y_col), 4, (0, 0, 255), -1)
        cv2.putText(color_img, "Ball", (x_col, y_col - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # If you want a window pop-up, uncomment:
        # cv2.imshow("Ping Pong Tracking", color_img)
        # cv2.waitKey(1)

        self.publish_annotated_image(color_img)

    def publish_annotated_image(self, annotated):
        """Helper to publish an annotated OpenCV image as a ROS Image message."""
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header.stamp = self.get_clock().now().to_msg()
        self.annotated_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PingPongDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ping Pong Detector node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
