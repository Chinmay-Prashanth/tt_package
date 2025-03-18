#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from cv2 import aruco

# ------------------ Simple 3D Kalman Filter for (X, Y, Z) ------------------
class BallKalman:
    def __init__(self, dt=0.1):
        # State vector: [x, y, z, vx, vy, vz]^T
        self.dt = dt
        self.A = np.eye(6, dtype=np.float32)
        self.A[0,3] = dt
        self.A[1,4] = dt
        self.A[2,5] = dt

        # We measure only [x, y, z]
        self.H = np.zeros((3,6), dtype=np.float32)
        self.H[0,0] = 1.0
        self.H[1,1] = 1.0
        self.H[2,2] = 1.0

        self.P = np.eye(6, dtype=np.float32) * 1000.0
        self.Q = np.eye(6, dtype=np.float32) * 0.01
        self.R = np.eye(3, dtype=np.float32) * 0.05

        self.x = np.zeros((6,1), dtype=np.float32)

    def predict(self):
        self.x = self.A @ self.x
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, z):
        # z is [x, y, z]
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - (self.H @ self.x)
        self.x = self.x + K @ y
        I = np.eye(6, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P

    def get_state(self):
        return self.x  # [x, y, z, vx, vy, vz]^T


class PingPongArucoBox(Node):
    def __init__(self):
        super().__init__('ping_pong_aruco_box')

        self.bridge = CvBridge()

        # Latest images
        self.latest_color = None
        self.latest_depth = None

        # Subscribe to color & depth
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)

        # Publishers
        self.point_pub = self.create_publisher(PointStamped, '/ping_pong_ball_marker_frame', 10)
        self.annotated_pub = self.create_publisher(Image, '/ball_tracker/annotated_image', 10)

        # Timer for continuous detection/tracking (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Load YOLO
        self.model = YOLO("best.pt")
        self.get_logger().info("YOLO model loaded from best.pt.")

        # Color camera intrinsics
        self.color_width = 1280
        self.color_height = 720
        self.color_fx = 644.06097
        self.color_fy = 643.20557
        self.color_cx = 640.95575
        self.color_cy = 366.67944

        # Depth camera intrinsics
        self.depth_width = 848
        self.depth_height = 480
        self.depth_fx = 429.73431
        self.depth_fy = 429.73431
        self.depth_cx = 422.43372
        self.depth_cy = 238.09940

        self.conf_thresh = 0.3

        # ArUco dictionary & parameters (old approach)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        # If cornerRefinementMethod is available, enable it
        if hasattr(self.aruco_params, 'cornerRefinementMethod'):
            self.aruco_params.cornerRefinementMethod = getattr(aruco, 'CORNER_REFINE_SUBPIX', 1)

        self.marker_id = 1
        self.marker_length = 0.05  # 5 cm marker

        # YOLO bounding box / tracker
        self.tracking = False
        self.tracker = None
        self.bbox = (0, 0, 0, 0)

        # Store last valid marker pose
        self.marker_rvec = None
        self.marker_tvec = None

        # Kalman filter for ball in marker coords
        self.kf = BallKalman(dt=0.1)

        # Depth range checks (in meters)
        self.depth_min = 0.2
        self.depth_max = 5.0

        # -------------------------
        # Define your imaginary box in marker coords
        # We'll assume:
        #  - X axis = ± half the table's breadth
        #  - Y axis = 0..0.5 m (height)
        #  - Z axis = 0..(camera_distance - 0.5)
        # For example, table breadth ~1.52 m => half ~0.76
        # camera ~2.44 m => minus 0.5 => ~1.94
        self.table_half_breadth = 0.76  # half the table's breadth
        self.box_y_max = 0.5
        self.box_z_max = 1.94

        # For convenience, we define the 8 corners of the box in marker coords
        # We'll draw this box in the color image
        self.box_corners_marker = self.define_box_corners(
            x_min=-self.table_half_breadth, x_max=self.table_half_breadth,
            y_min=0.0,                   y_max=self.box_y_max,
            z_min=0.0,                   z_max=self.box_z_max
        )
        self.box_edges = [
            (0,1), (0,2), (1,3), (2,3),  # bottom face
            (4,5), (4,6), (5,7), (6,7),  # top face
            (0,4), (1,5), (2,6), (3,7)   # vertical edges
        ]

    # ---------------- HELPER FUNCTIONS ----------------

    def define_box_corners(self, x_min, x_max, y_min, y_max, z_min, z_max):
        """
        Return 8 corners in marker coords, forming a rectangular prism.
        Index them like:
            0: (x_min, y_min, z_min)
            1: (x_max, y_min, z_min)
            2: (x_min, y_max, z_min)
            3: (x_max, y_max, z_min)
            4: (x_min, y_min, z_max)
            5: (x_max, y_min, z_max)
            6: (x_min, y_max, z_max)
            7: (x_max, y_max, z_max)
        """
        return np.array([
            [x_min, y_min, z_min],
            [x_max, y_min, z_min],
            [x_min, y_max, z_min],
            [x_max, y_max, z_min],
            [x_min, y_min, z_max],
            [x_max, y_min, z_max],
            [x_min, y_max, z_max],
            [x_max, y_max, z_max]
        ], dtype=np.float32)

    def color_callback(self, msg: Image):
        try:
            color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_color = color_img
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")

    def depth_callback(self, msg: Image):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = depth_img
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def timer_callback(self):
        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for color and depth frames...")
            return

        color_img = self.latest_color.copy()
        depth_img = self.latest_depth.copy()

        # 1) Detect ArUco
        corners, ids, _ = aruco.detectMarkers(color_img, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for i, marker_id in enumerate(ids):
                if marker_id[0] == self.marker_id:
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], self.marker_length,
                        self.get_camera_matrix(),
                        self.get_dist_coeffs()
                    )
                    rvec = rvecs[0][0]
                    tvec = tvecs[0][0]

                    dist_marker = np.linalg.norm(tvec)
                    if 0.1 < dist_marker < 10.0:
                        self.marker_rvec = rvec
                        self.marker_tvec = tvec

                    # Draw axis & corners
                    aruco.drawAxis(color_img, self.get_camera_matrix(), self.get_dist_coeffs(),
                                   rvec, tvec, 0.03)
                    cv2.polylines(color_img, [np.int32(corners[i])], True, (0,255,255), 2)
                    cv2.putText(color_img, f"Aruco ID={self.marker_id}",
                                tuple(corners[i][0][0].astype(int)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        # 2) YOLO detection for the ball
        detection_found = False
        results = self.model(color_img)
        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            confidences = boxes.conf.cpu().numpy()
            xyxy = boxes.xyxy.cpu().numpy()

            best_idx = np.argmax(confidences)
            best_conf = confidences[best_idx]
            if best_conf >= self.conf_thresh:
                x1, y1, x2, y2 = xyxy[best_idx].astype(int)
                w_ = x2 - x1
                h_ = y2 - y1
                if 5 < w_ < 200 and 5 < h_ < 200:
                    self.bbox = (x1, y1, w_, h_)
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(color_img, self.bbox)
                    self.tracking = True
                    detection_found = True
                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (0,255,0), 2)

        if not detection_found:
            if self.tracking and self.tracker is not None:
                success, tracked_bbox = self.tracker.update(color_img)
                if success:
                    self.bbox = tracked_bbox
                    x_, y_, w_, h_ = map(int, self.bbox)
                    cv2.rectangle(color_img, (x_, y_), (x_ + w_, y_ + h_), (255,0,0), 2)
                else:
                    self.tracking = False
                    self.tracker = None

        # 3) Compute 3D position
        x_col, y_col, w_col, h_col = map(int, self.bbox)
        center_x_col = x_col + w_col//2
        center_y_col = y_col + h_col//2
        if not (0 <= center_x_col < self.color_width and 0 <= center_y_col < self.color_height):
            # No valid box
            self.draw_3d_box_in_image(color_img)  # draw imaginary box
            self.publish_annotated_image(color_img)
            return

        depth_x = int(round(center_x_col * (self.depth_width / self.color_width)))
        depth_y = int(round(center_y_col * (self.depth_height / self.color_height)))
        if not (0 <= depth_x < self.depth_width and 0 <= depth_y < self.depth_height):
            self.draw_3d_box_in_image(color_img)
            self.publish_annotated_image(color_img)
            return

        depth_value = depth_img[depth_y, depth_x]
        if depth_value == 0:
            self.draw_3d_box_in_image(color_img)
            self.publish_annotated_image(color_img)
            return

        Z_c = depth_value / 1000.0
        if not (self.depth_min < Z_c < self.depth_max):
            self.draw_3d_box_in_image(color_img)
            self.publish_annotated_image(color_img)
            return

        X_c = (depth_x - self.depth_cx) * Z_c / self.depth_fx
        Y_c = (depth_y - self.depth_cy) * Z_c / self.depth_fy

        # 4) Transform to marker coords + Kalman + box check
        if self.marker_rvec is not None and self.marker_tvec is not None:
            ball_camera = np.array([[X_c], [Y_c], [Z_c]], dtype=np.float32)
            tvec = self.marker_tvec.reshape((3,1))
            R, _ = cv2.Rodrigues(self.marker_rvec)
            R_inv = R.T
            P_marker = R_inv @ (ball_camera - tvec)
            X_m = P_marker[0,0]
            Y_m = P_marker[1,0]
            Z_m = P_marker[2,0]

            self.kf.predict()
            z_meas = np.array([[X_m], [Y_m], [Z_m]], dtype=np.float32)
            self.kf.update(z_meas)
            x_est = self.kf.get_state()
            X_f, Y_f, Z_f = x_est[0,0], x_est[1,0], x_est[2,0]

            # Check if inside the box
            if self.is_in_box(X_f, Y_f, Z_f):
                self.get_logger().info(
                    f"Ball (filtered) in marker coords: X={X_f:.2f}, Y={Y_f:.2f}, Z={Z_f:.2f} [IN BOX]"
                )
            else:
                self.get_logger().info(
                    f"Ball (filtered) in marker coords: X={X_f:.2f}, Y={Y_f:.2f}, Z={Z_f:.2f} [OUT BOX]"
                )

            # Publish
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "aruco_marker_frame"
            point_msg.point.x = float(X_f)
            point_msg.point.y = float(Y_f)
            point_msg.point.z = float(Z_f)
            self.point_pub.publish(point_msg)

            # Also draw the imaginary 3D box
            self.draw_3d_box_in_image(color_img, R, tvec)

        else:
            # No marker pose yet
            self.draw_3d_box_in_image(color_img)
            self.get_logger().info(f"Ball in camera coords: X={X_c:.2f}, Y={Y_c:.2f}, Z={Z_c:.2f}")

        # Draw ball center in color image
        cv2.circle(color_img, (center_x_col, center_y_col), 4, (0,0,255), -1)
        cv2.putText(color_img, "Ball", (x_col, y_col - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)

        self.publish_annotated_image(color_img)

    def is_in_box(self, X_m, Y_m, Z_m):
        """
        Check if the point (X_m, Y_m, Z_m) is within:
         X in [-table_half_breadth..table_half_breadth]
         Y in [0..0.5]
         Z in [0..(2.44 - 0.5)=1.94], for example
        """
        return (
            -self.table_half_breadth <= X_m <= self.table_half_breadth and
            0.0 <= Y_m <= self.box_y_max and
            0.0 <= Z_m <= self.box_z_max
        )

    def draw_3d_box_in_image(self, color_img, R=None, t=None):
        """
        Project the 8 corners of the imaginary box from marker coords
        into the color image and draw edges, so you can see it in rqt_image_view.

        If we don't have R,t yet, we just skip.
        """
        if R is None or t is None:
            # No marker pose => can't project
            return

        # We'll transform each corner: P_camera = R * P_marker + t
        # Then project to image using pinhole model
        corners_2d = []
        for corner in self.box_corners_marker:
            corner_m = corner.reshape((3,1))  # (3,1)
            # camera coords
            corner_c = R @ corner_m + t.reshape((3,1))  # 3x1
            Xc, Yc, Zc = corner_c[0,0], corner_c[1,0], corner_c[2,0]
            if Zc <= 0.01:
                # behind camera or too close => skip
                corners_2d.append(None)
                continue
            # project
            u = self.color_fx * (Xc / Zc) + self.color_cx
            v = self.color_fy * (Yc / Zc) + self.color_cy
            corners_2d.append((int(u), int(v)))

        # Draw edges
        for (i1, i2) in self.box_edges:
            p1 = corners_2d[i1]
            p2 = corners_2d[i2]
            if p1 is not None and p2 is not None:
                cv2.line(color_img, p1, p2, (255,0,255), 2)

    def publish_annotated_image(self, annotated):
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header.stamp = self.get_clock().now().to_msg()
        self.annotated_pub.publish(annotated_msg)

    def get_camera_matrix(self):
        return np.array([
            [self.color_fx, 0,              self.color_cx],
            [0,              self.color_fy, self.color_cy],
            [0,              0,             1]
        ], dtype=np.float32)

    def get_dist_coeffs(self):
        # If you have real distortion, fill them here
        return np.zeros((5,), dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = PingPongArucoBox()
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
