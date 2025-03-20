#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from tf2_ros import TransformBroadcaster
from collections import deque

def rotation_matrix_to_quaternion(R):
    """Convert a rotation matrix to quaternion (w, x, y, z)"""
    trace = np.trace(R)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return w, x, y, z

class IntegratedDetector(Node):
    def __init__(self):
        super().__init__('integrated_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize the ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Optimize detector parameters for RealSense D455
        self.aruco_params.adaptiveThreshWinSizeMin = 23
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.05
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 9
        self.aruco_params.cornerRefinementMaxIterations = 50
        self.aruco_params.cornerRefinementMinAccuracy = 0.05
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Initialize YOLO model for ball detection
        self.model = YOLO("best.pt")
        self.get_logger().info("YOLO model loaded from best.pt")
        
        # Latest images
        self.latest_color = None
        self.latest_depth = None
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # ArUco marker parameters
        self.marker_size = 0.20  # 20cm marker size
        self.pose_history = deque(maxlen=30)
        self.last_stable_pose = None
        self.initial_wall_normal = None
        self.last_marker_transform = None
        
        # Filtering parameters
        self.translation_threshold = 0.03
        self.rotation_threshold = 0.15
        self.alpha = 0.95
        self.wall_normal_tolerance = 0.2
        
        # Ball tracking parameters
        self.conf_thresh = 0.3
        self.tracking = False
        self.tracker = None
        self.bbox = (0, 0, 0, 0)
        
        # Table dimensions
        self.TABLE_WIDTH = 2.74
        self.TABLE_HEIGHT = 1.525
        self.TABLE_LENGTH = 2.74
        
        # Create subscribers
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # Create publishers
        self.debug_image_pub = self.create_publisher(Image, 'integrated_debug_image', 10)
        self.ball_pose_pub = self.create_publisher(PointStamped, 'ball_in_table_frame', 10)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Processing timer (30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.process_frame)
        
        self.get_logger().info('Integrated detector initialized')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters received')

    def color_callback(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color conversion error: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def stabilize_wall_orientation(self, rvec, tvec):
        """Stabilize orientation for wall-mounted marker"""
        rot_mat, _ = cv2.Rodrigues(rvec)
        normal = rot_mat[:, 2]
        
        if self.initial_wall_normal is None:
            self.initial_wall_normal = normal
            return rvec
        
        normal_diff = np.arccos(np.clip(np.dot(normal, self.initial_wall_normal), -1.0, 1.0))
        
        if abs(normal_diff) < self.wall_normal_tolerance:
            z_axis = self.initial_wall_normal
        else:
            z_axis = 0.99 * self.initial_wall_normal + 0.01 * normal
            z_axis = z_axis / np.linalg.norm(z_axis)
        
        y_temp = np.array([0, 1, 0])
        x_axis = np.cross(y_temp, z_axis)
        x_norm = np.linalg.norm(x_axis)
        
        if x_norm < 1e-6:
            y_temp = np.array([0, 0, 1])
            x_axis = np.cross(y_temp, z_axis)
        
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        stabilized_rot = np.column_stack((x_axis, y_axis, z_axis))
        stabilized_rvec, _ = cv2.Rodrigues(stabilized_rot)
        
        return stabilized_rvec.flatten()

    def filter_pose(self, tvec, rvec):
        """Apply enhanced temporal filtering to reduce pose jitter"""
        if self.last_stable_pose is None:
            self.last_stable_pose = (tvec.copy(), rvec.copy())
            self.pose_history.append((tvec, rvec))
            return tvec, rvec
        
        rvec = self.stabilize_wall_orientation(rvec, tvec)
        
        t_diff = np.linalg.norm(tvec - self.last_stable_pose[0])
        r_diff = np.linalg.norm(rvec - self.last_stable_pose[1])
        
        if t_diff < self.translation_threshold * 1.5 and r_diff < self.rotation_threshold * 1.5:
            filtered_tvec = 0.98 * self.last_stable_pose[0] + 0.02 * tvec
            filtered_rvec = 0.98 * self.last_stable_pose[1] + 0.02 * rvec
        else:
            if len(self.pose_history) > 0:
                weights = np.exp(np.linspace(-3, 0, len(self.pose_history)))
                weights = weights / np.sum(weights)
                
                recent_tvecs = np.array([pose[0] for pose in self.pose_history])
                recent_rvecs = np.array([pose[1] for pose in self.pose_history])
                
                avg_tvec = np.average(recent_tvecs, axis=0, weights=weights)
                avg_rvec = np.average(recent_rvecs, axis=0, weights=weights)
                
                filtered_tvec = 0.9 * avg_tvec + 0.1 * tvec
                filtered_rvec = 0.9 * avg_rvec + 0.1 * rvec
            else:
                filtered_tvec = tvec
                filtered_rvec = rvec
        
        self.pose_history.append((filtered_tvec, filtered_rvec))
        self.last_stable_pose = (filtered_tvec.copy(), filtered_rvec.copy())
        
        return filtered_tvec, filtered_rvec

    def draw_table_box(self, image, rvec, tvec):
        """Draw table tennis table bounding box"""
        half_width = self.TABLE_WIDTH / 2
        half_height = self.TABLE_HEIGHT / 2
        
        table_points = np.float32([
            [-half_width, -half_height, 0],
            [half_width, -half_height, 0],
            [half_width, half_height, 0],
            [-half_width, half_height, 0],
            [-half_width, -half_height, self.TABLE_LENGTH],
            [half_width, -half_height, self.TABLE_LENGTH],
            [half_width, half_height, self.TABLE_LENGTH],
            [-half_width, half_height, self.TABLE_LENGTH],
            [0, 0, 0],
            [0, 0, 0.2]
        ])
        
        try:
            img_points, _ = cv2.projectPoints(table_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
            
            h, w = image.shape[:2]
            img_points = np.clip(img_points, -1e6, 1e6)
            img_points = img_points.reshape(-1, 2)
            
            valid_points = np.all(np.isfinite(img_points), axis=1)
            if not np.all(valid_points):
                return image
            
            img_points = np.clip(img_points, 0, [w-1, h-1])
            img_points = img_points.astype(np.int32)
            
            # Draw box
            cv2.polylines(image, [img_points[:4].reshape(-1,1,2)], True, (0, 255, 255), 2)
            cv2.polylines(image, [img_points[4:8].reshape(-1,1,2)], True, (0, 255, 255), 2)
            
            for i in range(4):
                cv2.line(image, tuple(img_points[i]), tuple(img_points[i+4]), (0, 255, 255), 1)
            
            # Draw orientation indicator
            cv2.line(image, tuple(img_points[8]), tuple(img_points[9]), (255, 0, 0), 2)
            
        except Exception as e:
            self.get_logger().warn(f'Error drawing table box: {str(e)}')
        
        return image

    def transform_point_to_table_frame(self, point_camera, marker_rvec, marker_tvec):
        """Transform a point from camera frame to table frame"""
        # Convert rotation vector to matrix
        R_marker_to_camera, _ = cv2.Rodrigues(marker_rvec)
        
        # Create transformation matrix from camera to marker
        T_camera_to_marker = np.eye(4)
        T_camera_to_marker[:3, :3] = R_marker_to_camera.T
        T_camera_to_marker[:3, 3] = -R_marker_to_camera.T @ marker_tvec
        
        # Transform point to marker frame
        point_homogeneous = np.append(point_camera, 1)
        point_marker = T_camera_to_marker @ point_homogeneous
        
        return point_marker[:3]

    def process_frame(self):
        # Check each component separately since camera_matrix is a numpy array
        if self.latest_color is None or self.latest_depth is None or self.camera_matrix is None:
            return

        try:
            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            vis_image = color_img.copy()
            
            # 1. Detect ArUco marker
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            marker_detected = False
            if ids is not None:
                marker_idx = np.where(ids == 189)[0]
                if len(marker_idx) > 0:
                    marker_detected = True
                    idx = marker_idx[0]
                    
                    # Get marker pose
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[idx]], self.marker_size, self.camera_matrix, self.dist_coeffs)
                    
                    # Filter pose
                    tvec, rvec = self.filter_pose(tvecs[0][0], rvecs[0][0])
                    
                    # Save marker transform
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    w, x, y, z = rotation_matrix_to_quaternion(rot_mat)
                    
                    transform = TransformStamped()
                    transform.header.stamp = self.get_clock().now().to_msg()
                    transform.header.frame_id = "camera_color_optical_frame"
                    transform.child_frame_id = 'table_frame'
                    transform.transform.translation.x = tvec[0]
                    transform.transform.translation.y = tvec[1]
                    transform.transform.translation.z = tvec[2]
                    transform.transform.rotation.w = w
                    transform.transform.rotation.x = x
                    transform.transform.rotation.y = y
                    transform.transform.rotation.z = z
                    
                    self.tf_broadcaster.sendTransform(transform)
                    self.last_marker_transform = (rvec, tvec)
                    
                    # Draw marker and table box
                    cv2.aruco.drawDetectedMarkers(vis_image, [corners[idx]], np.array([[189]]))
                    vis_image = self.draw_table_box(vis_image, rvec, tvec)
            
            # 2. Detect and track ping pong balls
            results = self.model(color_img)
            ball_detected = False
            
            if results and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                confidences = boxes.conf.cpu().numpy()
                xyxy = boxes.xyxy.cpu().numpy()
                
                best_idx = np.argmax(confidences)
                best_conf = confidences[best_idx]
                
                if best_conf >= self.conf_thresh:
                    x1, y1, x2, y2 = xyxy[best_idx].astype(int)
                    w = x2 - x1
                    h = y2 - y1
                    
                    self.bbox = (x1, y1, w, h)
                    self.tracker = cv2.TrackerCSRT_create()
                    self.tracker.init(color_img, self.bbox)
                    self.tracking = True
                    ball_detected = True
                    
                    # Get ball center in image coordinates
                    center_x = int(x1 + w/2)
                    center_y = int(y1 + h/2)
                    
                    # Map to depth image coordinates
                    depth_x = int(round(center_x * (depth_img.shape[1] / color_img.shape[1])))
                    depth_y = int(round(center_y * (depth_img.shape[0] / color_img.shape[0])))
                    
                    if 0 <= depth_x < depth_img.shape[1] and 0 <= depth_y < depth_img.shape[0]:
                        depth_value = depth_img[depth_y, depth_x]
                        if depth_value > 0:
                            # Convert depth to meters
                            Z = depth_value / 1000.0
                            
                            # Calculate X and Y using depth camera intrinsics
                            X = (depth_x - self.camera_matrix[0, 2]) * Z / self.camera_matrix[0, 0]
                            Y = (depth_y - self.camera_matrix[1, 2]) * Z / self.camera_matrix[1, 1]
                            
                            # Transform ball position to table frame if marker is detected
                            if marker_detected and self.last_marker_transform is not None:
                                marker_rvec, marker_tvec = self.last_marker_transform
                                ball_camera = np.array([X, Y, Z])
                                ball_table = self.transform_point_to_table_frame(ball_camera, marker_rvec, marker_tvec)
                                
                                # Publish ball position in table frame
                                point_msg = PointStamped()
                                point_msg.header.stamp = self.get_clock().now().to_msg()
                                point_msg.header.frame_id = "table_frame"
                                point_msg.point.x = float(ball_table[0])
                                point_msg.point.y = float(ball_table[1])
                                point_msg.point.z = float(ball_table[2])
                                self.ball_pose_pub.publish(point_msg)
                                
                                # Draw ball detection
                                cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.circle(vis_image, (center_x, center_y), 4, (0, 0, 255), -1)
                                cv2.putText(vis_image, f"Ball ({ball_table[0]:.2f}, {ball_table[1]:.2f}, {ball_table[2]:.2f})",
                                          (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            elif self.tracking and self.tracker is not None:
                success, tracked_bbox = self.tracker.update(color_img)
                if success:
                    x, y, w, h = map(int, tracked_bbox)
                    cv2.rectangle(vis_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                else:
                    self.tracking = False
                    self.tracker = None
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.header.frame_id = "camera_color_optical_frame"
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in process_frame: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Integrated Detector node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 