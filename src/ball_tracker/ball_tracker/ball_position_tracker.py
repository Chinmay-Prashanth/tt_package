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

class BallPositionTracker(Node):
    def __init__(self):
        super().__init__('ball_position_tracker')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize the ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Optimize detector parameters for RealSense D455 and stability
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
        self.marker_size = 0.15  # 15cm marker size
        
        # Enhanced pose filtering parameters for better stability
        self.pose_history = deque(maxlen=60)  # Increased history size (2 seconds at 30 FPS)
        self.last_stable_pose = None
        self.initial_wall_normal = None
        self.translation_threshold = 0.02  # Reduced threshold for more stability (2cm)
        self.rotation_threshold = 0.1  # Reduced threshold (~5.7 degrees)
        self.alpha = 0.98  # Increased weight for current stable pose
        self.wall_normal_tolerance = 0.15  # Reduced tolerance for more stable orientation
        
        # FPS monitoring
        self.last_frame_time = self.get_clock().now()
        self.fps_history = deque(maxlen=30)
        
        # Ball tracking parameters
        self.conf_thresh = 0.3
        self.tracking = False
        self.tracker = None
        self.bbox = (0, 0, 0, 0)
        
        # Enhanced ball position filtering
        self.ball_position_history = deque(maxlen=15)  # Increased history for better smoothing
        self.position_weights = None  # Will be initialized when history is filled
        self.min_depth_confidence = 0.7  # Minimum confidence for depth measurements
        self.position_threshold = 0.05  # 5cm threshold for position jumps
        self.last_valid_position = None
        
        # Status tracking
        self.tracking_status = "No Ball Detected"
        self.detection_confidence = 0.0
        self.relative_box_position = None  # Position relative to box coordinates
        
        # Add prediction parameters
        self.g = -9.81  # gravity acceleration
        self.e = 0.87   # coefficient of restitution
        self.ball_radius = 0.02  # 2cm radius
        self.table_height = 0.76  # table height in meters
        self.table_length = 2.74  # table length in meters
        self.table_width = 1.525  # table width in meters
        
        # Prediction tracking
        self.last_ball_pos = None
        self.last_ball_vel = None
        self.last_ball_time = None
        self.predicted_positions = []
        
        # Create subscribers with larger queue sizes
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 30)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 30)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 30)
        
        # Create publishers with larger queue sizes
        self.debug_image_pub = self.create_publisher(Image, 'ball_tracking_visualization', 30)
        self.ball_position_pub = self.create_publisher(PointStamped, 'ball_position', 30)
        self.initial_position_pub = self.create_publisher(PointStamped, 'ball_initial_position', 30)
        self.predicted_trajectory_pub = self.create_publisher(
            PointStamped, 'predicted_ball_position', 30)
        
        # Processing timer (strict 30 Hz)
        period = 1.0/30.0  # Exactly 30 Hz
        self.timer = self.create_timer(period, self.process_frame)
        
        self.get_logger().info('Ball position tracker initialized at 30 Hz')

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

    def draw_marker_info(self, image, corners, rvec, tvec):
        """Draw marker center, axes, and orientation info"""
        # Draw marker outline
        corners_array = np.array(corners[0])  # Convert to numpy array
        cv2.aruco.drawDetectedMarkers(image, [corners_array], np.array([[189]]))
        
        # Draw coordinate axes
        cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        
        # Draw marker center - fix the center point calculation
        center = np.mean(corners_array.reshape(-1, 2), axis=0).astype(int)
        cv2.circle(image, (center[0], center[1]), 5, (0, 0, 255), -1)
        cv2.putText(image, "Marker Center", (center[0] + 10, center[1]),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw distance from camera
        distance = np.linalg.norm(tvec)
        cv2.putText(image, f"Marker dist: {distance:.2f}m", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return image

    def transform_point_to_marker(self, point_camera, marker_rvec, marker_tvec):
        """Transform a point from camera frame to marker frame"""
        R_marker_to_camera, _ = cv2.Rodrigues(marker_rvec)
        
        # Ensure marker_tvec is a 3D vector
        marker_tvec = marker_tvec.reshape(3, 1)
        
        # Create transformation matrix from camera to marker
        T_camera_to_marker = np.eye(4)
        T_camera_to_marker[:3, :3] = R_marker_to_camera.T
        T_camera_to_marker[:3, 3] = (-R_marker_to_camera.T @ marker_tvec).flatten()
        
        # Transform point to marker frame
        point_homogeneous = np.append(point_camera, 1)
        point_marker = T_camera_to_marker @ point_homogeneous
        
        return point_marker[:3]

    def check_initial_position(self, ball_position):
        """Check if ball is at initial position (0.1m from marker)"""
        distance = np.linalg.norm(ball_position)
        return abs(distance - 0.1) < 0.02  # 2cm tolerance

    def draw_table_box(self, image, rvec, tvec):
        """Draw table tennis table end bounding box starting from marker"""
        # Table tennis table dimensions (in meters)
        TABLE_WIDTH = 1.5  # Increased width for better scale
        TABLE_HEIGHT = 1.0  # Increased height for better scale
        BOX_DEPTH = 2.0    # Increased depth for better perspective
        
        # Define 3D points for box
        half_width = TABLE_WIDTH / 2
        half_height = TABLE_HEIGHT / 2
        
        # Create points for box (centered exactly on marker)
        box_points = np.float32([
            # Front face (centered on marker)
            [-half_width, -half_height, 0],    # Bottom left
            [half_width, -half_height, 0],     # Bottom right
            [half_width, half_height, 0],      # Top right
            [-half_width, half_height, 0],     # Top left
            
            # Back face
            [-half_width, -half_height, BOX_DEPTH], # Bottom left
            [half_width, -half_height, BOX_DEPTH],  # Bottom right
            [half_width, half_height, BOX_DEPTH],   # Top right
            [-half_width, half_height, BOX_DEPTH],  # Top left
            
            # Center points
            [0, 0, 0],     # Marker center
            [0, 0, BOX_DEPTH]  # Back center
        ])
        
        try:
            # Project 3D points to image plane
            img_points, _ = cv2.projectPoints(box_points, rvec, tvec, 
                                           self.camera_matrix, self.dist_coeffs)
            
            # Ensure points are within image bounds
            h, w = image.shape[:2]
            img_points = np.clip(img_points, -1e6, 1e6)
            img_points = img_points.reshape(-1, 2)
            
            # Convert to integer coordinates
            img_points = np.clip(img_points, 0, [w-1, h-1]).astype(np.int32)
            
            # Get marker center (this is the actual ArUco marker center)
            marker_center = img_points[8].astype(int)
            
            # Draw front face (yellow)
            cv2.polylines(image, [img_points[:4]], True, (0, 255, 255), 2)
            
            # Draw back face (yellow, slightly transparent)
            cv2.polylines(image, [img_points[4:8]], True, (0, 200, 200), 1)
            
            # Draw connecting lines between faces
            for i in range(4):
                # Dashed lines for depth perspective
                pt1 = tuple(img_points[i])
                pt2 = tuple(img_points[i+4])
                
                # Draw dashed lines by creating multiple segments
                num_segments = 20
                for j in range(num_segments):
                    if j % 2 == 0:  # Draw only even segments
                        start_pt = (int(pt1[0] + (pt2[0] - pt1[0]) * j / num_segments),
                                  int(pt1[1] + (pt2[1] - pt1[1]) * j / num_segments))
                        end_pt = (int(pt1[0] + (pt2[0] - pt1[0]) * (j + 1) / num_segments),
                                 int(pt1[1] + (pt2[1] - pt1[1]) * (j + 1) / num_segments))
                        cv2.line(image, start_pt, end_pt, (0, 255, 255), 1)
            
            # Draw marker center (red cross)
            size = 15
            cv2.line(image, 
                    (marker_center[0] - size, marker_center[1]),
                    (marker_center[0] + size, marker_center[1]),
                    (0, 0, 255), 2)
            cv2.line(image, 
                    (marker_center[0], marker_center[1] - size),
                    (marker_center[0], marker_center[1] + size),
                    (0, 0, 255), 2)
            
            # Draw center line connecting front to back (green)
            cv2.line(image, 
                    tuple(marker_center),
                    tuple(img_points[9].astype(int)),
                    (0, 255, 0), 1)
            
            # Draw box dimensions
            cv2.putText(image, f"Box: {TABLE_WIDTH}x{TABLE_HEIGHT}x{BOX_DEPTH}m", 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
            # Draw coordinate axes at marker center
            cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs, 
                            rvec, tvec, 0.3)  # Increased axis size
            
        except Exception as e:
            self.get_logger().warn(f'Error drawing box: {str(e)}')
        
        return image

    def stabilize_wall_orientation(self, rvec, tvec):
        """Stabilize orientation for wall-mounted marker to ensure Z-axis points towards camera"""
        # The camera's viewing direction in camera coordinates (points out from camera)
        camera_view = np.array([0, 0, 1])
        
        # Get current rotation matrix
        rot_mat, _ = cv2.Rodrigues(rvec)
        
        # We want the marker's Z-axis to point towards the camera
        # So it should be opposite to the camera's viewing direction
        z_axis = -camera_view  # This ensures the blue axis points towards the camera
        
        # Use the world's up direction as a reference for Y
        world_up = np.array([0, 1, 0])
        
        # Calculate X axis as cross product of world up and Z axis
        x_axis = np.cross(world_up, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Calculate Y axis to ensure right-handed coordinate system
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Construct new rotation matrix with Z pointing towards camera
        stabilized_rot = np.column_stack((x_axis, y_axis, z_axis))
        
        # Convert back to rotation vector
        stabilized_rvec, _ = cv2.Rodrigues(stabilized_rot)
        
        return stabilized_rvec.flatten()

    def filter_pose(self, tvec, rvec):
        """Apply enhanced temporal filtering to reduce pose jitter"""
        if self.last_stable_pose is None:
            self.last_stable_pose = (tvec.copy(), rvec.copy())
            self.pose_history.append((tvec, rvec))
            return tvec, rvec
        
        # Stabilize orientation for wall-mounted marker
        rvec = self.stabilize_wall_orientation(rvec, tvec)
        
        # Calculate the difference from last stable pose
        t_diff = np.linalg.norm(tvec - self.last_stable_pose[0])
        r_diff = np.linalg.norm(rvec - self.last_stable_pose[1])
        
        # Very strong filtering for better stability
        if t_diff < self.translation_threshold and r_diff < self.rotation_threshold:
            # Extremely strong smoothing for small changes
            filtered_tvec = 0.99 * self.last_stable_pose[0] + 0.01 * tvec
            filtered_rvec = 0.99 * self.last_stable_pose[1] + 0.01 * rvec
        else:
            # For larger changes, use weighted moving average with more history
            if len(self.pose_history) > 0:
                # Use exponential weighting with stronger decay
                weights = np.exp(np.linspace(-4, 0, len(self.pose_history)))
                weights = weights / np.sum(weights)
                
                recent_tvecs = np.array([pose[0] for pose in self.pose_history])
                recent_rvecs = np.array([pose[1] for pose in self.pose_history])
                
                # Weighted average with more weight on history
                avg_tvec = np.average(recent_tvecs, axis=0, weights=weights)
                avg_rvec = np.average(recent_rvecs, axis=0, weights=weights)
                
                # Stronger filtering
                filtered_tvec = 0.95 * avg_tvec + 0.05 * tvec
                filtered_rvec = 0.95 * avg_rvec + 0.05 * rvec
            else:
                filtered_tvec = tvec
                filtered_rvec = rvec
        
        # Update history and stable pose
        self.pose_history.append((filtered_tvec, filtered_rvec))
        self.last_stable_pose = (filtered_tvec.copy(), filtered_rvec.copy())
        
        return filtered_tvec, filtered_rvec

    def filter_ball_position(self, new_position):
        """Apply enhanced filtering to ball position"""
        if self.last_valid_position is None:
            self.last_valid_position = new_position
            return new_position
        
        # Check for unrealistic jumps
        position_diff = np.linalg.norm(new_position - self.last_valid_position)
        if position_diff > self.position_threshold:
            # Use weighted average if jump is too large
            filtered_position = 0.9 * self.last_valid_position + 0.1 * new_position
        else:
            # Add to history
            self.ball_position_history.append(new_position)
            
            if len(self.ball_position_history) >= 5:
                # Create exponential weights if not initialized
                if self.position_weights is None or len(self.position_weights) != len(self.ball_position_history):
                    weights = np.exp(np.linspace(-2, 0, len(self.ball_position_history)))
                    self.position_weights = weights / np.sum(weights)
                
                # Calculate weighted average of recent positions
                positions = np.array(self.ball_position_history)
                filtered_position = np.average(positions, axis=0, weights=self.position_weights)
            else:
                filtered_position = new_position
        
        self.last_valid_position = filtered_position
        return filtered_position

    def calculate_box_relative_position(self, ball_position):
        """Calculate ball position relative to the box coordinates"""
        # Box dimensions
        TABLE_WIDTH = 2.74
        TABLE_HEIGHT = 1.525
        
        # Calculate relative position in percentage
        x_rel = (ball_position[0] + TABLE_WIDTH/2) / TABLE_WIDTH * 100
        y_rel = (ball_position[1] + TABLE_HEIGHT/2) / TABLE_HEIGHT * 100
        z_rel = ball_position[2] / TABLE_WIDTH * 100
        
        return x_rel, y_rel, z_rel

    def draw_status_info(self, image, ball_position=None, confidence=0.0):
        """Draw enhanced status information on image"""
        # Draw FPS
        avg_fps = np.mean(self.fps_history) if len(self.fps_history) > 0 else 0
        cv2.putText(image, f"FPS: {avg_fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw tracking status
        cv2.putText(image, f"Status: {self.tracking_status}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw confidence
        cv2.putText(image, f"Confidence: {confidence:.2f}", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        if ball_position is not None and self.relative_box_position is not None:
            x_rel, y_rel, z_rel = self.relative_box_position
            
            # Draw absolute position
            cv2.putText(image, f"Ball Position (m):", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(image, f"X: {ball_position[0]:.3f}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(image, f"Y: {ball_position[1]:.3f}", (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(image, f"Z: {ball_position[2]:.3f}", (10, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw relative position
            cv2.putText(image, f"Relative Position (%):", (image.shape[1]-200, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(image, f"Width: {x_rel:.1f}%", (image.shape[1]-200, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(image, f"Height: {y_rel:.1f}%", (image.shape[1]-200, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            cv2.putText(image, f"Depth: {z_rel:.1f}%", (image.shape[1]-200, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        return image

    def predict_trajectory(self, pos, vel, dt):
        """Predict ball trajectory using physics model"""
        # Initial conditions
        x0, y0, z0 = pos
        vx0, vy0, vz0 = vel
        
        # Predict next position
        x = x0 + vx0 * dt
        y = y0 + vy0 * dt
        z = z0 + vz0 * dt + 0.5 * self.g * dt * dt
        
        # Predict next velocity
        vx = vx0
        vy = vy0
        vz = vz0 + self.g * dt
        
        # Check for table bounce
        if z < self.table_height + self.ball_radius and vz < 0:
            # Bounce with coefficient of restitution
            vz = -vz * self.e
            z = self.table_height + self.ball_radius
        
        return np.array([x, y, z]), np.array([vx, vy, vz])

    def calculate_velocity(self, current_pos, current_time):
        """Calculate ball velocity from positions and timestamps"""
        if (self.last_ball_pos is not None and 
            self.last_ball_time is not None and 
            current_time > self.last_ball_time):
            
            dt = (current_time - self.last_ball_time).nanoseconds / 1e9
            if dt > 0:
                velocity = (current_pos - self.last_ball_pos) / dt
                return velocity
        return None

    def process_frame(self):
        # Calculate FPS
        current_time = self.get_clock().now()
        dt = (current_time - self.last_frame_time).nanoseconds / 1e9
        fps = 1.0 / dt if dt > 0 else 0
        self.fps_history.append(fps)
        self.last_frame_time = current_time
        
        # Skip if we don't have all required data
        if self.latest_color is None or self.latest_depth is None or self.camera_matrix is None:
            return

        try:
            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            vis_image = color_img.copy()
            
            # Calculate and display average FPS
            avg_fps = np.mean(self.fps_history) if len(self.fps_history) > 0 else 0
            cv2.putText(vis_image, f"FPS: {avg_fps:.1f}", (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 1. Detect ArUco marker
            gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            marker_detected = False
            if ids is not None and len(corners) > 0:
                marker_idx = np.where(ids == 189)[0]
                if len(marker_idx) > 0:
                    marker_detected = True
                    idx = marker_idx[0]
                    
                    # Get marker pose
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[idx:idx+1], self.marker_size, self.camera_matrix, self.dist_coeffs)
                    
                    # Apply temporal filtering to stabilize pose
                    tvec, rvec = self.filter_pose(tvecs[0], rvecs[0])
                    
                    # Draw marker information
                    vis_image = self.draw_marker_info(vis_image, [corners[idx]], rvec, tvec)
                    
                    # Draw table and box visualization
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
                    self.tracking_status = "Ball Detected"
                    self.detection_confidence = best_conf
                    
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
                            
                            # Transform ball position to marker frame if marker is detected
                            if marker_detected:
                                ball_camera = np.array([X, Y, Z])
                                ball_marker = self.transform_point_to_marker(ball_camera, rvec, tvec)
                                
                                # Apply enhanced filtering
                                filtered_position = self.filter_ball_position(ball_marker)
                                
                                # Calculate relative position
                                self.relative_box_position = self.calculate_box_relative_position(filtered_position)
                                
                                # Update visualization and publish
                                point_msg = PointStamped()
                                point_msg.header.stamp = self.get_clock().now().to_msg()
                                point_msg.header.frame_id = "marker_frame"
                                point_msg.point.x = float(filtered_position[0])
                                point_msg.point.y = float(filtered_position[1])
                                point_msg.point.z = float(filtered_position[2])
                                self.ball_position_pub.publish(point_msg)
                                
                                # Draw enhanced status information
                                vis_image = self.draw_status_info(vis_image, filtered_position, best_conf)
                                
                                # Draw ball detection with coordinates
                                cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.circle(vis_image, (center_x, center_y), 4, (0, 0, 255), -1)
                                cv2.putText(vis_image, 
                                          f"Ball pos: ({filtered_position[0]:.2f}, {filtered_position[1]:.2f}, {filtered_position[2]:.2f})",
                                          (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                                # Calculate velocity
                                velocity = self.calculate_velocity(filtered_position, current_time)
                                
                                if velocity is not None:
                                    # Store current state
                                    self.last_ball_vel = velocity
                                    
                                    # Predict future positions
                                    predicted_pos = filtered_position.copy()
                                    predicted_vel = velocity.copy()
                                    self.predicted_positions = []
                                    
                                    # Predict next 30 frames (1 second at 30 FPS)
                                    for i in range(30):
                                        dt = 1.0/30.0  # 30 FPS
                                        predicted_pos, predicted_vel = self.predict_trajectory(
                                            predicted_pos, predicted_vel, dt)
                                        
                                        # Store prediction
                                        self.predicted_positions.append(predicted_pos.copy())
                                        
                                        # Publish predicted position
                                        pred_msg = PointStamped()
                                        pred_msg.header.stamp = current_time.to_msg()
                                        pred_msg.header.frame_id = "marker_frame"
                                        pred_msg.point.x = float(predicted_pos[0])
                                        pred_msg.point.y = float(predicted_pos[1])
                                        pred_msg.point.z = float(predicted_pos[2])
                                        self.predicted_trajectory_pub.publish(pred_msg)
                                
                                # Update last position and time
                                self.last_ball_pos = filtered_position
                                self.last_ball_time = current_time
                
            elif self.tracking and self.tracker is not None:
                success, tracked_bbox = self.tracker.update(color_img)
                if success:
                    self.tracking_status = "Tracking"
                    x, y, w, h = map(int, tracked_bbox)
                    cv2.rectangle(vis_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                else:
                    self.tracking_status = "Lost Track"
                    self.tracking = False
                    self.tracker = None
            else:
                self.tracking_status = "No Ball Detected"
                self.detection_confidence = 0.0
                self.relative_box_position = None
            
            # Always draw status info even if no ball is detected
            vis_image = self.draw_status_info(vis_image)
            
            # Publish visualization
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
    node = BallPositionTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ball Position Tracker node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 