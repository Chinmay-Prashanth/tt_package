#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
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

def draw_table_end(image, rvec, tvec, camera_matrix, dist_coeffs):
    """Draw table tennis table end bounding box starting from marker center"""
    # Table tennis table dimensions (in meters)
    TABLE_WIDTH = 2.74  # Standard table width
    TABLE_HEIGHT = 1.525  # Standard table height
    
    # Define 3D points for table end (starting from marker)
    half_width = TABLE_WIDTH / 2
    half_height = TABLE_HEIGHT / 2
    
    # Create points for the table end plane
    # Note: The marker is at (0,0,0) and the box extends in positive Z direction
    table_points = np.float32([
        [-half_width, -half_height, 0],    # Bottom left at marker plane
        [half_width, -half_height, 0],     # Bottom right at marker plane
        [half_width, half_height, 0],      # Top right at marker plane
        [-half_width, half_height, 0],     # Top left at marker plane
        [-half_width, -half_height, 2.74], # Bottom left at end
        [half_width, -half_height, 2.74],  # Bottom right at end
        [half_width, half_height, 2.74],   # Top right at end
        [-half_width, half_height, 2.74],  # Top left at end
        [0, 0, 0],                         # Marker center
        [0, 0, 0.2]                        # Normal vector point
    ])
    
    # Project 3D points to image plane
    img_points, _ = cv2.projectPoints(table_points, rvec, tvec, camera_matrix, dist_coeffs)
    img_points = img_points.astype(np.int32)
    
    # Draw the front rectangle (at marker)
    cv2.polylines(image, [img_points[:4]], True, (0, 255, 255), 2)  # Yellow color
    
    # Draw the back rectangle (at end)
    cv2.polylines(image, [img_points[4:8]], True, (0, 255, 255), 2)  # Yellow color
    
    # Draw the connecting lines
    for i in range(4):
        cv2.line(image,
                tuple(img_points[i][0]),
                tuple(img_points[i+4][0]),
                (0, 255, 255), 1)
    
    # Draw normal vector (blue line showing orientation)
    cv2.line(image, 
            tuple(img_points[8][0]),  # Center point
            tuple(img_points[9][0]),  # Normal vector point
            (255, 0, 0), 2)          # Blue color
    
    # Draw center lines on front face
    center = img_points[8][0]  # Use marker center
    
    # Calculate corners in image coordinates for front face
    left = img_points[0][0][0]
    right = img_points[1][0][0]
    top = img_points[3][0][1]
    bottom = img_points[0][0][1]
    
    # Draw horizontal and vertical center lines on front face
    cv2.line(image, (left, int(center[1])), (right, int(center[1])), (0, 255, 255), 1)  # Horizontal
    cv2.line(image, (int(center[0]), top), (int(center[0]), bottom), (0, 255, 255), 1)  # Vertical
    
    return image

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize the ArUco dictionary (using 5x5_1000 to support ID 189)
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
        self.aruco_params.cornerRefinementWinSize = 9  # Increased for better corner detection
        self.aruco_params.cornerRefinementMaxIterations = 50  # Increased for better accuracy
        self.aruco_params.cornerRefinementMinAccuracy = 0.05  # More strict corner refinement
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera matrix and distortion coefficients (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Create subscribers
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)
        
        # Create publishers
        self.marker_pose_pub = self.create_publisher(
            PoseArray,
            'aruco_poses',
            10)
        
        self.debug_image_pub = self.create_publisher(
            Image,
            'aruco_debug_image',
            10)
        
        # Initialize transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Marker size in meters
        self.marker_size = 0.20  # 20cm marker size
        
        # Initialize pose filtering with larger history
        self.pose_history = deque(maxlen=30)  # Increased history size for more stability
        self.last_stable_pose = None
        self.initial_wall_normal = None
        
        # Filtering parameters (adjusted for wall-mounted marker)
        self.translation_threshold = 0.03  # 3cm threshold for position changes
        self.rotation_threshold = 0.15  # ~8.6 degrees threshold for rotation changes
        self.alpha = 0.95  # Very high weight for current stable pose
        self.wall_normal_tolerance = 0.2  # Tolerance for wall normal direction changes
        
        self.get_logger().info('ArUco detector node initialized with DICT_5X5_1000')
        self.get_logger().info(f'Marker size set to: {self.marker_size} meters')
        self.get_logger().info('Looking for marker ID 189')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters received')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Distortion coefficients: {self.dist_coeffs}')

    def stabilize_wall_orientation(self, rvec, tvec):
        """Stabilize orientation for wall-mounted marker"""
        rot_mat, _ = cv2.Rodrigues(rvec)
        
        # Extract the normal vector (Z-axis of the marker)
        normal = rot_mat[:, 2]
        
        if self.initial_wall_normal is None:
            self.initial_wall_normal = normal
            return rvec
        
        # Check if the normal has changed significantly
        normal_diff = np.arccos(np.clip(np.dot(normal, self.initial_wall_normal), -1.0, 1.0))
        
        if abs(normal_diff) < self.wall_normal_tolerance:
            # Use the initial wall normal
            z_axis = self.initial_wall_normal
        else:
            # Gradually adjust the normal
            z_axis = 0.95 * self.initial_wall_normal + 0.05 * normal
            z_axis = z_axis / np.linalg.norm(z_axis)
        
        # Reconstruct rotation matrix with stabilized normal
        y_axis = np.array([0, 1, 0])  # Assume roughly vertical alignment
        x_axis = np.cross(y_axis, z_axis)
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
        
        # Stabilize orientation for wall-mounted marker
        rvec = self.stabilize_wall_orientation(rvec, tvec)
        
        # Calculate the difference from last stable pose
        t_diff = np.linalg.norm(tvec - self.last_stable_pose[0])
        r_diff = np.linalg.norm(rvec - self.last_stable_pose[1])
        
        # If change is smaller than threshold, use heavily weighted average
        if t_diff < self.translation_threshold and r_diff < self.rotation_threshold:
            filtered_tvec = self.alpha * self.last_stable_pose[0] + (1 - self.alpha) * tvec
            filtered_rvec = self.alpha * self.last_stable_pose[1] + (1 - self.alpha) * rvec
        else:
            # For larger changes, use weighted moving average with more history
            if len(self.pose_history) > 0:
                # Calculate weighted average of recent poses with exponential weights
                weights = np.exp(np.linspace(-2, 0, len(self.pose_history)))
                weights = weights / np.sum(weights)
                
                recent_tvecs = np.array([pose[0] for pose in self.pose_history])
                recent_rvecs = np.array([pose[1] for pose in self.pose_history])
                
                avg_tvec = np.average(recent_tvecs, axis=0, weights=weights)
                avg_rvec = np.average(recent_rvecs, axis=0, weights=weights)
                
                # Heavily weight the average for smoother transitions
                filtered_tvec = 0.8 * avg_tvec + 0.2 * tvec
                filtered_rvec = 0.8 * avg_rvec + 0.2 * rvec
            else:
                filtered_tvec = tvec
                filtered_rvec = rvec
        
        # Update history and stable pose
        self.pose_history.append((filtered_tvec, filtered_rvec))
        self.last_stable_pose = (filtered_tvec.copy(), filtered_rvec.copy())
        
        return filtered_tvec, filtered_rvec

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('Waiting for camera calibration parameters...')
            return

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, _ = self.detector.detectMarkers(gray)
            
            # Create clean visualization image
            vis_image = cv_image.copy()
            
            if ids is not None:
                # Find marker 189
                marker_idx = np.where(ids == 189)[0]
                
                if len(marker_idx) > 0:
                    idx = marker_idx[0]
                    
                    # Estimate pose
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[idx]],
                        self.marker_size,
                        self.camera_matrix,
                        self.dist_coeffs
                    )
                    
                    # Apply temporal filtering
                    tvec, rvec = self.filter_pose(tvecs[0][0], rvecs[0][0])
                    
                    # Convert rotation vector to matrix
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    
                    # Convert to quaternion
                    w, x, y, z = rotation_matrix_to_quaternion(rot_mat)
                    
                    # Create and publish transform
                    transform = TransformStamped()
                    transform.header = msg.header
                    transform.header.frame_id = "camera_color_optical_frame"
                    transform.child_frame_id = 'aruco_marker_189'
                    transform.transform.translation.x = tvec[0]
                    transform.transform.translation.y = tvec[1]
                    transform.transform.translation.z = tvec[2]
                    transform.transform.rotation.w = w
                    transform.transform.rotation.x = x
                    transform.transform.rotation.y = y
                    transform.transform.rotation.z = z
                    
                    self.tf_broadcaster.sendTransform(transform)
                    
                    # Draw marker and axes
                    cv2.aruco.drawDetectedMarkers(vis_image, [corners[idx]], np.array([[189]]))
                    cv2.drawFrameAxes(
                        vis_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        0.1
                    )
                    
                    # Draw table end bounding box
                    vis_image = draw_table_end(vis_image, rvec, tvec, 
                                            self.camera_matrix, self.dist_coeffs)
                    
                    # Add distance measurement
                    distance = np.linalg.norm(tvec)
                    cv2.putText(vis_image, f'ID: 189 ({distance:.2f}m)', 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                              1, (0, 255, 0), 2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            debug_msg.header = msg.header
            debug_msg.header.frame_id = "camera_color_optical_frame"
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 