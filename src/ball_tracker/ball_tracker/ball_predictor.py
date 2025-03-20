#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import deque

class ExtendedKalmanFilter:
    def __init__(self, dt=1/120.0):  # High frequency for precision
        # State vector: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)
        
        # Optimized uncertainties for accuracy
        self.P = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])  # Increased initial uncertainty
        
        # Process noise - increased to allow for more dynamic motion
        self.Q = np.diag([0.001, 0.001, 0.001, 0.01, 0.01, 0.01])
        
        # Measurement noise - decreased to trust measurements more
        self.R = np.diag([0.0005, 0.0005, 0.0005])
        
        # Gravity
        self.g = -9.81
        
        # Time step
        self.dt = dt
        
        # Physics parameters
        self.e = 0.87  # coefficient of restitution
        self.table_height = 0.76  # table height (m)
        self.ball_radius = 0.02  # ball radius (m)
    
    def predict(self):
        """Predict next state based on physical model"""
        # Extract current state
        x, y, z, vx, vy, vz = self.state
        dt = self.dt
        
        # Apply motion model
        # Position update
        x_pred = x + vx * dt
        y_pred = y + vy * dt
        z_pred = z + vz * dt + 0.5 * self.g * dt**2
        
        # Velocity update
        vx_pred = vx
        vy_pred = vy
        vz_pred = vz + self.g * dt
        
        # Check for table bounce
        if z_pred < self.table_height + self.ball_radius and vz_pred < 0:
            # Bounce with coefficient of restitution
            vz_pred = -vz_pred * self.e
            z_pred = self.table_height + self.ball_radius
        
        # New state prediction
        self.state = np.array([x_pred, y_pred, z_pred, vx_pred, vy_pred, vz_pred])
        
        # Compute Jacobian of motion model
        F = np.eye(6)
        F[0, 3] = dt  # dx/dvx
        F[1, 4] = dt  # dy/dvy
        F[2, 5] = dt  # dz/dvz
        
        # Update covariance with process noise
        self.P = F @ self.P @ F.T + self.Q
        
        return self.state.copy()
    
    def update(self, measurement):
        """Update state based on measurement"""
        # Measurement matrix (we observe position but not velocity)
        H = np.zeros((3, 6))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # z
        
        # Calculate innovation (measurement residual)
        y = measurement - H @ self.state
        
        # Calculate innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Calculate Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P
        
        return self.state.copy()
    
    def predict_trajectory(self, steps):
        """Predict future trajectory for multiple steps ahead"""
        predictions = []
        
        # Start from current state
        curr_state = self.state.copy()
        
        for _ in range(steps):
            # Apply motion model
            x, y, z, vx, vy, vz = curr_state
            dt = self.dt
            
            # Position update
            x_pred = x + vx * dt
            y_pred = y + vy * dt
            z_pred = z + vz * dt + 0.5 * self.g * dt**2
            
            # Velocity update
            vx_pred = vx
            vy_pred = vy
            vz_pred = vz + self.g * dt
            
            # Check for table bounce
            if z_pred < self.table_height + self.ball_radius and vz_pred < 0:
                # Bounce with coefficient of restitution
                vz_pred = -vz_pred * self.e
                z_pred = self.table_height + self.ball_radius
            
            # Update current state for next iteration
            curr_state = np.array([x_pred, y_pred, z_pred, vx_pred, vy_pred, vz_pred])
            
            # Store position prediction
            predictions.append(curr_state[:3].copy())
        
        return np.array(predictions)
    
    def find_target_distance_intersection(self, target_distance, max_steps=100):
        """Find when trajectory reaches target distance from origin"""
        predictions = self.predict_trajectory(max_steps)
        
        # Calculate distance from origin for each prediction
        distances = np.linalg.norm(predictions, axis=1)
        
        # Find closest match to target distance
        idx = np.argmin(np.abs(distances - target_distance))
        
        if abs(distances[idx] - target_distance) < 0.05:  # 5cm tolerance
            return predictions[idx], idx * self.dt
        
        return None, None

class BallPredictor(Node):
    def __init__(self):
        super().__init__('ball_predictor')
        
        # Physics parameters
        self.g = -9.81  # gravity acceleration (m/s^2)
        self.e = 0.87   # coefficient of restitution
        self.ball_radius = 0.02  # ball radius (m)
        
        # Table dimensions
        self.table_height = 0.76  # table height (m)
        self.table_length = 2.74  # table length (m)
        self.table_width = 1.525  # table width (m)
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Initialize EKF
        self.ekf = ExtendedKalmanFilter(dt=1/60.0)  # 60Hz internal update rate
        
        # Prediction parameters
        self.target_distance = 0.2  # Target distance from camera (m)
        self.prediction_tolerance = 0.02  # 2cm tolerance for predictions
        self.max_prediction_steps = 120  # 2 seconds at 60Hz
        
        # State tracking
        self.last_pos = None
        self.last_vel = None
        self.last_time = None
        self.pos_history = deque(maxlen=30)  # Keep last 30 positions
        self.time_history = deque(maxlen=30)  # Corresponding timestamps
        
        # Trajectory visualization
        self.predicted_trajectory = None
        self.trajectory_colors = None
        
        # Visualization parameters
        self.vis_image = None
        self.latest_color_image = None
        self.color_image_lock = False
        
        # Debug counters
        self.total_callbacks = 0
        self.total_predictions = 0
        self.last_status_time = self.get_clock().now()
        
        # Create subscribers
        self.ball_pos_sub = self.create_subscription(
            PointStamped,
            'ball_position',
            self.ball_position_callback,
            10)
        
        self.color_img_sub = self.create_subscription(
            Image,
            'ball_tracking_visualization',
            self.color_image_callback,
            10)
        
        # Create publishers
        self.predicted_pos_pub = self.create_publisher(
            PointStamped,
            'predicted_ball_position',
            10)
        
        self.vis_image_pub = self.create_publisher(
            Image,
            'prediction_visualization',
            10)
        
        # Create timers
        self.prediction_timer = self.create_timer(1.0/30.0, self.predict_callback)
        self.visualization_timer = self.create_timer(1.0/30.0, self.visualization_callback)
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        self.get_logger().info('Ball predictor node initialized with EKF')
        self.get_logger().info(f'Target prediction distance: {self.target_distance}m')

    def ball_position_callback(self, msg):
        """Handle incoming ball position messages"""
        self.total_callbacks += 1
        current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        current_time = self.get_clock().now()
        
        # Log incoming position
        self.get_logger().info(f'Received ball position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})')
        
        # Add to history
        self.pos_history.append(current_pos)
        self.time_history.append(current_time)
        
        # Update EKF with new measurement
        if len(self.pos_history) >= 2:
            # If this is our second measurement, initialize velocity
            if len(self.pos_history) == 2:
                dt = (self.time_history[1] - self.time_history[0]).nanoseconds / 1e9
                if dt > 0:
                    velocity = (self.pos_history[1] - self.pos_history[0]) / dt
                    # Initialize EKF state with position and initial velocity
                    initial_state = np.concatenate([current_pos, velocity])
                    self.ekf.state = initial_state
                    self.get_logger().info(f'EKF initialized with velocity: {velocity}')
            
            # Update the filter
            self.ekf.update(current_pos)
            
            # Store the current state
            self.last_pos = self.ekf.state[:3]
            self.last_vel = self.ekf.state[3:6]
            self.last_time = current_time
            
            # Log EKF state after update
            self.get_logger().info(
                f'EKF State - Pos: ({self.last_pos[0]:.3f}, {self.last_pos[1]:.3f}, {self.last_pos[2]:.3f}), '
                f'Vel: ({self.last_vel[0]:.3f}, {self.last_vel[1]:.3f}, {self.last_vel[2]:.3f})'
            )
        else:
            # First measurement, just store it
            self.last_pos = current_pos
            self.last_time = current_time
            self.get_logger().info('Received first ball position')

    def color_image_callback(self, msg):
        """Handle incoming color image messages"""
        if not self.color_image_lock:
            try:
                self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f"Color image conversion error: {e}")

    def predict_callback(self):
        """Generate and publish predictions using EKF"""
        current_time = self.get_clock().now()
        
        # Skip if we don't have initialized EKF
        if self.last_pos is None or self.last_time is None:
            # Publish null prediction
            pred_msg = PointStamped()
            pred_msg.header.stamp = current_time.to_msg()
            pred_msg.header.frame_id = "marker_frame"
            pred_msg.point.x = 0.0
            pred_msg.point.y = 0.0
            pred_msg.point.z = 0.0
            self.predicted_pos_pub.publish(pred_msg)
            return
        
        # If we have an initialized EKF, use it to predict
        if len(self.pos_history) >= 2:
            # Run EKF prediction to advance internal state
            predicted_state = self.ekf.predict()
            
            # Generate trajectory prediction
            self.predicted_trajectory = self.ekf.predict_trajectory(self.max_prediction_steps)
            
            # Find point at target distance
            target_pos, target_time = self.ekf.find_target_distance_intersection(
                self.target_distance, self.max_prediction_steps)
            
            # Generate color gradient for trajectory visualization
            if self.predicted_trajectory is not None and len(self.predicted_trajectory) > 0:
                self.trajectory_colors = []
                for i in range(len(self.predicted_trajectory)):
                    # Green to red gradient based on time
                    ratio = i / len(self.predicted_trajectory)
                    r = int(255 * ratio)
                    g = int(255 * (1 - ratio))
                    b = 0
                    self.trajectory_colors.append((b, g, r))
            
            # Publish prediction point
            pred_msg = PointStamped()
            pred_msg.header.stamp = current_time.to_msg()
            pred_msg.header.frame_id = "marker_frame"
            
            if target_pos is not None:
                self.total_predictions += 1
                pred_msg.point.x = float(target_pos[0])
                pred_msg.point.y = float(target_pos[1])
                pred_msg.point.z = float(target_pos[2])
                
                # Log prediction
                distance = np.linalg.norm(target_pos)
                self.get_logger().info(
                    f'Prediction at {distance:.3f}m: '
                    f'({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}) '
                    f'in {target_time:.3f}s'
                )
            else:
                # If no intersection found, publish predicted state position
                pred_msg.point.x = float(predicted_state[0])
                pred_msg.point.y = float(predicted_state[1])
                pred_msg.point.z = float(predicted_state[2])
                self.get_logger().info(
                    f'No target distance intersection found, publishing predicted state: '
                    f'({predicted_state[0]:.3f}, {predicted_state[1]:.3f}, {predicted_state[2]:.3f})'
                )
            
            self.predicted_pos_pub.publish(pred_msg)

    def visualization_callback(self):
        """Create and publish visualization of current tracking and predictions"""
        if self.latest_color_image is None:
            return
        
        # Lock color image to prevent updates during processing
        self.color_image_lock = True
        
        try:
            # Copy the image for visualization
            vis_image = self.latest_color_image.copy()
            
            # Draw history points
            if len(self.pos_history) > 1:
                for i in range(1, len(self.pos_history)):
                    # Draw past trajectory (green points, fading with time)
                    alpha = 0.5 * i / len(self.pos_history)
                    color = (0, int(255 * alpha), 0)
                    
                    # We don't have camera parameters to project 3D points to image
                    # Instead, we'll add a visualization overlay in the corner
                    
            # Add prediction overlay in the bottom-right corner
            h, w = vis_image.shape[:2]
            overlay_w = 300
            overlay_h = 300
            offset_x = w - overlay_w - 10
            offset_y = h - overlay_h - 10
            
            # Create overlay background
            cv2.rectangle(vis_image, 
                         (offset_x, offset_y), 
                         (offset_x + overlay_w, offset_y + overlay_h), 
                         (0, 0, 0), -1)
            cv2.rectangle(vis_image, 
                         (offset_x, offset_y), 
                         (offset_x + overlay_w, offset_y + overlay_h), 
                         (255, 255, 255), 2)
            
            # Draw coordinate system axes
            axis_length = 50
            origin_x = offset_x + overlay_w // 2
            origin_y = offset_y + overlay_h // 2
            
            # X-axis (red)
            cv2.arrowedLine(vis_image, 
                          (origin_x, origin_y), 
                          (origin_x + axis_length, origin_y), 
                          (0, 0, 255), 2)
            cv2.putText(vis_image, "X", 
                       (origin_x + axis_length + 5, origin_y + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Y-axis (green)
            cv2.arrowedLine(vis_image, 
                          (origin_x, origin_y), 
                          (origin_x, origin_y - axis_length), 
                          (0, 255, 0), 2)
            cv2.putText(vis_image, "Y", 
                       (origin_x - 15, origin_y - axis_length - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Z-axis (blue)
            cv2.arrowedLine(vis_image, 
                          (origin_x, origin_y), 
                          (origin_x - int(axis_length * 0.7), origin_y - int(axis_length * 0.7)), 
                          (255, 0, 0), 2)
            cv2.putText(vis_image, "Z", 
                       (origin_x - int(axis_length * 0.7) - 20, origin_y - int(axis_length * 0.7) - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Draw table surface
            table_pts = np.array([
                [origin_x - 80, origin_y + 30],
                [origin_x + 80, origin_y + 30],
                [origin_x + 80, origin_y - 70],
                [origin_x - 80, origin_y - 70]
            ], dtype=np.int32)
            cv2.polylines(vis_image, [table_pts], True, (120, 80, 0), 2)
            
            # Add table label
            cv2.putText(vis_image, "Table", 
                       (origin_x - 25, origin_y + 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 80, 0), 2)
            
            # Draw predicted trajectory if available
            if self.predicted_trajectory is not None and len(self.predicted_trajectory) > 0:
                scale_factor = 80  # Scale factor to fit in our overlay
                
                # Draw each point in the trajectory
                prev_x, prev_y = None, None
                
                for i, (pos, color) in enumerate(zip(self.predicted_trajectory, self.trajectory_colors)):
                    # Convert 3D point to 2D overlay coordinates
                    # X axis points right, Y axis points up, Z axis points at an angle
                    point_x = origin_x + int(pos[0] * scale_factor)
                    point_y = origin_y - int(pos[1] * scale_factor) - int(pos[2] * scale_factor * 0.5)
                    
                    # Draw point
                    cv2.circle(vis_image, (point_x, point_y), 2, color, -1)
                    
                    # Connect points with lines
                    if prev_x is not None and prev_y is not None:
                        cv2.line(vis_image, (prev_x, prev_y), (point_x, point_y), color, 1)
                    
                    prev_x, prev_y = point_x, point_y
            
            # Draw EKF state information
            if self.last_pos is not None and self.last_vel is not None:
                # Position
                cv2.putText(vis_image, "Current Position (m):", 
                           (offset_x + 10, offset_y + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(vis_image, f"X: {self.last_pos[0]:.3f}", 
                           (offset_x + 10, offset_y + 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(vis_image, f"Y: {self.last_pos[1]:.3f}", 
                           (offset_x + 10, offset_y + 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(vis_image, f"Z: {self.last_pos[2]:.3f}", 
                           (offset_x + 10, offset_y + 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Velocity
                cv2.putText(vis_image, "Current Velocity (m/s):", 
                           (offset_x + 10, offset_y + 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(vis_image, f"X: {self.last_vel[0]:.3f}", 
                           (offset_x + 10, offset_y + 130), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(vis_image, f"Y: {self.last_vel[1]:.3f}", 
                           (offset_x + 10, offset_y + 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(vis_image, f"Z: {self.last_vel[2]:.3f}", 
                           (offset_x + 10, offset_y + 170), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add additional status info
            cv2.putText(vis_image, f"EKF Prediction", 
                       (10, h - 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.putText(vis_image, f"Target Distance: {self.target_distance}m", 
                       (10, h - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Publish visualization
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            vis_msg.header.stamp = self.get_clock().now().to_msg()
            vis_msg.header.frame_id = "camera_color_optical_frame"
            self.vis_image_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f"Visualization error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        # Unlock color image
        self.color_image_lock = False

    def status_callback(self):
        """Print status information every second"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_status_time).nanoseconds / 1e9
        
        # Calculate rates
        callback_rate = self.total_callbacks / dt if dt > 0 else 0
        prediction_rate = self.total_predictions / dt if dt > 0 else 0
        
        self.get_logger().info(
            f'Status: Received {self.total_callbacks} positions ({callback_rate:.1f} Hz), '
            f'Made {self.total_predictions} predictions ({prediction_rate:.1f} Hz)'
        )
        
        # Reset counters
        self.total_callbacks = 0
        self.total_predictions = 0
        self.last_status_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = BallPredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Ball Predictor node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 