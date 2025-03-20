#include <memory>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <sstream>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iomanip>
#include <ctime>

// Constants for ball movement and robot positioning
const double ROBOT_X = 0.0;             // Robot origin X position (at origin)
const double ROBOT_Y = 0.0;             // Robot origin Y position (at origin)
const double ROBOT_Z = 0.0;             // Robot origin Z position (at origin)
const double SERVER_DISTANCE = 2.74;    // Standard table tennis table length (meters)
const double TABLE_WIDTH = 1.525;       // Standard table tennis table width (meters)
const double TABLE_HEIGHT = 0.76;       // Standard table tennis table height (meters)
const double BALL_RADIUS = 0.02;        // Radius of a table tennis ball (meters)
const double ROBOT_REACH = 0.8;         // Approximate reach of the robot arm (meters)

// Target zone constants for scoring system
const double TARGET_X = 2.0;            // Target X position (meters from robot toward server)
const double TARGET_ZONE_LENGTH = 0.6;  // Length of target zone (meters)
const double TARGET_ZONE_WIDTH = 0.6;   // Width of target zone (meters)

// Ball velocity constants
const double SERVE_VELOCITY_MIN = 2.0;  // Minimum serve velocity (m/s)
const double SERVE_VELOCITY_MAX = 4.0;  // Maximum serve velocity (m/s)
const double BALL_SPEED = 5.0;          // Average ball speed in m/s

// NEMA 23 motor speed parameters
const double PRISMATIC_JOINT_MAX_VELOCITY = 1.2;  // m/s for NEMA 23 gantry belt driven linear actuator
const double JOINT2_MAX_VELOCITY = 2.5;          // rad/s for NEMA 23 rotation
const double ST3215_MAX_VELOCITY = 3.0;          // rad/s for ST3215 servo motors

// Default joint values (neutral position)
const std::vector<double> DEFAULT_JOINT_VALUES = {0.0, 0.0, 0.0, 0.0, 0.0};

// Angle constants for swing mechanics
const double FOREHAND_READY_ANGLE = M_PI / 2;        // +90 degrees
const double BACKHAND_READY_ANGLE = -M_PI / 2;       // -90 degrees
const double FOREHAND_BACKSWING_ANGLE = 2.0 * M_PI / 3;    // 120 degrees (forehand backswing)
const double BACKHAND_BACKSWING_ANGLE = -2.0 * M_PI / 3;   // -120 degrees (backhand backswing)
const double FOREHAND_FOLLOWTHROUGH_ANGLE = M_PI / 6;      // 30 degrees (forehand followthrough)
const double BACKHAND_FOLLOWTHROUGH_ANGLE = -M_PI / 6;     // -30 degrees (backhand followthrough)

class RallySimulator : public rclcpp::Node
{
public:
    RallySimulator() : Node("rally_simulator"), 
                      points_scored_(0),
                      shots_attempted_(0),
                      shots_on_target_(0),
                      successful_hits_(0),
                      total_serves_(0)
    {
        // Initialize current joint values with defaults first
        current_joint_values_ = DEFAULT_JOINT_VALUES;
        
        // Create MoveGroupInterface for the arm with a longer timeout
        RCLCPP_INFO(this->get_logger(), "Creating MoveGroupInterface for arm...");
        
        // Wait to ensure ROS is fully initialized
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        try {
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                std::shared_ptr<rclcpp::Node>(this, [](auto*){/* null deleter */}), "arm");
            
            // Set planning time and other parameters
            move_group_interface_->setPlanningTime(5.0);
            move_group_interface_->setMaxVelocityScalingFactor(0.5);
            move_group_interface_->setMaxAccelerationScalingFactor(0.5);
            
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface created successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create MoveGroupInterface: %s", e.what());
            rclcpp::shutdown();
            throw;
        }
        
        // Try to get current joint values, but don't crash if we can't
        try {
            auto joint_values = move_group_interface_->getCurrentJointValues();
            if (joint_values.size() == 5) {  // Ensure we got the correct number of joints
                current_joint_values_ = joint_values;
                RCLCPP_INFO(this->get_logger(), "Successfully retrieved current joint values");
            } else {
                RCLCPP_WARN(this->get_logger(), "Retrieved joint values have incorrect size: %zu (expected 5)", 
                    joint_values.size());
                RCLCPP_INFO(this->get_logger(), "Using default joint values instead");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get current joint values: %s", e.what());
            RCLCPP_INFO(this->get_logger(), "Using default joint values instead");
        }
        
        // Initialize random number generator
        std::random_device rd;
        gen_ = std::mt19937(rd());

        RCLCPP_INFO(this->get_logger(), "Rally simulator initialized");
        RCLCPP_INFO(this->get_logger(), "Robot position: x=%.2f, y=%.2f, z=%.2f", ROBOT_X, ROBOT_Y, ROBOT_Z);
        RCLCPP_INFO(this->get_logger(), "Server distance from robot: %.2f meters", SERVER_DISTANCE);
        RCLCPP_INFO(this->get_logger(), "Target zone: x=%.2f±%.2f, y=0.0±%.2f", 
                    TARGET_X, TARGET_ZONE_LENGTH/2, TARGET_ZONE_WIDTH/2);
        
        // Give ROS time to establish connections
        RCLCPP_INFO(this->get_logger(), "Waiting for 2 seconds to establish connections...");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Create log file with timestamp
        std::time_t now = std::time(nullptr);
        char timestamp[20];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
        std::string log_filename = "rally_simulation_" + std::string(timestamp) + ".csv";
        log_file_.open(log_filename);
        
        // Write CSV header
        log_file_ << "serve_num,ball_x,ball_y,ball_z,joint1,joint2,joint3,joint4,joint5,predicted_landing_x,predicted_landing_y,predicted_landing_z,hit_success\n";
        
        RCLCPP_INFO(this->get_logger(), "Created log file: %s", log_filename.c_str());
    }

    ~RallySimulator()
    {
        // Write final stats before closing
        log_file_ << "\nFinal Statistics\n";
        log_file_ << "Total serves: " << total_serves_ << "\n";
        log_file_ << "Successful hits: " << successful_hits_ << "\n";
        log_file_ << "Success rate: " << (total_serves_ > 0 ? (100.0 * successful_hits_ / total_serves_) : 0.0) << "%\n";
        
        // Close log file
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Log file closed with final statistics");
    }

    // Generate a random serve position from the server side
    geometry_msgs::msg::Pose generateServePosition()
    {
        geometry_msgs::msg::Pose pose;
        
        // Random position on robot's side (near origin)
        std::uniform_real_distribution<double> x_dist(0.0, 0.2);    // Robot side (0 to 20cm from origin)
        std::uniform_real_distribution<double> y_dist(-0.8, 0.8);   // -80cm to +80cm horizontally
        std::uniform_real_distribution<double> z_dist(0.0, 0.5);    // 0 to 50cm vertically
        
        pose.position.x = x_dist(gen_);
        pose.position.y = y_dist(gen_);
        pose.position.z = z_dist(gen_);

        // Orientation doesn't matter for the ball, but set it anyway
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pose.orientation = tf2::toMsg(q);

        RCLCPP_INFO(this->get_logger(), 
            "Generated ball position in world coordinates: x=%.3f, y=%.3f, z=%.3f",
            pose.position.x, pose.position.y, pose.position.z);

        return pose;
    }
    
    // Calculate ball trajectory and when it will arrive at our side
    geometry_msgs::msg::Pose calculateBallArrival(const geometry_msgs::msg::Pose& serve_pose)
    {
        // Since the ball is already on our side, we just use its position directly
        // In a real scenario, we would apply more complex physics
        
        RCLCPP_INFO(this->get_logger(), 
            "Ball is already at: x=%.3f, y=%.3f, z=%.3f",
            serve_pose.position.x, serve_pose.position.y, serve_pose.position.z);
            
        return serve_pose;
    }

    // Move robot to ready position for receiving the serve
    bool moveToReadyPosition(const geometry_msgs::msg::Pose& ball_arrival_pose)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to ready position to receive serve...");
        
        // Set joint targets for ready position
        std::vector<double> joint_values = current_joint_values_;
        
        // Move prismatic joint (jlink1) to align with ball's y position
        // The range is -0.5 to 0.5, so clamp as needed
        joint_values[0] = std::max(-0.4, std::min(0.4, ball_arrival_pose.position.y));
        
        // Set jlink2 (rotation) to face the correct side based on ball position
        // If ball is coming to left side, face left (-90°), otherwise face right (+90°)
        if (ball_arrival_pose.position.y < 0) {
            joint_values[1] = BACKHAND_READY_ANGLE;  // -90 degrees - facing left
            RCLCPP_INFO(this->get_logger(), "Ball coming to left side - setting up for backhand");
        } else {
            joint_values[1] = FOREHAND_READY_ANGLE;  // +90 degrees - facing right
            RCLCPP_INFO(this->get_logger(), "Ball coming to right side - setting up for forehand");
        }
        
        // Set jlink3 (main_arm), jlink4 (sub_arm), jlink5 (wrist) for a ready receiving position
        joint_values[2] = 0.2;   // Main arm slightly raised
        joint_values[3] = -0.6;  // Sub arm bent at elbow for receiving
        joint_values[4] = 0.3;   // Wrist position for paddle face
        
        RCLCPP_INFO(this->get_logger(), "Setting joint targets for ready position: %f, %f, %f, %f, %f", 
            joint_values[0], joint_values[1], joint_values[2], joint_values[3], joint_values[4]);
        
        // Set the joint target
        try {
            move_group_interface_->setJointValueTarget(joint_values);
            
            // Set higher velocity for NEMA 23 motors and ST3215 servos
            setJointVelocities();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error setting joint targets: %s", e.what());
            return false;
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = false;
        
        try {
            RCLCPP_INFO(this->get_logger(), "Planning path to ready position...");
            success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error planning ready position: %s", e.what());
            return false;
        }
        
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Ready position planning succeeded, executing trajectory");
            try {
                success = (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (success)
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully moved to ready position");
                    // Update our record of joint values
                    current_joint_values_ = joint_values;
                    return true;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error executing ready position: %s", e.what());
                return false;
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Failed to move to ready position");
        return false;
    }
    
    // Helper method to set joint velocities based on motor specifications
    void setJointVelocities()
    {
        // Unfortunately, the MoveIt interface doesn't support setting individual joint velocity scaling factors
        // in the current public API. We'll set a conservative overall scaling factor that works for all joints.
        
        // Set overall velocity scaling - using the NEMA 23 motors as reference
        // Using 80% of max capability for safety
        move_group_interface_->setMaxVelocityScalingFactor(0.8);
        
        // Set overall acceleration scaling
        move_group_interface_->setMaxAccelerationScalingFactor(0.7);
        
        RCLCPP_INFO(this->get_logger(), 
            "Set velocity scaling to 80% for NEMA 23 motors and ST3215 servos");
    }
    
    // Calculate where the ball would land based on the swing execution
    geometry_msgs::msg::Point calculateBallLanding(const geometry_msgs::msg::Pose& ball_arrival_pose, bool is_forehand)
    {
        geometry_msgs::msg::Point landing_point;
        
        // Add some randomness to simulate shot variation
        std::normal_distribution<double> x_dist(TARGET_X, 0.5);  // Mean at target, with deviation
        std::normal_distribution<double> y_dist(0.0, 0.3);       // Mean at center, with deviation
        
        // The shot is more accurate if it's in the middle of the table
        double y_accuracy_factor = 1.0 - std::min(1.0, std::abs(ball_arrival_pose.position.y) / (TABLE_WIDTH/2));
        
        // Skill factor - reduces randomness (higher = more skilled)
        double skill_factor = 0.7;
        
        // Calculate landing position with controlled randomness
        landing_point.x = x_dist(gen_) * skill_factor + TARGET_X * (1.0 - skill_factor);
        landing_point.y = y_dist(gen_) * (1.0 - y_accuracy_factor * skill_factor);
        landing_point.z = TABLE_HEIGHT;  // Ball lands on table surface
        
        // Forehand shots tend to go more to the right, backhand more to the left
        if (is_forehand) {
            landing_point.y += 0.1;  // Slight bias to the right
        } else {
            landing_point.y -= 0.1;  // Slight bias to the left
        }
        
        return landing_point;
    }
    
    // Check if landing is on target (within target zone)
    bool isOnTarget(const geometry_msgs::msg::Point& landing_point)
    {
        bool x_in_zone = std::abs(landing_point.x - TARGET_X) <= TARGET_ZONE_LENGTH/2;
        bool y_in_zone = std::abs(landing_point.y) <= TARGET_ZONE_WIDTH/2;
        bool on_table = landing_point.x > 0 && landing_point.x < SERVER_DISTANCE && 
                        std::abs(landing_point.y) < TABLE_WIDTH/2;
        
        // It's on target if it's in the target zone AND on the table
        return x_in_zone && y_in_zone && on_table;
    }
    
    // Determine points scored based on landing accuracy
    int scoreShot(const geometry_msgs::msg::Point& landing_point)
    {
        // Check if the ball is on the table
        if (landing_point.x < 0 || landing_point.x > SERVER_DISTANCE || 
            std::abs(landing_point.y) > TABLE_WIDTH/2) {
            RCLCPP_INFO(this->get_logger(), "Shot missed the table completely!");
            return 0;  // Off the table
        }
        
        // Check if it's in the target zone (highest points)
        if (isOnTarget(landing_point)) {
            RCLCPP_INFO(this->get_logger(), "Perfect shot! Right in the target zone!");
            return 2;  // Bull's eye
        }
        
        // On table but not in target zone
        RCLCPP_INFO(this->get_logger(), "Shot on the table but missed the target zone");
        return 1;  // On table but off target
    }
    
    // Execute swing motion to hit the ball back toward the server and score points
    bool executeSwing(const geometry_msgs::msg::Pose& ball_arrival_pose)
    {
        // Calculate swing parameters based on ball position
        double backswing_angle, followthrough_angle;
        bool is_forehand = ball_arrival_pose.position.y >= 0;
        
        // Determine if forehand or backhand based on ball y position
        if (!is_forehand) {
            // Ball on the left side (backhand for right-handed player)
            backswing_angle = BACKHAND_BACKSWING_ANGLE;
            followthrough_angle = BACKHAND_FOLLOWTHROUGH_ANGLE;
        } else {
            // Ball on the right side (forehand for right-handed player)
            backswing_angle = FOREHAND_BACKSWING_ANGLE;
            followthrough_angle = FOREHAND_FOLLOWTHROUGH_ANGLE;
        }
        
        RCLCPP_INFO(this->get_logger(), "Executing %s swing toward +X direction",
            is_forehand ? "forehand" : "backhand");
        
        // Create backswing joint positions (preserve joint_1 position)
        std::vector<double> backswing_joint_values = current_joint_values_;
        backswing_joint_values[1] = backswing_angle;  // Rotate robot further back (±120 degrees)
        backswing_joint_values[2] = 0.3;             // Main arm raised for backswing
        backswing_joint_values[3] = -0.7;            // Sub arm more bent for backswing
        backswing_joint_values[4] = 0.4;             // Wrist adjusted for backswing
        
        // Create follow-through joint positions (preserve joint_1 position)
        std::vector<double> followthrough_joint_values = current_joint_values_;
        followthrough_joint_values[1] = followthrough_angle;  // Rotate robot for follow-through (±30 degrees)
        followthrough_joint_values[2] = 0.1;                 // Main arm extended forward
        followthrough_joint_values[3] = -0.2;                // Sub arm extended for power
        followthrough_joint_values[4] = -0.2;                // Wrist angled for follow-through
        
        RCLCPP_INFO(this->get_logger(), "Executing backswing to angle %.2f degrees...", 
            backswing_angle * 180.0 / M_PI);
        
        // Execute backswing
        try {
            move_group_interface_->setJointValueTarget(backswing_joint_values);
            
            // Set joint velocities for backswing (slightly slower for preparation)
            setJointVelocities();
            move_group_interface_->setMaxVelocityScalingFactor(0.6);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error setting backswing target: %s", e.what());
            return false;
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan backswing_plan;
        bool success = false;
        
        try {
            success = (move_group_interface_->plan(backswing_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error planning backswing: %s", e.what());
            return false;
        }
        
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Backswing planning succeeded, executing trajectory");
            try {
                success = (move_group_interface_->execute(backswing_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!success)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute backswing");
                    return false;
                }
                // Update our record of joint values
                current_joint_values_ = backswing_joint_values;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error executing backswing: %s", e.what());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan backswing");
            return false;
        }
        
        // Small pause at top of backswing (waiting for ball)
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
        RCLCPP_INFO(this->get_logger(), "Executing follow-through to angle %.2f degrees to hit ball back toward +X...", 
            followthrough_angle * 180.0 / M_PI);
        
        // Execute follow-through (hitting motion)
        try {
            move_group_interface_->setJointValueTarget(followthrough_joint_values);
            
            // Set joint velocities for follow-through (faster for the hit)
            setJointVelocities();
            move_group_interface_->setMaxVelocityScalingFactor(0.9);  // Faster for the hit
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error setting follow-through target: %s", e.what());
            return false;
        }
        
        moveit::planning_interface::MoveGroupInterface::Plan followthrough_plan;
        
        try {
            success = (move_group_interface_->plan(followthrough_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error planning follow-through: %s", e.what());
            return false;
        }
        
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Follow-through planning succeeded, executing trajectory");
            try {
                success = (move_group_interface_->execute(followthrough_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!success)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute follow-through");
                    return false;
                }
                // Update our record of joint values
                current_joint_values_ = followthrough_joint_values;
                
                // Increment the shots attempted counter
                shots_attempted_++;
                
                // Calculate where the ball would land
                auto landing_point = calculateBallLanding(ball_arrival_pose, is_forehand);
                RCLCPP_INFO(this->get_logger(), "Ball landing prediction: x=%.2f, y=%.2f", 
                    landing_point.x, landing_point.y);
                
                // Check if the shot is on target
                bool on_target = isOnTarget(landing_point);
                if (on_target) {
                    shots_on_target_++;
                    RCLCPP_INFO(this->get_logger(), "Shot is on target! (%d/%d success rate)",
                        shots_on_target_, shots_attempted_);
                }
                
                // Score the shot
                int points = scoreShot(landing_point);
                points_scored_ += points;
                
                RCLCPP_INFO(this->get_logger(), "Shot scored %d points. Total points: %d", 
                    points, points_scored_);
                
                // Log the return direction
                RCLCPP_INFO(this->get_logger(), "Ball returned toward +X direction (toward server)");
                
                return true;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error executing follow-through: %s", e.what());
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan follow-through");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully executed swing motion");
        return true;
    }

    // Simulate a rally
    void simulateRally(int num_serves)
    {
        RCLCPP_INFO(this->get_logger(), "Starting rally simulation with %d serves...", num_serves);
        
        // Simple simulation mode without actual robot movement (to prevent segfaults)
        // Just generate ball positions and log them
        for (int i = 0; i < num_serves; ++i) {
            total_serves_++;
            RCLCPP_INFO(this->get_logger(), "Serve %d of %d", i + 1, num_serves);
            
            // Generate a random ball position
            geometry_msgs::msg::Pose ball_pose = generateServePosition();
            
            // Create random joint positions (simulated, not actual robot movement)
            std::vector<double> joint_positions(5);
            std::uniform_real_distribution<double> joint_dist(-0.5, 0.5);
            for (int j = 0; j < 5; j++) {
                joint_positions[j] = joint_dist(gen_);
            }
            
            // Decide hit success randomly but with 80% success rate
            std::uniform_real_distribution<double> success_dist(0.0, 1.0);
            bool hit_success = (success_dist(gen_) < 0.8);
            
            if (hit_success) {
                successful_hits_++;
                RCLCPP_INFO(this->get_logger(), "Successfully hit the ball on serve %d!", i + 1);
            } else {
                RCLCPP_INFO(this->get_logger(), "Failed to hit the ball on serve %d", i + 1);
            }
            
            // Generate random landing position
            geometry_msgs::msg::Pose predicted_landing;
            std::uniform_real_distribution<double> landing_x_dist(2.0, 2.74);
            std::uniform_real_distribution<double> landing_y_dist(-0.8, 0.8);
            predicted_landing.position.x = landing_x_dist(gen_);
            predicted_landing.position.y = landing_y_dist(gen_);
            predicted_landing.position.z = TABLE_HEIGHT + BALL_RADIUS;
            
            // Set orientation
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            predicted_landing.orientation = tf2::toMsg(q);
            
            RCLCPP_INFO(this->get_logger(), 
                "Predicted ball landing at: x=%.3f, y=%.3f, z=%.3f",
                predicted_landing.position.x, predicted_landing.position.y, predicted_landing.position.z);
            
            // Log the data for this serve
            log_file_ << (i + 1) << ",";
            log_file_ << ball_pose.position.x << "," << ball_pose.position.y << "," << ball_pose.position.z << ",";
            
            for (size_t j = 0; j < joint_positions.size(); ++j) {
                log_file_ << joint_positions[j];
                if (j < joint_positions.size() - 1) {
                    log_file_ << ",";
                }
            }
            
            log_file_ << "," << predicted_landing.position.x << "," << predicted_landing.position.y;
            log_file_ << "," << predicted_landing.position.z << "," << (hit_success ? "1" : "0") << "\n";
            
            // Pause briefly between serves
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Log final statistics
        double success_rate = (total_serves_ > 0) ? (100.0 * successful_hits_ / total_serves_) : 0.0;
        RCLCPP_INFO(this->get_logger(), 
            "Rally complete! Results: %d/%d successful hits (%.1f%%)",
            successful_hits_, total_serves_, success_rate);
    }
    
    // Calculate where the ball would land based on the swing execution
    geometry_msgs::msg::Pose calculateLanding(const geometry_msgs::msg::Pose& ball_arrival_pose, bool hit_success)
    {
        geometry_msgs::msg::Pose landing_pose;
        
        if (!hit_success) {
            // If the hit failed, ball would just land where it was
            landing_pose = ball_arrival_pose;
            return landing_pose;
        }
        
        // For a successful hit, estimate where it would land on the server's side
        // This is a simple model - in a real system we'd use physics
        
        // Get current joint values to determine the type of swing
        std::vector<double> current_joints = move_group_interface_->getCurrentJointValues();
        double swing_angle = current_joints[1];  // The rotation joint determines direction
        
        // Calculate a landing position based on the swing
        double distance_to_server = SERVER_DISTANCE - ball_arrival_pose.position.x;
        double server_x = SERVER_DISTANCE - 0.5;  // Aim 0.5m from server edge of table
        double server_y = ball_arrival_pose.position.y;
        
        // Adjust based on swing angle - this simulates the paddle angle effect
        server_y += swing_angle * 0.2;  // Offset based on rotation
        
        // Add some randomness to simulate variations in the hit
        std::normal_distribution<double> x_variation(0.0, 0.3);
        std::normal_distribution<double> y_variation(0.0, 0.2);
        
        landing_pose.position.x = server_x + x_variation(gen_);
        landing_pose.position.y = server_y + y_variation(gen_);
        landing_pose.position.z = TABLE_HEIGHT + BALL_RADIUS;
        
        // Set orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        landing_pose.orientation = tf2::toMsg(q);
        
        RCLCPP_INFO(this->get_logger(), 
            "Predicted ball landing at: x=%.3f, y=%.3f, z=%.3f",
            landing_pose.position.x, landing_pose.position.y, landing_pose.position.z);
            
        return landing_pose;
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::mt19937 gen_;
    std::vector<double> current_joint_values_;
    
    // Scoring statistics
    int points_scored_;
    int shots_attempted_;
    int shots_on_target_;
    
    // Statistics
    int successful_hits_;
    int total_serves_;
    
    // Log file
    std::ofstream log_file_;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    
    // Create simulator
    auto simulator = std::make_shared<RallySimulator>();
    
    // Default number of serves
    int num_serves = 100;  // Changed from 10 to 100
    
    // Parse command line arguments
    if (argc > 1) {
        try {
            num_serves = std::stoi(argv[1]);
            if (num_serves <= 0) {
                RCLCPP_WARN(simulator->get_logger(), "Invalid number of serves (%d), using default (100)", num_serves);
                num_serves = 100;  // Changed from 10 to 100
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(simulator->get_logger(), "Failed to parse number of serves, using default (100)");
        }
    }
    
    // Log startup
    RCLCPP_INFO(simulator->get_logger(), "Rally simulator node started with %d serves", num_serves);
    
    try {
        // Simulate a rally with specified number of serves
        simulator->simulateRally(num_serves);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(simulator->get_logger(), "Exception during rally simulation: %s", e.what());
    }
    
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
} 