#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <random>
#include <cmath>
#include <iomanip>   // For fixed and setprecision

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  // Initialize ROS 2 with simulated time enabled.
  auto node_options = rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}});
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tt_motion_rl_improved", node_options);
  auto logger = rclcpp::get_logger("tt_motion_rl_improved");

  // Create the MoveGroupInterface for the "arm" planning group.
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  // Increased planning time from 10.0 to 20.0 seconds.
  move_group.setPlanningTime(25.0);

  // Define a fixed orientation for the target pose.
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, 0.0);  // Adjust as needed.
  geometry_msgs::msg::Quaternion fixed_orientation = tf2::toMsg(quat);

  // Set up a random generator for ball positions.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis_ball_x(-0.5, 0.5);
  std::uniform_real_distribution<double> dis_ball_y(-0.5, 0.5);
  std::uniform_real_distribution<double> dis_ball_z(0.0, 0.5);


  const int num_iterations = 25000;
  const int max_retries = 5;
  const double hit_threshold = 0.5;  // Distance threshold for a successful hit

  // Open a CSV file to log full transitions.
  std::ofstream csv_file("tt_rl_dataset.csv");
  if (!csv_file.is_open())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open tt_rl_dataset.csv for writing.");
    return 1;
  }
  
  // Write the CSV header.
  csv_file << "iteration,"
           << "state_ball_x,state_ball_y,state_ball_z,"
           << "state_ee_x,state_ee_y,state_ee_z,";
  for (int j = 0; j < 6; j++)
    csv_file << "state_joint" << (j+1) << (j < 5 ? "," : "");
  csv_file << ","
           << "action_x,action_y,action_z,"
           << "action_orient_x,action_orient_y,action_orient_z,action_orient_w,"
           << "reward,done,"
           << "next_ball_x,next_ball_y,next_ball_z,"
           << "next_ee_x,next_ee_y,next_ee_z,";
  for (int j = 0; j < 6; j++)
    csv_file << "next_joint" << (j+1) << (j < 5 ? "," : "");
  csv_file << "\n";

  for (int iter = 0; iter < num_iterations; iter++)
  {
    // Generate a random ball position.
    double ball_x = dis_ball_x(gen);
    double ball_y = dis_ball_y(gen);
    double ball_z = dis_ball_z(gen);

    // Get current end-effector pose.
    auto current_pose_stamped = move_group.getCurrentPose();
    auto current_pose = current_pose_stamped.pose;

    // Get current joint positions.
    std::vector<double> current_joints = move_group.getCurrentJointValues();

    // Create the target pose (action) using the ball position and fixed orientation.
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = ball_x;
    target_pose.position.y = ball_y;
    target_pose.position.z = ball_z;
    target_pose.orientation = fixed_orientation;

    // Log the intended action values.
    double action_x = target_pose.position.x;
    double action_y = target_pose.position.y;
    double action_z = target_pose.position.z;
    double action_orient_x = target_pose.orientation.x;
    double action_orient_y = target_pose.orientation.y;
    double action_orient_z = target_pose.orientation.z;
    double action_orient_w = target_pose.orientation.w;

    // Log ball position with one decimal place using "%.1f"
    RCLCPP_INFO(node->get_logger(), "Iteration %d: Ball position: (%.1f, %.1f, %.1f)",
                iter, ball_x, ball_y, ball_z);

    // Set the target pose.
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = false;
    for (int attempt = 0; attempt < max_retries; attempt++)
    {
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "Plan computed for iteration %d on attempt %d.", iter, attempt + 1);
        if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(node->get_logger(), "Iteration %d executed successfully.", iter);
          plan_success = true;
          break;
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "Execution failed on attempt %d for iteration %d.", attempt + 1, iter);
        }
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Planning failed on attempt %d for iteration %d.", attempt + 1, iter);
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    double reward = 0.0;
    bool done = false;

    // Get next state values after executing the action.
    auto next_pose_stamped = move_group.getCurrentPose();
    auto next_pose = next_pose_stamped.pose;
    std::vector<double> next_joints = move_group.getCurrentJointValues();

    double dx = next_pose.position.x - ball_x;
    double dy = next_pose.position.y - ball_y;
    double dz = next_pose.position.z - ball_z;
    double distance_error = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (plan_success)
    {
      reward = -distance_error;  // Closer means higher reward (less negative)
      if (distance_error < hit_threshold)
      {
        done = true;
        RCLCPP_INFO(node->get_logger(), "Iteration %d: Hit achieved! Distance error = %.3f", iter, distance_error);
      }
      else
      {
        done = false;
        RCLCPP_INFO(node->get_logger(), "Iteration %d: Missed. Distance error = %.3f", iter, distance_error);
      }
    }
    else
    {
      reward = -100.0;
      done = false;
      RCLCPP_WARN(node->get_logger(), "Iteration %d: Planning/Execution failed. Assigned reward = -100", iter);
    }

    // Log the full transition.
    std::ostringstream line;
    // State: ball position (formatted to one decimal place).
    line << iter << ","
         << std::fixed << std::setprecision(1) << ball_x << "," << ball_y << "," << ball_z << ","
         << std::resetiosflags(std::ios::fixed);
    // State: current EE position.
    line << current_pose.position.x << "," << current_pose.position.y << "," << current_pose.position.z << ",";
    // State: current joint positions.
    for (size_t i = 0; i < current_joints.size(); i++)
    {
      line << current_joints[i] << (i < current_joints.size()-1 ? "," : ",");
    }
    // Action: target pose (position and orientation).
    line << action_x << "," << action_y << "," << action_z << ","
         << action_orient_x << "," << action_orient_y << "," << action_orient_z << "," << action_orient_w << ",";
    // Reward and done.
    line << reward << "," << done << ",";
    // Next state: new ball position (formatted to one decimal place).
    double next_ball_x = dis_ball_x(gen);
    double next_ball_y = dis_ball_y(gen);
    double next_ball_z = dis_ball_z(gen);
    line << std::fixed << std::setprecision(1)
         << next_ball_x << "," << next_ball_y << "," << next_ball_z << ","
         << std::resetiosflags(std::ios::fixed);
    // Next state: new EE position.
    line << next_pose.position.x << "," << next_pose.position.y << "," << next_pose.position.z << ",";
    // Next state: new joint positions.
    for (size_t i = 0; i < next_joints.size(); i++)
    {
      line << next_joints[i] << (i < next_joints.size()-1 ? "," : "");
    }
    line << "\n";
    
    csv_file << line.str();

    // Clear the pose targets for the next iteration.
    move_group.clearPoseTargets();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  csv_file.close();
  RCLCPP_INFO(node->get_logger(), "RL trials completed. Transitions saved to tt_rl_dataset.csv");

  rclcpp::shutdown();
  return 0;
}
