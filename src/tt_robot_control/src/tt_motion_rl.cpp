#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <random>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  // Initialize ROS 2.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tt_motion_rl");
  auto logger = rclcpp::get_logger("tt_motion_rl");

  // Create the MoveGroupInterface for the "arm" planning group.
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  // Increase planning time to allow for sufficient IK/plan computation.
  move_group.setPlanningTime(10.0);

  // Define a fixed orientation for hitting.
  // Adjust the roll, pitch, and yaw as needed.
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, 0.0);  // Change these values if needed.
  geometry_msgs::msg::Quaternion fixed_orientation = tf2::toMsg(quat);

  // Set up a random generator for ball positions.
  std::random_device rd;
  std::mt19937 gen(rd());
  // Define the ball's workspace region (adjust these ranges to your needs).
  std::uniform_real_distribution<double> dis_ball_x(0.2, 0.4);
  std::uniform_real_distribution<double> dis_ball_y(-0.2, 0.2);
  std::uniform_real_distribution<double> dis_ball_z(0.1, 0.3);

  const int num_iterations = 10000;  // Number of RL trials (iterations)
  const int max_retries = 1;        // Number of planning attempts per trial

  // Open a file to log rewards (iteration, ball position, reward).
  std::ofstream reward_file("tt_motion_rl_rewards.txt");
  if (!reward_file.is_open())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open tt_motion_rl_rewards.txt for writing.");
    return 1;
  }

  for (int iter = 0; iter < num_iterations; iter++)
  {
    // Generate a random ball position.
    double ball_x = dis_ball_x(gen);
    double ball_y = dis_ball_y(gen);
    double ball_z = dis_ball_z(gen);

    // Create a target Cartesian pose corresponding to the ball position.
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = ball_x;  
    target_pose.position.y = ball_y;
    target_pose.position.z = ball_z;
    target_pose.orientation = fixed_orientation;

    RCLCPP_INFO(node->get_logger(), "Iteration %d: Ball position: (%.3f, %.3f, %.3f)",
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
    if (plan_success)
    {
      // After execution, get the current end-effector pose.
      auto current_pose_stamped = move_group.getCurrentPose();
      auto current_pose = current_pose_stamped.pose;
      double dx = current_pose.position.x - ball_x;
      double dy = current_pose.position.y - ball_y;
      double dz = current_pose.position.z - ball_z;
      double distance_error = std::sqrt(dx * dx + dy * dy + dz * dz);
      reward = -distance_error;  // Reward: negative distance (closer = higher reward)
      RCLCPP_INFO(node->get_logger(), "Iteration %d: Distance error = %.3f, Reward = %.3f",
                  iter, distance_error, reward);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Iteration %d: Planning/Execution failed. Reward = -100", iter);
      reward = -100.0;
    }

    // Log iteration number, ball position, and reward.
    reward_file << iter << " " << ball_x << " " << ball_y << " " << ball_z << " " << reward << "\n";

    // Clear the pose targets for the next iteration.
    move_group.clearPoseTargets();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  reward_file.close();
  RCLCPP_INFO(node->get_logger(), "RL trials completed. Rewards saved to tt_motion_rl_rewards.txt");

  rclcpp::shutdown();
  return 0;
}
