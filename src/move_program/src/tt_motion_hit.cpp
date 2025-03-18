#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

int main(int argc, char **argv)
{
  // Initialize ROS 2.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tt_motion_with_ball");
  auto logger = rclcpp::get_logger("tt_motion_with_ball");

  // Create the MoveGroupInterface for the "arm" planning group.
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  move_group.setPlanningTime(5.0);

  // Retrieve joint names; we expect 6 joints.
  std::vector<std::string> joint_names = move_group.getJointNames();
  if (joint_names.size() != 6)
  {
    RCLCPP_ERROR(node->get_logger(), "Expected 6 joints, but found %zu.", joint_names.size());
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Found %zu joints.", joint_names.size());

  // Define base key poses for a table tennis stroke.
  // The order is: Ready, Cocking, Impact, Follow-Through.
  // These joint configurations (in radians/meters) are examples; adjust them as needed.
  std::vector<double> ready_pose          = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> cocking_pose        = {0.1, -0.5, 0.5, -0.3, 0.2, 0.3};
  std::vector<double> impact_pose_base    = {-0.1, 0.5, 0.1, 0.2, -0.2, -0.3};
  std::vector<double> follow_through_pose = {0.0, 0.0, -0.2, 0.0, 0.0, 0.0};

  // Open an output file to store trajectories and ball positions.
  std::ofstream traj_file("tt_ball_trajectories.txt");
  if (!traj_file.is_open())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open tt_ball_trajectories.txt for writing.");
    return 1;
  }

  // Set up random generators for ball positions.
  // Assume the ball appears in a region of the robot's workspace.
  std::random_device rd;
  std::mt19937 gen(rd());
  // For example, ball_x and ball_y in [-0.3, 0.3] and ball_z in [0.15, 0.25].
  std::uniform_real_distribution<double> dis_ball_x(-0.3, 0.3);
  std::uniform_real_distribution<double> dis_ball_y(-0.3, 0.3);
  std::uniform_real_distribution<double> dis_ball_z(0.15, 0.25);

  const int max_retries = 1;
  const int num_hits = 1000;  // Number of ball hits (iterations).

  // For each ball hit, generate a ball position and perform the stroke sequence.
  for (int hit = 0; hit < num_hits; hit++)
  {
    // Generate a random ball position.
    double ball_x = dis_ball_x(gen);
    double ball_y = dis_ball_y(gen);
    double ball_z = dis_ball_z(gen);
    
    traj_file << "Ball hit " << hit << ": ball position = ("
              << ball_x << ", " << ball_y << ", " << ball_z << ")\n";

    // Compute the impact pose by modifying the base impact pose using the ball position.
    // Here, scaling factors (k1, k2, k3) determine how strongly the ball position affects the joint values.
    double k1 = 0.2, k2 = 0.2, k3 = 0.2;
    std::vector<double> impact_pose(6);
    impact_pose[0] = impact_pose_base[0] + k1 * ball_x;
    impact_pose[1] = impact_pose_base[1] + k2 * ball_y;
    impact_pose[2] = impact_pose_base[2] + k3 * ball_z;
    // For joints 4, 5, and 6, use the base values.
    impact_pose[3] = impact_pose_base[3];
    impact_pose[4] = impact_pose_base[4];
    impact_pose[5] = impact_pose_base[5];

    // Create a sequence of poses to execute the stroke.
    std::vector<std::vector<double>> sequence = {ready_pose, cocking_pose, impact_pose, follow_through_pose};

    // Execute each step in the sequence.
    for (size_t step = 0; step < sequence.size(); step++)
    {
      std::vector<double> joint_target = sequence[step];
      std::ostringstream oss;
      oss << "Hit " << hit << ", Step " << step << ": ";
      for (size_t j = 0; j < joint_target.size(); j++)
      {
        oss << joint_names[j] << ": " << joint_target[j] << "  ";
      }
      RCLCPP_INFO(node->get_logger(), "%s", oss.str().c_str());

      bool plan_success = false;
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      for (int attempt = 0; attempt < max_retries; attempt++)
      {
        move_group.setJointValueTarget(joint_target);
        if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(node->get_logger(), "Plan computed for hit %d, step %zu on attempt %d. Executing...", hit, step, attempt + 1);
          if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_INFO(node->get_logger(), "Hit %d, step %zu reached successfully.", hit, step);
            plan_success = true;
            break;
          }
          else
          {
            RCLCPP_ERROR(node->get_logger(), "Execution failed on attempt %d for hit %d, step %zu.", attempt + 1, hit, step);
          }
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "Planning failed on attempt %d for hit %d, step %zu.", attempt + 1, hit, step);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      if (plan_success)
      {
        // Save the trajectory details.
        traj_file << "Trajectory for hit " << hit << ", step " << step << ":\n";
        auto &joint_traj = plan.trajectory_.joint_trajectory;
        traj_file << "  Joint Names: ";
        for (const auto &name : joint_traj.joint_names)
        {
          traj_file << name << " ";
        }
        traj_file << "\n";
        for (const auto &point : joint_traj.points)
        {
          traj_file << "  Time from start: " << point.time_from_start.sec << "." 
                    << point.time_from_start.nanosec << " sec\n";
          traj_file << "  Positions: ";
          for (const auto &pos : point.positions)
          {
            traj_file << pos << " ";
          }
          traj_file << "\n";
        }
        traj_file << "\n";
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "Skipping hit %d, step %zu after %d failed attempts.", hit, step, max_retries);
      }

      // Pause briefly before the next step.
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  traj_file.close();
  RCLCPP_INFO(node->get_logger(), "Trajectories and ball positions have been saved to tt_ball_trajectories.txt");

  rclcpp::shutdown();
  return 0;
}
