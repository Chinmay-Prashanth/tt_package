#include <memory>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tt_motion");
  auto logger = rclcpp::get_logger("tt_motion");

  // Create the MoveGroupInterface for the "arm" planning group
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

  // Define key poses for a table tennis stroke.
  // Order: Ready, Cocking, Impact, Follow-Through.
  // Adjust these joint values (in radians/meters) to suit your robot.
  std::vector<std::vector<double>> key_poses = {
    // Ready Position: neutral configuration
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    // Cocking: pull paddle back
    {0.1, -0.5, 0.5, -0.3, 0.2, 0.3},
    // Impact: swing forward to hit the ball
    {-0.1, 0.5, 0.1, 0.2, -0.2, -0.3},
    // Follow-Through: continue the swing
    {0.0, 0.0, -0.2, 0.0, 0.0, 0.0}
  };

  // Open an output file to store trajectories.
  std::ofstream traj_file("tt_trajectories.txt");
  if (!traj_file.is_open())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open tt_trajectories.txt for writing.");
    return 1;
  }

  const int max_retries = 1;
  const int num_waypoints = 1000;  // Total steps
  int num_key_poses = key_poses.size();

  for (int i = 0; i < num_waypoints; i++)
  {
    // Cycle through key poses using modulo operator.
    std::vector<double> joint_target = key_poses[i % num_key_poses];

    // Log the target joint values.
    std::ostringstream oss;
    for (size_t j = 0; j < joint_target.size(); j++)
    {
      oss << joint_names[j] << ": " << joint_target[j] << "  ";
    }
    RCLCPP_INFO(node->get_logger(), "Moving to waypoint %d with joint targets: %s", i, oss.str().c_str());

    bool plan_success = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int attempt = 0; attempt < max_retries; attempt++)
    {
      move_group.setJointValueTarget(joint_target);
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "Plan computed for waypoint %d on attempt %d. Executing...", i, attempt + 1);
        if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(node->get_logger(), "Waypoint %d reached successfully.", i);
          plan_success = true;
          break;
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "Execution failed on attempt %d for waypoint %d.", attempt + 1, i);
        }
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Planning failed on attempt %d for waypoint %d.", attempt + 1, i);
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (plan_success)
    {
      // Write the trajectory data to the file.
      traj_file << "Trajectory for waypoint " << i << " (Key Pose index " << (i % num_key_poses) << "):\n";
      auto &joint_traj = plan.trajectory_.joint_trajectory;
      traj_file << "  Joint Names: ";
      for (const auto &name : joint_traj.joint_names)
      {
        traj_file << name << " ";
      }
      traj_file << "\n";

      // Write each trajectory point: time from start and joint positions.
      for (const auto &point : joint_traj.points)
      {
        traj_file << "  Time from start: " << point.time_from_start.sec << "." << point.time_from_start.nanosec << " sec\n";
        traj_file << "  Positions: ";
        for (const auto &pos : point.positions)
        {
          traj_file << pos << " ";
        }
        traj_file << "\n";
      }
      traj_file << "\n";  // Blank line between trajectories
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Skipping waypoint %d after %d failed attempts.", i, max_retries);
    }

    // Brief pause before moving to the next waypoint.
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  traj_file.close();
  RCLCPP_INFO(node->get_logger(), "Trajectories have been saved to tt_trajectories.txt");

  rclcpp::shutdown();
  return 0;
}
