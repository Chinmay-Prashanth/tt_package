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

int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto logger = rclcpp::get_logger("move_program");

  // Create the MoveGroupInterface for the "arm" planning group
  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
  move_group.setPlanningTime(5.0);

  // Retrieve joint names; expect 6 joints with custom limits as specified.
  std::vector<std::string> joint_names = move_group.getJointNames();
  if (joint_names.size() != 6)
  {
    RCLCPP_ERROR(node->get_logger(), "Expected 6 joints, but found %zu.", joint_names.size());
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Found %zu joints.", joint_names.size());

  // Setup random generators for each joint with specified limits.
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dis_joint1(-0.5, 0.5);          // Joint 1: -0.5 to +0.5 (meters)
  std::uniform_real_distribution<double> dis_joint2(-M_PI, M_PI);           // Joint 2: -π to π
  std::uniform_real_distribution<double> dis_joint3(-1.39626, 1.39626);     // Joint 3: -80° to 80° in radians
  std::uniform_real_distribution<double> dis_joint4(-M_PI/2, M_PI/2);         // Joint 4: -π/2 to π/2
  std::uniform_real_distribution<double> dis_joint5(-M_PI/2, M_PI/2);         // Joint 5: -π/2 to π/2
  std::uniform_real_distribution<double> dis_joint6(-M_PI, M_PI);             // Joint 6: -π to π

  // Open an output file to store trajectories.
  std::ofstream traj_file("trajectories.txt");
  if (!traj_file.is_open())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open trajectories.txt for writing.");
    return 1;
  }
  
  const int max_retries = 1;
  const int num_waypoints = 1000;  // 1000 steps

  for (int i = 0; i < num_waypoints; i++)
  {
    // Generate a joint target vector for 6 joints.
    std::vector<double> joint_target(6);
    joint_target[0] = dis_joint1(gen);
    joint_target[1] = dis_joint2(gen);
    joint_target[2] = dis_joint3(gen);
    joint_target[3] = dis_joint4(gen);
    joint_target[4] = dis_joint5(gen);
    joint_target[5] = dis_joint6(gen);

    // Build a string for logging the joint target values.
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
      // Write the trajectory data to file.
      traj_file << "Trajectory for waypoint " << i << ":\n";
      // Access the joint trajectory from the plan.
      auto& joint_traj = plan.trajectory_.joint_trajectory;
      traj_file << "  Joint Names: ";
      for (const auto &name : joint_traj.joint_names)
      {
        traj_file << name << " ";
      }
      traj_file << "\n";

      // For each trajectory point, write out the time from start and positions.
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
    
    // Brief pause before the next waypoint.
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  traj_file.close();
  RCLCPP_INFO(node->get_logger(), "Trajectories have been saved to trajectories.txt");
  
  rclcpp::shutdown();
  return 0;
}
