#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
 
  auto logger = rclcpp::get_logger("move_program");

  // Create a MoveGroupInterface for the "arm" planning group
  moveit::planning_interface::MoveGroupInterface move_group_interface(node, "arm");

  // Define a goal pose
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -3.14 / 2);  // Set orientation (roll, pitch, yaw)
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
  geometry_msgs::msg::Pose goal_pose;
  goal_pose.orientation = msg_quat;
  goal_pose.position.x = 0.4;
  goal_pose.position.y = -0.3;
  goal_pose.position.z = 0.15;  // Corrected: remove the extraneous 's'

  move_group_interface.setPoseTarget(goal_pose);

  // Plan to the goal pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planning_success = static_cast<bool>(move_group_interface.plan(plan));

  if (planning_success)
  {
    // Execute the planned trajectory
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "No plan found");
  }

  rclcpp::shutdown();
  return 0;
}
