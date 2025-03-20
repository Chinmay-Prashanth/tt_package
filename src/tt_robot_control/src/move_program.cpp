#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point_stamped.hpp>

class MoveProgram : public rclcpp::Node
{
public:
    MoveProgram() : Node("move_program")
    {
        // Create a MoveGroupInterface for the "arm" planning group
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::shared_ptr<rclcpp::Node>(this, [](auto*){/* null deleter */}), "arm");

        // Subscribe to predicted ball position
        ball_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "predicted_ball_position", 10,
            std::bind(&MoveProgram::ball_position_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Move program initialized, waiting for ball position...");
    }

    void ball_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Define a goal pose
        tf2::Quaternion tf2_quat;
        // For a 5-DOF robot, we'll keep the paddle facing down
        tf2_quat.setRPY(0, 0, -3.14 / 2);  // Set orientation (roll, pitch, yaw)
        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
        geometry_msgs::msg::Pose goal_pose;
        goal_pose.orientation = msg_quat;
        
        // Use predicted ball position as goal
        goal_pose.position.x = msg->point.x;
        goal_pose.position.y = msg->point.y;
        goal_pose.position.z = msg->point.z;

        RCLCPP_INFO(this->get_logger(), 
            "Received predicted ball position: x=%.3f, y=%.3f, z=%.3f",
            goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);

        // Set the goal pose
        move_group_interface_->setPoseTarget(goal_pose);

        // Plan to the goal pose
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool planning_success = static_cast<bool>(move_group_interface_->plan(plan));

        if (planning_success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing trajectory");
            // Execute the planned trajectory
            move_group_interface_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No plan found");
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball_pos_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveProgram>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
