#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <cmath>
#include <string>

using std::placeholders::_1;

class JointStateConverter : public rclcpp::Node
{
public:
  JointStateConverter()
  : Node("joint_state_converter")
  {
    // Subscribe to the /joint_states topic
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointStateConverter::joint_state_callback, this, _1));

    // Publisher for the converted joint states
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states_converted_js", 10);

    RCLCPP_INFO(this->get_logger(), "JointState Converter Node started.");
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto new_msg = sensor_msgs::msg::JointState();
    new_msg.header = msg->header;  // Retain header info (timestamp, frame, etc.)

    // Process each joint from the incoming message
    for (size_t i = 0; i < msg->name.size(); ++i) {
      // Process jlink1 (prismatic joint in meters, convert to centimeters)
      if (msg->name[i] == "jlink1") {
        new_msg.name.push_back(msg->name[i]);
        new_msg.position.push_back(msg->position[i] * 100.0);  // meters to centimeters
        if (i < msg->velocity.size()) {
          new_msg.velocity.push_back(msg->velocity[i]);
        }
        if (i < msg->effort.size()) {
          new_msg.effort.push_back(msg->effort[i]);
        }
      }
      // Process jlink2 to jlink4 (revolute joints in radians, convert to degrees and round)
      else if (msg->name[i] == "jlink2" ||
               msg->name[i] == "jlink3" ||
               msg->name[i] == "jlink4")
      {
        new_msg.name.push_back(msg->name[i]);
        double deg = std::round(msg->position[i] * (180.0 / M_PI));
        new_msg.position.push_back(deg);
        if (i < msg->velocity.size()) {
          new_msg.velocity.push_back(msg->velocity[i]);
        }
        if (i < msg->effort.size()) {
          new_msg.effort.push_back(msg->effort[i]);
        }
      }
      // Process jlink5 (revolute joint with clamped range [-10°, +10°])
      else if (msg->name[i] == "jlink5")
      {
        new_msg.name.push_back(msg->name[i]);
        double deg = std::round(msg->position[i] * (180.0 / M_PI));
        // Clamp the degree value to [-10, +10]
        if (deg < -10.0)
          deg = -10.0;
        else if (deg > 10.0)
          deg = 10.0;
        new_msg.position.push_back(deg);
        if (i < msg->velocity.size()) {
          new_msg.velocity.push_back(msg->velocity[i]);
        }
        if (i < msg->effort.size()) {
          new_msg.effort.push_back(msg->effort[i]);
        }
      }
    }
    // Publish the converted joint state message
    publisher_->publish(new_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateConverter>());
  rclcpp::shutdown();
  return 0;
}
