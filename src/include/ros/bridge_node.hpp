#pragma once

#include <rclcpp/rclcpp.hpp>
#include "estimator/estimator.hpp"
#include "controller/controller.hpp"

#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/joint_msg.hpp"

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode();

private:
  // Callback function
  void controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg);
  
  // Utils
  bool readSerialFrame();
  
  // ROS
  rclcpp::Subscription<custom_msgs::msg::ControlMsg>::SharedPtr control_sub_;
  rclcpp::Publisher<custom_msgs::msg::JointMsg>::SharedPtr joint_pub_;
  
  // Serial
  std::vector<Serial> input_;     // Input port for reading
  std::vector<Serial> output_;    // Output port for writing

  std::vector<uint8_t> buffer_;
};
