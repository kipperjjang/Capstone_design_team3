#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "bridge/serial.hpp"
#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/joint_msg.hpp"

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode();

private:
  // Callback function
  void controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg);
  void timerCallback();
  
  // Utils
  bool readSerialFrame();

  // ROS
  rclcpp::Subscription<custom_msgs::msg::ControlMsg>::SharedPtr control_sub_;
  rclcpp::Publisher<custom_msgs::msg::JointMsg>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Serial
  std::vector<std::unique_ptr<Serial>> input_;
  std::vector<std::unique_ptr<Serial>> output_;
  std::vector<uint8_t> buffer_;
  bool warned_about_protocol_{false};
};
