#pragma once

#include <rclcpp/rclcpp.hpp>
#include "estimator/estimator.hpp"
#include "controller/controller.hpp"

#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/imu_msg.hpp"

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode();

private:
  // Callback function
  void controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg);
  
  // Utils
  bool readIMUframe();
  
  // ROS
  rclcpp::Subscription<custom_msgs::msg::ControlMsg>::SharedPtr control_sub_;
  rclcpp::Publisher<custom_msgs::msg::ImuMsg>::SharedPtr imu_pub_;         // IMU data
  
  // Serial
  std::vector<Serial> serials_;
  std::vector<uint8_t> buffer_;

  // Data
  sensor_msgs::msg::Imu imu_;
};
