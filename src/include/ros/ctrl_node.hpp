#pragma once

#include <rclcpp/rclcpp.hpp>
#include "estimator/estimator.hpp"
#include "controller/controller.hpp"

#include "custom_msgs/msg/vision_msg.hpp"
#include "custom_msgs/msg/control_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"

class CtrlNode : public rclcpp::Node {
public:
  CtrlNode();

private:
  // Callback function
  void visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timerCallback();
  
  void publishControl(const ControlState &x);

  // ROS
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<custom_msgs::msg::ControlMsg>::SharedPtr ctrl_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Time last_imu_time_;
  bool first_imu_{true};

  // Sub-modules
  Estimator estimator_;
  Controller controller_;
};
