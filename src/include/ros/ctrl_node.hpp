#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "controller/controller.hpp"
#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/vision_msg.hpp"
#include "estimator/estimator.hpp"

class CtrlNode : public rclcpp::Node {
public:
  CtrlNode();

private:
  // Callback function
  void visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg);
  // void jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg);
  void timerCallback();
  
  void publishControl(const ControlState &x);

  // ROS
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_sub_;
  // rclcpp::Subscription<custom_msgs::msg::JointMsg>::SharedPtr joint_sub_;
  rclcpp::Publisher<custom_msgs::msg::ControlMsg>::SharedPtr ctrl_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<Estimator> estimator_;
  std::unique_ptr<Controller> controller_;
};
