#pragma once

#include "data/state/robot_state.hpp"
#include "estimator/estimator.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "custom_msgs/msg/test_debug.hpp"
#include "custom_msgs/msg/vision_msg.hpp"

class TestNode : public rclcpp::Node {
public:
  TestNode();
  virtual ~TestNode() = default;

private:
  void visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg);
  void timerCallback();
  void publishDebug(const RobotState &estimated_state, bool has_raw, bool predicted_only);

  // Estimator
  std::shared_ptr<Estimator> estimator_;
  RobotState last_raw_state_;
  bool has_raw_state_{false};

  // ROS interfaces
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_sub_;
  rclcpp::Publisher<custom_msgs::msg::TestDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
