#pragma once

#include "data/state/robot_state.hpp"
#include "estimator/estimator.hpp"

#include <cstddef>
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
  void recordResidual(const RobotState &raw_state, const RobotState &estimated_state);
  void logMetricsIfDue();

  // Estimator
  std::shared_ptr<Estimator> estimator_;
  RobotState last_raw_state_;
  bool has_raw_state_{false};
  double prediction_horizon_sec_{0.0};
  double metrics_log_period_sec_{1.0};
  rclcpp::Time last_metrics_log_time_;
  std::size_t measurement_count_{0};
  double residual_norm_sum_{0.0};
  double residual_norm_sq_sum_{0.0};
  double residual_norm_max_{0.0};

  // ROS interfaces
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_sub_;
  rclcpp::Publisher<custom_msgs::msg::TestDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
