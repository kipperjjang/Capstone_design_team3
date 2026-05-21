#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "controller/controller.hpp"
#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/joint_msg.hpp"
#include "custom_msgs/msg/test_debug.hpp"
#include "custom_msgs/msg/vision_msg.hpp"
#include "estimator/estimator.hpp"
#include "std_msgs/msg/bool.hpp"

class CtrlNode : public rclcpp::Node {
public:
  CtrlNode();

private:
  // Callback function
  void visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg);
  void jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg);
  void timerCallback();
  
  void publishControl(const ControlState &x);
  void publishDebug(const RobotState &estimated_state, const ControlState &control_state,
                    bool has_raw, bool predicted_only);
  void publishWebcamEnabled(bool enabled);
  bool processPicamMeasurement(const RobotState &vision_state);
  bool processWebcamMeasurement(const RobotState &vision_state);
  bool hasTarget(const RobotState &state) const;

  // ROS
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_sub_;
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_webcam_sub_;
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_picam_sub_;
  rclcpp::Subscription<custom_msgs::msg::JointMsg>::SharedPtr joint_sub_;
  rclcpp::Publisher<custom_msgs::msg::ControlMsg>::SharedPtr ctrl_pub_;
  rclcpp::Publisher<custom_msgs::msg::TestDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr webcam_enabled_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool first_joint_{false};
  Eigen::Vector2d joint_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d joint_vel_{Eigen::Vector2d::Zero()};

  RobotState last_raw_state_;
  bool has_raw_state_{false};

  RobotState latest_picam_state_;
  RobotState latest_webcam_state_;
  bool has_latest_picam_state_{false};
  bool has_latest_webcam_state_{false};
  double latest_picam_receive_time_{0.0};
  double latest_webcam_receive_time_{0.0};
  double last_processed_picam_receive_time_{0.0};
  double last_processed_webcam_receive_time_{0.0};
  double last_picam_time_{0.0};
  double webcam_pause_hold_sec_{0.5};
  std::unique_ptr<Estimator> estimator_;
  std::unique_ptr<Controller> controller_;
};
