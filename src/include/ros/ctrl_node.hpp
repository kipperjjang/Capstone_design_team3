#pragma once

#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "controller/controller.hpp"
#include "controller/fsm.hpp"
#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/joint_msg.hpp"
#include "custom_msgs/msg/test_debug.hpp"
#include "custom_msgs/msg/vision_msg.hpp"
#include "std_msgs/msg/bool.hpp"

class CtrlNode : public rclcpp::Node {
public:
  CtrlNode();

private:
  struct ControlTick {
    RobotState state;
    RobotState raw_state;
    ControlState control;
    bool has_raw_sample{false};
    double source_age{std::numeric_limits<double>::infinity()};
  };

  // Callback functions
  void visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg);
  void jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg);
  void timerCallback();

  // Utils
  ControlFSMInput buildFSMInput(double now) const;
  ControlTick executeMode(const ControlFSMOutput &fsm_output, double now);
  ControlTick trackPicam(double now);
  ControlTick holdPicam(double now, double picam_age);
  ControlTick searchWebcam(double now);
  ControlTick holdIdle(double now);
  RobotState filterPicamMeasurement(const RobotState &measurement);
  RobotState reusablePicamState(double now) const;

  void publishControl(const ControlState &control);
  void publishDebug(const ControlFSMInput &fsm_input, const ControlFSMOutput &fsm_output, const ControlTick &tick);
  void publishWebcamEnabled(bool enabled);

  void expirePicamLock(double now);
  bool isJointHoldMode(TrackingMode mode) const;
  void updateHoldJointTarget(TrackingMode mode);
  const Eigen::Vector2d& holdJointTarget() const;
  bool hasNewPicamTarget() const;
  bool hasFreshPicamTarget(double now) const;
  bool hasFreshWebcamTarget(double now) const;
  double picamAge(double now) const;
  double webcamAge(double now) const;
  RobotState withLatestJoint(RobotState state) const;
  bool shouldPublishControl(const ControlFSMOutput &fsm_output) const;

  // ROS
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_sub_;
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_webcam_sub_;
  rclcpp::Subscription<custom_msgs::msg::VisionMsg>::SharedPtr vision_picam_sub_;
  rclcpp::Subscription<custom_msgs::msg::JointMsg>::SharedPtr joint_sub_;
  rclcpp::Publisher<custom_msgs::msg::ControlMsg>::SharedPtr ctrl_pub_;
  rclcpp::Publisher<custom_msgs::msg::TestDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr webcam_enabled_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Joint
  bool first_joint_{false};
  Eigen::Vector2d joint_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d joint_vel_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d hold_joint_target_{Eigen::Vector2d::Zero()};
  bool has_hold_joint_target_{false};
  TrackingMode hold_joint_mode_{TrackingMode::IDLE};

  // Detection
  RobotState latest_picam_state_;
  RobotState latest_webcam_state_;
  bool has_latest_picam_state_{false};
  bool has_latest_webcam_state_{false};
  double last_consumed_picam_time_{0.0};
  bool has_picam_lock_{false};
  RobotState filtered_picam_state_;
  bool has_filtered_picam_state_{false};

  std::unique_ptr<Controller> controller_;
  std::unique_ptr<ControlFSM> fsm_;
};
