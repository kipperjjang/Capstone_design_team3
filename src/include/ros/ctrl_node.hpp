#pragma once

#include <limits>
#include <memory>
#include <string>

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
  enum class TrackingMode {
    IDLE,
    WEBCAM_SEARCH,
    PICAM_TRACK,
    PICAM_PREDICT,
    PICAM_HOLD,
  };

  enum class ControlSource {
    NONE,
    WEBCAM_DETECTION,
    PICAM_DETECTION,
    PICAM_PREDICTION,
    JOINT_HOLD,
  };

  struct TickDecision {
    TrackingMode mode{TrackingMode::IDLE};
    ControlSource source{ControlSource::NONE};
    RobotState state;
    RobotState raw_state;
    ControlState control;
    bool has_raw{false};
    bool predicted_only{false};
    bool webcam_enabled{true};
    double picam_age{std::numeric_limits<double>::infinity()};
    double webcam_age{std::numeric_limits<double>::infinity()};
    double source_age{std::numeric_limits<double>::infinity()};
  };

  // Callback function
  void visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg);
  void jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg);
  void timerCallback();
  
  void publishControl(const ControlState &x);
  void publishDebug(const TickDecision &decision);
  void publishWebcamEnabled(bool enabled);

  TickDecision chooseControl(double now);
  TickDecision usePicamMeasurement(double now);
  TickDecision usePicamPrediction(double now, double picam_age);
  TickDecision usePicamHold(double now, double picam_age);
  TickDecision useWebcamSearch(double now);
  TickDecision useIdleHold(double now);
  bool hasNewPicamDetection() const;
  bool hasFreshWebcamDetection(double now) const;
  double picamAge(double now) const;
  double webcamAge(double now) const;
  RobotState withLatestJoint(RobotState state) const;
  static const char* trackingModeName(TrackingMode mode);
  static const char* controlSourceName(ControlSource source);

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
  double last_processed_picam_detection_receive_time_{0.0};
  double last_picam_detection_time_{0.0};
  bool has_picam_lock_{false};
  std::unique_ptr<Estimator> estimator_;
  std::unique_ptr<Controller> controller_;
};
