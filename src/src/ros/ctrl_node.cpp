#include "ros/ctrl_node.hpp"

#include "data/config/control_config.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

using std::placeholders::_1;

CtrlNode::CtrlNode() : Node("control_node") {
  // Read Configuration
  std::string config_path;
  std::string vision_topic;
  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("vision_topic", "/vision");
  this->get_parameter("config_path", config_path);
  this->get_parameter("vision_topic", vision_topic);

  const EstimatorConfig estimator_config = EstimatorConfig::load(config_path);
  const ControlConfig controller_config = ControlConfig::load(config_path);
  
  // Initilaize
  estimator_ = std::make_unique<Estimator>(estimator_config);
  controller_ = std::make_unique<Controller>(controller_config);
  
  // Subscriber and Publisher
  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(
    vision_topic, 1, std::bind(&CtrlNode::visionCallback, this, _1));
  ctrl_pub_ = this->create_publisher<custom_msgs::msg::ControlMsg>("/control", 10);

  const int period = static_cast<int>(1000.0 / controller_config.hz);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&CtrlNode::timerCallback, this));
}

// void CtrlNode::jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg) {
//   // Cache joint data
//   auto now = this->now();
//   if (first_joint_) {
//     last_joint_time_ = now;
//     first_joint_ = false;
//   }
//   const double dt = (now - last_joint_time_).seconds();

//   joint_ = toEigen(msg->joint);
//   joint_vel_ = toEigen(msg->joint_vel);
//   if (estimator_.isInitialized()) {
//     estimator_.update(joint_, joint_vel_, dt); 
//   }
// }

void CtrlNode::visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
  if (msg == nullptr || msg->p.size() < 2) return;

  const bool has_measurement = msg->detected || msg->tracked;
  if (!has_measurement) return;

  RobotState vision_state(msg);

  if (!estimator_->isInitialized()) {
    estimator_->init(vision_state);
  } else {
    estimator_->update(vision_state);
  }

  // Run Controller
  const RobotState state = estimator_->getState(false);
  controller_->run(state);
  publishControl(controller_->getControl());
}

void CtrlNode::timerCallback() {
  // Update with the process model
  if (!estimator_->isInitialized()) return;
  
  // Update Estimator state (predicted state)
  const double t_now = this->now().seconds();
  estimator_->update(t_now);

  // Run Controller
  auto state = estimator_->getState(true);
  controller_->run(state);

  // Publish control input to the plant
  publishControl(controller_->getControl());
}

void CtrlNode::publishControl(const ControlState &x) {
  custom_msgs::msg::ControlMsg msg;
  msg.header.stamp = this->now();
  msg.u_yaw = static_cast<float>(x.u_yaw);
  msg.u_pitch = static_cast<float>(x.u_pitch);
  msg.fire = x.fire;
  msg.reload = x.reload;
  ctrl_pub_->publish(msg);
}
