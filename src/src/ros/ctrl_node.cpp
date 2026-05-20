#include "ros/ctrl_node.hpp"

#include "data/config/control_config.hpp"
#include "utils/eigen_utils.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>

using std::placeholders::_1;

namespace {
void fillVector2(std::array<double, 2> &output, const Eigen::Vector2d &input) {
  output[0] = input.x();
  output[1] = input.y();
}
}  // namespace

CtrlNode::CtrlNode() : Node("control_node") {
  // Read Configuration
  std::string config_path;
  std::string vision_topic;
  std::string vision_webcam_topic;
  std::string vision_picam_topic;
  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("vision_topic", "/vision");
  this->declare_parameter<std::string>("vision_webcam_topic", "/vision_webcam");
  this->declare_parameter<std::string>("vision_picam_topic", "/vision_picam");
  this->get_parameter("config_path", config_path);
  this->get_parameter("vision_topic", vision_topic);
  this->get_parameter("vision_webcam_topic", vision_webcam_topic);
  this->get_parameter("vision_picam_topic", vision_picam_topic);

  const EstimatorConfig estimator_config = EstimatorConfig::load(config_path);
  const ControlConfig controller_config = ControlConfig::load(config_path);
  
  // Initilaize
  estimator_ = std::make_unique<Estimator>(estimator_config);
  controller_ = std::make_unique<Controller>(controller_config);
  
  // Subscriber and Publisher
  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_topic, 1, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_webcam_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_webcam_topic, 1, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_picam_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_picam_topic, 1, std::bind(&CtrlNode::visionCallback, this, _1));
  joint_sub_ = this->create_subscription<custom_msgs::msg::JointMsg>("/joint", 1, std::bind(&CtrlNode::jointCallback, this, _1));
  ctrl_pub_ = this->create_publisher<custom_msgs::msg::ControlMsg>("/control", 1);
  debug_pub_ = this->create_publisher<custom_msgs::msg::TestDebug>("/test/debug", 1);

  const int period = static_cast<int>(1000.0 / controller_config.hz);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&CtrlNode::timerCallback, this));
}

void CtrlNode::jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg) {
  // Cache joint data
  if (!first_joint_) first_joint_ = true;

  joint_ = toEigen(msg->joint);
  joint_vel_ = toEigen(msg->joint_vel);
}

void CtrlNode::visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
  if (msg == nullptr || msg->p.size() < 2) return;

  const bool has_measurement = msg->detected || msg->tracked;
  if (!has_measurement) return;

  RobotState vision_state(msg);
  last_raw_state_ = vision_state;
  has_raw_state_ = true;
  
  // Current time
  const double t_now = this->now().seconds();
  vision_state.t = t_now;

  // Update estimator
  RobotState state;
  if (vision_state.camera == "picam") {
    if (!estimator_->isInitialized()) {
      estimator_->init(vision_state);
    } else {
      estimator_->update(vision_state);
    }
    // Joint value update
    estimator_->update(joint_, joint_vel_);
    state = estimator_->getState(false);

    // Store last picam time
    last_picam_time_ = t_now;
  } else {
    state = vision_state;
    state.joint = joint_;
    state.joint_vel = joint_vel_;

    // Store last picam time
    state.t = last_picam_time_;
  }

  controller_->run(state);
  publishControl(controller_->getControl());
  publishDebug(state, controller_->getControl(), true, false);
}

void CtrlNode::timerCallback() {
  // // Update with the process model
  if (!estimator_->isInitialized()) return;
  
  // Update Estimator state (predicted state)
  const double t_now = this->now().seconds();
  estimator_->update(t_now);

  // Run Controller
  auto state = estimator_->getState(true);
  // auto state = estimator_->getState(estimator_->config_.prediction_time);
  controller_->run(state);

  // Publish control input to the plant
  publishControl(controller_->getControl());
  publishDebug(state, controller_->getControl(), false, true);
}

void CtrlNode::publishControl(const ControlState &x) {
  custom_msgs::msg::ControlMsg msg;
  msg.header.stamp = this->now();
  msg.u_yaw = static_cast<float>(x.u_yaw);
  msg.u_pitch = static_cast<float>(x.u_pitch);
  msg.fire = x.fire;
  msg.reload = x.reload;
  msg.manual = false;
  ctrl_pub_->publish(msg);
}

void CtrlNode::publishDebug(const RobotState &estimated_state, const ControlState &control_state,
                            bool has_raw, bool predicted_only) {
  custom_msgs::msg::TestDebug msg;
  msg.header.stamp = this->now();
  msg.sample_time = (has_raw && has_raw_state_) ? last_raw_state_.t : estimated_state.t;
  msg.dt = estimated_state.dt;

  if (has_raw_state_) {
    fillVector2(msg.raw_p, last_raw_state_.p);
    fillVector2(msg.raw_v, last_raw_state_.v);
    fillVector2(msg.raw_a, last_raw_state_.a);
    msg.detected = last_raw_state_.detected;
    msg.tracked = last_raw_state_.tracked;
  }

  fillVector2(msg.estimated_p, estimated_state.p);
  fillVector2(msg.estimated_v, estimated_state.v);
  fillVector2(msg.estimated_a, estimated_state.a);

  msg.has_raw = has_raw && has_raw_state_;
  msg.estimator_initialized = estimator_->isInitialized();
  msg.predicted_only = predicted_only;

  msg.has_control = true;
  msg.u_yaw = control_state.u_yaw;
  msg.u_pitch = control_state.u_pitch;
  msg.fire = control_state.fire;
  msg.reload = control_state.reload;

  debug_pub_->publish(msg);
}
