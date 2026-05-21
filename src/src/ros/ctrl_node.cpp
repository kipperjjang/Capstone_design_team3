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
  this->declare_parameter<double>("webcam_pause_hold_sec", 0.0);
  this->get_parameter("config_path", config_path);
  this->get_parameter("vision_topic", vision_topic);
  this->get_parameter("vision_webcam_topic", vision_webcam_topic);
  this->get_parameter("vision_picam_topic", vision_picam_topic);

  const EstimatorConfig estimator_config = EstimatorConfig::load(config_path);
  const ControlConfig controller_config = ControlConfig::load(config_path);

  // Initilaize
  estimator_ = std::make_unique<Estimator>(estimator_config);
  controller_ = std::make_unique<Controller>(controller_config);
  this->get_parameter("webcam_pause_hold_sec", webcam_pause_hold_sec_);
  if (webcam_pause_hold_sec_ <= 0.0) {
    webcam_pause_hold_sec_ = estimator_config.max_time_gap;
  }

  // Subscriber and Publisher
  const auto qos_latest = rclcpp::QoS(rclcpp::KeepLast(1));
  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_webcam_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_webcam_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_picam_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_picam_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  joint_sub_ = this->create_subscription<custom_msgs::msg::JointMsg>("/joint", qos_latest, std::bind(&CtrlNode::jointCallback, this, _1));
  ctrl_pub_ = this->create_publisher<custom_msgs::msg::ControlMsg>("/control", qos_latest);
  debug_pub_ = this->create_publisher<custom_msgs::msg::TestDebug>("/test/debug", qos_latest);
  webcam_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/vision_webcam/enabled", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

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
  if (msg == nullptr) return;
  if ((msg->detected || msg->tracked) && msg->p.size() < 2) return;

  RobotState vision_state(msg);
  
  const double receive_time = this->now().seconds();
  vision_state.t = receive_time;

  if (vision_state.camera == "picam") {
    latest_picam_state_ = vision_state;
    latest_picam_receive_time_ = receive_time;
    has_latest_picam_state_ = true;
  } else {
    latest_webcam_state_ = vision_state;
    latest_webcam_receive_time_ = receive_time;
    has_latest_webcam_state_ = true;
  }
}

bool CtrlNode::hasTarget(const RobotState &state) const {
  return state.detected || state.tracked;
}

bool CtrlNode::processPicamMeasurement(const RobotState &vision_state) {
  if (!hasTarget(vision_state)) return false;

  last_raw_state_ = vision_state;
  has_raw_state_ = true;

  if (!estimator_->isInitialized()) {
    estimator_->init(vision_state);
  } else {
    estimator_->update(vision_state);
  }

  RobotState state = estimator_->isInitialized() ? estimator_->getState(false) : vision_state;
  state.joint = joint_;
  state.joint_vel = joint_vel_;
  last_picam_time_ = vision_state.t;

  controller_->run(state);
  publishControl(controller_->getControl());
  publishDebug(state, controller_->getControl(), true, false);
  return true;
}

bool CtrlNode::processWebcamMeasurement(const RobotState &vision_state) {
  if (!hasTarget(vision_state)) return false;

  RobotState state = vision_state;
  state.joint = joint_;
  state.joint_vel = joint_vel_;
  if (last_picam_time_ > 0.0) {
    state.dt = vision_state.t - last_picam_time_;
  }

  last_raw_state_ = state;
  has_raw_state_ = true;

  controller_->run(state);
  publishControl(controller_->getControl());
  publishDebug(state, controller_->getControl(), true, false);
  return true;
}

void CtrlNode::timerCallback() {
  const double t_now = this->now().seconds();
  const bool has_recent_picam_target =
      last_picam_time_ > 0.0 && t_now - last_picam_time_ <= estimator_->config_.max_time_gap;
  publishWebcamEnabled(!(last_picam_time_ > 0.0 && t_now - last_picam_time_ <= webcam_pause_hold_sec_));

  bool ran_controller = false;
  if (has_latest_picam_state_ && latest_picam_receive_time_ > last_processed_picam_receive_time_) {
    last_processed_picam_receive_time_ = latest_picam_receive_time_;
    ran_controller = processPicamMeasurement(latest_picam_state_);
  }

  if (!ran_controller && estimator_->isInitialized()) {
    estimator_->update(t_now);
    if (estimator_->isInitialized()) {
      auto state = estimator_->getState(true);
      state.joint = joint_;
      state.joint_vel = joint_vel_;
      controller_->run(state);
      publishControl(controller_->getControl());
      publishDebug(state, controller_->getControl(), false, true);
      ran_controller = true;
    }
  }

  if (!ran_controller && !has_recent_picam_target && has_latest_webcam_state_ &&
      latest_webcam_receive_time_ > last_processed_webcam_receive_time_ &&
      t_now - latest_webcam_receive_time_ <= estimator_->config_.max_time_gap) {
    last_processed_webcam_receive_time_ = latest_webcam_receive_time_;
    ran_controller = processWebcamMeasurement(latest_webcam_state_);
  }

  if (!ran_controller) {
    publishControl(controller_->getControl());
  }
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

void CtrlNode::publishWebcamEnabled(bool enabled) {
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  webcam_enabled_pub_->publish(msg);
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
