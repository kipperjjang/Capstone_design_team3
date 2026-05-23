#include "ros/ctrl_node.hpp"

#include "data/config/control_config.hpp"
#include "utils/eigen_utils.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

using std::placeholders::_1;

namespace {
void fillVector2(std::array<double, 2> &output, const Eigen::Vector2d &input) {
  output[0] = input.x();
  output[1] = input.y();
}
}  // namespace

CtrlNode::CtrlNode() : Node("control_node") {
  std::string config_path;
  std::string vision_topic, vision_webcam_topic, vision_picam_topic;
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

  estimator_ = std::make_unique<Estimator>(estimator_config);
  controller_ = std::make_unique<Controller>(controller_config);
  fsm_ = std::make_unique<ControlFSM>(controller_config);

  const auto qos_latest = rclcpp::QoS(rclcpp::KeepLast(1));
  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_webcam_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_webcam_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_picam_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_picam_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  joint_sub_ = this->create_subscription<custom_msgs::msg::JointMsg>("/joint", qos_latest, std::bind(&CtrlNode::jointCallback, this, _1));

  ctrl_pub_ = this->create_publisher<custom_msgs::msg::ControlMsg>("/control", qos_latest);
  debug_pub_ = this->create_publisher<custom_msgs::msg::TestDebug>("/test/debug", qos_latest);
  webcam_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision_webcam/enabled", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  const int period = std::max(1, static_cast<int>(1000.0 / controller_config.hz));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&CtrlNode::timerCallback, this));
}

void CtrlNode::jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg) {
  if (msg == nullptr) return;
  if (!first_joint_) first_joint_ = true;

  joint_ = toEigen(msg->joint);
  joint_vel_ = toEigen(msg->joint_vel);
  if (estimator_) {
    estimator_->updateJoint(joint_, joint_vel_, this->now().seconds());
  }
}

void CtrlNode::visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
  if (msg == nullptr) return;
  if (msg->detected && msg->p.size() < 2) return;

  RobotState vision_state(msg);
  if (!vision_state.detected) return;

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

void CtrlNode::timerCallback() {
  const double now = this->now().seconds();
  expirePicamLock(now);

  const ControlFSMInput fsm_input = buildFSMInput(now);
  const ControlFSMOutput fsm_output = fsm_->update(fsm_input);
  updateHoldJointTarget(fsm_output.mode);
  const ControlTick tick = executeMode(fsm_output, now);

  publishWebcamEnabled(fsm_output.webcam_enabled);
  publishControl(tick.control);
  publishDebug(fsm_input, fsm_output, tick);
}

bool CtrlNode::hasNewPicamTarget() const {
  return has_latest_picam_state_ && latest_picam_state_.detected &&latest_picam_receive_time_ > last_processed_picam_target_receive_time_;
}

double CtrlNode::picamAge(double now) const {
  if (!has_picam_lock_) return std::numeric_limits<double>::infinity();
  return now - last_picam_target_time_;
}

double CtrlNode::webcamAge(double now) const {
  if (!has_latest_webcam_state_ || !latest_webcam_state_.detected) {
    return std::numeric_limits<double>::infinity();
  }
  return now - latest_webcam_receive_time_;
}

bool CtrlNode::hasFreshWebcamTarget(double now) const {
  return webcamAge(now) <= controller_->config().webcam_measurement_max_age;
}

void CtrlNode::expirePicamLock(double now) {
  if (!has_picam_lock_) return;
  if (picamAge(now) > controller_->config().picam_track_hold_sec) {
    has_picam_lock_ = false;
  }
}

bool CtrlNode::isJointHoldMode(TrackingMode mode) const {
  return mode == TrackingMode::IDLE || mode == TrackingMode::PICAM_HOLD;
}

void CtrlNode::updateHoldJointTarget(TrackingMode mode) {
  if (!isJointHoldMode(mode)) {
    has_hold_joint_target_ = false;
    hold_joint_mode_ = mode;
    return;
  }

  if (has_hold_joint_target_ && hold_joint_mode_ == mode) {
    return;
  }

  if (!first_joint_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "No /joint sample received yet; holding zero joint target.");
  }

  hold_joint_target_ = first_joint_ ? joint_ : Eigen::Vector2d::Zero();
  hold_joint_mode_ = mode;
  has_hold_joint_target_ = true;
}

const Eigen::Vector2d& CtrlNode::holdJointTarget() const {
  return has_hold_joint_target_ ? hold_joint_target_ : joint_;
}

RobotState CtrlNode::withLatestJoint(RobotState state) const {
  state.joint = joint_;
  state.joint_vel = joint_vel_;
  return state;
}

ControlFSMInput CtrlNode::buildFSMInput(double now) const {
  ControlFSMInput input;
  input.has_new_picam_target = hasNewPicamTarget();
  input.has_picam_lock = has_picam_lock_;
  input.estimator_initialized = estimator_ && estimator_->isInitialized();
  input.has_fresh_webcam_target = hasFreshWebcamTarget(now);
  input.picam_age = picamAge(now);
  input.webcam_age = webcamAge(now);
  return input;
}

CtrlNode::ControlTick CtrlNode::executeMode(const ControlFSMOutput &fsm_output, double now) {
  switch (fsm_output.mode) {
    case TrackingMode::PICAM_TRACK:
      return trackPicam(now);
    case TrackingMode::PICAM_PREDICT:
      return predictPicam(now, picamAge(now));
    case TrackingMode::PICAM_HOLD:
      return holdPicam(now, picamAge(now));
    case TrackingMode::WEBCAM_SEARCH:
      return searchWebcam(now);
    case TrackingMode::IDLE:
    default:
      return holdIdle(now);
  }
}

CtrlNode::ControlTick CtrlNode::trackPicam(double now) {
  ControlTick tick;
  tick.source_age = now - latest_picam_receive_time_;

  RobotState measurement = latest_picam_state_;
  last_processed_picam_target_receive_time_ = latest_picam_receive_time_;
  last_picam_target_time_ = measurement.t;
  has_picam_lock_ = true;

  // Update estimator state with latest measurement
  estimator_->update(measurement);

  tick.raw_state = measurement;
  tick.has_raw = true;
  tick.state = estimator_->isInitialized() ? estimator_->updatedState() : withLatestJoint(measurement);
  tick.state.camera = "picam";
  tick.state.detected = measurement.detected;
  tick.control = controller_->computePicamPixelTrack(tick.state);
  return tick;
}

CtrlNode::ControlTick CtrlNode::predictPicam(double now, double picam_age) {
  ControlTick tick;
  tick.source_age = picam_age;
  tick.state = estimator_->updatedState();  // No prediction currently
  tick.state.camera = "picam";
  tick.state.detected = false;
  tick.state.process = true;
  tick.state.dt = picam_age;
  tick.control = controller_->computePicamPixelTrack(tick.state);
  return tick;
}

CtrlNode::ControlTick CtrlNode::holdPicam(double now, double picam_age) {
  (void)now;
  ControlTick tick;
  tick.source_age = picam_age;
  if (estimator_->isInitialized()) {
    tick.state = estimator_->updatedState();            // Get updated state with last measurement
  } else if (has_latest_picam_state_) {
    tick.state = withLatestJoint(latest_picam_state_);  // If failed to call estimator, use raw previous measurement
  } else {
    tick.state = withLatestJoint(RobotState());         // If failed, use default RobotState
  }
  tick.state.camera = "picam";
  tick.state.detected = false;
  tick.state.process = false;
  tick.state.dt = picam_age;
  tick.control = controller_->computeJointHold(holdJointTarget());
  return tick;
}

CtrlNode::ControlTick CtrlNode::searchWebcam(double now) {
  ControlTick tick;
  tick.source_age = webcamAge(now);
  RobotState measurement = latest_webcam_state_;

  tick.raw_state = measurement;
  tick.has_raw = true;
  tick.state = withLatestJoint(measurement);  // For webcam, use pure measurement
  tick.state.camera = "webcam";
  tick.state.dt = webcamAge(now);
  tick.control = controller_->computeWebcamAngleSearch(tick.state);
  return tick;
}

CtrlNode::ControlTick CtrlNode::holdIdle(double now) {
  ControlTick tick;
  tick.source_age = 0.0;
  tick.state = withLatestJoint(RobotState());
  tick.state.t = now;
  tick.control = controller_->computeJointHold(holdJointTarget());
  return tick;
}

void CtrlNode::publishControl(const ControlState &control) {
  custom_msgs::msg::ControlMsg msg;
  msg.header.stamp = this->now();
  msg.u_yaw = static_cast<float>(control.u_yaw);
  msg.u_pitch = static_cast<float>(control.u_pitch);
  msg.ispixel = control.isPixel;
  ctrl_pub_->publish(msg);
}

void CtrlNode::publishWebcamEnabled(bool enabled) {
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  webcam_enabled_pub_->publish(msg);
}

void CtrlNode::publishDebug(const ControlFSMInput &fsm_input, const ControlFSMOutput &fsm_output, const ControlTick &tick) {
  custom_msgs::msg::TestDebug msg;
  msg.header.stamp = this->now();
  msg.sample_time = tick.has_raw ? tick.raw_state.t : tick.state.t;
  msg.dt = tick.state.dt;

  if (tick.has_raw) {
    fillVector2(msg.raw_p, tick.raw_state.p);
    fillVector2(msg.raw_v, tick.raw_state.v);
    fillVector2(msg.raw_a, tick.raw_state.a);
    msg.detected = tick.raw_state.detected;
  } else {
    msg.detected = tick.state.detected;
  }

  fillVector2(msg.estimated_p, tick.state.p);
  fillVector2(msg.estimated_v, tick.state.v);
  fillVector2(msg.estimated_a, tick.state.a);

  msg.has_raw = tick.has_raw;
  msg.estimator_initialized = estimator_->isInitialized();
  msg.predicted_only = fsm_output.predicted_only;

  msg.has_control = true;
  msg.u_yaw = tick.control.u_yaw;
  msg.u_pitch = tick.control.u_pitch;
  msg.tracking_mode = ControlFSM::trackingModeName(fsm_output.mode);
  msg.control_source = ControlFSM::controlSourceName(fsm_output.source);
  msg.is_pixel = tick.control.isPixel;
  msg.webcam_enabled = fsm_output.webcam_enabled;
  msg.picam_age = std::isfinite(fsm_input.picam_age) ? fsm_input.picam_age : -1.0;
  msg.webcam_age = std::isfinite(fsm_input.webcam_age) ? fsm_input.webcam_age : -1.0;
  msg.source_age = std::isfinite(tick.source_age) ? tick.source_age : -1.0;

  debug_pub_->publish(msg);
}
