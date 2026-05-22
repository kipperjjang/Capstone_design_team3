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
  // Read Configuration
  std::string config_path;
  std::string vision_topic, vision_webcam_topic, vision_picam_topic;
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
  estimator_  = std::make_unique<Estimator>(estimator_config);
  controller_ = std::make_unique<Controller>(controller_config);

  // Subscriber and Publisher
  const auto qos_latest = rclcpp::QoS(rclcpp::KeepLast(1));
  vision_sub_         = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_webcam_sub_  = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_webcam_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  vision_picam_sub_   = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_picam_topic, qos_latest, std::bind(&CtrlNode::visionCallback, this, _1));
  joint_sub_          = this->create_subscription<custom_msgs::msg::JointMsg>("/joint", qos_latest, std::bind(&CtrlNode::jointCallback, this, _1));

  ctrl_pub_           = this->create_publisher<custom_msgs::msg::ControlMsg>("/control", qos_latest);
  debug_pub_          = this->create_publisher<custom_msgs::msg::TestDebug>("/test/debug", qos_latest);
  webcam_enabled_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision_webcam/enabled", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  const int period = static_cast<int>(1000.0 / controller_config.hz);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&CtrlNode::timerCallback, this));
}

void CtrlNode::jointCallback(const custom_msgs::msg::JointMsg::SharedPtr msg) {
  // Cache joint data
  if (!first_joint_) first_joint_ = true;

  joint_      = toEigen(msg->joint);
  joint_vel_  = toEigen(msg->joint_vel);
}

void CtrlNode::visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
  if (msg == nullptr) return;
  if (!msg->detected || msg->p.size() < 2) return;

  RobotState vision_state(msg);
  
  const double receive_time = this->now().seconds();
  vision_state.t = receive_time;

  if (vision_state.camera == "picam") {
    latest_picam_state_         = vision_state;
    latest_picam_receive_time_  = receive_time;
    has_latest_picam_state_     = true;
  } else {
    latest_webcam_state_        = vision_state;
    latest_webcam_receive_time_ = receive_time;
    has_latest_webcam_state_    = true;
  }
}

bool CtrlNode::hasNewPicamDetection() const {
  return has_latest_picam_state_ &&
         latest_picam_state_.detected &&
         latest_picam_receive_time_ > last_processed_picam_detection_receive_time_;
}

double CtrlNode::picamAge(double now) const {
  if (!has_picam_lock_) return std::numeric_limits<double>::infinity();
  return now - last_picam_detection_time_;
}

double CtrlNode::webcamAge(double now) const {
  if (!has_latest_webcam_state_ || !latest_webcam_state_.detected) {
    return std::numeric_limits<double>::infinity();
  }
  return now - latest_webcam_receive_time_;
}

bool CtrlNode::hasFreshWebcamDetection(double now) const {
  return webcamAge(now) <= controller_->config().webcam_measurement_max_age;
}

RobotState CtrlNode::withLatestJoint(RobotState state) const {
  state.joint = joint_;
  state.joint_vel = joint_vel_;
  return state;
}

CtrlNode::TickDecision CtrlNode::usePicamMeasurement(double now) {
  TickDecision decision;
  decision.mode = TrackingMode::PICAM_TRACK;
  decision.source = ControlSource::PICAM_DETECTION;
  decision.webcam_enabled = false;
  decision.webcam_age = webcamAge(now);
  decision.source_age = now - latest_picam_receive_time_;
  decision.picam_age = decision.source_age;

  RobotState vision_state = latest_picam_state_;
  last_raw_state_ = vision_state;
  has_raw_state_ = true;
  last_processed_picam_detection_receive_time_ = latest_picam_receive_time_;
  last_picam_detection_time_ = vision_state.t;
  has_picam_lock_ = true;

  if (!estimator_->isInitialized()) {
    estimator_->init(vision_state);
  } else {
    estimator_->update(vision_state);
  }

  decision.raw_state = vision_state;
  decision.has_raw = true;
  decision.state = withLatestJoint(estimator_->isInitialized() ? estimator_->getState(false) : vision_state);
  decision.state.camera = "picam";
  decision.state.detected = true;
  decision.state.tracked = false;
  decision.control = controller_->computePicamPixelTrack(decision.state);
  return decision;
}

CtrlNode::TickDecision CtrlNode::usePicamPrediction(double now, double picam_age) {
  TickDecision decision;
  decision.mode = TrackingMode::PICAM_PREDICT;
  decision.source = ControlSource::PICAM_PREDICTION;
  decision.webcam_enabled = false;
  decision.predicted_only = true;
  decision.picam_age = picam_age;
  decision.webcam_age = webcamAge(now);
  decision.source_age = picam_age;

  estimator_->update(now);
  if (!estimator_->isInitialized()) {
    return usePicamHold(now, picam_age);
  }

  decision.state = withLatestJoint(estimator_->getState(true));
  decision.state.camera = "picam";
  decision.state.detected = false;
  decision.state.tracked = false;
  decision.state.process = true;
  decision.state.dt = picam_age;
  decision.control = controller_->computePicamPixelTrack(decision.state);
  return decision;
}

CtrlNode::TickDecision CtrlNode::usePicamHold(double now, double picam_age) {
  TickDecision decision;
  decision.mode = TrackingMode::PICAM_HOLD;
  decision.source = ControlSource::JOINT_HOLD;
  decision.webcam_enabled = false;
  decision.picam_age = picam_age;
  decision.webcam_age = webcamAge(now);
  decision.source_age = picam_age;

  if (estimator_->isInitialized()) {
    decision.state = withLatestJoint(estimator_->getState(false));
  } else if (has_latest_picam_state_) {
    decision.state = withLatestJoint(latest_picam_state_);
  } else {
    decision.state = withLatestJoint(RobotState());
  }
  decision.state.camera = "picam";
  decision.state.detected = false;
  decision.state.tracked = false;
  decision.state.process = false;
  decision.state.dt = picam_age;
  decision.control = controller_->computeJointHold(joint_);
  return decision;
}

CtrlNode::TickDecision CtrlNode::useWebcamSearch(double now) {
  TickDecision decision;
  decision.mode = TrackingMode::WEBCAM_SEARCH;
  decision.source = ControlSource::WEBCAM_DETECTION;
  decision.webcam_enabled = true;
  decision.picam_age = picamAge(now);
  decision.webcam_age = webcamAge(now);
  decision.source_age = decision.webcam_age;

  RobotState vision_state = latest_webcam_state_;
  decision.raw_state = vision_state;
  decision.has_raw = true;
  decision.state = withLatestJoint(vision_state);
  decision.state.camera = "webcam";
  decision.state.dt = decision.picam_age;
  decision.control = controller_->computeWebcamAngleSearch(decision.state);

  last_raw_state_ = vision_state;
  has_raw_state_ = true;
  return decision;
}

CtrlNode::TickDecision CtrlNode::useIdleHold(double now) {
  TickDecision decision;
  decision.mode = TrackingMode::IDLE;
  decision.source = ControlSource::JOINT_HOLD;
  decision.webcam_enabled = true;
  decision.picam_age = picamAge(now);
  decision.webcam_age = webcamAge(now);
  decision.source_age = 0.0;
  decision.state = withLatestJoint(RobotState());
  decision.state.t = now;
  decision.control = controller_->computeJointHold(joint_);
  return decision;
}

CtrlNode::TickDecision CtrlNode::chooseControl(double now) {
  if (hasNewPicamDetection()) {
    return usePicamMeasurement(now);
  }

  const double age = picamAge(now);
  if (has_picam_lock_ && estimator_->isInitialized() &&
      age <= controller_->config().picam_prediction_max_sec) {
    return usePicamPrediction(now, age);
  }

  if (has_picam_lock_ && age <= controller_->config().picam_track_hold_sec) {
    return usePicamHold(now, age);
  }

  if (hasFreshWebcamDetection(now)) {
    return useWebcamSearch(now);
  }

  return useIdleHold(now);
}

void CtrlNode::timerCallback() {
  const double t_now = this->now().seconds();
  const TickDecision decision = chooseControl(t_now);
  publishWebcamEnabled(decision.webcam_enabled);
  publishControl(decision.control);
  publishDebug(decision);
}

void CtrlNode::publishControl(const ControlState &x) {
  custom_msgs::msg::ControlMsg msg;
  msg.header.stamp = this->now();
  msg.u_yaw = static_cast<float>(x.u_yaw);
  msg.u_pitch = static_cast<float>(x.u_pitch);
  msg.fire = x.fire;
  msg.reload = x.reload;
  msg.manual = false;
  msg.ispixel = x.isPixel;
  ctrl_pub_->publish(msg);
}

void CtrlNode::publishWebcamEnabled(bool enabled) {
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  webcam_enabled_pub_->publish(msg);
}

const char* CtrlNode::trackingModeName(TrackingMode mode) {
  switch (mode) {
    case TrackingMode::IDLE:
      return "IDLE";
    case TrackingMode::WEBCAM_SEARCH:
      return "WEBCAM_SEARCH";
    case TrackingMode::PICAM_TRACK:
      return "PICAM_TRACK";
    case TrackingMode::PICAM_PREDICT:
      return "PICAM_PREDICT";
    case TrackingMode::PICAM_HOLD:
      return "PICAM_HOLD";
    default:
      return "UNKNOWN";
  }
}

const char* CtrlNode::controlSourceName(ControlSource source) {
  switch (source) {
    case ControlSource::NONE:
      return "NONE";
    case ControlSource::WEBCAM_DETECTION:
      return "WEBCAM_DETECTION";
    case ControlSource::PICAM_DETECTION:
      return "PICAM_DETECTION";
    case ControlSource::PICAM_PREDICTION:
      return "PICAM_PREDICTION";
    case ControlSource::JOINT_HOLD:
      return "JOINT_HOLD";
    default:
      return "UNKNOWN";
  }
}

void CtrlNode::publishDebug(const TickDecision &decision) {
  custom_msgs::msg::TestDebug msg;
  msg.header.stamp = this->now();
  msg.sample_time = decision.has_raw ? decision.raw_state.t : decision.state.t;
  msg.dt = decision.state.dt;

  if (decision.has_raw) {
    fillVector2(msg.raw_p, decision.raw_state.p);
    fillVector2(msg.raw_v, decision.raw_state.v);
    fillVector2(msg.raw_a, decision.raw_state.a);
    msg.detected = decision.raw_state.detected;
    msg.tracked = decision.raw_state.tracked;
  } else {
    msg.detected = decision.state.detected;
    msg.tracked = decision.state.tracked;
  }

  fillVector2(msg.estimated_p, decision.state.p);
  fillVector2(msg.estimated_v, decision.state.v);
  fillVector2(msg.estimated_a, decision.state.a);

  msg.has_raw = decision.has_raw;
  msg.estimator_initialized = estimator_->isInitialized();
  msg.predicted_only = decision.predicted_only;

  msg.has_control = true;
  msg.u_yaw = decision.control.u_yaw;
  msg.u_pitch = decision.control.u_pitch;
  msg.fire = decision.control.fire;
  msg.reload = decision.control.reload;
  msg.tracking_mode = trackingModeName(decision.mode);
  msg.control_source = controlSourceName(decision.source);
  msg.is_pixel = decision.control.isPixel;
  msg.webcam_enabled = decision.webcam_enabled;
  msg.picam_age = std::isfinite(decision.picam_age) ? decision.picam_age : -1.0;
  msg.webcam_age = std::isfinite(decision.webcam_age) ? decision.webcam_age : -1.0;
  msg.source_age = std::isfinite(decision.source_age) ? decision.source_age : -1.0;

  debug_pub_->publish(msg);
}
