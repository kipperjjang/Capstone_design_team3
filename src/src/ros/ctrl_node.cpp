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

  const ControlConfig controller_config = ControlConfig::load(config_path);

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
  static size_t count = 1;
  
  // LPF alpha
  const double alpha = 0.3;

  joint_ = alpha * joint_ + (1-alpha) * toEigen(msg->joint);
  joint_vel_ = alpha * joint_vel_ + (1-alpha) * toEigen(msg->joint_vel);

  joint_buffer_ = joint_buffer_ + joint_vel_;
  joint_mean_ = joint_buffer_ / count;
  if (count >= 100) {
    joint_buffer_.setZero();
    count = 1;
  } else {
    count++;
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
    has_latest_picam_state_ = true;
  } else {
    latest_webcam_state_ = vision_state;
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
  if (shouldPublishControl(fsm_output)) {
    publishControl(tick.control);
  }

  publishDebug(fsm_input, fsm_output, tick);
}

bool CtrlNode::hasNewPicamTarget() const {
  return has_latest_picam_state_ && latest_picam_state_.detected && latest_picam_state_.t > last_consumed_picam_time_;
}

bool CtrlNode::hasFreshPicamTarget(double now) const {
  if (hasNewPicamTarget()) {
    return true;
  }

  return has_picam_lock_ && has_filtered_picam_state_ && picamAge(now) <= controller_->config().picam_track_reuse_sec;
}

double CtrlNode::picamAge(double now) const {
  if (!has_picam_lock_ || !has_filtered_picam_state_) {
    return std::numeric_limits<double>::infinity();
  }
  return now - filtered_picam_state_.t;
}

double CtrlNode::webcamAge(double now) const {
  if (!has_latest_webcam_state_ || !latest_webcam_state_.detected) {
    return std::numeric_limits<double>::infinity();
  }
  return now - latest_webcam_state_.t;
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

  // No need to update
  if (has_hold_joint_target_ && hold_joint_mode_ == mode) {
    return;
  }

  if (!first_joint_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No /joint sample received yet; skipping joint hold control.");
    has_hold_joint_target_ = false;
    hold_joint_mode_ = mode;
    return;
  }

  hold_joint_target_ = joint_;
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

RobotState CtrlNode::filterPicamMeasurement(const RobotState &measurement) {
  RobotState state = withLatestJoint(measurement);

  const ControlConfig &config = controller_->config();
  const bool should_reset = !has_filtered_picam_state_ || state.t - filtered_picam_state_.t > config.picam_lpf_reset_gap_sec;

  if (!config.picam_lpf_enabled || should_reset) {
    filtered_picam_state_ = state;
    filtered_picam_state_.dt = 0.0;
    has_filtered_picam_state_ = true;
    return filtered_picam_state_;
  }

  const double dt = std::max(0.0, state.t - filtered_picam_state_.t);
  filtered_picam_state_.p = config.picam_lpf_alpha * filtered_picam_state_.p + (1.0 - config.picam_lpf_alpha) * state.p;
  filtered_picam_state_.t = state.t;
  filtered_picam_state_.dt = dt;
  filtered_picam_state_.img_center = state.img_center;
  filtered_picam_state_.camera = state.camera;
  filtered_picam_state_.detected = state.detected;
  filtered_picam_state_.joint = state.joint;
  filtered_picam_state_.joint_vel = state.joint_vel;
  return filtered_picam_state_;
}

RobotState CtrlNode::reusablePicamState(double now) const {
  RobotState state = withLatestJoint(filtered_picam_state_);
  state.detected = false;
  state.dt = std::max(0.0, now - filtered_picam_state_.t);
  return state;
}

// Eigen::Vector2d CtrlNode::webcamSearchOffset(double now) const {
//   const ControlConfig &config = controller_->config();
//   const double half_angle = (M_PI / 180) * config.webcam_search_box_half_angle_deg;

//   const double requested_step = (M_PI / 180) * config.webcam_search_box_step_deg;
//   const int half_steps = std::max(1, static_cast<int>(std::ceil(half_angle / requested_step)));
//   const double step = half_angle / static_cast<double>(half_steps);
  
//   std::vector<Eigen::Vector2d> pattern;
//   pattern:reserve(static_cast<size_t>((2 * half_steps + 1) * (2 * half_steps + 1)));
//   pattern.push_back(Eigen::Vector2d::Zero());
//   for (int pitch_idx = -half_steps; pitch_idx <= half_steps; ++pitch_idx) {
//     for (int yaw_idx = -half_steps; yaw_idx <= half_steps; ++yaw_idx) {
//       if (yaw_idx == 0 && pitch_idx == 0) {
//         continue;
//       }
//     }
//     pattern.emplace_back(static_cast<double>(yaw_idx) * step, static_cast<double>(pitch) * step);
//   }

// }


ControlFSMInput CtrlNode::buildFSMInput(double now) const {
  ControlFSMInput input;
  input.has_fresh_picam_target = hasFreshPicamTarget(now);
  input.has_picam_lock = has_picam_lock_;
  input.has_fresh_webcam_target = hasFreshWebcamTarget(now);
  input.picam_age = picamAge(now);
  input.webcam_age = webcamAge(now);
  return input;
}

CtrlNode::ControlTick CtrlNode::executeMode(const ControlFSMOutput &fsm_output, double now) {
  switch (fsm_output.mode) {
    case TrackingMode::PICAM_TRACK:
      return trackPicam(now);
    case TrackingMode::PICAM_HOLD:
      return holdPicam(now, picamAge(now));
    case TrackingMode::WEBCAM_SEARCH:
      return searchWebcam(now);
    case TrackingMode::IDLE:
    default:
      return holdIdle(now);
  }
}

void CtrlNode::decideFire(const RobotState &state, ControlState &control) {
  const auto config = controller_->config();
  // Check if fire
  bool fire = true;

  // pixel error; bbox
  Eigen::Vector2d bbox_half_size = state.bbox / 2;
  double bbox_half_width = bbox_half_size(0);
  double bbox_half_height = bbox_half_size(1);
  
  // Pixel error check (box)
  Eigen::Vector2d pixel_error = (state.p - config.img_offset - state.img_center).cwiseAbs();
  // if (pixel_error(0) < bbox_half_width && pixel_error(1) < bbox_half_height) {
  //   fire &= true;:w
  // } else {
  //   fire &= false;
  // }

  // Pixel error check (circle)
  // const double length = std::min(bbox_half_width, bbox_half_height);
  // if (pixel_error.norm() < length) {// && pixel_error.norm() > 0.1 * length ) {
  //   fire &= true;
  // } else {
  //   fire &= false;
  // }

  // std::cout << "1\t" << fire;

  // Joint velocity에 따라 offset (buffer 0.5s); 
  // std::cout << joint_mean_.transpose() << std::endl;
  
  // if (joint_vel_(1) < 0.0) {
  //   control.u_pitch = control.u_pitch + config.pixel_offset(1);
  // }
  // else if (joint_mean_(1) <= config.edge_joint_threshold(1)) {
  //   fire &= false;
  // }

  //   std::cout << "2\t" << fire;
  // if (joint_vel_(0) < config.edge_joint_threshold(0)) {
  //   control.u_yaw = control.u_yaw - config.pixel_offset(0);
  // } else if (joint_vel_(0) > config.edge_joint_threshold(0)) {
  //   control.u_yaw = control.u_yaw + config.pixel_offset(0);
  // } 
  
  // if (joint_vel_.norm() < config.vel_norm_threshold) {
  //    fire &= false;
  // }
  
  // std::cout << "3\t" << fire << std::endl;
  control.update(fire);
}


CtrlNode::ControlTick CtrlNode::trackPicam(double now) {
  ControlTick tick;
  const bool has_new_measurement = hasNewPicamTarget();

  if (has_new_measurement) {
    RobotState measurement = latest_picam_state_;
    last_consumed_picam_time_ = measurement.t;
    has_picam_lock_ = true;

    tick.raw_state = measurement;
    tick.has_raw_sample = true;
    tick.source_age = now - measurement.t;
    tick.state = filterPicamMeasurement(measurement);
  } else if (has_filtered_picam_state_) {
    tick.source_age = picamAge(now);
    tick.state = reusablePicamState(now);
  } else {
    tick.source_age = picamAge(now);
    tick.state = withLatestJoint(RobotState());
  }

  tick.control = controller_->computePicamPixelTrack(tick.state);
  decideFire(tick.state, tick.control);
  
  tick.state.camera = "picam";
  return tick;
}

CtrlNode::ControlTick CtrlNode::holdPicam(double now, double picam_age) {
  ControlTick tick;
  tick.source_age = picam_age;
  if (has_filtered_picam_state_) {
    tick.state = reusablePicamState(now);
  } else if (has_latest_picam_state_) {
    tick.state = withLatestJoint(latest_picam_state_);
  } else {
    tick.state = withLatestJoint(RobotState());
  }
  tick.state.camera = "picam";
  tick.state.detected = false;
  tick.state.dt = picam_age;
  tick.control = controller_->computeJointHold(holdJointTarget());
  return tick;
}

CtrlNode::ControlTick CtrlNode::searchWebcam(double now) {
  ControlTick tick;
  tick.source_age = webcamAge(now);
  RobotState measurement = latest_webcam_state_;

  tick.raw_state = measurement;
  tick.has_raw_sample = true;
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

bool CtrlNode::shouldPublishControl(const ControlFSMOutput &fsm_output) const {
  if (fsm_output.source == ControlSource::JOINT_HOLD && !first_joint_) {
    return false;
  }
  return true;
}

void CtrlNode::publishControl(const ControlState &control) {
  custom_msgs::msg::ControlMsg msg;
  msg.header.stamp = this->now();
  msg.u_yaw = static_cast<float>(control.u_yaw);
  msg.u_pitch = static_cast<float>(control.u_pitch);
  msg.ispixel = control.isPixel;
  msg.fire = control.fire;
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
  msg.sample_time = tick.has_raw_sample ? tick.raw_state.t : tick.state.t;
  msg.dt = std::isfinite(tick.source_age) ? tick.source_age : -1.0;

  if (tick.has_raw_sample) {
    fillVector2(msg.raw_p, tick.raw_state.p);
    msg.detected = tick.raw_state.detected;
  } else {
    msg.detected = tick.state.detected;
  }

  fillVector2(msg.filtered_p, tick.state.p);

  msg.u_yaw = tick.control.u_yaw;
  msg.u_pitch = tick.control.u_pitch;
  msg.tracking_mode = ControlFSM::trackingModeName(fsm_output.mode);
  msg.control_source = ControlFSM::controlSourceName(fsm_output.source);
  msg.picam_age = std::isfinite(fsm_input.picam_age) ? fsm_input.picam_age : -1.0;
  msg.webcam_age = std::isfinite(fsm_input.webcam_age) ? fsm_input.webcam_age : -1.0;
  msg.source_age = std::isfinite(tick.source_age) ? tick.source_age : -1.0;

  debug_pub_->publish(msg);
}
