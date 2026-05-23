#include "ros/test_node.hpp"

#include "data/config/estimator_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

using std::placeholders::_1;

namespace {
void fillVector2(std::array<double, 2> &output, const Eigen::Vector2d &input) {
  output[0] = input.x();
  output[1] = input.y();
}
}  // namespace

TestNode::TestNode() : Node("test_node") {
  std::string config_path;
  std::string vision_topic;
  std::string debug_topic;

  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("vision_topic", "/vision");
  this->declare_parameter<std::string>("debug_topic", "/test/debug");
  this->declare_parameter<double>("prediction_horizon_sec", -1.0);
  this->declare_parameter<double>("metrics_log_period_sec", 1.0);
  this->get_parameter("config_path", config_path);
  this->get_parameter("vision_topic", vision_topic);
  this->get_parameter("debug_topic", debug_topic);

  EstimatorConfig estimator_config = EstimatorConfig::load(config_path);
  prediction_horizon_sec_ = this->get_parameter("prediction_horizon_sec").as_double();
  if (prediction_horizon_sec_ < 0.0) {
    prediction_horizon_sec_ = estimator_config.prediction_time;
  }
  metrics_log_period_sec_ = std::max(0.0, this->get_parameter("metrics_log_period_sec").as_double());
  last_metrics_log_time_ = this->now();

  estimator_ = std::make_shared<Estimator>(estimator_config);

  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_topic, 1, std::bind(&TestNode::visionCallback, this, _1));
  debug_pub_ = this->create_publisher<custom_msgs::msg::TestDebug>(debug_topic, 1);

  const int period_ms = std::max(1, static_cast<int>(1000.0 / estimator_config.hz));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&TestNode::timerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Estimator-only test node: vision_topic=%s debug_topic=%s prediction_horizon=%.3fs",
    vision_topic.c_str(), debug_topic.c_str(), prediction_horizon_sec_);
}

void TestNode::visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
  if (!msg) return;

  if (msg->p.size() < 2) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Ignoring /vision message with fewer than two position values.");
    return;
  }

  RobotState raw_state(msg);
  if (raw_state.t <= 0.0) {
    raw_state.t = this->now().seconds();
  }

  last_raw_state_ = raw_state;
  has_raw_state_ = true;

  if (!raw_state.detected) {
    return;
  }

  if (!estimator_->isInitialized()) {
    estimator_->init(raw_state);
  } else {
    estimator_->update(raw_state);
  }

  const RobotState estimated_state = estimator_->updatedState();
  recordResidual(raw_state, estimated_state);
  publishDebug(estimated_state, true, false);
  logMetricsIfDue();
}

void TestNode::timerCallback() {
  if (!estimator_->isInitialized()) return;

  const double target_time = this->now().seconds() + prediction_horizon_sec_;
  const RobotState estimated_state = estimator_->predictedState(target_time);
  publishDebug(estimated_state, false, true);
  logMetricsIfDue();
}

void TestNode::publishDebug(const RobotState &estimated_state, bool has_raw, bool predicted_only) {
  custom_msgs::msg::TestDebug msg;
  msg.header.stamp = this->now();
  msg.sample_time = has_raw ? last_raw_state_.t : estimated_state.t;
  msg.dt = estimated_state.dt;

  if (has_raw_state_) {
    fillVector2(msg.raw_p, last_raw_state_.p);
    fillVector2(msg.raw_v, last_raw_state_.v);
    fillVector2(msg.raw_a, last_raw_state_.a);
    msg.detected = last_raw_state_.detected;
  }
  fillVector2(msg.estimated_p, estimated_state.p);
  fillVector2(msg.estimated_v, estimated_state.v);
  fillVector2(msg.estimated_a, estimated_state.a);

  msg.has_raw = has_raw;
  msg.estimator_initialized = estimator_->isInitialized();
  msg.predicted_only = predicted_only;

  msg.has_control = false;
  msg.u_yaw = 0.0;
  msg.u_pitch = 0.0;
  msg.tracking_mode = predicted_only ? "ESTIMATOR_PREDICT" : "ESTIMATOR_UPDATE";
  msg.control_source = "ESTIMATOR_TEST";
  msg.is_pixel = true;
  msg.webcam_enabled = false;
  msg.picam_age = -1.0;
  msg.webcam_age = -1.0;
  msg.source_age = predicted_only ? estimated_state.dt : 0.0;

  debug_pub_->publish(msg);
}

void TestNode::recordResidual(const RobotState &raw_state, const RobotState &estimated_state) {
  const double residual_norm = (estimated_state.p - raw_state.p).norm();
  residual_norm_sum_ += residual_norm;
  residual_norm_sq_sum_ += residual_norm * residual_norm;
  residual_norm_max_ = std::max(residual_norm_max_, residual_norm);
  ++measurement_count_;
}

void TestNode::logMetricsIfDue() {
  if (metrics_log_period_sec_ <= 0.0 || measurement_count_ == 0) {
    return;
  }

  const rclcpp::Time now = this->now();
  if ((now - last_metrics_log_time_).seconds() < metrics_log_period_sec_) {
    return;
  }

  const double count = static_cast<double>(measurement_count_);
  const double mean = residual_norm_sum_ / count;
  const double rmse = std::sqrt(residual_norm_sq_sum_ / count);
  RCLCPP_INFO(
    this->get_logger(),
    "Estimator residual: samples=%zu mean=%.3fpx rmse=%.3fpx max=%.3fpx initialized=%s",
    measurement_count_, mean, rmse, residual_norm_max_,
    estimator_->isInitialized() ? "true" : "false");

  last_metrics_log_time_ = now;
}
