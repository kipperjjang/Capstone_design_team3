#include "ros/test_node.hpp"

#include "data/config/estimator_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
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

  this->declare_parameter<std::string>("config_path", "");
  this->declare_parameter<std::string>("vision_topic", "/vision");
  this->get_parameter("config_path", config_path);
  this->get_parameter("vision_topic", vision_topic);

  EstimatorConfig estimator_config = EstimatorConfig::load(config_path);
  estimator_ = std::make_shared<Estimator>(estimator_config);

  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>(vision_topic, 1, std::bind(&TestNode::visionCallback, this, _1));
  debug_pub_ = this->create_publisher<custom_msgs::msg::TestDebug>("/test/debug", 1);

  const int period_ms = std::max(1, static_cast<int>(1000.0 / estimator_config.hz));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&TestNode::timerCallback, this));
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

  const bool has_measurement = raw_state.detected || raw_state.tracked;
  last_raw_state_ = raw_state;
  has_raw_state_ = true;

  if (has_measurement) {
    if (!estimator_->isInitialized()) {
      estimator_->init(raw_state);
    } else {
      estimator_->update(raw_state);
    }
  }

  const RobotState estimated_state = estimator_->getState(false);
  publishDebug(estimated_state, true, false);
}

void TestNode::timerCallback() {
  if (!estimator_->isInitialized()) return;

  estimator_->update(this->now().seconds());
  const RobotState estimated_state = estimator_->getState(true);
  publishDebug(estimated_state, false, true);
}

void TestNode::publishDebug(const RobotState &estimated_state, bool has_raw, bool predicted_only) {
  custom_msgs::msg::TestDebug msg;
  msg.header.stamp = this->now();
  msg.sample_time = has_raw ? last_raw_state_.t : estimated_state.t;
  msg.dt = estimated_state.dt;

  if (has_raw_state_) {
    fillVector2(msg.raw_p, last_raw_state_.p);
    fillVector2(msg.raw_v, last_raw_state_.v);
    msg.detected = last_raw_state_.detected;
    msg.tracked = last_raw_state_.tracked;
  }
  fillVector2(msg.estimated_p, estimated_state.p);
  fillVector2(msg.estimated_v, estimated_state.v);

  msg.has_raw = has_raw;
  msg.estimator_initialized = estimator_->isInitialized();
  msg.predicted_only = predicted_only;

  msg.has_control = false;
  msg.u_yaw = 0.0;
  msg.u_pitch = 0.0;
  msg.fire = false;
  msg.reload = false;

  debug_pub_->publish(msg);
}
