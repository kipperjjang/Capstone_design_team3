#include "ros/ctrl_node.hpp"
using std::placeholders::_1;

CtrlNode::CtrlNode() : Node("control_node") {
  // Read Configuration
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "");
  this->get_parameter("config_path", config_path);
  EstimatorConfig estimator_config = EstimatorConfig::load(config_path);
  ControllerConfig controller_config = ControllerConfig::load(config_path);

  // Initilaize
  estimator_ = Estimator(estimator_config);
  controller_ = Controller(controller_config);

  // Subscriber and Publisher
  vision_sub_ = this->create_subscription<custom_msgs::msg::VisionMsg>("/vision", 1, std::bind(&CtrlNode::visionCallback, this, _1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1, std::bind(&CtrlNode::imuCallback, this, _1));
  
  control_pub_ = this->create_publisher<custom_msgs::msg::ControlMsg>("/control", 1);

  // Timer
  const int period = static_cast<int>(1000.0 / controller_config.frequency);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&CtrlNode::timerCallback, this));
}

void CtrlNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Store and process imu data
  Eigen::Vector2d omega = toEigen(msg->angular_velocity).tail<2>();
  estimator_.update(omega);
}

void CtrlNode::visionCallback(const custom_msgs::msg::VisionMsg::SharedPtr msg) {
  const bool has_meas = msg->detected || msg->tracked;
  if (!has_meas) return;

  // Vision State
  auto vision_state = RobotState(msg);

  // Update Estimator state
  if (!estimator_.isInitialized()) {
    estimator_.init(vision_state);
  } else {
    estimator_.update(vision_state);
  }

  // Run Controller
  auto state = estimator_.getState();
  controller_.run(state);

  // Publish control input to the plant
  publishControl(controller_.getControl());
}

void CtrlNode::timerCallback() {
  // Update with the process model
  if (!estimator_.isInitialized()) return;
  
  // Update Estimator state (predicted state)
  const double t_now = this->now().seconds();
  estimator_.update(t_now);

  // Run Controller
  auto state = estimator_.getState();
  controller_.run(state);

  // Publish control input to the plant
  publishControl(controller_.getControl());
}

void CtrlNode::publishControl(const ControlState &x) {
  custom_msgs::msg::ControlMsg msg;
  msg.header.stamp = this->now();
  msg.u_yaw = x.u_yaw;
  msg.u_pitch = x.u_pitch;
  msg.fire = x.fire;
  msg.reload = x.reload;
  control_pub_->publish(msg);
}