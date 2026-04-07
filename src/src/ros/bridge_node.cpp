#include "ros/bridge_node.hpp"

BridgeNode::BridgeNode() : Node("bridge_node") {
  // Read Configuration
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "");
  this->get_parameter("config_path", config_path);
  PortConfig port_config = PortConfig::load(config_path);

  // Initilaize
  // serial_ = Serial(port_config);
  
  // ROS subscriber and publisher
  control_sub_ = this->create_subscription<custom_msgs::msg::ControlMsg>("/control", 1, std::bind(&BridgeNodwe::controlCallback, this, _1));
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 1);

  // Timer
  const int period = static_cast<int>(1000.0 / port_config.watchdog_frequency);
  watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(period), std::bind(&CtrlNode::timerCallback, this));
}

void BridgeNode::controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg) {
  std::vector<uint8_t> data;

  // multiple port???
  // Write seril on corresponding ports

}

void BridgeNode::timerCallback() {
  // Read IMU Port
  if (!readImuframe()) return;
  imu_pub_->publish(imu_);
}

bool BridgeNode::readImuFrame() {
  // Read IMU frame on serial port
  // Fix imu_ 

  // Check if whole IMU frame has read
  imu_.header = this->now();
  imu_.orientation = ...
  imu_.angular_velocity = ...
  imu_.linear_acceleration = ...

  return true;
}
