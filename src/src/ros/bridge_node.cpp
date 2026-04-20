#include "ros/bridge_node.hpp"

BridgeNode::BridgeNode() : Node("bridge_node") {
  // Read Configuration
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "");
  this->get_parameter("config_path", config_path);
  PortConfigList port_config = PortConfigList::load(config_path);
  
  // Input port
  for (auto iconfig : port_config.input) {
    input_.push_back(Serial(iconfig));
  }
  // Output port
  for (auto oconfig : port_config.output) {
    output_.push_back(Serial(oconfig));
  }
  
  // ROS subscriber and publisher
  control_sub_ = this->create_subscription<custom_msgs::msg::ControlMsg>("/control", 1, std::bind(&BridgeNodwe::controlCallback, this, _1));
  joint_pub_ = this->create_publisher<custom_msgs::msg::JointMsg>("/joint", 1);

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
  if (!readSerialFrame()) return;


  // Publish Joint data
  custom_msgs::msg::JointMsg msg;
  msg.header.stamp = this->now();
  msg.yaw = ..
  msg.pitch = ..
  joint_pub_->publish(msg);
}

bool BridgeNode::readSerial() {
  // Read data
  for (auto iport : input_) {
    iport.readSerial(buffer.data(), size);
  }

  return true;
}
