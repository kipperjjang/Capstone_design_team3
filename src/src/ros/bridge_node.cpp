#include "ros/bridge_node.hpp"

#include "data/config/port_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>

using std::placeholders::_1;

BridgeNode::BridgeNode() : Node("bridge_node") {
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "");
  this->get_parameter("config_path", config_path);

  const PortConfigList port_config = PortConfigList::load(config_path);
  for (const auto &iconfig : port_config.input) {
    input_.push_back(std::make_unique<Serial>(iconfig));
  }
  for (const auto &oconfig : port_config.output) {
    output_.push_back(std::make_unique<Serial>(oconfig));
  }

  control_sub_ = this->create_subscription<custom_msgs::msg::ControlMsg>(
    "/control", 10, std::bind(&BridgeNode::controlCallback, this, _1));
  joint_pub_ = this->create_publisher<custom_msgs::msg::JointMsg>("/joint", 10);

  const int period_ms = std::max(1, static_cast<int>(1000.0 / port_config.watchdog_frequency));
  watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&BridgeNode::timerCallback, this));
  buffer_.reserve(1024);
}

void BridgeNode::controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg) {
  if (msg == nullptr) {
    return;
  }

  if (!warned_about_protocol_) {
    RCLCPP_WARN(
      this->get_logger(),
      "BridgeNode is running in safe stub mode. Serial frame encoding/decoding is intentionally not implemented.");
    warned_about_protocol_ = true;
  }
}

void BridgeNode::timerCallback() {
  (void)readSerialFrame();
}

bool BridgeNode::readSerialFrame() {
  std::array<uint8_t, 256> read_buffer{};
  bool received_bytes = false;

  for (auto &iport : input_) {
    if (!iport || !iport->isOpen()) {
      continue;
    }

    const ssize_t bytes_read = iport->readSerial(read_buffer.data(), read_buffer.size());
    if (bytes_read > 0) {
      received_bytes = true;
      buffer_.insert(buffer_.end(), read_buffer.begin(), read_buffer.begin() + bytes_read);
    }
  }

  if (received_bytes) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Serial data received, but joint frame parsing is not implemented in the safe stub.");
    buffer_.clear();
  }

  return received_bytes;
}
