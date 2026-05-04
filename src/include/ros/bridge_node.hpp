#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "bridge/serial.hpp"
#include "custom_msgs/msg/control_msg.hpp"
#include "custom_msgs/msg/joint_msg.hpp"

class BridgeNode : public rclcpp::Node {
public:
  BridgeNode();

private:
  // Callback function
  void controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg);
  void timerCallback();
  
  // Utils
  bool writeSerialFrame(const custom_msgs::msg::ControlMsg &msg);
  bool writeFrameToSerial(const std::vector<uint8_t> &frame);
  bool readSerialFrame();
  bool parseReadFrames(std::vector<uint8_t> &buffer);

  // ROS
  rclcpp::Subscription<custom_msgs::msg::ControlMsg>::SharedPtr control_sub_;
  rclcpp::Publisher<custom_msgs::msg::JointMsg>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Serial
  std::unique_ptr<Serial> serial_;
  std::vector<uint8_t> buffer_;
  std::vector<uint8_t> last_write_frame_;
  uint8_t write_seq_{0x11};
  bool previous_read_status_{true};
};
