#include "ros/bridge_node.hpp"

#include "data/config/port_config.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <functional>

using std::placeholders::_1;

namespace {

constexpr std::array<uint8_t, 2> kSof{{0xAA, 0x55}};
constexpr size_t  kHeaderSize = 5;
constexpr size_t  kCrcSize = 2;
constexpr size_t  kWritePayloadSize = 9;
constexpr size_t  kReadPayloadSize = 25;
constexpr size_t  kMinFrameSize = kHeaderSize + kCrcSize;
constexpr size_t  kPrevStatusOffset = 2;
constexpr size_t  kLenOffset = 3;
constexpr size_t  kSeqOffset = 4;
constexpr uint8_t kStatusOk = 0x00;
constexpr uint8_t kStatusFailed = 0x01;

uint16_t crc16Ccitt(const uint8_t *data, size_t size) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }

  return crc;
}

void appendBytes(std::vector<uint8_t> &frame, const void *data, size_t size) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  frame.insert(frame.end(), bytes, bytes + size);
}

std::vector<uint8_t> makeControlFrame(
    const custom_msgs::msg::ControlMsg &msg,
    bool previous_read_status,
    uint8_t seq) {
  std::vector<uint8_t> frame;
  frame.reserve(kHeaderSize + kWritePayloadSize + kCrcSize);

  frame.insert(frame.end(), kSof.begin(), kSof.end());
  frame.push_back(previous_read_status ? kStatusOk : kStatusFailed);
  frame.push_back(static_cast<uint8_t>(kWritePayloadSize));
  frame.push_back(seq);

  appendBytes(frame, &msg.u_yaw, sizeof(msg.u_yaw));
  appendBytes(frame, &msg.u_pitch, sizeof(msg.u_pitch));
  // appendBytes(frame, &msg.u_yaw_dot, sizeof(msg.u_yaw_dot));
  // appendBytes(frame, &msg.u_pitch_dot, sizeof(msg.u_pitch_dot));

  uint8_t flags = 0x00;
  if (!msg.ispixel) {
    flags |= 0x01;
  }
  frame.push_back(flags);

  const uint16_t crc = crc16Ccitt(frame.data() + kSof.size(), frame.size() - kSof.size());

  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return frame;
}

}  // namespace

BridgeNode::BridgeNode() : Node("bridge_node") {
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "");
  this->get_parameter("config_path", config_path);

  const PortConfig port_config = PortConfig::load(config_path);
  serial_ = std::make_unique<Serial>(port_config);

  const auto qos_latest = rclcpp::QoS(rclcpp::KeepLast(1));
  control_sub_  = this->create_subscription<custom_msgs::msg::ControlMsg>("/control", qos_latest, std::bind(&BridgeNode::controlCallback, this, _1));
  joint_pub_    = this->create_publisher<custom_msgs::msg::JointMsg>("/joint", qos_latest);

  const int read_period_ms  = std::max(1, static_cast<int>(1000.0 / port_config.watchdog_frequency));
  const int write_period_ms = std::max(1, static_cast<int>(1000.0 / port_config.write_frequency));
  read_timer_   = this->create_wall_timer(std::chrono::milliseconds(read_period_ms), std::bind(&BridgeNode::readTimerCallback, this));
  write_timer_  = this->create_wall_timer(std::chrono::milliseconds(write_period_ms), std::bind(&BridgeNode::writeTimerCallback, this));
  buffer_.reserve(1024);
}

void BridgeNode::controlCallback(const custom_msgs::msg::ControlMsg::SharedPtr msg) {
  if (msg == nullptr) return;

  latest_control_ = *msg;
  has_latest_control_ = true;
}

void BridgeNode::readTimerCallback() {
  (void)readSerialFrame();
}

void BridgeNode::writeTimerCallback() {
  if (!has_latest_control_) return;

  (void)writeSerialFrame(latest_control_);
}

bool BridgeNode::writeSerialFrame(const custom_msgs::msg::ControlMsg &msg) {
  last_write_frame_ = makeControlFrame(msg, previous_read_status_, write_seq_);
  ++write_seq_;

  return writeFrameToSerial(last_write_frame_);
}

bool BridgeNode::writeFrameToSerial(const std::vector<uint8_t> &frame) {
  if (frame.empty() || !serial_ || !serial_->isOpen()) {
    return false;
  }

  // Write frame
  const ssize_t bytes_written = serial_->writeSerial(frame);
  if (bytes_written == static_cast<ssize_t>(frame.size())) {
    return true;
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to write a complete control frame to serial port.");
  return false;
}

bool BridgeNode::readSerialFrame() {
  std::array<uint8_t, 256> read_buffer{};
  if (!serial_ || !serial_->isOpen()) return false;

  // Read Serial Frame
  const ssize_t bytes_read = serial_->readSerial(read_buffer.data(), read_buffer.size());
  if (bytes_read <= 0) {
    return false;
  }

  buffer_.insert(buffer_.end(), read_buffer.begin(), read_buffer.begin() + bytes_read);
  return parseReadFrames(buffer_);
}

bool BridgeNode::parseReadFrames(std::vector<uint8_t> &buffer) {
  bool parsed_frame = false;

  // Extract and publish data from the read frame
  while (buffer.size() >= kMinFrameSize) {
    auto sof = std::search(buffer.begin(), buffer.end(), kSof.begin(), kSof.end());
    if (sof == buffer.end()) {
      const size_t keep = std::min<size_t>(buffer.size(), kSof.size() - 1);
      buffer.erase(buffer.begin(), buffer.end() - keep);
      return parsed_frame;
    }

    // SOF check
    if (sof != buffer.begin()) {
      buffer.erase(buffer.begin(), sof);
    }

    // Header size check
    if (buffer.size() < kHeaderSize) {
      return parsed_frame;
    }

    // Len check
    const uint8_t payload_size = buffer[kLenOffset];
    if (payload_size != kReadPayloadSize) {
      previous_read_status_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dropping serial frame with unexpected payload length: %u", static_cast<unsigned int>(payload_size));
      buffer.erase(buffer.begin());
      continue;
    }

    // Frame size check
    const size_t frame_size = kHeaderSize + payload_size + kCrcSize;
    if (buffer.size() < frame_size) {
      return parsed_frame;
    }

    // CRC16 check
    uint16_t received_crc = 0;
    std::memcpy(&received_crc, buffer.data() + frame_size - kCrcSize, sizeof(received_crc));
    const uint16_t computed_crc = crc16Ccitt(buffer.data() + kSof.size(), frame_size - kSof.size() - kCrcSize);
    if (received_crc != computed_crc) {
      previous_read_status_ = false;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Dropping serial frame with invalid CRC16.");
      buffer.erase(buffer.begin());
      continue;
    }

    // Previous read status check
    previous_read_status_ = true;
    const bool remote_received_previous_write = buffer[kPrevStatusOffset] != kStatusFailed;
    if (!remote_received_previous_write && !last_write_frame_.empty()) {
      (void)writeFrameToSerial(last_write_frame_);
    }

    // Read data
    std::array<float, 6> values{};
    std::memcpy(values.data(), buffer.data() + kHeaderSize, values.size() * sizeof(float));

    // Publish joint values
    custom_msgs::msg::JointMsg msg;
    msg.header.stamp = this->now();
    msg.joint = {values[0], values[3]};
    msg.joint_vel = {values[1], values[4]};
    msg.joint_torque = {values[2], values[5]};
    joint_pub_->publish(msg);

    parsed_frame = true;
    buffer.erase(buffer.begin(), buffer.begin() + frame_size);
  }

  return parsed_frame;
}
