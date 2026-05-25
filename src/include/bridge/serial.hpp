#pragma once

#include "data/config/port_config.hpp"

#include <cstddef>
#include <cstdint>
#include <sys/types.h>
#include <vector>

class Serial {
public:
  explicit Serial(const PortConfig &config);
  ~Serial() { closeSerialPort(); }

  bool isOpen() const { return serial_port_fd_ >= 0; }

  ssize_t readSerial(uint8_t *buffer, std::size_t size);
  ssize_t writeSerial(const uint8_t *data, std::size_t size);
  ssize_t writeSerial(const std::vector<uint8_t> &data);

private:
  bool openSerialPort();
  bool configureSerialPort();
  void closeSerialPort();

  // Raw transport only; BridgeNode owns protocol framing.
  PortConfig config_;

  int serial_port_fd_{-1};
};
