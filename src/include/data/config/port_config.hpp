#pragma once

#include <string>

struct PortConfig {
  std::string name = "serial";
  std::string port = "/dev/ttyUSB0";
  int baud = 115200;
  int data_bits = 8;
  int stop_bits = 1;
  int parity = 0;  // 0: none, 1: odd, 2: even

  double watchdog_frequency = 1000;

  static PortConfig load(const std::string &path);
};
