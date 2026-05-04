#pragma once

#include <string>

struct PortConfig {
  std::string name;
  std::string port;
  int baud{115200};
  int data_bits{8};
  int stop_bits{1};
  int parity{0};

  PortConfig() = default;
  PortConfig(const std::string &_name, const std::string &_port, int _baud, int _nd, int _ns, int _p)
      : name(_name), port(_port), baud(_baud), data_bits(_nd), stop_bits(_ns), parity(_p) {
  }

  double watchdog_frequency{1000.0};
  static PortConfig load(const std::string &path);
};
