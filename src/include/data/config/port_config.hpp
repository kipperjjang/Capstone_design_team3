#pragma once

#include <string>

struct PortConfig {
  std::string name;
  std::string read_port, write_port;
  int baud{115200};
  int data_bits{8};
  int stop_bits{1};
  int parity{0};

  PortConfig() = default;
  PortConfig(const std::string &_name, const std::string &_rport, const std::string &_wport, int _baud, int _nd, int _ns, int _p)
      : name(_name), read_port(_rport), write_port(_wport), baud(_baud), data_bits(_nd), stop_bits(_ns), parity(_p) {
  }

  double watchdog_frequency{1000.0};
  double write_frequency{50.0};
  static PortConfig load(const std::string &path);
};
