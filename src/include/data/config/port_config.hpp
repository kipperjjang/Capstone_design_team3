#pragma once

#include <string>
#include <vector>

struct PortConfig {
  std::string name;
  std::string port;
  int baud;
  int data_bits;
  int stop_bits;
  int parity;       // 0: none, 1: odd, 2: even
  bool isInput;

  PortConfig(std::string &_name, std::string &_port, int _baud, int _nd, int _ns, int _p, bool _isInput) {
    name = _name;
    port = _port;
    baud = _baud;
    data_bits = _nd;
    stop_bits = _ns;
    parity = _p;
    isInput = _isInput;
  }
};

struct PortConfigList {
  std::vector<PortConfig> input;
  std::vector<PortConfig> output;

  double watchdog_frequency = 1000;
  static PortConfigList load(const std::string &path);
};