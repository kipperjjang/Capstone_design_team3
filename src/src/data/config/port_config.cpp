#include "data/config/port_config.hpp"

#include <yaml-cpp/yaml.h>

PortConfig PortConfig::load(const std::string &path) {
  PortConfig config;
  if (path.empty()) {
    return config;
  }
  
  const YAML::Node file = YAML::LoadFile(path);
  const YAML::Node serial = file["bridge"]["serial"];

  config.name = serial["name"].as<std::string>();
  config.port = serial["port"].as<std::string>();
  config.baud = serial["baud"].as<int>();
  config.data_bits = serial["data_bits"].as<int>();
  config.stop_bits = serial["stop_bits"].as<int>();
  config.parity = serial["parity"].as<int>();
  config.watchdog_frequency = serial["watchdog_frequency"].as<double>();

  return config;
}
