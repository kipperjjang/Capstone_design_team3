#include "data/config/port_config.hpp"

#include <yaml-cpp/yaml.h>

PortConfig PortConfig::load(const std::string &path) {
  const YAML::Node file = YAML::LoadFile(path);

  const YAML::Node serial = file["bridge"]["serial"];
  PortConfig config(
    serial["name"].as<std::string>(),
    serial["port"].as<std::string>(),
    serial["baud"].as<int>(),
    serial["data_bits"].as<int>(),
    serial["stop_bits"].as<int>(),
    serial["parity"].as<int>());

  // Watchdog frequency; rate of bridge node
  config.watchdog_frequency = serial["watchdog_frequency"].as<double>();

  return config;
}
