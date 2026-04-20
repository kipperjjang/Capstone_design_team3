#include "data/config/port_config.hpp"

#include <yaml-cpp/yaml.h>

PortConfigList PortConfigList::load(const std::string &path) {
  PortConfigList config;
  
  const YAML::Node file = YAML::LoadFile(path);

  const YAML::Node i = file["bridge"]["serial"]["input"];
  const YAML::Node o = file["bridge"]["serial"]["output"];

  // Input ports configuration
  auto iname = i["name"].as<std::vector<std::string>>();
  auto iport = i["port"].as<std::vector<std::string>>();
  auto ibaud = i["baud"].as<std::vector<int>>();
  auto idata = i["data_bits"].as<std::vector<int>>();
  auto istop = i["stop_bits"].as<std::vector<int>>();
  auto iparity = i["parity"].as<std::vector<int>>();
  for (size_t idx = 0; idx < iname.size(); ++idx) {
    config.input.push_back(PortConfig(iname[idx], iport[idx], ibaud[idx], idata[idx], istop[idx], istop[idx], iparity[idx], true));
  }

  // Output ports configuration
  auto oname = o["name"].as<std::vector<std::string>>();
  auto oport = o["port"].as<std::vector<std::string>>();
  auto obaud = o["baud"].as<std::vector<int>>();
  auto odata = o["data_bits"].as<std::vector<int>>();
  auto ostop = o["stop_bits"].as<std::vector<int>>();
  auto oparity = o["parity"].as<std::vector<int>>();
  for (size_t odx = 0; odx < oname.size(); ++odx) {
    config.output.push_back(PortConfig(oname[odx], oport[odx], obaud[odx], odata[odx], ostop[odx], ostop[odx], oparity[odx], false));
  }

  // Watchdog frequency; rate of bridge node
  config.watchdog_frequency = serial["watchdog_frequency"].as<double>();

  return config;
}
