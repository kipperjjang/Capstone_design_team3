#pragma once

#include "data/config/control_config.hpp"
#include "data/state/control_state.hpp"
#include "data/state/robot_state.hpp"

class Controller {
public:
  explicit Controller(const ControlConfig &config) : config_(config) {}

  ControlState computeWebcamAngleSearch(const RobotState &webcam_state) const;
  ControlState computePicamPixelTrack(const RobotState &picam_state) const;
  ControlState computeJointHold(const Eigen::Vector2d &joint) const;
  const ControlConfig& config() const { return config_; }
  
private:
  ControlConfig config_;
};
