#pragma once

#include "controller/fsm.hpp"
#include "data/config/control_config.hpp"
#include "data/state/control_state.hpp"
#include "data/state/robot_state.hpp"

class Controller {
public:
  Controller(const ControlConfig &config) : fsm_(config), config_(config) {}

  void run(const RobotState &state);
  const ControlState& getControl() const { return u_; }

private:
  FSM fsm_;

  // Controller configuration
  ControlConfig config_;

  // State
  RobotState state_;
  RobotState aim_state_;
  
  // Control state
  ControlState u_;
};
