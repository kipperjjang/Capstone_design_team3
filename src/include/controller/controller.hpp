#pragma once

#include "data/config/controller_config.hpp"
#include "data/state/robot_state.hpp"
#include "data/state/control_state.hpp"
#include "data/state/fsm_state.hpp"
#include "controller/fsm.hpp"

#include "utils/lpf.hpp"

class Controller {
public:
  Controller(const ControlConfig &config) : config_(config) {
    fsm_ = FSM(config_);
  }

  void run(const RobotState &state);
  const ControlState& getControl() { return u_; }

private:
  // FSM model
  FSM fsm_;

  // Controller configuration
  ControlConfig config_;

  // State
  RobotState state_;
  RobotState error_, error_aim_;
  
  // Control state
  ControlState u_;
}