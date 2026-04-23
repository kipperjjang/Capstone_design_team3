#pragma once

#include <string>

#include "data/config/control_config.hpp"
#include "data/state/fsm_state.hpp"
#include "data/state/robot_state.hpp"

class FSM {
public:
  FSM(const ControlConfig &config) : config_(config) {}

  void update(const RobotState &state);

  // Utils
  FSMState getFSMState() const { return fsm_state_; }
  std::string getFSMStateName() const { return whichState(fsm_state_); }

private:
  ControlConfig config_;
  FSMState fsm_state_{FSMState::SEARCH};
};
