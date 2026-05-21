#include "controller/fsm.hpp"
#include <iostream>

void FSM::update(const RobotState &state) {
  const double dt = state.dt;
  const bool has_target = state.detected || state.tracked;
  const double err_p = (state.img_center - state.p).norm();
  const double err_v = state.v.norm();
  const std::string cam = state.camera;
  const bool has_picam_target = cam == "picam" && has_target;

  switch (fsm_state_) {
    case FSMState::SEARCH:
      // std::cout << "search" << std::endl;
      if (has_picam_target) {
        fsm_state_ = FSMState::TRACK;
      }
      break;

    case FSMState::TRACK:
      // std::cout << dt << std::endl;
      if (!has_picam_target && dt > config_.ctrl_max_time_gap) {
        fsm_state_ = FSMState::SEARCH;
      }
      break;

    case FSMState::AIM:
      if (!has_picam_target && dt > config_.max_time_gap) {
        fsm_state_ = FSMState::SEARCH;
      } else if (err_p < config_.err_p_fire && err_v < config_.err_v_track) {
        fsm_state_ = FSMState::RELOAD;
      } else if (err_p >= config_.err_p_track) {
        fsm_state_ = FSMState::TRACK;
      }
      break;

    case FSMState::RELOAD:
      fsm_state_ = has_picam_target ? FSMState::TRACK : FSMState::SEARCH;
      break;

    default:
      std::cout << "search" << std::endl;
      fsm_state_ = FSMState::SEARCH;
      break;
  }
}
