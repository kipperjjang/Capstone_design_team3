#pragma once

#include <string>

/* FSM State */
enum class FSMState {
  SEARCH,
  TRACK,
  AIM,
  RELOAD,
  FAIL
};

// Utility function for FSM
inline std::string whichState(const FSMState state) {
  switch (state) {
    case FSMState::SEARCH:
      return "SEARCH";
      break;
    case FSMState::TRACK:
      return "TRACK";
      break;
    case FSMState::AIM:
      return "AIM";
      break;
    case FSMState::RELOAD:
      return "RELOAD";
      break;
    default:
      return "Unknown";
      break;
  }
}