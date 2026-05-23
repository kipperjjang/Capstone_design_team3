#include "controller/fsm.hpp"

ControlFSMOutput ControlFSM::update(const ControlFSMInput &input) {
  if (input.has_new_picam_target) {
    output_ = {
      TrackingMode::PICAM_TRACK,
      ControlSource::PICAM_DETECTION,
      false,
      false,
    };
    return output_;
  }

  if (input.has_picam_lock && input.estimator_initialized &&
      input.picam_age <= config_.picam_prediction_max_sec) {
    output_ = {
      TrackingMode::PICAM_PREDICT,
      ControlSource::PICAM_PREDICTION,
      false,
      true,
    };
    return output_;
  }

  if (input.has_picam_lock && input.picam_age <= config_.picam_track_hold_sec) {
    output_ = {
      TrackingMode::PICAM_HOLD,
      ControlSource::JOINT_HOLD,
      false,
      false,
    };
    return output_;
  }

  if (input.has_fresh_webcam_target) {
    output_ = {
      TrackingMode::WEBCAM_SEARCH,
      ControlSource::WEBCAM_DETECTION,
      true,
      false,
    };
    return output_;
  }

  output_ = {
    TrackingMode::IDLE,
    ControlSource::JOINT_HOLD,
    true,
    false,
  };
  return output_;
}

const char* ControlFSM::trackingModeName(TrackingMode mode) {
  switch (mode) {
    case TrackingMode::IDLE:
      return "IDLE";
    case TrackingMode::WEBCAM_SEARCH:
      return "WEBCAM_SEARCH";
    case TrackingMode::PICAM_TRACK:
      return "PICAM_TRACK";
    case TrackingMode::PICAM_PREDICT:
      return "PICAM_PREDICT";
    case TrackingMode::PICAM_HOLD:
      return "PICAM_HOLD";
    default:
      return "UNKNOWN";
  }
}

const char* ControlFSM::controlSourceName(ControlSource source) {
  switch (source) {
    case ControlSource::NONE:
      return "NONE";
    case ControlSource::WEBCAM_DETECTION:
      return "WEBCAM_DETECTION";
    case ControlSource::PICAM_DETECTION:
      return "PICAM_DETECTION";
    case ControlSource::PICAM_PREDICTION:
      return "PICAM_PREDICTION";
    case ControlSource::JOINT_HOLD:
      return "JOINT_HOLD";
    default:
      return "UNKNOWN";
  }
}
