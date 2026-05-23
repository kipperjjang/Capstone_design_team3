#include "controller/fsm.hpp"

// Check status when failed to track during Picam tracking
TrackingMode ControlFSM::fallbackMode(const ControlFSMInput &input) const {
  return input.has_fresh_webcam_target ? TrackingMode::WEBCAM_SEARCH : TrackingMode::IDLE;
}

ControlFSMOutput ControlFSM::update(const ControlFSMInput &input) {
  // Prioritize picam tracking
  if (input.has_new_picam_target) {
    mode_ = TrackingMode::PICAM_TRACK;
    output_ = outputForMode(mode_);
    return output_;
  }

  switch (mode_) {
    case TrackingMode::IDLE:
      mode_ = input.has_fresh_webcam_target ? TrackingMode::WEBCAM_SEARCH : TrackingMode::IDLE;
      break;

    case TrackingMode::WEBCAM_SEARCH:
      mode_ = input.has_fresh_webcam_target ? TrackingMode::WEBCAM_SEARCH : TrackingMode::IDLE;
      break;

    case TrackingMode::PICAM_TRACK:
      if (!input.has_picam_lock) {
        mode_ = fallbackMode(input);          // Expired lock time (lock time; tracking holding time)
      } else if (input.estimator_initialized && input.picam_age <= config_.picam_prediction_max_sec) {
        mode_ = TrackingMode::PICAM_PREDICT;  // Prediction
      } else if (input.picam_age <= config_.picam_track_hold_sec) {
        mode_ = TrackingMode::PICAM_HOLD;     // Holding for YOLO detection
      } else {
        mode_ = fallbackMode(input);
      }
      break;

    case TrackingMode::PICAM_PREDICT:
      if (!input.has_picam_lock) {
        mode_ = fallbackMode(input);          // Expired lock time
      } else if (input.estimator_initialized && input.picam_age <= config_.picam_prediction_max_sec) {
        mode_ = TrackingMode::PICAM_PREDICT;  // Maintain mode
      } else if (input.picam_age <= config_.picam_track_hold_sec) {
        mode_ = TrackingMode::PICAM_HOLD;     // Holding for YOLO detection
      } else {
        mode_ = fallbackMode(input);
      }
      break;

    case TrackingMode::PICAM_HOLD:
      if (input.has_picam_lock && input.picam_age <= config_.picam_track_hold_sec) {
        mode_ = TrackingMode::PICAM_HOLD;
      } else {
        mode_ = fallbackMode(input);
      }
      break;

    default:
      mode_ = fallbackMode(input);
      break;
  }

  output_ = outputForMode(mode_);
  return output_;
}

ControlFSMOutput ControlFSM::outputForMode(TrackingMode mode) const {
  switch (mode) {
    case TrackingMode::WEBCAM_SEARCH:
      return {
        TrackingMode::WEBCAM_SEARCH,
        ControlSource::WEBCAM_DETECTION,
        true,     // webcam enabled
        false,    // no prediction
      };

    case TrackingMode::PICAM_TRACK:
      return {
        TrackingMode::PICAM_TRACK,
        ControlSource::PICAM_DETECTION,
        false,    // webcam disabled
        false,    // no prediction
      };

    case TrackingMode::PICAM_PREDICT:
      return {
        TrackingMode::PICAM_PREDICT,
        ControlSource::PICAM_PREDICTION,
        false,    // webcam disabled
        true,     // prediction
      };

    case TrackingMode::PICAM_HOLD:
      return {
        TrackingMode::PICAM_HOLD,
        ControlSource::JOINT_HOLD,
        false,    // webcam disabled
        false,    // no prediction
      };

    case TrackingMode::IDLE:
    default:
      return {
        TrackingMode::IDLE,
        ControlSource::JOINT_HOLD,
        true,     // webcam enabled
        false,    // no prediction
      };
  }
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
