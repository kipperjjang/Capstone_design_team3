#include "controller/fsm.hpp"

// Check status when failed to track during Picam tracking
TrackingMode ControlFSM::fallbackMode(const ControlFSMInput &input) const {
  return input.has_fresh_webcam_target ? TrackingMode::WEBCAM_SEARCH : TrackingMode::IDLE;
}

ControlFSMOutput ControlFSM::update(const ControlFSMInput &input) {
  // Prioritize picam tracking
  if (input.has_fresh_picam_target) {
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
      } else if (input.picam_age <= config_.picam_track_reuse_sec) {
        mode_ = TrackingMode::PICAM_TRACK;    // Reuse last filtered YOLO briefly
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
      };

    case TrackingMode::PICAM_TRACK:
      return {
        TrackingMode::PICAM_TRACK,
        ControlSource::PICAM_MEASUREMENT,
        false,    // webcam disabled
      };

    case TrackingMode::PICAM_HOLD:
      return {
        TrackingMode::PICAM_HOLD,
        ControlSource::JOINT_HOLD,
        false,    // webcam disabled
      };

    case TrackingMode::IDLE:
    default:
      return {
        TrackingMode::IDLE,
        ControlSource::JOINT_HOLD,
        true,     // webcam enabled
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
    case ControlSource::PICAM_MEASUREMENT:
      return "PICAM_MEASUREMENT";
    case ControlSource::JOINT_HOLD:
      return "JOINT_HOLD";
    default:
      return "UNKNOWN";
  }
}
