#pragma once

#include <limits>

#include "data/config/control_config.hpp"

enum class TrackingMode {
  IDLE,
  WEBCAM_SEARCH,
  PICAM_TRACK,
  PICAM_HOLD,
};

enum class ControlSource {
  NONE,
  WEBCAM_DETECTION,
  PICAM_MEASUREMENT,
  JOINT_HOLD,
};

struct ControlFSMInput {
  bool has_fresh_picam_target{false};
  bool has_picam_lock{false};
  bool has_fresh_webcam_target{false};
  double picam_age{std::numeric_limits<double>::infinity()};
  double webcam_age{std::numeric_limits<double>::infinity()};
};

struct ControlFSMOutput {
  TrackingMode mode{TrackingMode::IDLE};
  ControlSource source{ControlSource::NONE};
  bool webcam_enabled{true};
};

class ControlFSM {
public:
  explicit ControlFSM(const ControlConfig &config) : config_(config) {}

  ControlFSMOutput update(const ControlFSMInput &input);
  ControlFSMOutput output() const { return output_; }
  TrackingMode mode() const { return mode_; }

  static const char* trackingModeName(TrackingMode mode);
  static const char* controlSourceName(ControlSource source);

private:
  ControlFSMOutput outputForMode(TrackingMode mode) const;
  TrackingMode fallbackMode(const ControlFSMInput &input) const;

  ControlConfig config_;
  ControlFSMOutput output_;
  TrackingMode mode_{TrackingMode::IDLE};
};
