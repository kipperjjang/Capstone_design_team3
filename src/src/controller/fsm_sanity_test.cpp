#include "controller/fsm.hpp"

#include <cassert>

namespace {

ControlConfig testConfig() {
  ControlConfig config;
  config.picam_prediction_max_sec = 0.25;
  config.picam_track_hold_sec = 1.0;
  return config;
}

ControlFSMInput baseInput() {
  ControlFSMInput input;
  input.has_new_picam_target = false;
  input.has_picam_lock = false;
  input.estimator_initialized = false;
  input.has_fresh_webcam_target = false;
  input.picam_age = 100.0;
  input.webcam_age = 100.0;
  return input;
}

void testPicamDetectionHasPriority() {
  ControlFSM fsm(testConfig());
  ControlFSMInput input = baseInput();
  input.has_new_picam_target = true;
  input.has_picam_lock = true;
  input.estimator_initialized = true;
  input.has_fresh_webcam_target = true;
  input.picam_age = 0.1;

  const ControlFSMOutput output = fsm.update(input);
  assert(output.mode == TrackingMode::PICAM_TRACK);
  assert(output.source == ControlSource::PICAM_DETECTION);
  assert(!output.webcam_enabled);
  assert(!output.predicted_only);
}

void testPredictionWindow() {
  ControlFSM fsm(testConfig());
  ControlFSMInput input = baseInput();
  input.has_picam_lock = true;
  input.estimator_initialized = true;
  input.picam_age = 0.2;

  const ControlFSMOutput output = fsm.update(input);
  assert(output.mode == TrackingMode::PICAM_PREDICT);
  assert(output.source == ControlSource::PICAM_PREDICTION);
  assert(!output.webcam_enabled);
  assert(output.predicted_only);
}

void testHoldWindowAfterPrediction() {
  ControlFSM fsm(testConfig());
  ControlFSMInput input = baseInput();
  input.has_picam_lock = true;
  input.estimator_initialized = true;
  input.picam_age = 0.5;
  input.has_fresh_webcam_target = true;

  const ControlFSMOutput output = fsm.update(input);
  assert(output.mode == TrackingMode::PICAM_HOLD);
  assert(output.source == ControlSource::JOINT_HOLD);
  assert(!output.webcam_enabled);
  assert(!output.predicted_only);
}

void testWebcamSearchAfterHoldExpires() {
  ControlFSM fsm(testConfig());
  ControlFSMInput input = baseInput();
  input.has_picam_lock = true;
  input.estimator_initialized = true;
  input.picam_age = 1.5;
  input.has_fresh_webcam_target = true;

  const ControlFSMOutput output = fsm.update(input);
  assert(output.mode == TrackingMode::WEBCAM_SEARCH);
  assert(output.source == ControlSource::WEBCAM_DETECTION);
  assert(output.webcam_enabled);
  assert(!output.predicted_only);
}

void testIdleWhenNoTarget() {
  ControlFSM fsm(testConfig());
  const ControlFSMOutput output = fsm.update(baseInput());

  assert(output.mode == TrackingMode::IDLE);
  assert(output.source == ControlSource::JOINT_HOLD);
  assert(output.webcam_enabled);
  assert(!output.predicted_only);
}

}  // namespace

int main() {
  testPicamDetectionHasPriority();
  testPredictionWindow();
  testHoldWindowAfterPrediction();
  testWebcamSearchAfterHoldExpires();
  testIdleWhenNoTarget();
  return 0;
}
