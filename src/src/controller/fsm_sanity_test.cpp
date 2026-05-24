#include "controller/fsm.hpp"

#include <cassert>

namespace {

ControlConfig testConfig() {
  ControlConfig config;
  config.picam_track_reuse_sec = 0.25;
  config.picam_track_hold_sec = 1.0;
  return config;
}

ControlFSMInput baseInput() {
  ControlFSMInput input;
  input.picam_age = 100.0;
  input.webcam_age = 100.0;
  return input;
}

ControlFSMInput newPicamInput() {
  ControlFSMInput input = baseInput();
  input.has_fresh_picam_target = true;
  input.has_picam_lock = true;
  input.picam_age = 0.0;
  return input;
}

ControlFSMInput picamLockInput(double picam_age) {
  ControlFSMInput input = baseInput();
  input.has_picam_lock = true;
  input.picam_age = picam_age;
  return input;
}

void assertOutput(
    const ControlFSMOutput &output,
    TrackingMode mode,
    ControlSource source,
    bool webcam_enabled) {
  assert(output.mode == mode);
  assert(output.source == source);
  assert(output.webcam_enabled == webcam_enabled);
}

void testIdleToWebcamSearch() {
  ControlFSM fsm(testConfig());
  ControlFSMInput input = baseInput();
  input.has_fresh_webcam_target = true;

  assertOutput(
      fsm.update(input),
      TrackingMode::WEBCAM_SEARCH,
      ControlSource::WEBCAM_DETECTION,
      true);
  assert(fsm.mode() == TrackingMode::WEBCAM_SEARCH);
}

void testWebcamSearchToIdleWhenTargetStales() {
  ControlFSM fsm(testConfig());
  ControlFSMInput input = baseInput();
  input.has_fresh_webcam_target = true;
  fsm.update(input);

  assertOutput(
      fsm.update(baseInput()),
      TrackingMode::IDLE,
      ControlSource::JOINT_HOLD,
      true);
}

void testPicamAcquireHasGlobalPriority() {
  ControlFSM fsm(testConfig());
  ControlFSMInput webcam = baseInput();
  webcam.has_fresh_webcam_target = true;
  fsm.update(webcam);

  assertOutput(
      fsm.update(newPicamInput()),
      TrackingMode::PICAM_TRACK,
      ControlSource::PICAM_MEASUREMENT,
      false);
}

void testPicamTrackReusesRecentMeasurement() {
  ControlFSM fsm(testConfig());
  fsm.update(newPicamInput());

  assertOutput(
      fsm.update(picamLockInput(0.1)),
      TrackingMode::PICAM_TRACK,
      ControlSource::PICAM_MEASUREMENT,
      false);
}

void testPicamTrackToHoldAfterReuseWindow() {
  ControlFSM fsm(testConfig());
  fsm.update(newPicamInput());
  fsm.update(picamLockInput(0.1));

  assertOutput(
      fsm.update(picamLockInput(0.5)),
      TrackingMode::PICAM_HOLD,
      ControlSource::JOINT_HOLD,
      false);
}

void testPicamHoldToWebcamSearchAfterLockExpires() {
  ControlFSM fsm(testConfig());
  fsm.update(newPicamInput());
  fsm.update(picamLockInput(0.1));
  fsm.update(picamLockInput(0.5));

  ControlFSMInput input = baseInput();
  input.has_picam_lock = false;
  input.has_fresh_webcam_target = true;

  assertOutput(
      fsm.update(input),
      TrackingMode::WEBCAM_SEARCH,
      ControlSource::WEBCAM_DETECTION,
      true);
}

void testPicamHoldToIdleAfterHoldTimeout() {
  ControlFSM fsm(testConfig());
  fsm.update(newPicamInput());
  fsm.update(picamLockInput(0.1));
  fsm.update(picamLockInput(0.5));

  ControlFSMInput input = picamLockInput(1.5);
  input.has_fresh_webcam_target = false;

  assertOutput(
      fsm.update(input),
      TrackingMode::IDLE,
      ControlSource::JOINT_HOLD,
      true);
}

void testPicamReacquiredDuringHold() {
  ControlFSM fsm(testConfig());
  fsm.update(newPicamInput());
  fsm.update(picamLockInput(0.1));
  fsm.update(picamLockInput(0.5));

  assertOutput(
      fsm.update(newPicamInput()),
      TrackingMode::PICAM_TRACK,
      ControlSource::PICAM_MEASUREMENT,
      false);
}

}  // namespace

int main() {
  testIdleToWebcamSearch();
  testWebcamSearchToIdleWhenTargetStales();
  testPicamAcquireHasGlobalPriority();
  testPicamTrackReusesRecentMeasurement();
  testPicamTrackToHoldAfterReuseWindow();
  testPicamHoldToWebcamSearchAfterLockExpires();
  testPicamHoldToIdleAfterHoldTimeout();
  testPicamReacquiredDuringHold();
  return 0;
}
