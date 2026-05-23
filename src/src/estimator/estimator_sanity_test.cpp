#include "estimator/estimator.hpp"

#include <Eigen/Dense>

#include <cassert>
#include <cmath>

namespace {

EstimatorConfig testConfig() {
  EstimatorConfig config{};
  config.hz = 50.0;
  config.q_acc = 1.0;
  config.r_detected = 1.0;
  config.r_tracked = 1.0;
  config.r_temp_vel = 0.01;
  config.r_temp_acc = 100.0;
  config.p0_pos = 1.0;
  config.p0_vel = 100.0;
  config.p0_acc = 100.0;
  config.max_time_gap = 1.0;
  config.prediction_time = 0.0;
  return config;
}

RobotState measurement(double t, const Eigen::Vector2d &p) {
  RobotState state;
  state.t = t;
  state.p = p;
  state.detected = true;
  state.tracked = false;
  state.img_center = Eigen::Vector2d(320.0, 240.0);
  state.camera = "picam";
  return state;
}

RobotState measurementWithVelocity(double t, const Eigen::Vector2d &p, const Eigen::Vector2d &v) {
  RobotState state = measurement(t, p);
  state.v = v;
  state.has_velocity = true;
  return state;
}

bool near(const Eigen::Vector2d &a, const Eigen::Vector2d &b, double eps = 1e-9) {
  return (a - b).norm() <= eps;
}

void testPredictionDoesNotMutatePosterior() {
  Estimator estimator(testConfig());
  estimator.init(measurementWithVelocity(0.0, Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(3.0, -1.0)));

  const RobotState before = estimator.updatedState();
  const RobotState predicted = estimator.predictedState(0.5);
  const RobotState after = estimator.updatedState();

  assert(near(before.p, after.p));
  assert(near(before.v, after.v));
  assert(!near(before.p, predicted.p));
}

void testUpdatedStateMatchesCorrectedCompatibilityName() {
  Estimator estimator(testConfig());
  estimator.init(measurementWithVelocity(0.0, Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(3.0, -1.0)));

  const RobotState updated = estimator.updatedState();
  const RobotState corrected = estimator.correctedState();

  assert(near(updated.p, corrected.p));
  assert(near(updated.v, corrected.v));
  assert(near(updated.a, corrected.a));
}

void testPredictionGapHoldsLastPredictedState() {
  Estimator estimator(testConfig());
  estimator.init(measurementWithVelocity(0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d(10.0, 0.0)));

  const RobotState last_prediction = estimator.predict(0.5);
  const RobotState gap_prediction = estimator.predict(1.5);

  assert(!estimator.isInitialized());
  assert(near(gap_prediction.p, last_prediction.p));
  assert(near(gap_prediction.v, last_prediction.v));
  assert(std::abs(gap_prediction.dt - 1.5) <= 1e-9);
}

void testMissingVelocityDiffersFromZeroVelocityMeasurement() {
  Estimator no_velocity_hint(testConfig());
  no_velocity_hint.init(measurementWithVelocity(0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d(10.0, 0.0)));
  no_velocity_hint.update(measurement(1.0, Eigen::Vector2d(10.0, 0.0)));

  Estimator zero_velocity_hint(testConfig());
  zero_velocity_hint.init(measurementWithVelocity(0.0, Eigen::Vector2d::Zero(), Eigen::Vector2d(10.0, 0.0)));
  zero_velocity_hint.update(measurementWithVelocity(1.0, Eigen::Vector2d(10.0, 0.0), Eigen::Vector2d::Zero()));

  assert(std::abs(no_velocity_hint.updatedState().v.x()) >
         std::abs(zero_velocity_hint.updatedState().v.x()));
}

void testGapReinitializesFromNewMeasurement() {
  Estimator estimator(testConfig());
  estimator.init(measurement(0.0, Eigen::Vector2d::Zero()));
  estimator.update(measurement(2.0, Eigen::Vector2d(7.0, 8.0)));

  assert(estimator.isInitialized());
  assert(near(estimator.updatedState().p, Eigen::Vector2d(7.0, 8.0)));
}

void testJointMetadataIsAttached() {
  Estimator estimator(testConfig());
  estimator.updateJoint(Eigen::Vector2d(0.1, -0.2), Eigen::Vector2d(0.3, -0.4), 0.0);
  estimator.init(measurement(0.0, Eigen::Vector2d::Zero()));

  const RobotState state = estimator.updatedState();
  assert(near(state.joint, Eigen::Vector2d(0.1, -0.2)));
  assert(near(state.joint_vel, Eigen::Vector2d(0.3, -0.4)));
}

}  // namespace

int main() {
  testPredictionDoesNotMutatePosterior();
  testUpdatedStateMatchesCorrectedCompatibilityName();
  testPredictionGapHoldsLastPredictedState();
  testMissingVelocityDiffersFromZeroVelocityMeasurement();
  testGapReinitializesFromNewMeasurement();
  testJointMetadataIsAttached();
  return 0;
}
