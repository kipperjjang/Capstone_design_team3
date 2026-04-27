#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import TestDebug
from custom_msgs.msg import VisionMsg


class TrainingDataSampler(Node):
  def __init__(self):
    super().__init__('prediction_training_data_sampler')

    self.declare_parameter('debug_topic', '/test/debug')
    self.declare_parameter('vision_topic', '/vision')
    self.declare_parameter('output_csv', '~/ros2_ws/src/capstone/src/scripts/prediction/data/prediction_training_samples.csv')
    self.declare_parameter('scenario_count', 500)
    self.declare_parameter('periods_per_scenario', 5.0)
    self.declare_parameter('test_vision_rate_hz', 30.0)
    self.declare_parameter('position_noise_std', 30.0)
    self.declare_parameter('random_seed', 0)
    self.declare_parameter('scenario_gap_sec', 0.8)
    self.declare_parameter('center_x', 320.0)
    self.declare_parameter('center_y', 240.0)
    self.declare_parameter('min_frequency_hz', 0.25)
    self.declare_parameter('max_frequency_hz', 1.20)
    self.declare_parameter('min_amplitude_x', 5.0)
    self.declare_parameter('max_amplitude_x', 140.0)
    self.declare_parameter('min_amplitude_y', 40.0)
    self.declare_parameter('max_amplitude_y', 170.0)

    self.debug_topic = self.get_parameter('debug_topic').value
    self.vision_topic = self.get_parameter('vision_topic').value
    self.output_csv = Path(str(self.get_parameter('output_csv').value)).expanduser()
    self.scenario_count = max(1, int(self.get_parameter('scenario_count').value))
    self.periods_per_scenario = max(0.5, float(self.get_parameter('periods_per_scenario').value))
    self.test_vision_rate_hz = max(1.0, float(self.get_parameter('test_vision_rate_hz').value))
    self.position_noise_std = max(0.0, float(self.get_parameter('position_noise_std').value))
    self.random_seed = int(self.get_parameter('random_seed').value)
    self.scenario_gap_sec = max(0.0, float(self.get_parameter('scenario_gap_sec').value))
    self.center_x = float(self.get_parameter('center_x').value)
    self.center_y = float(self.get_parameter('center_y').value)
    self.min_frequency_hz = float(self.get_parameter('min_frequency_hz').value)
    self.max_frequency_hz = float(self.get_parameter('max_frequency_hz').value)
    self.min_amplitude_x = float(self.get_parameter('min_amplitude_x').value)
    self.max_amplitude_x = float(self.get_parameter('max_amplitude_x').value)
    self.min_amplitude_y = float(self.get_parameter('min_amplitude_y').value)
    self.max_amplitude_y = float(self.get_parameter('max_amplitude_y').value)

    self.rng = (
      np.random.default_rng()
      if self.random_seed < 0
      else np.random.default_rng(self.random_seed))

    self.scenarios = [self.makeScenario(i) for i in range(self.scenario_count)]
    self.current_idx = -1
    self.current_scenario = None
    self.scenario_start_time = None
    self.scenario_abs_origin = None
    self.last_truth_position = None
    self.last_truth_velocity = None
    self.last_truth_rel_time = None
    self.last_written_time = None
    self.waiting_for_next = False
    self.next_start_wall_time = None
    self.finished = False

    self.output_csv.parent.mkdir(parents=True, exist_ok=True)
    self.csv_file = self.output_csv.open('w', newline='')
    self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=[
      'run_id',
      'scenario_id',
      'time',
      'est_x',
      'est_y',
      'est_vx',
      'est_vy',
      'est_ax',
      'est_ay',
      'truth_x',
      'truth_y',
      'frequency_hz',
      'post_hit',
    ])
    self.csv_writer.writeheader()

    self.debug_sub = self.create_subscription(
      TestDebug,
      self.debug_topic,
      self.debugCallback,
      10)
    self.vision_pub = self.create_publisher(VisionMsg, self.vision_topic, 10)

    self.publish_timer = self.create_timer(
      1.0 / self.test_vision_rate_hz,
      self.publishTestVision)
    self.monitor_timer = self.create_timer(0.05, self.monitorScenarios)

    self.startNextScenario()

  def destroy_node(self):
    if hasattr(self, 'csv_file') and not self.csv_file.closed:
      self.csv_file.flush()
      self.csv_file.close()
    super().destroy_node()

  def nowSeconds(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def randomUniform(self, low, high):
    return float(self.rng.uniform(low, high))

  def makeSinParams(self, direction_scale=1.0):
    base_frequency = self.randomUniform(self.min_frequency_hz, self.max_frequency_hz)
    return {
      'frequency_hz': base_frequency,
      'amp_x1': self.randomUniform(self.min_amplitude_x, self.max_amplitude_x) * direction_scale,
      'amp_y1': self.randomUniform(self.min_amplitude_y, self.max_amplitude_y),
      'amp_x2': self.randomUniform(0.0, 0.35 * self.max_amplitude_x),
      'amp_y2': self.randomUniform(0.0, 0.35 * self.max_amplitude_y),
      'phase_x1': self.randomUniform(0.0, 2.0 * math.pi),
      'phase_y1': self.randomUniform(0.0, 2.0 * math.pi),
      'phase_x2': self.randomUniform(0.0, 2.0 * math.pi),
      'phase_y2': self.randomUniform(0.0, 2.0 * math.pi),
      'freq_ratio_x': self.randomUniform(0.75, 1.35),
      'freq_ratio_y': self.randomUniform(0.85, 1.25),
    }

  def makeScenario(self, scenario_id):
    pre = self.makeSinParams(direction_scale=self.randomUniform(0.15, 0.75))
    post = self.makeSinParams(direction_scale=self.randomUniform(0.65, 1.20))
    duration = self.periods_per_scenario / max(pre['frequency_hz'], 1e-6)
    hit_time = duration * self.randomUniform(0.45, 0.65)
    return {
      'id': scenario_id,
      'run_id': 'scenario_%04d' % scenario_id,
      'duration': duration,
      'hit_time': hit_time,
      'pre': pre,
      'post': post,
    }

  def evaluateParams(self, params, t):
    wx = 2.0 * math.pi * params['frequency_hz'] * params['freq_ratio_x']
    wy = 2.0 * math.pi * params['frequency_hz'] * params['freq_ratio_y']
    return np.array([
      self.center_x
      + params['amp_x1'] * math.sin(wx * t + params['phase_x1'])
      + params['amp_x2'] * math.sin(2.0 * wx * t + params['phase_x2']),
      self.center_y
      + params['amp_y1'] * math.sin(wy * t + params['phase_y1'])
      + params['amp_y2'] * math.sin(2.0 * wy * t + params['phase_y2']),
    ], dtype=float)

  def groundTruthPosition(self, scenario, rel_time):
    if rel_time < scenario['hit_time']:
      return self.evaluateParams(scenario['pre'], rel_time)

    hit_pos = self.evaluateParams(scenario['pre'], scenario['hit_time'])
    post_zero = self.evaluateParams(scenario['post'], 0.0)
    post_pos = self.evaluateParams(scenario['post'], rel_time - scenario['hit_time'])
    return hit_pos + (post_pos - post_zero)

  def startNextScenario(self):
    self.current_idx += 1
    if self.current_idx >= len(self.scenarios):
      self.finished = True
      self.csv_file.flush()
      self.get_logger().info('Finished %d scenarios. CSV saved to %s' % (
        len(self.scenarios),
        self.output_csv))
      rclpy.shutdown()
      return

    self.current_scenario = self.scenarios[self.current_idx]
    self.scenario_start_time = self.get_clock().now()
    self.scenario_abs_origin = None
    self.last_truth_position = None
    self.last_truth_velocity = None
    self.last_truth_rel_time = None
    self.last_written_time = None
    self.waiting_for_next = False
    self.get_logger().info(
      'Starting %s/%d duration %.3fs' %
      (self.current_scenario['run_id'], self.scenario_count, self.current_scenario['duration']))

  def publishTestVision(self):
    if self.finished or self.waiting_for_next or self.current_scenario is None:
      return

    now = self.get_clock().now()
    rel_time = (now - self.scenario_start_time).nanoseconds * 1e-9
    stamp_time = now.nanoseconds * 1e-9
    if self.scenario_abs_origin is None:
      self.scenario_abs_origin = stamp_time - rel_time

    if rel_time > self.current_scenario['duration']:
      self.waiting_for_next = True
      self.next_start_wall_time = self.nowSeconds() + self.scenario_gap_sec
      self.get_logger().info('Finished %s' % self.current_scenario['run_id'])
      return

    truth_position = self.groundTruthPosition(self.current_scenario, rel_time)
    velocity = None
    acceleration = None
    if self.last_truth_position is not None and self.last_truth_rel_time is not None:
      dt = rel_time - self.last_truth_rel_time
      if dt > 0.0:
        velocity = (truth_position - self.last_truth_position) / dt
        if self.last_truth_velocity is not None:
          acceleration = (velocity - self.last_truth_velocity) / dt

    observed_position = truth_position.copy()
    if self.position_noise_std > 0.0:
      observed_position += self.rng.normal(0.0, self.position_noise_std, size=2)

    msg = VisionMsg()
    msg.header.stamp = now.to_msg()
    msg.detected = True
    msg.tracked = True
    msg.p = observed_position.tolist()
    msg.v = velocity.tolist() if velocity is not None else []
    msg.a = acceleration.tolist() if acceleration is not None else []
    msg.bbox = [0.0, 0.0]
    msg.confidence = 1.0
    msg.covariance = []
    msg.source_mode = 'prediction_training_data'
    self.vision_pub.publish(msg)

    self.last_truth_position = truth_position
    self.last_truth_velocity = velocity
    self.last_truth_rel_time = rel_time

  def debugCallback(self, msg):
    if (
      self.finished
      or self.waiting_for_next
      or self.current_scenario is None
      or self.scenario_abs_origin is None
      or msg.predicted_only
      or not msg.estimator_initialized
    ):
      return

    rel_time = float(msg.sample_time) - self.scenario_abs_origin
    if not 0.0 <= rel_time <= self.current_scenario['duration']:
      return
    if self.last_written_time is not None and rel_time <= self.last_written_time + 1e-6:
      return
    if len(msg.estimated_p) < 2 or len(msg.estimated_v) < 2 or len(msg.estimated_a) < 2:
      return

    truth_position = self.groundTruthPosition(self.current_scenario, rel_time)
    self.csv_writer.writerow({
      'run_id': self.current_scenario['run_id'],
      'scenario_id': self.current_scenario['id'],
      'time': '%.9f' % rel_time,
      'est_x': '%.9f' % msg.estimated_p[0],
      'est_y': '%.9f' % msg.estimated_p[1],
      'est_vx': '%.9f' % msg.estimated_v[0],
      'est_vy': '%.9f' % msg.estimated_v[1],
      'est_ax': '%.9f' % msg.estimated_a[0],
      'est_ay': '%.9f' % msg.estimated_a[1],
      'truth_x': '%.9f' % truth_position[0],
      'truth_y': '%.9f' % truth_position[1],
      'frequency_hz': '%.9f' % self.current_scenario['pre']['frequency_hz'],
      'post_hit': int(rel_time >= self.current_scenario['hit_time']),
    })
    self.last_written_time = rel_time

  def monitorScenarios(self):
    if self.finished or not self.waiting_for_next:
      return
    if self.nowSeconds() >= self.next_start_wall_time:
      self.startNextScenario()


def main():
  rclpy.init()
  node = TrainingDataSampler()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()

