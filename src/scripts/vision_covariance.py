#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
import yaml

from custom_msgs.msg import VisionMsg


@dataclass
class VisionSample:
  t: float
  p: np.ndarray
  confidence: float
  bbox: np.ndarray


def stamp_to_seconds(stamp):
  return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def matrix_summary(matrix):
  matrix = np.asarray(matrix, dtype=float)
  diag = np.diag(matrix)
  return {
    'matrix': matrix.tolist(),
    'diag': diag.tolist(),
    'std': np.sqrt(np.maximum(diag, 0.0)).tolist(),
  }


def finite_matrix(matrix):
  matrix = np.asarray(matrix, dtype=float)
  return np.nan_to_num(matrix, nan=0.0, posinf=0.0, neginf=0.0)


class VisionCovarianceCalib(Node):
  def __init__(self):
    super().__init__('vision_covariance_calib')

    self.declare_parameter('vision_topic', '/vision_picam')
    self.declare_parameter('mode', 'static')
    self.declare_parameter('sample_count', 300)
    self.declare_parameter('min_samples', 50)
    self.declare_parameter('output_path', '/tmp/vision_covariance.yaml')
    self.declare_parameter('smooth_window', 21)
    self.declare_parameter('min_confidence', -1.0)
    self.declare_parameter('max_jump_px', -1.0)

    self.vision_topic = self.get_parameter('vision_topic').value
    self.mode = str(self.get_parameter('mode').value).lower()
    self.sample_count = max(1, int(self.get_parameter('sample_count').value))
    self.min_samples = max(2, int(self.get_parameter('min_samples').value))
    self.output_path = self.get_parameter('output_path').value
    self.smooth_window = max(3, int(self.get_parameter('smooth_window').value))
    self.min_confidence = float(self.get_parameter('min_confidence').value)
    self.max_jump_px = float(self.get_parameter('max_jump_px').value)

    if self.mode not in ('static', 'smooth'):
      raise ValueError("mode must be 'static' or 'smooth'")

    self.samples = []
    self.rejected_samples = 0
    self.reject_reasons = {
      'not_detected': 0,
      'missing_position': 0,
      'low_confidence': 0,
      'jump': 0,
    }
    self.finished = False

    self.sub = self.create_subscription(
      VisionMsg,
      self.vision_topic,
      self.vision_callback,
      1)

    self.get_logger().info(
      f'Collecting {self.sample_count} detected samples from {self.vision_topic} '
      f'in {self.mode} mode.')

  def now_seconds(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def reject(self, reason):
    self.rejected_samples += 1
    if reason in self.reject_reasons:
      self.reject_reasons[reason] += 1

  def vision_callback(self, msg):
    if self.finished:
      return

    if not msg.detected:
      self.reject('not_detected')
      return

    if len(msg.p) < 2:
      self.reject('missing_position')
      return

    if self.min_confidence >= 0.0 and msg.confidence < self.min_confidence:
      self.reject('low_confidence')
      return

    p = np.array(msg.p[:2], dtype=float)
    if self.max_jump_px > 0.0 and self.samples:
      jump = np.linalg.norm(p - self.samples[-1].p)
      if jump > self.max_jump_px:
        self.reject('jump')
        return

    t = stamp_to_seconds(msg.header.stamp)
    if t <= 0.0:
      t = self.now_seconds()

    bbox = np.array(msg.bbox[:2], dtype=float) if len(msg.bbox) >= 2 else np.zeros(2)
    self.samples.append(VisionSample(t=t, p=p, confidence=float(msg.confidence), bbox=bbox))

    if len(self.samples) % 50 == 0 or len(self.samples) == self.sample_count:
      self.get_logger().info(f'Accepted samples: {len(self.samples)}/{self.sample_count}')

    if len(self.samples) >= self.sample_count:
      self.finished = True
      self.finish()

  def finish(self):
    if len(self.samples) < self.min_samples:
      self.get_logger().error(
        f'Only collected {len(self.samples)} samples, but min_samples={self.min_samples}.')
      rclpy.shutdown()
      return

    try:
      result = self.compute_result()
      self.print_result(result)
      self.write_yaml(result)
    finally:
      rclpy.shutdown()

  def compute_result(self):
    times = np.array([sample.t for sample in self.samples], dtype=float)
    positions = np.vstack([sample.p for sample in self.samples])
    positive_dt = np.diff(times)
    positive_dt = positive_dt[positive_dt > 0.0]
    median_dt = float(np.median(positive_dt)) if positive_dt.size > 0 else 0.0
    fps = 1.0 / median_dt if median_dt > 0.0 else 0.0

    if self.mode == 'smooth':
      reference = self.smooth_positions(positions)
    else:
      reference = np.repeat(np.mean(positions, axis=0, keepdims=True), len(positions), axis=0)

    residual = positions - reference
    position_covariance = finite_matrix(np.cov(residual.T))

    if median_dt > 0.0:
      velocity_covariance = finite_matrix(2.0 * position_covariance / (median_dt * median_dt))
    else:
      velocity_covariance = np.zeros((2, 2), dtype=float)

    velocity_covariance_empirical = None
    if self.mode == 'smooth' and len(positions) >= 3:
      dt = np.diff(times)
      valid = dt > 0.0
      if np.count_nonzero(valid) >= 2:
        v_raw = np.diff(positions, axis=0)[valid] / dt[valid, None]
        v_ref = np.diff(reference, axis=0)[valid] / dt[valid, None]
        velocity_covariance_empirical = finite_matrix(np.cov((v_raw - v_ref).T))

    result = {
      'vision_covariance': {
        'topic': self.vision_topic,
        'mode': self.mode,
        'samples': len(self.samples),
        'rejected_samples': self.rejected_samples,
        'reject_reasons': self.reject_reasons,
        'median_dt': median_dt,
        'fps': fps,
        'mean_pixel': np.mean(positions, axis=0).tolist(),
        'position_covariance': matrix_summary(position_covariance),
        'velocity_covariance_from_position': matrix_summary(velocity_covariance),
      }
    }

    if velocity_covariance_empirical is not None:
      result['vision_covariance']['velocity_covariance_empirical'] = matrix_summary(
        velocity_covariance_empirical)

    if self.mode == 'smooth':
      result['vision_covariance']['smoother'] = {
        'type': 'moving_average',
        'window': self.effective_smooth_window(len(positions)),
      }

    return result

  def effective_smooth_window(self, n):
    window = min(self.smooth_window, n)
    if window % 2 == 0:
      window -= 1
    if window > n:
      window = n if n % 2 == 1 else n - 1
    return max(3, window)

  def smooth_positions(self, positions):
    window = self.effective_smooth_window(len(positions))
    half_window = window // 2
    smoothed = np.zeros_like(positions)
    for axis in range(2):
      padded = np.pad(positions[:, axis], (half_window, half_window), mode='edge')
      kernel = np.full(window, 1.0 / window, dtype=float)
      smoothed[:, axis] = np.convolve(padded, kernel, mode='valid')
    return smoothed

  def print_result(self, result):
    data = result['vision_covariance']
    pos = data['position_covariance']
    vel = data['velocity_covariance_from_position']
    self.get_logger().info('=== Vision Pixel Covariance Result ===')
    self.get_logger().info(f"mode: {data['mode']}")
    self.get_logger().info(f"topic: {data['topic']}")
    self.get_logger().info(f"accepted_samples: {data['samples']}")
    self.get_logger().info(f"rejected_samples: {data['rejected_samples']}")
    self.get_logger().info(f"median_dt: {data['median_dt']:.6f} sec")
    self.get_logger().info(f"fps: {data['fps']:.2f}")
    self.get_logger().info(f"mean_pixel: {data['mean_pixel']}")
    self.get_logger().info(f"position_covariance: {pos['matrix']}")
    self.get_logger().info(f"position_std: {pos['std']}")
    self.get_logger().info(f"velocity_covariance_from_position: {vel['matrix']}")
    self.get_logger().info(f"velocity_std_from_position: {vel['std']}")
    if 'velocity_covariance_empirical' in data:
      emp = data['velocity_covariance_empirical']
      self.get_logger().info(f"velocity_covariance_empirical: {emp['matrix']}")
      self.get_logger().info(f"velocity_std_empirical: {emp['std']}")

  def write_yaml(self, result):
    directory = os.path.dirname(self.output_path)
    if directory:
      os.makedirs(directory, exist_ok=True)

    with open(self.output_path, 'w', encoding='utf-8') as f:
      yaml.safe_dump(result, f, sort_keys=False)

    self.get_logger().info(f'Saved covariance result to {self.output_path}')


def main(args=None):
  rclpy.init(args=args)
  node = VisionCovarianceCalib()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    if len(node.samples) >= node.min_samples and not node.finished:
      node.finished = True
      node.finish()
  finally:
    if rclpy.ok():
      rclpy.shutdown()
    node.destroy_node()


if __name__ == '__main__':
  main()
