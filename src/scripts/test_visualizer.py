#!/usr/bin/env python3
from collections import deque
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import TestDebug
from custom_msgs.msg import VisionMsg


class TestVisualizer(Node):
  def __init__(self):
    super().__init__('test_visualizer')

    self.declare_parameter('debug_topic', '/test/debug')
    self.declare_parameter('width', 1280)
    self.declare_parameter('height', 760)
    self.declare_parameter('history_size', 100)
    self.declare_parameter('label_stride', 10)
    self.declare_parameter('use_test_vision', True)
    self.declare_parameter('vision_topic', '/vision')
    self.declare_parameter('use_fixed_view', True)
    self.declare_parameter('view_min_x', 0.0)
    self.declare_parameter('view_max_x', 640.0)
    self.declare_parameter('view_min_y', 0.0)
    self.declare_parameter('view_max_y', 480.0)
    self.declare_parameter('test_vision_rate_hz', 30.0)
    self.declare_parameter('test_center_x', 320.0)
    self.declare_parameter('test_center_y', 240.0)
    self.declare_parameter('test_amplitude_x', 120.0)
    self.declare_parameter('test_amplitude_y', 80.0)
    self.declare_parameter('test_frequency_hz', 0.5)
    self.declare_parameter('test_position_noise_std', 30.0)

    self.debug_topic = self.get_parameter('debug_topic').value
    self.width = self.get_parameter('width').value
    self.height = self.get_parameter('height').value
    self.history_size = self.get_parameter('history_size').value
    self.label_stride = max(1, self.get_parameter('label_stride').value)
    self.use_test_vision = self.get_parameter('use_test_vision').value
    self.vision_topic = self.get_parameter('vision_topic').value
    self.use_fixed_view = self.get_parameter('use_fixed_view').value
    self.view_min_x = self.get_parameter('view_min_x').value
    self.view_max_x = self.get_parameter('view_max_x').value
    self.view_min_y = self.get_parameter('view_min_y').value
    self.view_max_y = self.get_parameter('view_max_y').value
    self.test_vision_rate_hz = self.get_parameter('test_vision_rate_hz').value
    self.test_center_x = self.get_parameter('test_center_x').value
    self.test_center_y = self.get_parameter('test_center_y').value
    self.test_amplitude_x = self.get_parameter('test_amplitude_x').value
    self.test_amplitude_y = self.get_parameter('test_amplitude_y').value
    self.test_frequency_hz = self.get_parameter('test_frequency_hz').value
    self.test_position_noise_std = self.get_parameter('test_position_noise_std').value

    self.raw_history = deque(maxlen=self.history_size)
    self.true_history = deque(maxlen=self.history_size)
    self.kf_history = deque(maxlen=self.history_size)
    self.latest_control = None
    self.test_vision_start_time = self.get_clock().now()
    self.last_test_position = None
    self.last_test_sample_time = None
    self.last_test_velocity = None
    self.time_origin = None

    self.debug_sub = self.create_subscription(
      TestDebug,
      self.debug_topic,
      self.debugCallback,
      10)
    self.vision_pub = self.create_publisher(VisionMsg, self.vision_topic, 10)

    self.test_vision_timer = None
    if self.use_test_vision:
      period = 1.0 / max(self.test_vision_rate_hz, 1.0)
      self.test_vision_timer = self.create_timer(period, self.publishTestVision)

    self.draw_timer = self.create_timer(1.0 / 30.0, self.draw)

  def publishTestVision(self):
    now = self.get_clock().now()
    sample_time = (now - self.test_vision_start_time).nanoseconds * 1e-9
    stamp_time = now.nanoseconds * 1e-9
    phase = 2.0 * math.pi * self.test_frequency_hz * sample_time

    # Ellipse scenario
    position = np.array([
      self.test_center_x + self.test_amplitude_x * math.cos(phase)+ self.test_amplitude_y/3 * math.sin(2*phase) + self.test_amplitude_y/2 * math.sin(3*phase),
      self.test_center_y + self.test_amplitude_y * math.sin(phase) + self.test_amplitude_y/3 * math.cos(2*phase) + self.test_amplitude_y/2 * math.cos(3*phase),
    ], dtype=float)

    # Vertical linear oscillation scenario
    # period = 1.0 / max(self.test_frequency_hz, 1e-6)
    # cycle = (sample_time / period) % 1.0
    # if cycle < 0.5:
    #   normalized_y = -1.0 + 4.0 * cycle
    # else:
    #   normalized_y = 3.0 - 4.0 * cycle
    
    # position = np.array([
    #   self.test_center_x,
    #   self.test_center_y + self.test_amplitude_y * normalized_y,
    # ], dtype=float)

    velocity = None
    acceleration = None
    if self.last_test_position is not None and self.last_test_sample_time is not None:
      dt = sample_time - self.last_test_sample_time
      if dt > 0.0:
        velocity = (position - self.last_test_position) / dt
        if self.last_test_velocity is not None:
          acceleration = (velocity - self.last_test_velocity) / dt

    observed_position = position.copy()
    if self.test_position_noise_std > 0.0:
      observed_position += np.random.normal(
        0.0, self.test_position_noise_std, size=2)

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
    msg.source_mode = 'test_visualizer'

    self.vision_pub.publish(msg)
    self.appendTrueSample(stamp_time, position)
    self.appendRawSample(
      stamp_time,
      observed_position,
      velocity if velocity is not None else np.zeros(2, dtype=float))
    self.last_test_position = position
    self.last_test_sample_time = sample_time
    self.last_test_velocity = velocity

  def debugCallback(self, msg):
    if msg.has_raw and not self.use_test_vision:
      self.appendRawSample(
        msg.sample_time,
        np.array(msg.raw_p, dtype=float),
        np.array(msg.raw_v, dtype=float))

    if msg.estimator_initialized:
      self.kf_history.append((
        msg.sample_time,
        np.array(msg.estimated_p, dtype=float),
        msg.predicted_only))

    if msg.has_control:
      self.latest_control = msg

  def appendRawSample(self, sample_time, position, velocity):
    if self.time_origin is None:
      self.time_origin = sample_time
    self.raw_history.append((sample_time, position, velocity))

  def appendTrueSample(self, sample_time, position):
    if self.time_origin is None:
      self.time_origin = sample_time
    self.true_history.append((sample_time, position))

  def relativeTime(self, sample_time):
    if self.time_origin is None:
      return 0.0
    return sample_time - self.time_origin

  def getDisplayWindow(self):
    raw_window = list(self.raw_history)[-2:]
    if not raw_window:
      return [], []

    if len(raw_window) == 1:
      target_start = raw_window[0][0]
      target_end = raw_window[0][0]
      kf_window = list(self.kf_history)[-1:]
      return raw_window, kf_window

    target_start = raw_window[0][0]
    target_end = raw_window[1][0]
    kf_window = [
      (sample_time, point, predicted_only)
      for sample_time, point, predicted_only in self.kf_history
      if target_start <= sample_time <= target_end
    ]
    return raw_window, kf_window

  def collectPoints(self, raw_window, kf_window, true_period):
    points = []
    points.extend(point for _, point, _ in raw_window)
    points.extend(point for _, point, _ in kf_window)
    points.extend(point for _, point in true_period)
    return points

  def getOnePeriodWindow(self):
    if not self.raw_history:
      return [], [], [], 1.0

    period = 1.0 / max(self.test_frequency_hz, 1e-6)
    latest_time = self.raw_history[-1][0]
    start_time = latest_time - period

    raw_period = [
      (sample_time, point, velocity)
      for sample_time, point, velocity in self.raw_history
      if sample_time >= start_time
    ]
    kf_period = [
      (sample_time, point, predicted_only)
      for sample_time, point, predicted_only in self.kf_history
      if sample_time >= start_time
    ]
    true_period = [
      (sample_time, point)
      for sample_time, point in self.true_history
      if sample_time >= start_time
    ]
    return raw_period, kf_period, true_period, period

  def getFixedBounds(self):
    min_xy = np.array([self.view_min_x, self.view_min_y], dtype=float)
    max_xy = np.array([self.view_max_x, self.view_max_y], dtype=float)

    if max_xy[0] <= min_xy[0]:
      max_xy[0] = min_xy[0] + 1.0
    if max_xy[1] <= min_xy[1]:
      max_xy[1] = min_xy[1] + 1.0

    return min_xy, max_xy

  def toPixel(self, point, min_xy, max_xy, rect):
    left, top, right, bottom = rect
    span = np.maximum(max_xy - min_xy, 1.0)
    plot_w = max(1, right - left)
    plot_h = max(1, bottom - top)
    normalized = (point - min_xy) / span

    x = int(left + normalized[0] * plot_w)
    y = int(bottom - normalized[1] * plot_h)
    return x, y

  def drawTargetWindow(self, image, raw_window, min_xy, max_xy, rect):
    for idx, (sample_time, point, _) in enumerate(raw_window):
      pixel = self.toPixel(point, min_xy, max_xy, rect)
      is_current = idx == len(raw_window) - 1
      color = (0, 180, 0) if is_current else (120, 200, 120)
      if self.use_test_vision and self.test_position_noise_std > 0.0:
        label = 'noisy current' if is_current else 'noisy prev'
      else:
        label = 'target current' if is_current else 'target prev'
      radius = 6 if is_current else 4
      thickness = -1 if is_current else 2
      cv2.circle(image, pixel, radius, color, thickness)

      text = f'{label} (t={self.relativeTime(sample_time):.2f}s)'
      cv2.putText(
        image, text, (pixel[0] + 6, pixel[1] - 6),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

  def drawTrueTrajectory(self, image, true_period, min_xy, max_xy, rect):
    if len(true_period) < 2:
      return

    color = (200, 200, 200)
    previous_pixel = self.toPixel(true_period[0][1], min_xy, max_xy, rect)
    for _, point in true_period[1:]:
      pixel = self.toPixel(point, min_xy, max_xy, rect)
      cv2.line(image, previous_pixel, pixel, color, 1, cv2.LINE_AA)
      previous_pixel = pixel

  def drawKalmanWindow(self, image, kf_window, min_xy, max_xy, rect):
    for sample_time, point, predicted_only in kf_window:
      pixel = self.toPixel(point, min_xy, max_xy, rect)
      label = 'pred' if predicted_only else 'kf'
      color = (0, 0, 255) if predicted_only else (255, 0, 0)
      cv2.circle(image, pixel, 4, color, -1)
      text = f'{label} (t={self.relativeTime(sample_time):.2f}s)'
      cv2.putText(
        image, text, (pixel[0] + 6, pixel[1] + 14),
        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)

  def drawTimeValuePlot(self, image, raw_period, kf_period, true_period, period, rect, axis_index, axis_label):
    left, top, right, bottom = rect
    cv2.rectangle(image, (left, top), (right, bottom), (220, 220, 220), 1)
    cv2.putText(image, f'one period {axis_label}-value plot', (left, top - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (70, 70, 70), 1)

    axis_values = [point[axis_index] for _, point, _ in raw_period]
    axis_values.extend(point[axis_index] for _, point, _ in kf_period)
    axis_values.extend(point[axis_index] for _, point in true_period)
    if not axis_values:
      cv2.putText(image, 'no samples', (left + 20, top + 30),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)
      return

    min_value = min(axis_values)
    max_value = max(axis_values)
    if max_value <= min_value:
      max_value = min_value + 1.0
    value_padding = max(1.0, 0.1 * (max_value - min_value))
    min_value -= value_padding
    max_value += value_padding

    def to_plot(sample_time, axis_value):
      time_in_period = self.relativeTime(sample_time) % period
      normalized_x = time_in_period / period
      normalized_y = (axis_value - min_value) / max(max_value - min_value, 1.0)
      x = int(left + normalized_x * (right - left))
      y = int(bottom - normalized_y * (bottom - top))
      return x, y

    if len(true_period) >= 2:
      previous_pixel = to_plot(true_period[0][0], true_period[0][1][axis_index])
      for sample_time, point in true_period[1:]:
        pixel = to_plot(sample_time, point[axis_index])
        cv2.line(image, previous_pixel, pixel, (200, 200, 200), 1, cv2.LINE_AA)
        previous_pixel = pixel

    for sample_time, point, _ in raw_period:
      pixel = to_plot(sample_time, point[axis_index])
      cv2.circle(image, pixel, 3, (0, 180, 0), -1)

    for sample_time, point, predicted_only in kf_period:
      pixel = to_plot(sample_time, point[axis_index])
      color = (0, 0, 255) if predicted_only else (255, 0, 0)
      cv2.circle(image, pixel, 3, color, -1)

    cv2.putText(image, '0', (left, bottom + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1)
    cv2.putText(image, f'{period:.2f}s', (right - 40, bottom + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1)
    cv2.putText(image, f'{axis_label} max {max_value:.1f}', (left + 8, top + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1)
    cv2.putText(image, f'{axis_label} min {min_value:.1f}', (left + 8, bottom - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (90, 90, 90), 1)

  def drawLegend(self, image):
    target_label = 'noisy target samples' if self.use_test_vision and self.test_position_noise_std > 0.0 else 'target current/prev'
    cv2.putText(image, target_label, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 150, 0), 2)
    cv2.putText(image, 'true trajectory', (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 2)
    cv2.putText(image, 'kalman filter', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    cv2.putText(image, 'prediction only', (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    mode = 'synthetic vision' if self.use_test_vision else 'real vision'
    cv2.putText(image, f'mode: {mode}', (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)
    view_mode = 'fixed view' if self.use_fixed_view else 'tracking view'
    cv2.putText(image, view_mode, (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)
    cv2.putText(image, 'time is relative to first target sample', (20, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)

    if self.use_fixed_view:
      bounds_text = (
        f'view x:[{self.view_min_x:.0f}, {self.view_max_x:.0f}] '
        f'y:[{self.view_min_y:.0f}, {self.view_max_y:.0f}]')
      cv2.putText(image, bounds_text, (20, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)

    if self.latest_control is not None:
      text = (
        f'u=({self.latest_control.u_yaw:.3f}, {self.latest_control.u_pitch:.3f}) '
        f'fire={self.latest_control.fire} reload={self.latest_control.reload}')
      control_y = 230 if self.use_fixed_view else 205
      cv2.putText(image, text, (20, control_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (70, 70, 70), 1)

  def draw(self):
    image = np.full((self.height, self.width, 3), 255, dtype=np.uint8)
    self.drawLegend(image)

    raw_window, kf_window = self.getDisplayWindow()
    raw_period, kf_period, true_period, period = self.getOnePeriodWindow()
    points = self.collectPoints(raw_window, kf_window, true_period)
    if not points:
      cv2.putText(
        image, 'Waiting for target/debug data ...', (20, self.height // 2),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 80, 80), 2)
      cv2.imshow('test_visualizer', image)
      cv2.waitKey(1)
      return

    if self.use_fixed_view:
      min_xy, max_xy = self.getFixedBounds()
    else:
      points_array = np.vstack(points)
      min_xy = points_array.min(axis=0)
      max_xy = points_array.max(axis=0)
      padding = np.maximum((max_xy - min_xy) * 0.1, 1.0)
      min_xy -= padding
      max_xy += padding

    top_y = 240
    bottom_y = self.height - 60
    spatial_rect = (60, top_y, 620, bottom_y)
    plot_left = 670
    plot_gap = 28
    mid_y = top_y + (bottom_y - top_y - plot_gap) // 2
    time_x_rect = (plot_left, top_y, self.width - 60, mid_y)
    time_y_rect = (plot_left, mid_y + plot_gap, self.width - 60, bottom_y)

    cv2.rectangle(image, (spatial_rect[0], spatial_rect[1]), (spatial_rect[2], spatial_rect[3]), (220, 220, 220), 1)
    self.drawTrueTrajectory(image, true_period, min_xy, max_xy, spatial_rect)
    self.drawTargetWindow(image, raw_window, min_xy, max_xy, spatial_rect)
    self.drawKalmanWindow(image, kf_window, min_xy, max_xy, spatial_rect)

    self.drawTimeValuePlot(image, raw_period, kf_period, true_period, period, time_x_rect, 0, 'x')
    self.drawTimeValuePlot(image, raw_period, kf_period, true_period, period, time_y_rect, 1, 'y')

    cv2.imshow('test_visualizer', image)
    key = cv2.waitKey(1)
    if key in (ord('q'), 27):
      rclpy.shutdown()


def main():
  rclpy.init()
  node = TestVisualizer()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
