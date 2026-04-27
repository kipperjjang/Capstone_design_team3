#!/usr/bin/env python3
from collections import deque

from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import TestDebug
from sensor_msgs.msg import Image


class VisionTest(Node):
  def __init__(self):
    super().__init__('vision_test')

    self.declare_parameter('mode', 'offline') # offline, realtime
    self.declare_parameter('debug_topic', '/test/debug')
    self.declare_parameter('image_topic', '/vision/image')
    self.declare_parameter('collection_duration_sec', 20.0)
    self.declare_parameter('history_size', 100)
    self.declare_parameter('draw_prediction_included', True)
    self.declare_parameter('width', 1280)
    self.declare_parameter('height', 760)
    self.declare_parameter('use_fixed_view', True)
    self.declare_parameter('view_min_x', 0.0)
    self.declare_parameter('view_max_x', 640.0)
    self.declare_parameter('view_min_y', 0.0)
    self.declare_parameter('view_max_y', 480.0)

    self.mode = self.get_parameter('mode').value
    self.debug_topic = self.get_parameter('debug_topic').value
    self.image_topic = self.get_parameter('image_topic').value
    self.collection_duration_sec = float(self.get_parameter('collection_duration_sec').value)
    self.history_size = max(2, int(self.get_parameter('history_size').value))
    self.draw_prediction_included = bool(self.get_parameter('draw_prediction_included').value)
    self.width = int(self.get_parameter('width').value)
    self.height = int(self.get_parameter('height').value)
    self.use_fixed_view = bool(self.get_parameter('use_fixed_view').value)
    self.view_min_x = float(self.get_parameter('view_min_x').value)
    self.view_max_x = float(self.get_parameter('view_max_x').value)
    self.view_min_y = float(self.get_parameter('view_min_y').value)
    self.view_max_y = float(self.get_parameter('view_max_y').value)

    if self.mode not in ('offline', 'realtime'):
      self.get_logger().warn(
        "Unknown mode '%s'; falling back to realtime." % self.mode)
      self.mode = 'realtime'

    self.bridge = CvBridge()
    self.time_origin = None
    self.offline_start_time = self.nowSeconds()
    self.plots_rendered = False
    self.latest_image = None
    self.latest_image_stamp = None

    self.raw_samples = []
    self.estimated_samples = []
    self.raw_history = deque(maxlen=self.history_size)
    self.estimated_history = deque(maxlen=self.history_size)

    self.debug_sub = self.create_subscription(
      TestDebug,
      self.debug_topic,
      self.debugCallback,
      10)

    self.image_sub = None
    self.draw_timer = None
    self.monitor_timer = None

    if self.mode == 'realtime':
      self.image_sub = self.create_subscription(
        Image,
        self.image_topic,
        self.imageCallback,
        10)
      self.draw_timer = self.create_timer(1.0 / 30.0, self.drawRealtime)
      self.get_logger().info('Running realtime vision visualization.')
    else:
      self.monitor_timer = self.create_timer(0.05, self.monitorOffline)
      self.get_logger().info(
        'Collecting %.2fs of /test/debug data for offline plots.' %
        self.collection_duration_sec)

  def nowSeconds(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def stampSeconds(self, stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9

  def relativeTime(self, sample_time):
    if self.time_origin is None:
      self.time_origin = sample_time
    return sample_time - self.time_origin

  def debugCallback(self, msg):
    sample_time = float(msg.sample_time)
    rel_time = self.relativeTime(sample_time)

    if msg.has_raw:
      raw_point = np.array(msg.raw_p, dtype=float)
      raw_sample = (sample_time, rel_time, raw_point)
      self.raw_samples.append(raw_sample)
      self.raw_history.append(raw_sample)

    if msg.estimator_initialized:
      estimated_point = np.array(msg.estimated_p, dtype=float)
      estimated_sample = (
        sample_time,
        rel_time,
        estimated_point,
        bool(msg.predicted_only))
      self.estimated_samples.append(estimated_sample)
      self.estimated_history.append(estimated_sample)

  def imageCallback(self, msg):
    self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.latest_image_stamp = self.stampSeconds(msg.header.stamp)

  def monitorOffline(self):
    if self.plots_rendered:
      return

    if self.nowSeconds() - self.offline_start_time < self.collection_duration_sec:
      return

    self.plots_rendered = True
    self.get_logger().info(
      'Rendering offline plots with %d raw and %d estimated samples.' %
      (len(self.raw_samples), len(self.estimated_samples)))
    self.showOfflinePlots()
    rclpy.shutdown()

  def estimatedLineSamples(self, samples):
    if self.draw_prediction_included:
      return list(samples)
    return [sample for sample in samples if not sample[3]]

  def sampleArrays(self, samples, has_prediction_flag=False):
    if not samples:
      return np.array([], dtype=float), np.empty((0, 2), dtype=float)

    if has_prediction_flag:
      times = np.asarray([sample[1] for sample in samples], dtype=float)
      points = np.asarray([sample[2] for sample in samples], dtype=float)
    else:
      times = np.asarray([sample[1] for sample in samples], dtype=float)
      points = np.asarray([sample[2] for sample in samples], dtype=float)
    return times, points

  def plotOfflineTrajectory(self, ax, raw_times, raw_points,
                            estimated_times, estimated_points,
                            prediction_times, prediction_points):
    if len(raw_points) > 0:
      ax.plot(raw_points[:, 0], raw_points[:, 1], color='tab:green',
              linewidth=1.4, alpha=0.55)
      ax.scatter(raw_points[:, 0], raw_points[:, 1], color='tab:green',
                 s=18, alpha=0.65, label='raw vision')

    if len(estimated_points) > 0:
      ax.plot(estimated_points[:, 0], estimated_points[:, 1],
              color='tab:blue', linewidth=2.2, label='estimated state')

    if len(prediction_points) > 0:
      ax.scatter(prediction_points[:, 0], prediction_points[:, 1],
                 color='tab:red', s=18, alpha=0.75, label='prediction only')

    ax.set_xlabel('x [px]')
    ax.set_ylabel('y [px]')
    ax.set_title('Image-plane trajectory')
    ax.grid(True, alpha=0.3)
    if self.use_fixed_view:
      ax.set_xlim(self.view_min_x, self.view_max_x)
      ax.set_ylim(self.view_min_y, self.view_max_y)
    if len(raw_points) == 0 and len(estimated_points) == 0:
      ax.text(0.5, 0.5, 'No samples collected',
              transform=ax.transAxes, ha='center', va='center')

  def plotOfflineAxis(self, ax, axis_index, axis_label,
                      raw_times, raw_points,
                      estimated_times, estimated_points,
                      prediction_times, prediction_points):
    if len(raw_points) > 0:
      ax.scatter(raw_times, raw_points[:, axis_index], color='tab:green',
                 s=18, alpha=0.65, label='raw vision')
      ax.plot(raw_times, raw_points[:, axis_index], color='tab:green',
              linewidth=1.2, alpha=0.45)

    if len(estimated_points) > 0:
      ax.plot(estimated_times, estimated_points[:, axis_index],
              color='tab:blue', linewidth=2.0, label='estimated state')

    if len(prediction_points) > 0:
      ax.scatter(prediction_times, prediction_points[:, axis_index],
                 color='tab:red', s=18, alpha=0.75, label='prediction only')

    ax.set_xlabel('time [s]')
    ax.set_ylabel('%s [px]' % axis_label)
    ax.set_title('%s over time' % axis_label)
    ax.grid(True, alpha=0.3)
    if len(raw_points) == 0 and len(estimated_points) == 0:
      ax.text(0.5, 0.5, 'No samples collected',
              transform=ax.transAxes, ha='center', va='center')

  def showOfflinePlots(self):
    raw_times, raw_points = self.sampleArrays(self.raw_samples)
    line_samples = self.estimatedLineSamples(self.estimated_samples)
    estimated_times, estimated_points = self.sampleArrays(line_samples, True)
    prediction_samples = [sample for sample in self.estimated_samples if sample[3]]
    prediction_times, prediction_points = self.sampleArrays(prediction_samples, True)

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    fig.subplots_adjust(top=0.78, wspace=0.28)
    ax_xy, ax_x, ax_y = axes

    self.plotOfflineTrajectory(
      ax_xy,
      raw_times,
      raw_points,
      estimated_times,
      estimated_points,
      prediction_times,
      prediction_points)
    self.plotOfflineAxis(
      ax_x,
      0,
      'x',
      raw_times,
      raw_points,
      estimated_times,
      estimated_points,
      prediction_times,
      prediction_points)
    self.plotOfflineAxis(
      ax_y,
      1,
      'y',
      raw_times,
      raw_points,
      estimated_times,
      estimated_points,
      prediction_times,
      prediction_points)

    handles, labels = ax_xy.get_legend_handles_labels()
    if handles:
      fig.legend(
        handles,
        labels,
        ncol=min(len(labels), 3),
        loc='upper center',
        bbox_to_anchor=(0.5, 0.92),
        frameon=False)
    fig.suptitle('Vision / Kalman Trajectory Summary', fontsize=14, y=0.99)
    plt.show()

  def getFixedBounds(self):
    min_xy = np.array([self.view_min_x, self.view_min_y], dtype=float)
    max_xy = np.array([self.view_max_x, self.view_max_y], dtype=float)
    if max_xy[0] <= min_xy[0]:
      max_xy[0] = min_xy[0] + 1.0
    if max_xy[1] <= min_xy[1]:
      max_xy[1] = min_xy[1] + 1.0
    return min_xy, max_xy

  def getDynamicBounds(self, points):
    if not points:
      return self.getFixedBounds()

    point_array = np.vstack(points)
    min_xy = point_array.min(axis=0)
    max_xy = point_array.max(axis=0)
    padding = np.maximum((max_xy - min_xy) * 0.1, 1.0)
    return min_xy - padding, max_xy + padding

  def getDisplayBounds(self, points):
    if self.use_fixed_view:
      return self.getFixedBounds()
    return self.getDynamicBounds(points)

  def toPanelPixel(self, point, min_xy, max_xy, rect):
    left, top, right, bottom = rect
    span = np.maximum(max_xy - min_xy, 1.0)
    normalized = (point - min_xy) / span
    x = int(left + normalized[0] * max(1, right - left))
    y = int(bottom - normalized[1] * max(1, bottom - top))
    return x, y

  def drawPolyline(self, image, points, color, thickness=1):
    if len(points) < 2:
      return

    for idx in range(1, len(points)):
      cv2.line(image, points[idx - 1], points[idx], color, thickness, cv2.LINE_AA)

  def drawImageOverlay(self):
    if self.latest_image is None:
      return None

    image = self.latest_image.copy()
    raw_points = [
      (int(round(sample[2][0])), int(round(sample[2][1])))
      for sample in self.raw_history
    ]
    filtered_points = [
      (int(round(sample[2][0])), int(round(sample[2][1])))
      for sample in self.estimated_history
      if self.draw_prediction_included or not sample[3]
    ]
    prediction_points = [
      (int(round(sample[2][0])), int(round(sample[2][1])))
      for sample in self.estimated_history
      if sample[3]
    ]

    self.drawPolyline(image, raw_points, (0, 180, 0), 1)
    self.drawPolyline(image, filtered_points, (255, 0, 0), 2)
    self.drawPolyline(image, prediction_points, (0, 0, 255), 1)

    if self.raw_history:
      self.drawOverlayPoint(image, self.raw_history[-1][2], (0, 180, 0), 'raw', (8, -8))

    latest_filtered = next(
      (sample for sample in reversed(self.estimated_history) if not sample[3]),
      None)
    if latest_filtered is not None:
      self.drawOverlayPoint(image, latest_filtered[2], (255, 0, 0), 'est', (8, 16))

    latest_prediction = next(
      (sample for sample in reversed(self.estimated_history) if sample[3]),
      None)
    if latest_prediction is not None:
      self.drawOverlayPoint(image, latest_prediction[2], (0, 0, 255), 'pred', (8, 36))

    return image

  def drawOverlayPoint(self, image, point, color, label, offset):
    pixel = (int(round(point[0])), int(round(point[1])))
    cv2.circle(image, pixel, 5, color, -1)
    cv2.putText(
      image,
      label,
      (pixel[0] + offset[0], pixel[1] + offset[1]),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.45,
      color,
      1,
      cv2.LINE_AA)

  def placeImageInRect(self, canvas, image, rect):
    left, top, right, bottom = rect
    panel_h = max(1, bottom - top)
    panel_w = max(1, right - left)

    image_h, image_w = image.shape[:2]
    scale = min(panel_w / max(1, image_w), panel_h / max(1, image_h))
    size = (
      max(1, int(round(image_w * scale))),
      max(1, int(round(image_h * scale))))
    resized = cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)
    x_offset = left + (panel_w - resized.shape[1]) // 2
    y_offset = top + (panel_h - resized.shape[0]) // 2
    canvas[y_offset:y_offset + resized.shape[0],
           x_offset:x_offset + resized.shape[1]] = resized

  def drawSpatialFallback(self, canvas, rect, min_xy, max_xy):
    cv2.rectangle(canvas, (rect[0], rect[1]), (rect[2], rect[3]), (220, 220, 220), 1)

    raw_pixels = [
      self.toPanelPixel(sample[2], min_xy, max_xy, rect)
      for sample in self.raw_history
    ]
    estimated_pixels = [
      self.toPanelPixel(sample[2], min_xy, max_xy, rect)
      for sample in self.estimated_history
      if self.draw_prediction_included or not sample[3]
    ]
    prediction_pixels = [
      self.toPanelPixel(sample[2], min_xy, max_xy, rect)
      for sample in self.estimated_history
      if sample[3]
    ]

    self.drawPolyline(canvas, raw_pixels, (0, 180, 0), 1)
    self.drawPolyline(canvas, estimated_pixels, (255, 0, 0), 2)
    self.drawPolyline(canvas, prediction_pixels, (0, 0, 255), 1)

    if raw_pixels:
      cv2.circle(canvas, raw_pixels[-1], 5, (0, 180, 0), -1)
    if estimated_pixels:
      cv2.circle(canvas, estimated_pixels[-1], 5, (255, 0, 0), -1)
    if prediction_pixels:
      cv2.circle(canvas, prediction_pixels[-1], 5, (0, 0, 255), -1)

    cv2.putText(
      canvas,
      'Waiting for /vision/image ...',
      (rect[0] + 20, rect[1] + 32),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.65,
      (90, 90, 90),
      2,
      cv2.LINE_AA)

  def drawLegend(self, canvas):
    cv2.putText(canvas, 'raw vision', (24, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 150, 0), 2, cv2.LINE_AA)
    cv2.putText(canvas, 'estimated state', (24, 56), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(canvas, 'prediction only', (24, 82), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(canvas, 'mode: realtime', (250, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (70, 70, 70), 1, cv2.LINE_AA)
    cv2.putText(canvas, 'q / Esc: quit', (250, 56), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (70, 70, 70), 1, cv2.LINE_AA)

  def collectRealtimePoints(self):
    points = [sample[2] for sample in self.raw_history]
    points.extend(sample[2] for sample in self.estimated_history)
    return points

  def drawTimePlot(self, canvas, rect, axis_index, axis_label):
    left, top, right, bottom = rect
    cv2.rectangle(canvas, (left, top), (right, bottom), (220, 220, 220), 1)
    cv2.putText(canvas, '%s over time' % axis_label, (left, top - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (70, 70, 70), 1, cv2.LINE_AA)

    raw_samples = list(self.raw_history)
    estimated_samples = [
      sample for sample in self.estimated_history
      if self.draw_prediction_included or not sample[3]
    ]
    prediction_samples = [sample for sample in self.estimated_history if sample[3]]
    all_samples = raw_samples + estimated_samples + prediction_samples

    if not all_samples:
      cv2.putText(canvas, 'no samples', (left + 20, top + 32),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1, cv2.LINE_AA)
      return

    min_time = min(sample[1] for sample in all_samples)
    max_time = max(sample[1] for sample in all_samples)
    if max_time <= min_time:
      max_time = min_time + 1.0

    values = [sample[2][axis_index] for sample in all_samples]
    min_value = min(values)
    max_value = max(values)
    if max_value <= min_value:
      max_value = min_value + 1.0
    padding = max(1.0, 0.1 * (max_value - min_value))
    min_value -= padding
    max_value += padding

    def to_plot(sample):
      x_norm = (sample[1] - min_time) / max(max_time - min_time, 1e-9)
      y_norm = (sample[2][axis_index] - min_value) / max(max_value - min_value, 1e-9)
      x = int(left + x_norm * max(1, right - left))
      y = int(bottom - y_norm * max(1, bottom - top))
      return x, y

    self.drawPolyline(canvas, [to_plot(sample) for sample in raw_samples], (0, 180, 0), 1)
    self.drawPolyline(canvas, [to_plot(sample) for sample in estimated_samples], (255, 0, 0), 2)
    self.drawPolyline(canvas, [to_plot(sample) for sample in prediction_samples], (0, 0, 255), 1)

    for samples, color in (
      (raw_samples[-1:], (0, 180, 0)),
      (estimated_samples[-1:], (255, 0, 0)),
      (prediction_samples[-1:], (0, 0, 255)),
    ):
      for sample in samples:
        cv2.circle(canvas, to_plot(sample), 3, color, -1)

  def drawRealtime(self):
    canvas = np.full((self.height, self.width, 3), 255, dtype=np.uint8)
    self.drawLegend(canvas)

    top_y = 112
    bottom_y = self.height - 44
    visual_rect = (40, top_y, min(640, self.width // 2), bottom_y)
    plot_left = visual_rect[2] + 50
    plot_right = self.width - 50
    plot_gap = 32
    mid_y = top_y + (bottom_y - top_y - plot_gap) // 2
    time_x_rect = (plot_left, top_y, plot_right, mid_y)
    time_y_rect = (plot_left, mid_y + plot_gap, plot_right, bottom_y)

    points = self.collectRealtimePoints()
    min_xy, max_xy = self.getDisplayBounds(points)

    overlay_image = self.drawImageOverlay()
    if overlay_image is not None:
      cv2.rectangle(canvas, (visual_rect[0], visual_rect[1]),
                    (visual_rect[2], visual_rect[3]), (220, 220, 220), 1)
      self.placeImageInRect(canvas, overlay_image, visual_rect)
    else:
      self.drawSpatialFallback(canvas, visual_rect, min_xy, max_xy)

    self.drawTimePlot(canvas, time_x_rect, 0, 'x')
    self.drawTimePlot(canvas, time_y_rect, 1, 'y')

    cv2.imshow('vision_test', canvas)
    key = cv2.waitKey(1)
    if key in (ord('q'), 27):
      rclpy.shutdown()


def main():
  rclpy.init()
  node = VisionTest()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
