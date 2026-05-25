#!/usr/bin/env python3
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CompressedImage


class TrackTest(Node):
  def __init__(self):
    super().__init__('track_test')

    self.declare_parameter('image_topic', '/vision/image')
    self.declare_parameter('window_name', 'track_test')
    self.declare_parameter('fallback_width', 480)
    self.declare_parameter('fallback_height', 640)
    self.declare_parameter('view_scale', 0.6)
    self.declare_parameter('hz_window_size', 60)
    self.declare_parameter('draw_rate_hz', 30.0)
    self.declare_parameter('stale_timeout_sec', 0.5)

    self.image_topic = self.get_parameter('image_topic').value
    self.window_name = self.get_parameter('window_name').value
    self.fallback_width = int(self.get_parameter('fallback_width').value)
    self.fallback_height = int(self.get_parameter('fallback_height').value)
    self.view_scale = float(self.get_parameter('view_scale').value)
    self.hz_window_size = max(2, int(self.get_parameter('hz_window_size').value))
    self.draw_rate_hz = float(self.get_parameter('draw_rate_hz').value)
    self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)

    self.latest_image = None
    self.latest_image_time = None
    self.image_times = deque(maxlen=self.hz_window_size)

    image_qos = QoSProfile(
      history=HistoryPolicy.KEEP_LAST,
      depth=1,
      reliability=ReliabilityPolicy.BEST_EFFORT,
    )

    self.create_subscription(CompressedImage, self.image_topic, self.imageCallback, image_qos)
    self.get_logger().info(f'Subscribed to compressed image topic: {self.image_topic}')

    period = 1.0 / max(self.draw_rate_hz, 1.0)
    self.draw_timer = self.create_timer(period, self.draw)

  def nowSeconds(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def averageHz(self, timestamps):
    if len(timestamps) < 2:
      return 0.0

    elapsed = timestamps[-1] - timestamps[0]
    if elapsed <= 1e-6:
      return 0.0

    return (len(timestamps) - 1) / elapsed

  def imageCallback(self, msg):
    np_arr = np.frombuffer(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if image is None:
      return

    self.latest_image = image
    self.latest_image_time = self.nowSeconds()
    self.image_times.append(self.latest_image_time)

  def makeCanvas(self):
    if self.latest_image is not None:
      return self.latest_image.copy()

    canvas = np.full(
      (self.fallback_height, self.fallback_width, 3),
      245,
      dtype=np.uint8)
    cv2.putText(
      canvas,
      f'Waiting for {self.image_topic} ...',
      (20, self.fallback_height // 2),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.7,
      (90, 90, 90),
      1,
      cv2.LINE_AA)
    return canvas

  def drawStatus(self, image):
    now = self.nowSeconds()
    if self.latest_image_time is None:
      text = 'image: waiting'
      text_color = (0, 0, 255)
    else:
      age = now - self.latest_image_time
      hz = self.averageHz(self.image_times)
      text = f'image: {hz:.1f}Hz'
      text_color = (255, 255, 255) if age <= self.stale_timeout_sec else (0, 0, 255)

    cv2.putText(
      image,
      text,
      (16, 26),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.55,
      text_color,
      1,
      cv2.LINE_AA)

  def drawLegend(self, image):
    h = image.shape[0]
    cv2.putText(image, 'q / Esc: quit', (16, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

  def draw(self):
    image = self.makeCanvas()
    image = cv2.resize(
        image,
        None,
        fx=self.view_scale,
        fy=self.view_scale,
        interpolation=cv2.INTER_AREA
    )
    self.height, self.width, _ = image.shape

    # print(self.width, self.height)

    self.drawStatus(image)
    self.drawLegend(image)

    cv2.imshow(self.window_name, image)
    key = cv2.waitKey(1)
    if key in (ord('q'), 27):
      rclpy.shutdown()


def main():
  rclpy.init()
  node = TrackTest()
  try:
    rclpy.spin(node)
  finally:
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == '__main__':
  main()
