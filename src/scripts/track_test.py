#!/usr/bin/env python3
from collections import deque

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from custom_msgs.msg import TestDebug
from custom_msgs.msg import VisionMsg
from sensor_msgs.msg import Image


class TrackTest(Node):
  def __init__(self):
    super().__init__('track_test')

    self.declare_parameter('vision_topic', '/vision')
    self.declare_parameter('image_topic', '/vision/image')
    self.declare_parameter('debug_topic', '/test/debug')
    self.declare_parameter('window_name', 'track_test')
    self.declare_parameter('fallback_width', 640)
    self.declare_parameter('fallback_height', 480)
    self.declare_parameter('history_size', 120)
    self.declare_parameter('draw_rate_hz', 30.0)
    self.declare_parameter('stale_timeout_sec', 0.5)
    self.declare_parameter('velocity_arrow_scale', 0.05)
    self.declare_parameter('max_arrow_length', 90.0)

    self.vision_topic = self.get_parameter('vision_topic').value
    self.image_topic = self.get_parameter('image_topic').value
    self.debug_topic = self.get_parameter('debug_topic').value
    self.window_name = self.get_parameter('window_name').value
    self.fallback_width = int(self.get_parameter('fallback_width').value)
    self.fallback_height = int(self.get_parameter('fallback_height').value)
    self.history_size = max(2, int(self.get_parameter('history_size').value))
    self.draw_rate_hz = float(self.get_parameter('draw_rate_hz').value)
    self.stale_timeout_sec = float(self.get_parameter('stale_timeout_sec').value)
    self.velocity_arrow_scale = float(self.get_parameter('velocity_arrow_scale').value)
    self.max_arrow_length = float(self.get_parameter('max_arrow_length').value)

    self.bridge = CvBridge()
    self.latest_image = None
    self.latest_vision = None
    self.latest_debug = None
    self.latest_image_time = None
    self.latest_vision_time = None
    self.latest_debug_time = None

    self.yolo_history = deque(maxlen=self.history_size)
    self.estimate_history = deque(maxlen=self.history_size)

    self.create_subscription(Image, self.image_topic, self.imageCallback, 10)
    self.create_subscription(VisionMsg, self.vision_topic, self.visionCallback, 10)
    self.create_subscription(TestDebug, self.debug_topic, self.debugCallback, 10)

    period = 1.0 / max(self.draw_rate_hz, 1.0)
    self.draw_timer = self.create_timer(period, self.draw)

  def nowSeconds(self):
    return self.get_clock().now().nanoseconds * 1e-9

  def imageCallback(self, msg):
    self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.latest_image_time = self.nowSeconds()

  def visionCallback(self, msg):
    self.latest_vision = msg
    self.latest_vision_time = self.nowSeconds()
    if msg.detected and len(msg.p) >= 2:
      self.yolo_history.append(np.array(msg.p[:2], dtype=float))

  def debugCallback(self, msg):
    self.latest_debug = msg
    self.latest_debug_time = self.nowSeconds()
    if msg.estimator_initialized:
      self.estimate_history.append(np.array(msg.estimated_p, dtype=float))

  def makeCanvas(self):
    if self.latest_image is not None:
      return self.latest_image.copy()

    canvas = np.full(
      (self.fallback_height, self.fallback_width, 3),
      245,
      dtype=np.uint8)
    cv2.putText(
      canvas,
      'Waiting for /vision/image ...',
      (20, self.fallback_height // 2),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.7,
      (90, 90, 90),
      1,
      cv2.LINE_AA)
    return canvas

  def drawTrail(self, image, history, color, thickness):
    if len(history) < 2:
      return

    previous = self.pointToPixel(history[0])
    for point in list(history)[1:]:
      current = self.pointToPixel(point)
      cv2.line(image, previous, current, color, thickness, cv2.LINE_AA)
      previous = current

  def pointToPixel(self, point):
    return int(round(point[0])), int(round(point[1]))

  def drawPoint(self, image, point, color, label, offset):
    pixel = self.pointToPixel(point)
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

  def drawVelocityArrow(self, image, point, velocity, color, label):
    velocity = np.array(velocity, dtype=float)
    norm = float(np.linalg.norm(velocity))
    if norm <= 1e-6:
      return

    delta = velocity * self.velocity_arrow_scale
    delta_norm = float(np.linalg.norm(delta))
    if delta_norm > self.max_arrow_length:
      delta *= self.max_arrow_length / delta_norm

    start = self.pointToPixel(point)
    end = self.pointToPixel(point + delta)
    cv2.arrowedLine(image, start, end, color, 2, cv2.LINE_AA, tipLength=0.5)
    cv2.putText(
      image,
      label,
      (end[0] + 6, end[1] + 4),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.42,
      color,
      1,
      cv2.LINE_AA)

  def drawYolo(self, image):
    msg = self.latest_vision
    if msg is None or len(msg.p) < 2:
      return

    point = np.array(msg.p[:2], dtype=float)
    self.drawTrail(image, self.yolo_history, (0, 180, 0), 1)

    if msg.detected and len(msg.bbox) >= 2:
      half_w = float(msg.bbox[0]) * 0.5
      half_h = float(msg.bbox[1]) * 0.5
      top_left = self.pointToPixel(point - np.array([half_w, half_h]))
      bottom_right = self.pointToPixel(point + np.array([half_w, half_h]))
      cv2.rectangle(image, top_left, bottom_right, (0, 180, 0), 2)

    self.drawPoint(image, point, (0, 180, 0), 'YOLO', (8, -8))

    if len(msg.v) >= 2:
      self.drawVelocityArrow(
        image,
        point,
        np.array(msg.v[:2], dtype=float),
        (0, 140, 0),
        'raw v')

  def drawEstimate(self, image):
    msg = self.latest_debug
    if msg is None or not msg.estimator_initialized:
      return

    point = np.array(msg.estimated_p, dtype=float)
    velocity = np.array(msg.estimated_v, dtype=float)
    color = (0, 0, 255) if msg.predicted_only else (255, 0, 0)
    label = 'pred' if msg.predicted_only else 'est'

    self.drawTrail(image, self.estimate_history, (255, 120, 120), 2)
    self.drawPoint(image, point, color, label, (8, 18))
    self.drawVelocityArrow(image, point, velocity, (255, 0, 0), 'est v')

  def drawStatus(self, image):
    rows = [
      ('image', self.latest_image_time, (255, 255, 255)),
      ('vision', self.latest_vision_time, (0, 220, 0)),
      ('ctrl debug', self.latest_debug_time, (220, 0, 0)),
    ]
    now = self.nowSeconds()
    y = 26

    for label, timestamp, color in rows:
      if timestamp is None:
        text = f'{label}: waiting'
        text_color = (0, 0, 255)
      else:
        age = now - timestamp
        hz = 1.0 / age if age > 1e-6 else 0.0
        text = f'{label}: {hz:.1f}Hz'
        text_color = color if age <= self.stale_timeout_sec else (0, 0, 255)

      cv2.putText(
        image,
        text,
        (16, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        text_color,
        1,
        cv2.LINE_AA)
      y += 24

    if self.latest_debug is not None and self.latest_debug.has_control:
      ctrl = self.latest_debug
      control_text = (
        f'u=({ctrl.u_yaw:.2f}, {ctrl.u_pitch:.2f}) '
        f'fire={ctrl.fire} reload={ctrl.reload}')
      cv2.putText(
        image,
        control_text,
        (16, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        1,
        cv2.LINE_AA)

  def drawLegend(self, image):
    h = image.shape[0]
    cv2.putText(image, 'q / Esc: quit', (16, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, 'YOLO bbox/center', (170, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1, cv2.LINE_AA)
    cv2.putText(image, 'ctrl estimate/velocity', (360, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 0, 0), 1, cv2.LINE_AA)

  def draw(self):
    image = self.makeCanvas()
    self.drawYolo(image)
    self.drawEstimate(image)
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
