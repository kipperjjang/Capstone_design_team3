#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ultralytics import YOLO

from custom_msgs.msg import VisionMsg
from sensor_msgs.msg import Image

from trajectory_utils import load_runtime_config


class VisionNode(Node):
  def __init__(self):
    super().__init__("vision_node")

    self.declare_parameter("config_path", "")
    self.declare_parameter("yolo_model_path", "yolo_models/robot_yolo_p4_416_combine/weights/best.pt")
    self.declare_parameter("publish_image", False)
    self.declare_parameter("max_motion_dt", 0.5)

    self.config_path = self.get_parameter("config_path").value
    self.config = load_runtime_config(self.config_path)
    vision_config = self.config.get("vision", {})

    self.publish_image = self.get_parameter("publish_image").value
    self.max_motion_dt = float(self.get_parameter("max_motion_dt").value)
    self.yolo_model_path = self.get_parameter("yolo_model_path").value

    self.target_class = int(vision_config.get("target_class", 1))
    self.confidence = float(vision_config.get("confidence", 0.2))
    self.box_size = np.zeros(2, dtype=np.float32)
    
    self.bridge = CvBridge()
    self.capture = cv2.VideoCapture(0)
    self.model = YOLO(self.yolo_model_path)
    self.position = np.zeros(2, dtype=np.float32)
    self.velocity = np.zeros(2, dtype=np.float32)
    self.acceleration = np.zeros(2, dtype=np.float32)
    self.has_velocity = False
    self.has_acceleration = False
    self.prev_position = None
    self.prev_velocity = None
    self.prev_sample_time = None
    self.is_detected = False
    self.img = None

    self.state_pub = self.create_publisher(VisionMsg, "/vision", 10)
    self.image_pub = self.create_publisher(Image, "/vision/image", 10) if self.publish_image else None

  def read_img(self):
    ret, frame = self.capture.read()
    if not ret:
      return False

    self.img = frame
    return True

  def detect_bell(self):
    self.is_detected = False
    self.position = np.zeros(2, dtype=np.float32)
    if self.img is None:
      return

    results = self.model(self.img, conf=self.confidence, classes=[self.target_class], verbose=False)
    if not results:
      return

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
      return

    confidences = boxes.conf.cpu().numpy()
    best_index = int(np.argmax(confidences))
    x1, y1, x2, y2 = boxes.xyxy.cpu().numpy()[best_index]
    self.box_size = np.abs(np.array([x2 - x1, y2 - y1]))
    self.position = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], dtype=np.float32)
    self.is_detected = True

  def reset_motion_estimate(self):
    self.velocity = np.zeros(2, dtype=np.float32)
    self.acceleration = np.zeros(2, dtype=np.float32)
    self.has_velocity = False
    self.has_acceleration = False
    self.prev_position = None
    self.prev_velocity = None
    self.prev_sample_time = None

  def update_motion_estimate(self, sample_time):
    if not self.is_detected:
      self.reset_motion_estimate()
      return

    if self.prev_position is not None and self.prev_sample_time is not None:
      dt = sample_time - self.prev_sample_time
      if 0.0 < dt <= self.max_motion_dt:
        self.velocity = ((self.position - self.prev_position) / dt).astype(np.float32)
        self.has_velocity = True

        if self.prev_velocity is not None:
          self.acceleration = ((self.velocity - self.prev_velocity) / dt).astype(np.float32)
          self.has_acceleration = True
        else:
          self.acceleration = np.zeros(2, dtype=np.float32)
          self.has_acceleration = False
      else:
        self.velocity = np.zeros(2, dtype=np.float32)
        self.acceleration = np.zeros(2, dtype=np.float32)
        self.has_velocity = False
        self.has_acceleration = False

    self.prev_position = self.position.copy()
    self.prev_velocity = self.velocity.copy() if self.has_velocity else None
    self.prev_sample_time = sample_time

  def publish(self):
    now = self.get_clock().now()
    sample_time = now.nanoseconds * 1e-9
    stamp = now.to_msg()
    self.update_motion_estimate(sample_time)

    msg = VisionMsg()
    msg.header.stamp = stamp
    msg.detected = self.is_detected
    msg.tracked = self.has_velocity
    msg.p = self.position.tolist()
    msg.v = self.velocity.tolist() if self.has_velocity else []
    msg.a = self.acceleration.tolist() if self.has_acceleration else []
    msg.bbox = self.box_size
    msg.confidence = float(self.confidence)
    msg.covariance = []
    msg.source_mode = "YOLO"
    self.state_pub.publish(msg)

    if self.image_pub is not None and self.img is not None:
      image_msg = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
      image_msg.header.stamp = stamp
      self.image_pub.publish(image_msg)

  def run(self):
    executor = SingleThreadedExecutor()
    executor.add_node(self)

    try:
      while rclpy.ok():
        if not self.read_img():
          executor.spin_once(timeout_sec=0.01)
          continue

        self.detect_bell()
        self.publish()
        executor.spin_once(timeout_sec=0.0)
    finally:
      executor.shutdown()
      self.capture.release()
      self.destroy_node()
      if rclpy.ok():
        rclpy.shutdown()


def main():
  rclpy.init()
  node = VisionNode()
  node.run()


if __name__ == "__main__":
  main()
