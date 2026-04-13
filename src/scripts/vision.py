#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from custom_msgs.msg import VisionMsg
from sensor_msgs.msg import Image

class VisionNode(Node):
  def __init__(self):
    super().__init__('vision_node')
    self.declare_parameter('publish_image', True)
    self.declare_parameter('image_topic', '/vision/image')
    self.declare_parameter('config_path', '')
    self.declare_parameter('yolo_model_path', 'yolo_models/robot_yolo_p4_416_combine/weights/best.pt')
  
    # Load configuration
    self.yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
    self.loadYAML(self.get_parameter('config_path').get_parameter_value().string_value)
    self.publish_image = self.get_parameter('publish_image').value
    self.image_topic = self.get_parameter('image_topic').value

    # OpenCV
    self.bridge = CvBridge()
    self.capture = cv2.VideoCapture(0) # /dev/video0
    self.isDetected = False
    self.img = None

    # YOLO
    self.model = YOLO(self.yolo_model_path)
    # np.array for relative positions of detected object with respect to the center of image
    self.rel_pos = np.empty((2,2))
    self.bbox = np.zeros(2)

    # ROS Publisher
    self.state_pub = self.create_publisher(VisionMsg, '/vision', 10)
    self.image_pub = None
    if self.publish_image:
      self.image_pub = self.create_publisher(Image, self.image_topic, 10)

  def loadYAML(self, path):
    with open(path, 'r') as f:
      config = yaml.safe_load(f)
    self.target_class = config['vision']['target_class']
    self.confidence = config['vision']['confidence']

  def readImg(self):
    ret, frame = self.capture.read()
    if not ret:
      return False

    height, width = frame.shape[:2]
    self.frame_center = np.array([width // 2, height // 2])

    self.img = frame
    return True

  def detectBell(self):
    # Classes : pillar [0] / bell [1]
    TARGET_CLASSES = (1)

    # Inference only for the classes in TARGET_CLASSES
    results = self.model(self.img, conf=self.confidence, classes=list(TARGET_CLASSES), verbose=False)
    r = results[0]
    boxes = r.boxes

    # Early exit if no detections
    if boxes is None or len(boxes) == 0:
      return

    # N : number of boxes
    xyxy = boxes.xyxy.cpu().numpy()  # shape: (N, 4)
    cls = boxes.cls.cpu().numpy().astype(np.int16)  # shape: (N,)
    conf = boxes.conf.cpu().numpy()  # shape: (N,)

    best_idx = {}  # {class_id: box_index}

    for i in range(len(cls)):
      cls_id = cls[i]

      prev_i = best_idx.get(cls_id)

      # Update if first occurrence or higher confidence found
      if prev_i is None or conf[i] > conf[prev_i]:
        best_idx[cls_id] = i

    # Store relative positions (one per class)
    for cls_id, i in best_idx.items():
      x1, y1, x2, y2 = xyxy[i].astype(np.int32)

      # Compute object center
      obj_center_x = (x1 + x2) / 2
      obj_center_y = (y1 + y2) / 2

      # Normalize horizontal offset to [-1, 1]
      relative_x = (obj_center_x - self.frame_center[0]) # / self.frame_center[0]
      relative_y = (obj_center_y - self.frame_center[1]) # / self.frame_center[1]

      cls_name = self.model.names[cls_id]
      score = conf[i]

      self.rel_pos[cls_id, 0] = relative_x
      self.rel_pos[cls_id, 1] = relative_y

    # if Success to detect
    self.isDetected = True
    return

  def trackBell(self):
    # Run Tracker ...
    
    # if Fail to track
    self.isDetected = False
    return

  def publish(self):
    # Publish pose
    stamp = self.get_clock().now().to_msg()

    msg = VisionMsg()
    msg.header.stamp = stamp
    msg.detected = self.isDetected
    msg.p = self.rel_pos[0]
    msg.confidence = self.confidence

    # data ...
    self.state_pub.publish(msg)

    if self.publish_image and self.img is not None:
      image_msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
      image_msg.header.stamp = stamp
      self.image_pub.publish(image_msg)

  def run(self):
    exec = SingleThreadedExecutor()
    exec.add_node(self)

    while (rclpy.ok()):
      # Read Image
      if not self.readImg():
        continue

      # Run Detection or Tracking
      if self.isDetected:
        self.trackBell()
      else:
        self.detectBell()

      # Publish msg
      self.publish()
        
    exec.shutdown()
    self.capture.release()
    self.destroy_node()
    rclpy.shutdown()


# Run simulation node
if __name__ == "__main__":
  rclpy.init()
  sim = VisionNode()
  sim.run()
