#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import time
import yaml
import numpy as np

from custom_msgs.msg import VisionMsg

class VisionNode(Node):
  def __init__(self, yolo_model_path):
    super().__init__('vision_node')
    # OpenCV
    self.bridge = CvBridge()
    self.capture = cv2.VideoCapture(0) # /dev/video0
    self.isDetected = False

    # YOLO
    self.model = YOLO(yolo_model_path)
    # np.array for relative positions of detected object with respect to the center of image
    self.rel_pos = np.empty((2,2))

    # ROS Publisher
    self.state_pub = self.create_publisher(VisionMsg, '/vision', 10)

  def readImg(self):
    ret, frame = self.capture.read()
    if not ret:
      return False

    height, width = frame.shape[:2]
    self.frame_center = np.array([width // 2, height // 2])

    self.img = frame

  def detectBell(self):
    # Classes : pillar [0] / bell [1]
    TARGET_CLASSES = (1) # TODO : config로 confidence score 및 고려할 class 설정할 수 있게 변경.

    # Inference only for the classes in TARGET_CLASSES
    results = model(frame, conf=0.2, classes=list(TARGET_CLASSES), verbose=False)
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

      cls_name = model.names[cls_id]
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
    msg = VisionMsg()
    # data ...
    self.state_pub.publish(msg)

  def run(self):
    exec = SingleThreadedExecutor()
    exec.add_node(self)

    while (rclpy.ok()):
      # Read Image
      self.readImg()

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
  yolo_model_path = "yolo_models/robot_yolo_p4_416_combine/weights/best.pt"

  rclpy.init()
  sim = VisionNode(yolo_model_path=yolo_model_path)
  sim.run()
