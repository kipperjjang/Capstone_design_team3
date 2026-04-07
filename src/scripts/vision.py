#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import time
import yaml
import numpy as np

from custom_msgs.msg import VisionMsg

class VisionNode(Node):
  def __init__(self):
    super().__init__('vision_node')
    # OpenCV
    self.bridge = CvBridge()
    self.capture = cv2.VideoCapture(0) # /dev/video0
    self.isDetected = False

    # ROS Publisher
    self.state_pub = self.create_publisher(VisionMsg, '/vision', 10)

  def readImg(self):
    ret, frame = self.capture.read()
    if not ret:
      return False
    self.img = frame

  def detectBell(self):
    # Run YOLO ...
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
  rclpy.init()
  sim = VisionNode()
  sim.run()
