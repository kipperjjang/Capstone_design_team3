#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
import threading
import time
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ultralytics import YOLO

from custom_msgs.msg import VisionMsg
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

def jetson_gstreamer_pipeline(
  sensor_id=0,
  capture_width=1280,
  capture_height=720,
  display_width=640,
  display_height=480,
  framerate=30,
  flip_method=0,
):
  return (
    f"nvarguscamerasrc sensor-id={sensor_id} ! "
    f"video/x-raw(memory:NVMM), "
    f"width=(int){capture_width}, "
    f"height=(int){capture_height}, "
    f"framerate=(fraction){framerate}/1 ! "
    f"nvvidconv flip-method={flip_method} ! "
    f"video/x-raw, "
    f"width=(int){display_width}, "
    f"height=(int){display_height}, "
    f"format=(string)BGRx ! "
    f"videoconvert ! "
    f"video/x-raw, format=(string)BGR ! "
    f"appsink drop=true max-buffers=1 sync=false"
)

# Picamera Reader
class PiCamReader:
  def __init__(self, pipeline=None):
    self.pipeline = pipeline
    self.cap = None
    self.frame = None

    self.running = False
    self.thread = None

    self.lock = threading.Lock()
    self.latest_frame = None
    self.latest_timestamp = 0.0
    self.frame_count = 0

  def start(self):
    # Open Gstreamer Camera
    self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

    # Cannot open Camera
    if not self.cap.isOpened():
      raise RuntimeError("Cannot open Jetson CSI camera with GStreamer.")

    # Run thread to save latest image
    self.running = True
    self.thread = threading.Thread(target=self._reader_loop, daemon=True)
    self.thread.start()

  def _reader_loop(self):
    while self.running:
      ret, frame = self.cap.read()

      if not ret:
        time.sleep(0.001)
        continue

      with self.lock:
        self.latest_frame = frame
        self.latest_timestamp = time.time()
        self.frame_count += 1

  def read(self):
    with self.lock:
      if self.latest_frame is None:
        return None

      return self.latest_frame.copy()

  def stop(self):
    self.running = False

    if self.thread is not None:
      self.thread.join(timeout=1.0)

    if self.cap is not None:
      self.cap.release()

# Webcam Reader
class WebCamReader:
  def __init__(self, src=1, width=1280, height=720, fps=30):
    self.src = src
    self.width = width
    self.height = height
    self.fps = fps

    self.cap = None
    self.frame = None

    self.running = False
    self.thread = None
    self.lock = threading.Lock()

  def start(self):
    # Open WebCAM
    self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)

    # MJPG configuration
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Cannot open camera
    if not self.cap.isOpened():
      raise RuntimeError(f"Camera /dev/video{self.src} could not be opened. " "Try src=1, src=2, or check v4l2-ctl --list-devices.")

    # Run thread to save latest image
    self.running = True
    self.thread = threading.Thread(target=self._reader_loop, daemon=True)
    self.thread.start()

  def _reader_loop(self):
    while self.running:
      ret, frame = self.cap.read()
      
      if not ret:
        raise RuntimeError("Camera opened, but first frame could not be read.")

      with self.lock:
        self.ret = ret
        self.frame = frame

  def read(self):
    with self.lock:
      if self.frame is None:
        return None
      return self.frame.copy()    
      
  def stop(self):
    self.running = False

    if self.thread is not None:
      self.thread.join(timeout=1.0)

    if self.cap is not None:
      self.cap.release()

class VisionNode(Node):
  def __init__(self):
    super().__init__("vision_node")

    self.declare_parameter("config_path", "")
    self.declare_parameter("yolo_path", "/home/capstonet3/ros2_ws/src/capstone/yolo_models/robot_yolo_p4_416_combine_new/weights/best.engine")
    self.declare_parameter("camera_type", 0)

    self.declare_parameter("publish_image", False)
    self.declare_parameter("vision_topic", "/vision")
    self.declare_parameter("image_topic", "/vision/image")
    self.declare_parameter("enabled_topic", "/vision_webcam/enabled")
    
    # Initialize ROS Parameters
    self.config_path      = self.get_parameter("config_path").value
    self.yolo_model_path  = self.get_parameter("yolo_path").value
    self.camera_type      = self.get_parameter("camera_type").value
    
    self.publish_image    = self.get_parameter("publish_image").value    
    self.vision_topic     = self.get_parameter("vision_topic").value
    self.image_topic      = self.get_parameter("image_topic").value
    self.enabled_topic    = self.get_parameter("enabled_topic").value
    
    # Read Confi File
    self.load_config(self.config_path)
    
    # Initialize YOLO metric variables
    self.box_size         = np.zeros(2, dtype=np.float32)
    self.bridge           = CvBridge()
    self.model            = YOLO(self.yolo_model_path)
    self.position         = np.zeros(2, dtype=np.float32)
    self.velocity         = np.zeros(2, dtype=np.float32)
    self.acceleration     = np.zeros(2, dtype=np.float32)
    self.has_velocity     = False
    self.has_acceleration = False
    self.prev_position    = None
    self.prev_velocity    = None
    self.prev_sample_time = None
    self.is_detected      = False
    self.enabled          = True

    # Image related variables
    self.img        = None
    self.img_height = None
    self.img_width  = None
    self.img_center = None

    # Init and Start Camera Reader
    if self.camera_type == 0:
      self.reader = PiCamReader(
        jetson_gstreamer_pipeline(
          sensor_id       = 0,
          capture_width   = 1280,
          capture_height  = 720,
          display_width   = 1280,
          display_height  = 720,
          framerate       = 30,
          flip_method     = 0,
       )
      )
    elif self.camera_type == 1:
      self.reader = WebCamReader(src=1, width=1920, height=1080, fps=30)
    self.reader.start()

    # ROS Publisher and Subscriber
    self.state_pub    = self.create_publisher(VisionMsg, self.vision_topic, 1)
    self.image_pub    = self.create_publisher(Image, self.image_topic, 1) if self.publish_image else None
    self.enabled_sub  = None
    if self.camera_type == 1:
      self.enabled_sub = self.create_subscription(Bool, self.enabled_topic, self.enabledCallback, 1)

  def load_config(self, path):
    with open(path, "r") as f:
      data = yaml.safe_load(f) or {}
    vision_config      = data.get("vision", data)
    self.w_conf        = vision_config["weight_confidence"]
    self.w_dist        = vision_config["weight_distance"]
    self.max_motion_dt = vision_config["max_motion_dt"]

  def enabledCallback(self, msg):
    if msg is None:
      return
    was_enabled = self.enabled
    self.enabled = bool(msg.data)
    if was_enabled and not self.enabled:
      self.is_detected = False
      self.reset_motion_estimate()

  def read_img(self):
    frame = self.reader.read()

    if frame is None:
      time.sleep(0.001)
      return False

    if self.img_height is None or self.img_width is None:
      self.img_height, self.img_width = frame.shape[:2]
      self.img_center = np.array([self.img_width, self.img_height], dtype=np.float32) / 2.0

    if self.camera_type == 0:
      frame = cv2.rotate(frame, cv2.ROTATE_180)
      # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    self.img = frame
    return True
  
  def box_center(self, box):
    x1, y1, x2, y2 = box
    return np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], dtype=np.float32)

  def choose_best_box(self, boxes):
    candidates  = []
    xyxy        = boxes.xyxy.cpu().numpy()
    confidence  = boxes.conf.cpu().numpy()

    for box, conf in zip(xyxy, confidence):
      center  = self.box_center(box)

      if self.prev_position is not None:
        distance = np.linalg.norm(center - self.prev_position)
      else:
        distance = 0
      score = self.w_conf * conf - self.w_dist * distance
      candidates.append(score)
    
    best_idx = int(np.argmax(np.array(candidates)))
    return xyxy[best_idx]

  def detect_bell(self):
    self.is_detected  = False
    self.position     = np.zeros(2, dtype=np.float32)
    self.box_size     = np.zeros(2, dtype=np.float32)
    if self.img is None:
      return

    results = self.model(
      self.img,
      imgsz=416,
      conf=0.65,
      classes=[1],
      verbose=False,
    )

    if not results:
      return

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
      return

    # Select the best bbox
    box = self.choose_best_box(boxes)
    
    x1, y1, x2, y2    = box
    self.box_size     = np.abs(np.array([x2 - x1, y2 - y1]))
    self.position     = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], dtype=np.float32)
    self.is_detected  = True

  def reset_motion_estimate(self):
    self.velocity         = np.zeros(2, dtype=np.float32)
    self.acceleration     = np.zeros(2, dtype=np.float32)
    self.has_velocity     = False
    self.has_acceleration = False
    self.prev_position    = None
    self.prev_velocity    = None
    self.prev_sample_time = None

  def update_motion_estimate(self, sample_time):
    if not self.is_detected:
      self.reset_motion_estimate()
      return

    # LPF coefficient
    a_ = 0.8

    if self.prev_position is not None and self.prev_sample_time is not None:
      dt = sample_time - self.prev_sample_time
      if 0.0 < dt <= self.max_motion_dt:
        self.velocity     = a_ *self.velocity + (1-a_) * ((self.position - self.prev_position) / dt).astype(np.float32)
        self.has_velocity = True
        
        if self.prev_velocity is not None:
          self.acceleration     = a_ * self.acceleration + (1-a_) * ((self.velocity - self.prev_velocity) / dt).astype(np.float32)
          self.has_acceleration = True
        else:
          self.acceleration     = np.zeros(2, dtype=np.float32)
          self.has_acceleration = False
      else:
        # Violate Safe time Range, Reset
        self.reset_motion_estimate()

    # Cache data
    self.prev_position = self.position.copy()
    self.prev_velocity = self.velocity.copy() if self.has_velocity else None
    self.prev_sample_time = sample_time

  def publish(self):
    now         = self.get_clock().now()
    sample_time = now.nanoseconds * 1e-9
    stamp       = now.to_msg()
    self.update_motion_estimate(sample_time)

    # VisionMsg
    msg               = VisionMsg()
    msg.header.stamp  = stamp
    msg.detected      = self.is_detected
    msg.p             = self.position.tolist()
    msg.v             = self.velocity.tolist() if self.has_velocity else []
    msg.a             = self.acceleration.tolist() if self.has_acceleration else []
    msg.bbox          = self.box_size.tolist()
    msg.img_center    = self.img_center.tolist()
    msg.camera = "webcam" if self.camera_type else "picam"
    self.state_pub.publish(msg)

  def publishImage(self):
    if self.image_pub is not None and self.img is not None:
      image_msg = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
      image_msg.header.stamp = self.get_clock().now().to_msg()
      self.image_pub.publish(image_msg)

  def run(self):
    executor = SingleThreadedExecutor()
    executor.add_node(self)

    try:
      while rclpy.ok():
        if not self.read_img():
          executor.spin_once(timeout_sec=0.0)
          continue
        if not self.enabled:
          self.publishImage()
          executor.spin_once(timeout_sec=0.0)
          continue
        self.detect_bell()
        self.publish()
        self.publishImage()
        executor.spin_once(timeout_sec=0.0)
    finally:
      executor.shutdown()
      self.reader.stop()
      self.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()


def main():
  rclpy.init()
  node = VisionNode()
  node.run()

if __name__ == "__main__":
  main()
