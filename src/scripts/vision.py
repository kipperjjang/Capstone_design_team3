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
    self.declare_parameter("profile_timing", False)
    self.declare_parameter("profile_report_interval", 30)
    
    # Initialize ROS Parameters
    self.config_path      = self.get_parameter("config_path").value
    self.yolo_model_path  = self.get_parameter("yolo_path").value
    self.camera_type      = self.get_parameter("camera_type").value
    
    self.publish_image    = self.get_parameter("publish_image").value    
    self.vision_topic     = self.get_parameter("vision_topic").value
    self.image_topic      = self.get_parameter("image_topic").value
    self.enabled_topic    = self.get_parameter("enabled_topic").value
    self.profile_timing   = bool(self.get_parameter("profile_timing").value)
    self.profile_report_interval = max(1, int(self.get_parameter("profile_report_interval").value))
    
    # Read Confi File
    self.load_config(self.config_path)
    
    # Initialize YOLO metric variables
    self.box_size         = np.zeros(2, dtype=np.float32)
    self.bridge           = CvBridge()
    self.model            = YOLO(self.yolo_model_path)
    self.position         = np.zeros(2, dtype=np.float32)
    self.prev_position    = None
    self.prev_sample_time = None
    self.is_detected      = False
    self.enabled          = True

    # Image related variables
    self.img        = None
    self.img_height = None
    self.img_width  = None
    self.img_center = None

    # Timing profiler state.
    self.profile_count = 0
    self.profile_frame_count = 0
    self.profile_enabled_count = 0
    self.profile_detected_count = 0
    self.profile_sums = {
      "read": 0.0,
      "detect": 0.0,
      "publish": 0.0,
      "image_publish": 0.0,
      "spin": 0.0,
      "total": 0.0,
    }

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
          flip_method     = 2,
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

  def enabledCallback(self, msg):
    if msg is None:
      return
    was_enabled = self.enabled
    self.enabled = bool(msg.data)
    if was_enabled and not self.enabled:
      self.is_detected = False

  def read_img(self):
    frame = self.reader.read()

    if frame is None:
      time.sleep(0.001)
      return False

    if self.img_height is None or self.img_width is None:
      self.img_height, self.img_width = frame.shape[:2]
      self.img_center = np.array([self.img_width, self.img_height], dtype=np.float32) / 2.0

    self.img = frame
    return True
  
  def box_center(self, box):
    x1, y1, x2, y2 = box
    return np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0], dtype=np.float32)

  def choose_best_box(self, boxes):
    xyxy        = boxes.xyxy.cpu().numpy()
    confidence  = boxes.conf.cpu().numpy()

    best_score = -float("inf")
    best_idx = 0
    for idx, (box, conf) in enumerate(zip(xyxy, confidence)):
      center  = self.box_center(box)

      # Compute distance from previous detection
      if self.prev_position is not None:
        distance = np.linalg.norm(center - self.prev_position)
      else:
        distance = 0.0
      score = self.w_conf * conf - self.w_dist * distance
      
      # Check score
      if score > best_score:
        best_score = score
        best_idx = idx
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

  def publish(self):
    now         = self.get_clock().now()
    sample_time = now.nanoseconds * 1e-9
    stamp       = now.to_msg()

    # VisionMsg
    msg               = VisionMsg()
    msg.header.stamp  = stamp
    msg.detected      = self.is_detected
    msg.p             = self.position.tolist()
    msg.bbox          = self.box_size.tolist()
    msg.img_center    = self.img_center.tolist()
    msg.camera = "webcam" if self.camera_type else "picam"
    self.state_pub.publish(msg)

  def publishImage(self):
    if self.image_pub is not None and self.img is not None:
      image_msg = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
      image_msg.header.stamp = self.get_clock().now().to_msg()
      self.image_pub.publish(image_msg)

  def timingRow(self, label, value, total):
    ms = value * 1000.0
    pct = 100.0 * value / total if total > 1e-9 else 0.0
    return f"  {label:<13} {ms:7.2f} ms  {pct:5.1f}%"

  def updateTimingProfile(self, timings, frame_read=False, enabled=False, detected=False):
    if not self.profile_timing:
      return

    self.profile_count += 1
    self.profile_frame_count += 1 if frame_read else 0
    self.profile_enabled_count += 1 if enabled else 0
    self.profile_detected_count += 1 if detected else 0
    for key, value in timings.items():
      self.profile_sums[key] += value

    if self.profile_count < self.profile_report_interval:
      return

    avg = {
      key: value / self.profile_count
      for key, value in self.profile_sums.items()
    }
    fps = 1.0 / avg["total"] if avg["total"] > 1e-9 else 0.0
    camera = "webcam" if self.camera_type else "picam"
    rows = "\n".join([
      self.timingRow("total", avg["total"], avg["total"]),
      self.timingRow("read", avg["read"], avg["total"]),
      self.timingRow("detect", avg["detect"], avg["total"]),
      self.timingRow("publish", avg["publish"], avg["total"]),
      self.timingRow("image_pub", avg["image_publish"], avg["total"]),
      self.timingRow("spin", avg["spin"], avg["total"]),
    ])
    self.get_logger().info(
      "\n"
      f"vision timing [{camera}] avg over {self.profile_count} cycles\n"
      f"  fps={fps:.1f}  "
      f"frames={self.profile_frame_count}/{self.profile_count}  "
      f"enabled={self.profile_enabled_count}/{self.profile_count}  "
      f"detected={self.profile_detected_count}/{self.profile_count}\n"
      f"{rows}"
    )

    self.profile_count = 0
    self.profile_frame_count = 0
    self.profile_enabled_count = 0
    self.profile_detected_count = 0
    for key in self.profile_sums:
      self.profile_sums[key] = 0.0

  def run(self):
    executor = SingleThreadedExecutor()
    executor.add_node(self)

    try:
      while rclpy.ok():
        t0 = time.perf_counter()
        if not self.read_img():
          executor.spin_once(timeout_sec=0.0)
          continue

        t1 = time.perf_counter()
        if not self.enabled:
          self.publishImage()
          executor.spin_once(timeout_sec=0.0)
          continue

        self.detect_bell()
        t2 = time.perf_counter()
        self.publish()
        t3 = time.perf_counter()
        self.publishImage()
        t4 = time.perf_counter()
        executor.spin_once(timeout_sec=0.0)
        t5 = time.perf_counter()
        self.updateTimingProfile({
          "read": t1 - t0,
          "detect": t2 - t1,
          "publish": t3 - t2,
          "image_publish": t4 - t3,
          "spin": t5 - t4,
          "total": t5 - t0,
        }, frame_read=True, enabled=True, detected=self.is_detected)
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
