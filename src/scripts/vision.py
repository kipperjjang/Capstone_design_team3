#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
import threading
import time
from typing import Optional
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ultralytics import YOLO

from custom_msgs.msg import VisionMsg
from sensor_msgs.msg import Image

# from trajectory_utils import load_runtime_config

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

class WebCAMLatestFrameReader:
    def __init__(self, src=0, width=1280, height=720, fps=30):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)

        # MJPG configuration
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Camera /dev/video{self.src} could not be opened. "
                "Try src=1, src=2, or check v4l2-ctl --list-devices."
            )

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            
            if not ret:
              raise RuntimeError("Camera opened, but first frame could not be read.")
            
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()    

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None, None, None

            return self.latest_frame.copy(), self.latest_timestamp, self.frame_count
        
    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.cap is not None:
            self.cap.release()

class LatestFrameReader:
    def __init__(self, pipeline: Optional[any] = None):
        self.pipeline = pipeline
        self.cap = None
        self.running = False
        self.thread = None
        self.frame = None

        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_timestamp = 0.0
        self.frame_count = 0

    def start(self):
        if self.pipeline is not None:
          self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        else:
          self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            if self.pipeline is not None:
                raise RuntimeError("Cannot open Jetson CSI camera with GStreamer.")
            else:
                raise RuntimeError("Cannot open default camera.")

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

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is None:
                return None, None, None

            return self.latest_frame.copy(), self.latest_timestamp, self.frame_count

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
    self.declare_parameter("yolo_model_path_picam", "/home/capstonet3/ros2_ws/src/capstone/yolo_models/robot_yolo_p4_416_combine_new/weights/best.engine")
    self.declare_parameter("yolo_model_path_webcam", "/home/capstonet3/ros2_ws/src/capstone/yolo_models/robot_yolo_p4_416_combine_new_scaled_0_8/weights/best.engine")
    self.declare_parameter("publish_image", True)
    self.declare_parameter("max_motion_dt", 0.5)
    self.declare_parameter("camera_type", 0)  # "CSI" : 0 or "WebCAM" : 1
    self.declare_parameter("camera", "")
    self.declare_parameter("vision_topic", "/vision")
    self.declare_parameter("image_topic", "/vision/image")

    self.config_path = self.get_parameter("config_path").value
    # self.config = load_runtime_config(self.config_path)
    # vision_config = self.config.get("vision", {})

    self.publish_image = self.get_parameter("publish_image").value
    self.max_motion_dt = float(self.get_parameter("max_motion_dt").value)
    self.camera_type = self.get_parameter("camera_type").value
    self.camera = self.get_parameter("camera").value
    self.vision_topic = self.get_parameter("vision_topic").value
    self.image_topic = self.get_parameter("image_topic").value
    if not self.camera:
      self.camera = "picam" if self.camera_type == 0 else "webcam"
    if self.camera == "picam":
      self.yolo_model_path = self.get_parameter("yolo_model_path_picam").value
    else:
      self.yolo_model_path = self.get_parameter("yolo_model_path_webcam").value

    # self.target_class = int(vision_config.get("target_class", 1))
    # self.confidence = float(vision_config.get("confidence", 0.2))
    self.box_size = np.zeros(2, dtype=np.float32)
    
    self.bridge = CvBridge()
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
    self.img_height = None
    self.img_width = None
    self.img_center = None

    if self.camera_type == 0:
      self.reader = LatestFrameReader(
         jetson_gstreamer_pipeline(
          sensor_id=0,
          capture_width=1280,
          capture_height=720,
          display_width=1280,
          display_height=720,
          framerate=60,
          flip_method=0,
       )
      ) 
    elif self.camera_type == 1:
      self.reader = WebCAMLatestFrameReader(src=1, width=1920, height=1080, fps=30)

    self.reader.start()

    self.state_pub = self.create_publisher(VisionMsg, self.vision_topic, 10)
    self.image_pub = self.create_publisher(Image, self.image_topic, 10) if self.publish_image else None

  def read_img(self):
    if self.camera_type == 0:
      frame, frame_ts, frame_count = self.reader.get_latest_frame()
    elif self.camera_type == 1:
      ret, frame = self.reader.read()

    if frame is None:
      time.sleep(0.001)
      return False

    if self.img_height is None or self.img_width is None:
      self.img_height, self.img_width = frame.shape[:2]
      self.img_center = np.array([self.img_width, self.img_height], dtype=np.float32) / 2.0

    # anti clock 90
    # frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # updown flip
    if self.camera_type == 0:
     frame = cv2.rotate(frame, cv2.ROTATE_180)

    self.img = frame
    return True

  def detect_bell(self):
    self.is_detected = False
    self.position = np.zeros(2, dtype=np.float32)
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
      print("NN FAILED")
      return

    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
      return

    # confidences = boxes.conf.cpu().numpy()
    # self.confidence = np.max(confidences) 
    # best_index = int(np.argmax(confidences))
    x1, y1, x2, y2 = boxes.xyxy.cpu().numpy()[0]
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

    # LPF coefficient
    a_ = 0.8

    if self.prev_position is not None and self.prev_sample_time is not None:
      dt = sample_time - self.prev_sample_time
      if 0.0 < dt <= self.max_motion_dt:
        # LPF velocity
        self.velocity = a_ *self.velocity + (1-a_) * ((self.position - self.prev_position) / dt).astype(np.float32)
        self.has_velocity = True
        
        if self.prev_velocity is not None:
          # LPF acceleration
          self.acceleration = a_ * self.acceleration + (1-a_) * ((self.velocity - self.prev_velocity) / dt).astype(np.float32)
          self.has_acceleration = True
        else:
          self.acceleration = np.zeros(2, dtype=np.float32)
          self.has_acceleration = False
      else:
        # Reset
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
    msg.bbox = self.box_size.tolist()
    msg.img_center = self.img_center.tolist()
    # msg.confidence = float(self.confidence)
    msg.covariance = []
    msg.camera = self.camera
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
        self.read_img()
        self.detect_bell()
        if self.is_detected:
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
