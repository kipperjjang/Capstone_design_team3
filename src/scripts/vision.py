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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ultralytics import YOLO

from custom_msgs.msg import VisionMsg
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

# from trajectory_utils import load_runtime_config

def csicam_gstreamer_pipeline(
    sensor_id=0,
    width=1280,
    height=720,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int){width}, "
        f"height=(int){height}, "
        f"framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )

def webcam_gstreamer_pipeline(
    device="/dev/video0",
    width=1280,
    height=720,
    framerate=30,
):
    # TODO: Check whether HW-accelerated MJPEG decode is available on the target Jetson.
    # Commands for checking HW decoder
    ## Option 1
    # $ gst-inspect-1.0 nvv4l2decoder
    # $ gst-inspect-1.0 nvv4l2decoder | grep -i mjpeg
    # $ gst-inspect-1.0 nvv4l2decoder | less
    ## Option 2
    # $ GST_DEBUG=2 gst-launch-1.0 \
    # v4l2src device=/dev/video1 io-mode=2 ! \
    # image/jpeg,width=1280,height=720,framerate=30/1 ! \
    # nvv4l2decoder mjpeg=1 ! \
    # fakesink

    # If supported by the installed JetPack/GStreamer plugins, replace the CPU jpegdec path below with:
    #   v4l2src device={device} io-mode=2 !
    #   image/jpeg,width=(int){width},height=(int){height},framerate=(fraction){framerate}/1 !
    #   nvv4l2decoder mjpeg=1 !
    #   nvvidconv !
    #   video/x-raw, format=(string)BGRx !
    #   videoconvert !
    #   video/x-raw, format=(string)BGR !
    #   appsink drop=true max-buffers=1 sync=false
    # This can reduce CPU load and latency, but should be verified with gst-inspect-1.0 nvv4l2decoder
    # and a standalone gst-launch-1.0 test before replacing the current stable jpegdec pipeline.
    return (
        f"v4l2src device={device} io-mode=2 ! "
        f"image/jpeg, "
        f"width=(int){width}, "
        f"height=(int){height}, "
        f"framerate=(fraction){framerate}/1 ! "
        f"jpegdec ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )

# class WebCAMLatestFrameReader:
#     def __init__(self, src=0, width=1280, height=720, fps=30):
#         self.src = src
#         self.width = width
#         self.height = height
#         self.fps = fps
#
#         self.cap = None
#         self.frame = None
#         self.lock = threading.Lock()
#         self.running = False
#         self.thread = None
#
#     def start(self):
#         self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)
#
#         # MJPG configuration
#         self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
#         self.cap.set(cv2.CAP_PROP_FPS, self.fps)
#         self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
#
#         if not self.cap.isOpened():
#             raise RuntimeError(
#                 f"Camera /dev/video{self.src} could not be opened. "
#                 "Try src=1, src=2, or check v4l2-ctl --list-devices."
#             )
#
#         self.running = True
#         self.thread = threading.Thread(target=self._reader_loop, daemon=True)
#         self.thread.start()
#
#     def _reader_loop(self):
#         while self.running:
#             ret, frame = self.cap.read()
#
#             if not ret:
#               raise RuntimeError("Camera opened, but first frame could not be read.")
#
#             if ret:
#                 with self.lock:
#                     self.ret = ret
#                     self.frame = frame
#
#     def read(self):
#         with self.lock:
#             if self.frame is None:
#                 return False, None
#             return self.ret, self.frame.copy()
#
#     def get_latest_frame(self):
#         with self.lock:
#             if self.latest_frame is None:
#                 return None, None, None
#
#             return self.latest_frame.copy(), self.latest_timestamp, self.frame_count
#
#     def stop(self):
#         self.running = False
#
#         if self.thread is not None:
#             self.thread.join(timeout=1.0)
#
#         if self.cap is not None:
#             self.cap.release()

class LatestFrameReader:
  def __init__(self, pipeline=None):
    self.pipeline = pipeline
    self.cap = None
    self.running = False
    self.thread = None

    self.lock = threading.Lock()
    self.latest_frame = None

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

  def get_latest_frame(self):
      with self.lock:
          return self.latest_frame
      
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
    self.declare_parameter("yolo_model_path", "/home/capstonet3/ros2_ws/src/capstone/yolo_models/robot_yolo_p4_416_combine_new/weights/best.engine")
    self.declare_parameter("camera_type", 0)

    self.declare_parameter("publish_image", False)
    self.declare_parameter("vision_topic", "/vision")
    self.declare_parameter("image_topic", "/vision/image/compressed")
    self.declare_parameter("enabled_topic", "/vision_webcam/enabled")

    self.declare_parameter("profile_timing", False)
    self.declare_parameter("profile_report_interval", 30)
    self.declare_parameter("debug_image_width", 960)
    self.declare_parameter("debug_image_height", 540)
    self.declare_parameter("debug_image_fps", 10.0)
    self.declare_parameter("debug_jpeg_quality", 50) # range of the value : 0 ~ 100
    self.declare_parameter("debug_draw_text", False)
    
    # Initialize ROS Parameters
    self.config_path      = self.get_parameter("config_path").value
    self.yolo_model_path  = self.get_parameter("yolo_model_path").value
    self.camera_type      = self.get_parameter("camera_type").value
    
    self.publish_image    = self.get_parameter("publish_image").value    
    self.vision_topic     = self.get_parameter("vision_topic").value
    self.image_topic      = self.get_parameter("image_topic").value
    self.enabled_topic    = self.get_parameter("enabled_topic").value
    self.profile_timing   = bool(self.get_parameter("profile_timing").value)
    self.profile_report_interval = max(1, int(self.get_parameter("profile_report_interval").value))

    self.debug_image_width = int(self.get_parameter("debug_image_width").value)
    self.debug_image_height = int(self.get_parameter("debug_image_height").value)
    self.debug_image_fps = float(self.get_parameter("debug_image_fps").value)
    self.debug_jpeg_quality = int(self.get_parameter("debug_jpeg_quality").value)
    self.debug_draw_text = bool(self.get_parameter("debug_draw_text").value)
    self.debug_image_period = 1.0 / max(self.debug_image_fps, 1e-6)
    self.last_debug_image_time = 0.0
    
    # Read Confi File
    self.load_config(self.config_path)
    
    # Initialize YOLO metric variables
    self.box_size         = np.zeros(2, dtype=np.float32)
    self.bridge           = CvBridge()
    self.model            = YOLO(self.yolo_model_path, task="detect")
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

    if self.camera_type == 0:
      self.reader = LatestFrameReader(
         csicam_gstreamer_pipeline(
          sensor_id=0,
          width=1280,
          height=720,
          framerate=60,
          flip_method=0,
       )
      )
    elif self.camera_type == 1:
      self.reader = LatestFrameReader(
        webcam_gstreamer_pipeline(
          device="/dev/video1",
          width=1280,
          height=720,
          framerate=30,
        )
      )
    self.reader.start()

    self.state_pub = self.create_publisher(VisionMsg, self.vision_topic, 1)
    self.image_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    self.image_pub = self.create_publisher(CompressedImage, self.image_topic, self.image_qos) if self.publish_image else None
    self.enabled_sub = None
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
    frame = self.reader.get_latest_frame()

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

    # confidences = boxes.conf.cpu().numpy()
    # self.confidence = np.max(confidences)
    # best_index = int(np.argmax(confidences))
    x1, y1, x2, y2 = box
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
    """Publish an inference result image for remote tuning with minimum delay.

    This method is intentionally called after detect_bell(), so the published image
    already contains the latest model result. To reduce latency, the frame is first
    resized to the debug resolution and then the bbox is drawn on the smaller image.
    """
    if self.image_pub is None or self.img is None:
      return

    now_sec = time.time()
    if now_sec - self.last_debug_image_time < self.debug_image_period:
      return
    self.last_debug_image_time = now_sec

    src_h, src_w = self.img.shape[:2]

    if self.debug_image_width > 0 and self.debug_image_height > 0:
      out_w = self.debug_image_width
      out_h = self.debug_image_height
      debug_img = cv2.resize(self.img, (out_w, out_h), interpolation=cv2.INTER_AREA)
      scale_x = out_w / float(src_w)
      scale_y = out_h / float(src_h)
    else:
      debug_img = self.img
      out_h, out_w = debug_img.shape[:2]
      scale_x = 1.0
      scale_y = 1.0

    if self.is_detected and len(self.position) >= 2 and len(self.box_size) >= 2:
      cx = int(self.position[0] * scale_x)
      cy = int(self.position[1] * scale_y)
      bw = int(self.box_size[0] * scale_x)
      bh = int(self.box_size[1] * scale_y)
      x1 = max(0, int(cx - bw * 0.5))
      y1 = max(0, int(cy - bh * 0.5))
      x2 = min(out_w - 1, int(cx + bw * 0.5))
      y2 = min(out_h - 1, int(cy + bh * 0.5))

      cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
      cv2.circle(debug_img, (cx, cy), 4, (0, 0, 255), -1)

      if self.debug_draw_text:
        cv2.putText(
            debug_img,
            f"bell ({cx}, {cy})",
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    if self.img_center is not None:
      center_x = int(self.img_center[0] * scale_x)
      center_y = int(self.img_center[1] * scale_y)
      cv2.line(debug_img, (center_x, 0), (center_x, out_h - 1), (255, 0, 0), 1)
      cv2.line(debug_img, (0, center_y), (out_w - 1, center_y), (255, 0, 0), 1)

    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.debug_jpeg_quality)]
    success, encoded_img = cv2.imencode(".jpg", debug_img, encode_params)
    if not success:
      return

    image_msg = CompressedImage()
    image_msg.header.stamp = self.get_clock().now().to_msg()
    image_msg.format = "jpeg"
    image_msg.data = encoded_img.tobytes()
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
