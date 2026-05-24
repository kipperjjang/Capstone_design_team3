#!/usr/bin/env python3
"""Laptop-side compressed image viewer for Jetson vision debugging.

Usage
-----
1. Connect the laptop to the Jetson hotspot.

2. On the Jetson SSH terminal, run the vision node with lightweight image publish enabled:

   cd ~/ros2_ws
   export ROS_DOMAIN_ID=0
   export ROS_LOCALHOST_ONLY=0

   ros2 run capstone vision.py --ros-args \
     -p publish_image:=true \
     -p image_topic:=/vision/image/compressed \
     -p debug_image_width:=960 \
     -p debug_image_height:=540 \
     -p debug_image_fps:=10.0 \
     -p debug_jpeg_quality:=50

3. On the laptop terminal, run this viewer:

   cd ~/ros2_ws
   export ROS_DOMAIN_ID=0
   export ROS_LOCALHOST_ONLY=0

   ros2 run capstone image_debugger.py --ros-args \
     -p image_topic:=/vision/image/compressed

4. If latency is high over the hotspot, reduce bandwidth from the Jetson side:

   -p debug_image_width:=640 \
   -p debug_image_height:=360 \
   -p debug_image_fps:=5.0 \
   -p debug_jpeg_quality:=40

   # range of debug_jpeg_quality value : 0 ~ 100

Design notes
------------
- This node subscribes to sensor_msgs/CompressedImage, not raw sensor_msgs/Image.
- QoS uses BEST_EFFORT and depth=1 so old frames are dropped instead of queued.
- Press `q` or `Esc` in the OpenCV window to close the viewer.
- The viewer intentionally avoids latency overlay, FPS overlay, and repeated frame copies to minimize local display delay.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class ImageDebugger(Node):
    def __init__(self):
        super().__init__("image_debugger")

        self.declare_parameter("image_topic", "/vision/image/compressed")
        self.declare_parameter("window_name", "Vision Debug")

        self.image_topic = self.get_parameter("image_topic").value
        self.window_name = self.get_parameter("window_name").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos,
        )

        self.window_closed = False

        self.get_logger().info(f"Subscribed to compressed image topic: {self.image_topic}")
        self.get_logger().info("Press 'q' or Esc in the OpenCV window to quit.")

    def image_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            self.window_closed = True

    def spin_viewer_once(self) -> bool:
        rclpy.spin_once(self, timeout_sec=0.01)
        return not self.window_closed


def main():
    rclpy.init()
    node = ImageDebugger()

    try:
        while rclpy.ok():
            if not node.spin_viewer_once():
                break
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()