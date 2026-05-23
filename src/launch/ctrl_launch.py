from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Directory for the package
  pkg_dir = get_package_share_directory('capstone')
  
  # Path to configuration and model files
  config_path   = os.path.join(pkg_dir, 'config', 'params.yaml')
  yolo_picam    = "/home/capstonet3/ros2_ws/src/capstone/yolo_models/robot_yolo_p4_416_combine_new/weights/best.engine"
  yolo_webcam   = "/home/capstonet3/ros2_ws/src/capstone/yolo_models/robot_yolo_p4_416_combine_new_scaled_0_8/weights/best.engine"
  enable_debug  = LaunchConfiguration('enable_debug')
  publish_image = LaunchConfiguration('publish_image')
  enable_webcam = LaunchConfiguration('enable_webcam')
  enable_picam  = LaunchConfiguration('enable_picam')
  
  # HW-ROS bridge node
  bridge_node = Node(
    package     = 'capstone',
    executable  = 'bridge_node',
    name        = 'bridge_node',
    output      = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters  = [
      {'config_path':           config_path},
    ]
  )

  # Main control node
  ctrl_node = Node(
    package     = 'capstone',
    executable  = 'ctrl_node',
    name        = 'ctrl_node',
    output      = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters  = [
      {'config_path':           config_path},
      {'webcam_pause_hold_sec': 0.0},
    ],
  )

  # Webcam Vision Node - enable later when testing both cameras.
  vision_webcam_node = Node(
    package     = 'capstone',
    executable  = 'vision.py',
    name        = 'vision_webcam_node',
    output      = 'screen',
    condition   = IfCondition(enable_webcam),
    parameters  = [
      {'config_path':     config_path},
      {'yolo_model_path': yolo_webcam},
      {'camera_type':     1},
      {'vision_topic':    '/vision_webcam'},
      {'image_topic':     '/vision_webcam/image'},
      {'enabled_topic':   '/vision_webcam/enabled'},
      {'publish_image':   ParameterValue(publish_image, value_type=bool)},
    ],
  )

  # Pi Camera Vision Node
  vision_picam_node = Node(
    package     = 'capstone',
    executable  = 'vision.py',
    name        = 'vision_picam_node',
    output      = 'screen',
    condition   = IfCondition(enable_picam),
    parameters  = [
      {'config_path':     config_path},
      {'yolo_model_path': yolo_picam},
      {'camera_type':     0},
      {'vision_topic':    '/vision_picam'},
      {'image_topic':     '/vision_picam/image'},
      {'publish_image':   ParameterValue(publish_image, value_type=bool)},
    ],
  )

  # Test Node
  test_node1 = Node(
    package     = 'capstone',
    executable  = 'track_test.py',
    name        = 'track_picam_node',
    output      = 'screen',
    condition   = IfCondition(enable_debug),
    parameters  = [
      {'config_path':     config_path},
      {'vision_topic':    '/vision_picam'},
      {'image_topic':     '/vision_picam/image'},
      {'window_name':     'track_picam'},
    ],
  )

  test_node2 = Node(
    package     = 'capstone',
    executable  = 'track_test.py',
    name        = 'track_webcam_node',
    output      = 'screen',
    condition   = IfCondition(enable_debug),
    parameters  = [
      {'config_path':     config_path},
      {'vision_topic':    '/vision_webcam'},
      {'image_topic':     '/vision_webcam/image'},
      {'window_name':     'track_webcam'},
    ],
  )

  return LaunchDescription([
    DeclareLaunchArgument('enable_debug', default_value='true'),
    DeclareLaunchArgument('publish_image', default_value='true'),
    DeclareLaunchArgument('enable_webcam', default_value='false'),
    DeclareLaunchArgument('enable_picam', default_value='true'),
    bridge_node,
    ctrl_node,
    vision_webcam_node,
    vision_picam_node,
    test_node1,
    test_node2,
  ])
