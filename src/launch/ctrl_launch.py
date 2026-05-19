from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
  # Directory for the package
  pkg_dir = get_package_share_directory('capstone')
  
  # Path to configuration and model files
  config_path       = os.path.join(pkg_dir, 'config', 'params.yaml')
  
  # HW-ROS bridge node
  bridge_node = Node(
    package = 'capstone',
    executable = 'bridge_node',
    name = 'bridge_node',
    output = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters = [
      {'config_path':          config_path},
    ]
  )

  # Main control node
  ctrl_node = Node(
    package = 'capstone',
    executable = 'ctrl_node',
    name = 'ctrl_node',
    output = 'screen',
    prefix = 'xterm -e gdb -ex run --args',
    parameters = [
      {'config_path':          config_path},
    ],
  )

  # Webcam Vision Node - enable later when testing both cameras.
  # vision_webcam_node = Node(
  #   package = 'capstone',
  #   executable = 'vision.py',
  #   name = 'vision_webcam_node',
  #   output = 'screen',
  #   parameters = [
  #     {'config_path':      config_path},
  #     {'camera':           'webcam'},
  #     {'camera_type':      1},
  #     {'vision_topic':     '/vision_webcam'},
  #     {'image_topic':      '/vision_webcam/image'},
  #   ],
  # )

  # Pi Camera Vision Node
  vision_picam_node = Node(
    package = 'capstone',
    executable = 'vision.py',
    name = 'vision_picam_node',
    output = 'screen',
    parameters = [
      {'config_path':      config_path},
      {'camera':           'picam'},
      {'camera_type':      0},
      {'vision_topic':     '/vision_picam'},
      {'image_topic':      '/vision_picam/image'},
    ],
  )

  # Test Node
  test_node = Node(
    package = 'capstone',
    executable = 'track_test.py',
    name = 'test_node',
    output = 'screen',
    parameters = [
      {'config_path':      config_path},
      {'vision_topic':     '/vision_picam'},
      {'image_topic':      '/vision_picam/image'},
    ],
  )

  # return LaunchDescription([bridge_node, ctrl_node, vision_webcam_node, vision_picam_node, test_node])
  return LaunchDescription([bridge_node, ctrl_node, vision_picam_node, test_node])
