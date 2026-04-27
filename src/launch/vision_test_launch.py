from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Directory for the package
  pkg_dir = get_package_share_directory('capstone')
  
  # Path to configuration and model files
  config_path       = os.path.join(pkg_dir, 'config', 'params.yaml')

  # LaunchConfiguration

  # Vision Node
  vision_node = Node(
    package = 'capstone',
    executable = 'vision.py',
    name = 'vision_node',
    output = 'screen',
    parameters = [
      {'config_path':      config_path},
    ],
  )

  # Main control node
  test_node = Node(
    package = 'capstone',
    executable = 'test_node',
    name = 'test_node',
    output = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters = [
      {'config_path':          config_path},
    ],
  )
  
  # Test Node
  python_node = Node(
    package = 'capstone',
    executable = 'vision_test.py',
    name = 'python_node',
    output = 'screen',
  )

  # return LaunchDescription([test_node, python_node])
  return LaunchDescription([test_node, vision_node, python_node])
