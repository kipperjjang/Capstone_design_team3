from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # Directory for the package
  pkg_dir = get_package_share_directory('capstone')
  
  # Path to configuration and model files
  config_path       = os.path.join(pkg_dir, 'config', 'params.yaml')
  # urdf_path         = os.path.join(pkg_dir, 'models', 'urdf', 'uam_3dof.urdf')
  # xml_path          = os.path.join(pkg_dir, 'models', 'xml', 'uam_3dof.xml')

  # LaunchConfiguration
  
  # HW-ROS bridge node
  bridge_node = Node(
    package = 'capstone',
    executable = 'bridge',
    name = 'bridge_node',
    output = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters = [
      {'config_path':          config_path},
    ]
  )

  # Main control node
  ctrl_node = Node(
    package = 'capastone',
    executable = 'ctrl',
    name = 'ctrl_node',
    output = 'screen',
    # prefix = 'xterm -e gdb -ex run --args',
    parameters = [
      {'config_path':          config_path},
    ],
  )

  # Simulator Node
  vision_node = Node(
    package = 'capstone',
    executable = 'vision.py',
    name = 'vision_node',
    output = 'screen',
    parameters = [
      {'config_path':      config_path},
    ],
  )

  return LaunchDescription([bridge_node, ctrl_node, vision_node])
