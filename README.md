# Capstone_design_team3

## Quick Start
After clone this directory, on the ros2 workspace
```
colcon build --packages-up-to capstone

source ~/ros2_ws/install/setup.bash

ros2 launch capstone test_launch.py
```
Only test estimator performance.
Can change the frequency and other parameters in ```test_visualizer.py``` or ```config/params.yaml```