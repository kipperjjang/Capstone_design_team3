# Capstone_design_team3

## Quick Start
To clone ros2 package only, follow the commands below


**Clone ROS package**
```
# Clone structure only; no data
git clone --filter=blob:none --no-checkout https://github.com/kipperjjang/Capstone_design_team3.git
cd Capstone_design_team3

# Set directories to clone and pull
git sparse-checkout init --cone
git sparse-checkout set README.md custom_msgs src
git checkout origin/main
```
If already tracks ros2 packages, run lower codes only.

Rollback to pixel based KF


**Run test code**
```
colcon build --packages-up-to capstone
source ~/ros2_ws/install/setup.bash

ros2 launch capstone test_launch.py
```
You can change `test_visualizer.py` and `test_kalman_evo.py` in `test_launc.py`.

For `test_kalman_evo.py`, it checks performance of the kalman filter for some arbitrary trajectory by checking its APE(absolute pose error) and RPE(relative pose error). We can adjust parameters for this validation code such as sampling frequency, standard deviation or RPE steps.
