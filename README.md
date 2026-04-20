# Capstone_design_team3

## Quick Start
To clone ros2 package only, follow the commands below
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

Currently KF fixed into angle based state, therefore current estimator cannot be tested with current code.