
source /opt/ros/humble/setup.bash

sudo apt install -y libnanopb-dev libgrpc++-dev ros-humble-ros2-control ros-humble-ros2-controller

colcon build

source install/setup.bash

ros2 launch iiqka_moveit_palletizing moveit_palletizing.launch.py

