source /opt/ros/humble/setup.bash
colcon build
source ~/dev_ws/install/setup.bash

ros2 run panda_ros2_moveit2 control_arm.py