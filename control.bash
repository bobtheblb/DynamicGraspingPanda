( cd ~/dev_ws && colcon build )
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

ros2 run panda_ros2_moveit2 control_arm.py