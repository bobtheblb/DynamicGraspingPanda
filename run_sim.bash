source /opt/ros/humble/setup.bash
colcon build
source ~/dev_ws/install/setup.bash

echo "2" | ros2 launch panda_ros2_moveit2 panda_interface.launch.py