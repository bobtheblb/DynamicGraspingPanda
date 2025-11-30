source /opt/ros/humble/setup.bash
colcon build
source ~/dev_ws/install/setup.bash

ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box'}"
sleep 5
while true; do
    timeout 5 ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 5.0}"
    timeout 5 ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.55 --y -0.5 --z 0.76
    sleep 15
    timeout 5 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box'}"
    sleep 5
done