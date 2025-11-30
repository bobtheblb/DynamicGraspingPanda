#!/bin/bash

# Trap Ctrl+C and EXIT to clean up everything
trap "echo 'Stopping simulation...'; kill 0; exit" SIGINT EXIT

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

# Build workspace (optional)
colcon build
source ~/dev_ws/install/setup.bash

# Launch panda_interface in background
# Use setsid to ensure it's in its own process group
echo "2" | setsid ros2 launch panda_ros2_moveit2 panda_interface.launch.py > panda.log 2>&1 &
SIM_PID=$!
echo "Panda interface launched with PID $SIM_PID"

# Wait a few seconds for launch to initialize
sleep 5

# Infinite loop
while true; do
    # Spawn object
    ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.55 --y -0.5 --z 0.76

    # Turn on conveyor
    ros2 service call /CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 5.0}"

    # Wait a bit
    sleep 10

    # Delete object
    ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box'}"

    # Wait before next iteration
    sleep 5
done