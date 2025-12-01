( cd ~/dev_ws && colcon build )
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

xs=("a" "b" "c")
powers=("1" "2" "3")

while true; do
    while true; do
        echo "[CONVEYOR] Sending power command..."

        response=$(timeout 3 ros2 service call /CONVEYORPOWER \
            conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 10}" 2>/dev/null)

        if echo "$response" | grep -q "True"; then
            echo "[CONVEYOR] Power command succeeded."
            break
        fi

        echo "[CONVEYOR] Power command failed, retrying..."
        sleep 0.3
    done
    ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.33 --y -0.5 --z 0.76
    sleep 25

    while true; do
        echo "[SPAWNER] Deleting object..."

        response=$(timeout 3ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box'}" 2>/dev/null)

        if echo "$response" | grep -q "True"; then
            echo "[SPAWNER] Delete command succeeded."
            break
        fi

        echo "[SPAWNER] Delete, retrying..."
        sleep 0.3
    done

    sleep 5
done