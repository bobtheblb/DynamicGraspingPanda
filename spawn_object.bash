( cd ~/dev_ws && colcon build )
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash

xs=(0.23 0.33 0.43)
powers=(5 7.5 10)
trials=(1)
for x in "${xs[@]}"; do
    for power in "${powers[@]}"; do
        for trial in "${trials[@]}"; do
            echo CURRENT RUN = x: $x, power: $power, trial: $trial
            while true; do
                while true; do
                    echo "[CONVEYOR] Sending power command... $power"

                    response=$(timeout 3 ros2 service call /CONVEYORPOWER \
                        conveyorbelt_msgs/srv/ConveyorBeltControl "{power: $power}" 2>/dev/null)

                    if echo "$response" | grep -q "True"; then
                        echo "[CONVEYOR] Power command succeeded."
                        break
                    fi

                    echo "[CONVEYOR] Power command failed, retrying..."
                    sleep 0.3
                done

                ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x $x --y -0.5 --z 0.76
                sleep 25

                while true; do
                    echo "[SPAWNER] Deleting object..."

                    response=$(timeout 3 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'box'}" 2>/dev/null)

                    if echo "$response" | grep -q "success: True"; then
                        echo "[SPAWNER] Delete command succeeded."
                        break

                    # entity already gone
                    elif echo "$response" | grep -qi "does not exist"; then
                        echo "[SPAWNER] Box already gone."
                        break
                    fi

                    echo "[SPAWNER] Delete, retrying..."
                    sleep 0.3
                done

                sleep 5
            done
        done
    done
done
