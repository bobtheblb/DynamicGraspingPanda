#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Imports for Cartesian & Gripper Control
from ros2_data.action import MoveXYZW, MoveG

from gazebo_msgs.srv import GetEntityState

class CartesianPanda(Node):
    def __init__(self):
        super().__init__('cartesian_panda_client')
        
        # Configuration
        self.xyz_topic = '/MoveXYZW'
        self.gripper_topic = '/MoveG'
        
        # Clients
        self.get_logger().info('Initializing Cartesian Clients...')
        self._xyz_client = ActionClient(self, MoveXYZW, self.xyz_topic)
        self._gripper_client = ActionClient(self, MoveG, self.gripper_topic)
        
        self.get_logger().info('Waiting for Action Servers...')
        self._xyz_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('Servers Found! Ready to move.')

        self._box_state_client = self.create_client(GetEntityState, "ros2_grasp/get_entity_state")
        while not self._box_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for the service ...")

        self._box_state_req = GetEntityState.Request()
        self._box_state_req.name = 'box'
        self._box_state_req.reference_frame = 'world'

    def get_box_state(self):
        future = self._box_state_client.call_async(self._box_state_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result().state.pose.position
        return (res.x, res.y, res.z)

    def move_to_pose(self, x, y, z, yaw, pitch, roll, speed=1.0):
        """
        Moves the End Effector to a specific X,Y,Z coordinate (Meters)
        and Orientation (Degrees).
        """
        self.get_logger().info(f'\n>>> MOVING TO: X={x:.2f}, Y={y:.2f}, Z={z:.2f} | Pitch={pitch:.1f}')
        
        goal_msg = MoveXYZW.Goal()
        
        # Fill fields
        goal_msg.positionx = float(x)
        goal_msg.positiony = float(y)
        goal_msg.positionz = float(z)
        goal_msg.yaw = float(yaw)
        goal_msg.pitch = float(pitch)
        goal_msg.roll = float(roll)
        goal_msg.speed = float(speed)

        # Send Goal
        send_future = self._xyz_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if not send_future.done():
            self.get_logger().error('Goal Send Timed Out!')
            return False
            
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal Rejected!')
            return False
            
        self.get_logger().info('Goal Accepted. Moving...')
        
        # Wait for Result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=15.0)
        
        if res_future.done():
            self.get_logger().info('Movement Complete.')
            return True
        else:
            self.get_logger().error('Movement Timed Out!')
            return False

    def grip(self, width=0.0):
        """
        Control Gripper. 
        width: Float value in meters.
               0.0  = Closed
               0.08 = Open (Max width for Panda is ~8cm)
        """
        self.get_logger().info(f'Gripper Width: {width}')
        msg = MoveG.Goal()
        msg.goal = float(width) # FIX: Send float, not string
        
        self._gripper_client.send_goal_async(msg)
        time.sleep(1.0) # Wait for gripper to actuate

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPanda()

    try:
        (goal_x, goal_y, goal_z) = node.get_box_state()

        # 1. Open Hand (0.08m)
        node.grip(0.08)

        # 2. Ready Position (High up)
        # X=0.5, Z=0.6, Pitch=180 (Pointing straight down)
        node.move_to_pose(0.106, 0.0, 1.7, -90, -45, -90)
        time.sleep(0.5)

        node.move_to_pose(goal_x, goal_y, goal_z + 0.1, 0, 180, 0)
        time.sleep(0.5)

        # 3. Grasp Position (Low, near conveyor)
        # Z=0.25 is typically a safe grasp height for objects on the belt
        # node.move_to_pose(0.5, 0.0, 0.25, 0.0, 180.0, 0.0)
        # time.sleep(0.5)

        # 4. Close Hand (0.0m)
        # node.grip(0.0)

        # 5. Lift Object
        # node.move_to_pose(0.5, 0.0, 0.6, 0.0, 180.0, 0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()