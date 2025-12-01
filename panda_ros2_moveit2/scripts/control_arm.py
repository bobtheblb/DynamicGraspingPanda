#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Imports for Cartesian & Gripper Control
from ros2_data.action import MoveXYZW, MoveXYZ, MoveG
from ros2_grasping.action import Attacher

from gazebo_msgs.srv import GetEntityState

class CartesianPanda(Node):
    def __init__(self):
        super().__init__('cartesian_panda_client')
        
        # Configuration
        self.xyzw_topic = '/MoveXYZW'
        self.xyz_topic = '/MoveXYZ'
        self.gripper_topic = '/MoveG'
        self.attach_topic = '/Attacher'
        
        # Clients
        self.get_logger().info('Initializing Cartesian Clients...')
        self._xyzw_client = ActionClient(self, MoveXYZW, self.xyzw_topic)
        self._xyz_client = ActionClient(self, MoveXYZ, self.xyz_topic)
        self._gripper_client = ActionClient(self, MoveG, self.gripper_topic)
        self._attacher_client = ActionClient(self, Attacher, self.attach_topic)

        self.get_logger().info('Waiting for Action Servers...')
        self._xyzw_client.wait_for_server()
        self._xyz_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info('Servers Found! Ready to move.')

        self._box_state_client = self.create_client(GetEntityState, "ros2_grasp/get_entity_state")
        while not self._box_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for the service ...")

        self._box_state_req = GetEntityState.Request()
        self._box_state_req.name = 'box'
        self._box_state_req.reference_frame = 'world'

        self._planning_upper_bound = 4.5

    def get_box_state(self):
        future = self._box_state_client.call_async(self._box_state_req)
        rclpy.spin_until_future_complete(self, future)
        state = future.result().state
        pose = state.pose.position
        twist = state.twist.linear
        return ((pose.x, pose.y, pose.z), (twist.x, twist.y, twist.z))

    def move_to_xyzw(self, x, y, z, yaw, pitch, roll, speed=1.0):
        """
        Moves the End Effector to a specific X,Y,Z coordinate (Meters)
        and Orientation (Degrees).
        """
        self.get_logger().info("Moving to XYZW!")
        
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
        send_future = self._xyzw_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if not send_future.done():
            self.get_logger().error('[XYZW] Goal Send Timed Out!')
            return False
            
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[XYZW] Goal Rejected!')
            return False
            
        self.get_logger().info('[XYZW] Goal Accepted. Moving...')
        
        # Wait for Result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=15.0)
        
        if res_future.done():
            self.get_logger().info('[XYZW] Movement Complete.')
            return True
        else:
            self.get_logger().error('[XYZW] Movement Timed Out!')
            return False

    def move_to_xyz(self, x, y, z, speed=1.0):
        """
        Moves the End Effector to a specific X,Y,Z coordinate (Meters).
        """
        self.get_logger().info("Moving to XYZ")
        
        goal_msg = MoveXYZ.Goal()
        
        # Fill fields
        goal_msg.positionx = float(x)
        goal_msg.positiony = float(y)
        goal_msg.positionz = float(z)
        goal_msg.speed = float(speed)

        # Send Goal
        send_future = self._xyz_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if not send_future.done():
            self.get_logger().error('[XYZ] Goal Send Timed Out!')
            return False
            
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[XYZ] Goal Rejected!')
            return False
            
        self.get_logger().info('[XYZ] Goal Accepted. Moving...')
        
        # Wait for Result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=15.0)
        
        if res_future.done():
            self.get_logger().info('[XYZ] Movement Complete.')
            return True
        else:
            self.get_logger().error('[XYZ] Movement Timed Out!')
            return False

    def attach(self):
        self.get_logger().info("Attaching")
        msg = Attacher.Goal()
        msg.object = 'box'
        msg.endeffector = 'panda_leftfinger'

        send_future = self._attacher_client.send_goal_async(msg)
        # rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        # if not send_future.done():
        #     self.get_logger().error('[Attach] Goal Send Timed Out!')
        #     return False
            
        # goal_handle = send_future.result()
        # if not goal_handle.accepted:
        #     self.get_logger().error('[Attach] Goal Rejected!')
        #     return False
            
        # self.get_logger().info('[Attach] Goal Accepted. Moving...')
        
        # Wait for Result
        # res_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, res_future, timeout_sec=15.0)
        
        # if res_future.done():
        #     self.get_logger().info('[Attach] Movement Complete.')
        #     return True
        # else:
        #     self.get_logger().error('[Attach] Movement Timed Out!')
        #     return False

    def grip(self, width=0.0):
        """
        Control Gripper. 
        width: Float value in meters.
               0.0  = Closed
               0.08 = Open (Max width for Panda is ~8cm)
        """
        self.get_logger().info(f'Increasing Gripper Width to {width}')
        msg = MoveG.Goal()
        msg.goal = float(width) # FIX: Send float, not string
        
        send_future = self._gripper_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if not send_future.done():
            self.get_logger().error('[Grip] Goal Send Timed Out!')
            return False
            
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('[Grip] Goal Rejected!')
            return False
            
        self.get_logger().info('[Grip] Goal Accepted. Moving...')
        
        # Wait for Result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=15.0)
        
        if res_future.done():
            self.get_logger().info('[Grip] Movement Complete.')
            return True
        else:
            self.get_logger().error('[Grip] Movement Timed Out!')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPanda()

    print("Launched node")

    try:
        while True:
            # 2. Ready Position (High up)
            # X=0.5, Z=0.6, Pitch=180 (Pointing straight down)
            node.move_to_xyzw(0.33, 0.0, 0.76 + 0.5, 45, 180, 0)

            # Wait until box is back in grasping zone again
            (x, y, z), (vx, vy, vz) = node.get_box_state()
            while y < -0.5 or z > 0.76 + 0.2 or y > -0.3 or (x == 0 and y == 0 and z == 0):
                (x, y, z), (vx, vy, vz) = node.get_box_state()

                time.sleep(0.1)

            st = time.time()

            # 1. Open Hand
            node.grip(0.04)

            # Multi-step Predictive IK
            for height in (0.3, 0.1):
                (x, y, z), (vx, vy, vz) = node.get_box_state()
                # node.get_logger().info(f"Got box state: {((x, y, z), (vx, vy, vz))}")
                predicted_x = x
                predicted_y = y + vy * (node._planning_upper_bound)
                predicted_z = z
                # node.get_logger().info(f"Predicted box pose: {(predicted_x, predicted_y, predicted_z)}")

                node.move_to_xyzw(predicted_x, predicted_y, predicted_z + height, 45, 180, 0)

            # Wait for the cube to pass
            while True:
                (x, y, z), _ = node.get_box_state()
                if abs(y - predicted_y) <= 0.01:
                    node.grip(0.025)
                    node.attach()

                    node.move_to_xyzw(0.33, 0.0, 0.76 + 0.5, 45, 180, 0)
                    break

            grasping_time = time.time() - st
            node.get_logger().info(f"Completed grasp in {grasping_time} seconds")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()