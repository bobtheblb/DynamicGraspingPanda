#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState

class PandaControl(Node):
    def __init__(self):
        super().__init__("panda_control_node")

        self.client = self.create_client(GetEntityState, "ros2_grasp/get_entity_state")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for the service ...")

        self.req = GetEntityState.Request()
        self.req.name = 'box'
        self.req.reference_frame = 'world'

    def get_object_state(self):
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    panda_controller = PandaControl()
    while rclpy.ok():
        response = panda_controller.get_object_state()
        panda_controller.get_logger().info(f"Box XYZ: {response.state.pose.position}")

    panda_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()