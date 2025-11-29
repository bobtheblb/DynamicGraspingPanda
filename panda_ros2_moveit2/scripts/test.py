import rclpy
from rclpy.node import Node
import moveit_commander

def main():
    rclpy.init()
    robot = moveit_commander.RobotCommaner()

if __name__ == "__main__":
    main()