import rclpy
from rclpy.node import Node

from ros2_roboreg import RoboregServer


def main() -> None:
    rclpy.init()
    node = Node("roboreg_server")
    server = RoboregServer(node)
    rclpy.spin(node)
    rclpy.shutdown()
