import rclpy
from rclpy.node import Node

from ros2_meshreg import MeshregServer


def main() -> None:
    rclpy.init()
    node = Node("meshreg_server")
    server = MeshregServer(node)
    rclpy.spin(node)
    rclpy.shutdown()
