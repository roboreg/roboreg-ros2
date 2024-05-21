import rclpy

from ros2_roboreg.server import RoboregServer


def main() -> None:
    rclpy.init()
    server = RoboregServer("roboreg")
    rclpy.spin(server)
    rclpy.shutdown()
