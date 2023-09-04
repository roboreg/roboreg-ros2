import rclpy

from ros2_roboreg import RoboregServer


def main() -> None:
    rclpy.init()
    server = RoboregServer("roboreg")
    rclpy.spin(server)
    rclpy.shutdown()
