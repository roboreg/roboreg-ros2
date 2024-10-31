import rclpy

from ros2_roboreg.rr_monocular_depth import RoboregMonocularDepth


def main() -> None:
    rclpy.init()
    server = RoboregMonocularDepth("roboreg_monocular_depth")
    rclpy.spin(server)
    rclpy.shutdown()
