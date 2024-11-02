import rclpy

from ros2_roboreg.reg.monocular_depth import MonocularDepth


def main():
    rclpy.init(args=None)
    reg = MonocularDepth(node_name="roboreg")
    rclpy.spin(reg)
