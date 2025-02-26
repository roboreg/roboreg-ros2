import rclpy

from roboreg_nodes.reg.monocular_depth import MonocularDepth


def main():
    rclpy.init(args=None)
    reg = MonocularDepth(node_name="roboreg")
    rclpy.spin(reg)
