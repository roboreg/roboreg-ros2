import rclpy

from ros2_roboreg.reg.monocular_depth import MonocularDepth


def main():
    rclpy.init(args=None)
    reg = MonocularDepth(node_name="eye_to_hand_calibration")
    reg.initialize()
    rclpy.spin(reg)
