import rclpy

from ros2_roboreg.registration import MonocularDepthHEIC


def main():
    rclpy.init(args=None)
    heic = MonocularDepthHEIC(node_name="eye_to_hand_calibration")
    heic.initialize()
    rclpy.spin(heic)
