import rclpy

from ros2_roboreg.registration import StereoDepthHEIC


def main():
    rclpy.init(args=None)
    heic = StereoDepthHEIC(node_name="eye_to_hand_calibration")
    heic.initialize()
    rclpy.spin(heic)
