import rclpy

from ros2_roboreg.registration import StereoDepthE2HC


def main():
    rclpy.init(args=None)
    e2hc = StereoDepthE2HC(node_name="eye_to_hand_calibration")
    e2hc.initialize()
    rclpy.spin(e2hc)
