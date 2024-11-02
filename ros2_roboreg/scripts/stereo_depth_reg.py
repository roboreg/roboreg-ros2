import rclpy

from ros2_roboreg.reg.stereo_depth import StereoDepth


def main():
    rclpy.init(args=None)
    reg = StereoDepth(node_name="eye_to_hand_calibration")
    reg.initialize()
    rclpy.spin(reg)
