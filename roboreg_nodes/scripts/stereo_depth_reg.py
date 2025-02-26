import rclpy

from roboreg_nodes.reg.stereo_depth import StereoDepth


def main():
    rclpy.init(args=None)
    reg = StereoDepth(node_name="roboreg")
    rclpy.spin(reg)
