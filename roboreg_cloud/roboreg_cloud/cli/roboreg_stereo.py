import rclpy

from roboreg_cloud.stereo_depth_node import StereoDepthNode


def main():
    rclpy.init(args=None)
    try:
        reg = StereoDepthNode(node_name="roboreg_stereo_cloud")
        rclpy.spin(reg)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
