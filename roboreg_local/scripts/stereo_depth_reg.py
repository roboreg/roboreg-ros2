import rclpy

from roboreg_local.stereo_depth import StereoDepth


def main():
    rclpy.init(args=None)
    try:
        reg = StereoDepth(node_name="roboreg")
        rclpy.spin(reg)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
