import rclpy

from roboreg_cloud.monocular_depth_node import MonocularDepthNode


def main():
    rclpy.init(args=None)
    try:
        reg = MonocularDepthNode(node_name="roboreg_monocular_cloud")
        rclpy.spin(reg)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
