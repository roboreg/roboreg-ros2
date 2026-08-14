import rclpy

from roboreg_local.monocular_depth_node import MonocularDepthNode


def main():
    rclpy.init(args=None)
    try:
        reg = MonocularDepthNode(node_name="roboreg_monocular_local")
        rclpy.spin(reg)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
