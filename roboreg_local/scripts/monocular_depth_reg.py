import rclpy

from roboreg_local.monocular_depth import MonocularDepth


def main():
    rclpy.init(args=None)
    try:
        reg = MonocularDepth(node_name="roboreg")
        rclpy.spin(reg)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
