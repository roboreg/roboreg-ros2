import argparse

import numpy as np
import rclpy
import transformations
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("static_tf_broadcaster")
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    def send_transform(self, ht: np.ndarray, parent: str, child: str):
        xyz = ht[:3, 3]
        quaternion = transformations.quaternion_from_matrix(ht)  # w x y z convention
        self.tf_static_broadcaster.sendTransform(
            TransformStamped(
                header=Header(
                    frame_id=parent,
                    stamp=self.get_clock().now().to_msg(),
                ),
                child_frame_id=child,
                transform=Transform(
                    translation=Vector3(
                        x=xyz.item(0),
                        y=xyz.item(1),
                        z=xyz.item(2),
                    ),
                    rotation=Quaternion(
                        x=quaternion[1],
                        y=quaternion[2],
                        z=quaternion[3],
                        w=quaternion[0],
                    ),
                ),
            )
        )


def main():
    rclpy.init(args=None)
    parser = argparse.ArgumentParser()
    parser.add_argument("--parent", type=str, default="zed2_left_camera_frame")
    parser.add_argument("--child", type=str, default="/left/left_base_frame")
    parser.add_argument("--ht", type=str, required=True)
    args, unkown_args = parser.parse_known_args()

    static_tf_broadcaster = StaticTFBroadcaster()

    static_tf_broadcaster.get_logger().info(
        "Got target frames parent: {}, child: {}.".format(args.parent, args.child)
    )

    # load ht
    ht = np.load(args.ht)
    static_tf_broadcaster.get_logger().info(
        "Loaded homogeneous transform:\n{}".format(ht)
    )

    static_tf_broadcaster.send_transform(
        ht=ht,
        parent=args.parent,
        child=args.child,
    )

    try:
        rclpy.spin(static_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
