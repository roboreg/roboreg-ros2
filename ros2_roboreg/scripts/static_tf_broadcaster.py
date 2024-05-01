import argparse
import time

import numpy as np
import rclpy
import rclpy.time
import transformations
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__("static_tf_broadcaster")
        self.tf_static_broadcaster_ = StaticTransformBroadcaster(self)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

    def send_transform(
        self, ht: np.ndarray, parent: str, child: str, target_child: str = ""
    ):
        tf_tc_c = None
        ht_tc_c = np.eye(4)
        if target_child != "":
            while True and rclpy.ok():
                try:
                    tf_tc_c = self.tf_buffer_.lookup_transform(
                        child, target_child, rclpy.time.Time()
                    )
                    break
                except:
                    self.get_logger().info(
                        f"Waiting for transform from {target_child} to {child}."
                    )
                rclpy.spin_once(self)
                time.sleep(1.0)
        else:
            target_child = child

        if tf_tc_c is not None:
            ht_tc_c = transformations.quaternion_matrix(
                [
                    tf_tc_c.transform.rotation.w,
                    tf_tc_c.transform.rotation.x,
                    tf_tc_c.transform.rotation.y,
                    tf_tc_c.transform.rotation.z,
                ]
            )
            ht_tc_c[:3, 3] = [
                tf_tc_c.transform.translation.x,
                tf_tc_c.transform.translation.y,
                tf_tc_c.transform.translation.z,
            ]
            ht = np.dot(ht, ht_tc_c)

        xyz = ht[:3, 3]
        quaternion = transformations.quaternion_from_matrix(ht)  # w x y z convention
        self.tf_static_broadcaster_.sendTransform(
            TransformStamped(
                header=Header(
                    frame_id=parent,
                    stamp=self.get_clock().now().to_msg(),
                ),
                child_frame_id=target_child,
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
    parser.add_argument(
        "--parent",
        type=str,
        default="base_frame",
        help="Parent frame for published transform.",
    )
    parser.add_argument(
        "--child",
        type=str,
        default="camera_frame",
        help="Child frame for published transform.",
    )
    parser.add_argument(
        "--target_child",
        type=str,
        default="",
        help="Specify another target child than child, e.g. camera link.",
    )
    parser.add_argument(
        "--ht",
        type=str,
        required=True,
        help="Path to homogeneous transform. Expects a numpy file, i.e. *.npy.",
    )
    args, unkown_args = parser.parse_known_args()

    static_tf_broadcaster = StaticTFBroadcaster()

    static_tf_broadcaster.get_logger().info(
        "Got target frames parent: {}, child: {}, target child: {}.".format(
            args.parent, args.child, args.target_child
        )
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
        target_child=args.target_child,
    )

    try:
        rclpy.spin(static_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
