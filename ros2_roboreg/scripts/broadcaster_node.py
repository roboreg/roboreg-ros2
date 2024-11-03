import argparse

import numpy as np
import rclpy
from rclpy.node import Node

from ros2_roboreg.broadcaster import StaticTFBroadcaster


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

    node = Node("static_tf_broadcaster")
    static_tf_broadcaster = StaticTFBroadcaster(node=node)

    node.get_logger().info(
        "Got target frames parent: {}, child: {}, target child: {}.".format(
            args.parent, args.child, args.target_child
        )
    )

    # load ht
    ht = np.load(args.ht)
    node.get_logger().info("Loaded homogeneous transform:\n{}".format(ht))

    static_tf_broadcaster.broadcast_tf(
        ht=ht,
        parent=args.parent,
        child=args.child,
        target_child=args.target_child,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
