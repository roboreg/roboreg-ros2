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


class StaticTFBroadcaster:
    def __init__(self, node: Node):
        self._node = node
        self._tf_static_broadcaster = StaticTransformBroadcaster(self._node)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

    def broadcast_tf(
        self, ht: np.ndarray, parent: str, child: str, target_child: str = ""
    ):
        tf_tc_c = None
        ht_tc_c = np.eye(4)
        if target_child != "":
            while True and rclpy.ok():
                try:
                    tf_tc_c = self._tf_buffer.lookup_transform(
                        child, target_child, rclpy.time.Time()
                    )
                    break
                except:
                    self._node.get_logger().info(
                        f"Waiting for transform from {target_child} to {child}."
                    )
                rclpy.spin_once(self._node)
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
        self._tf_static_broadcaster.sendTransform(
            TransformStamped(
                header=Header(
                    frame_id=parent,
                    stamp=self._node.get_clock().now().to_msg(),
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
