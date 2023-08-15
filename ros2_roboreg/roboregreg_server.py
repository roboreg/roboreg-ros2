from typing import List, Tuple

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_srvs.srv import Trigger


class RoboregServer:
    SyncedDataType = Tuple[Image, JointState, PointCloud2]

    def __init__(self, node: Node) -> None:
        self._node: Node = node

        # data collection
        self._synced_data: self.SyncedDataType = (None, None, None)
        self._synced_data_list: List[self.SyncedDataType] = []

        # parameters
        self._delcare_parameters()
        self._get_parameters()

        # subscriptions
        self._create_subscriptions()

        # services
        self._create_services()

    def _delcare_parameters(self) -> None:
        if not self._node.has_parameter("sync_accuracy"):
            self._node.declare_parameter("sync_accuracy", 0.1)

    def _get_parameters(self) -> None:
        self._sync_accuracy = float(self._node.get_parameter("sync_accuracy").value)

    def _create_services(self) -> None:
        self.collect_service = self._node.create_service(
            Trigger, "collect", self._on_collect
        )
        self.register_service = self._node.create_service(
            Trigger, "register", self._on_register
        )

    def _create_subscriptions(self) -> None:
        self._image_sub = Subscriber(self._node, Image, "/image/rect")
        self._joint_state_sub = Subscriber(self._node, JointState, "/joint_states")
        self._point_cloud_sub = Subscriber(
            self._node, PointCloud2, "/point_cloud/registered"
        )

        self._approximate_time_sync = ApproximateTimeSynchronizer(
            [self._image_sub, self._joint_state_sub, self._point_cloud_sub],
            queue_size=1,
            slop=self._sync_accuracy,
        )
        self._approximate_time_sync.registerCallback(self._on_sync)

    def _on_sync(self, image: Image, joint_state: JointState, point_cloud: PointCloud2):
        self._synced_data = (image, joint_state, point_cloud)

    def _on_collect(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if (
            self._synced_data[0] is None
            or self._synced_data[1] is None
            or self._synced_data[1] is None
        ):
            response.success = False
            response.message = f"No data available yet. Maybe data not in sync. Synchronization accuracy: {self._sync_accuracy}."
            self.get_logger().warn(response.message)
            return response
        self._synced_data_list.append(self._synced_data)
        response.success = True
        response.message = (
            f"Added data with time stamp: {self._synced_data[0].header.stamp}"
        )
        self.get_logger().info(response.message)
        return response

    def _on_register(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if len(self._synced_data_list) == 0:
            response.success = False
            response.message = "No data collected yet."
            return response
