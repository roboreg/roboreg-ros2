import os
import pathlib
from typing import List, Tuple

import cv2
import cv_bridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_srvs.srv import Trigger

from ros2_roboreg_msgs.srv import SaveSyncedData


class RoboregServer(Node):
    SyncedDataType = Tuple[Image, JointState, PointCloud2]

    def __init__(self, node_name: str = "roboreg") -> None:
        super().__init__(node_name)

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
        if not self.has_parameter("sync_accuracy"):
            self.declare_parameter("sync_accuracy", 0.01)

    def _get_parameters(self) -> None:
        self._sync_accuracy = float(self.get_parameter("sync_accuracy").value)

    def _create_services(self) -> None:
        self.collect_service = self.create_service(
            Trigger, "~/collect", self._on_collect
        )
        self.register_service = self.create_service(
            Trigger, "~/register", self._on_register
        )
        self.save_synced_data_service = self.create_service(
            SaveSyncedData, "~/save_synced_data", self._on_save_synced_data
        )

    def _create_subscriptions(self) -> None:
        self._image_sub = Subscriber(self, Image, "/image/rect")
        self._joint_state_sub = Subscriber(self, JointState, "/joint_states")
        self._point_cloud_sub = Subscriber(self, PointCloud2, "/point_cloud/registered")

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

    def _on_save_synced_data(
        self, request: SaveSyncedData.Request, response: SaveSyncedData.Response
    ) -> SaveSyncedData.Response:
        if len(self._synced_data_list) == 0:
            response.success = False
            response.message = "No data collected yet."
            return response

        path = pathlib.Path(request.path)

        def write_synced_data():
            bridge = cv_bridge.CvBridge()

            def point_cloud_to_numpy(
                point_cloud: PointCloud2,
            ) -> Tuple[np.ndarray, np.ndarray]:
                self.get_logger().info("Converting point cloud to numpy.")
                self.get_logger().info(
                    f"Point cloud shape: {point_cloud.height} x {point_cloud.width}"
                )
                data = np.array(point_cloud.data, dtype=np.uint8)

                # offset + byte, step size = 4 * 4 bytes
                x_b0, x_b1, x_b2, x_b3 = (
                    data[point_cloud.fields[0].offset + 0 :: point_cloud.point_step],
                    data[point_cloud.fields[0].offset + 1 :: point_cloud.point_step],
                    data[point_cloud.fields[0].offset + 2 :: point_cloud.point_step],
                    data[point_cloud.fields[0].offset + 3 :: point_cloud.point_step],
                )
                y_b0, y_b1, y_b2, y_b3 = (
                    data[point_cloud.fields[1].offset + 0 :: point_cloud.point_step],
                    data[point_cloud.fields[1].offset + 1 :: point_cloud.point_step],
                    data[point_cloud.fields[1].offset + 2 :: point_cloud.point_step],
                    data[point_cloud.fields[1].offset + 3 :: point_cloud.point_step],
                )
                z_b0, z_b1, z_b2, z_b3 = (
                    data[point_cloud.fields[2].offset + 0 :: point_cloud.point_step],
                    data[point_cloud.fields[2].offset + 1 :: point_cloud.point_step],
                    data[point_cloud.fields[2].offset + 2 :: point_cloud.point_step],
                    data[point_cloud.fields[2].offset + 3 :: point_cloud.point_step],
                )
                rgb_b0, rgb_b1, rgb_b2, rgb_b3 = (
                    data[point_cloud.fields[3].offset + 0 :: point_cloud.point_step],
                    data[point_cloud.fields[3].offset + 1 :: point_cloud.point_step],
                    data[point_cloud.fields[3].offset + 2 :: point_cloud.point_step],
                    data[point_cloud.fields[3].offset + 3 :: point_cloud.point_step],
                )

                x = np.stack([x_b0, x_b1, x_b2, x_b3], axis=1)
                y = np.stack([y_b0, y_b1, y_b2, y_b3], axis=1)
                z = np.stack([z_b0, z_b1, z_b2, z_b3], axis=1)
                rgba = np.stack([rgb_b0, rgb_b1, rgb_b2, rgb_b3], axis=1)

                height, width = point_cloud.height, point_cloud.width

                x = x.flatten().view(dtype=np.float32).reshape((height, width))
                y = y.flatten().view(dtype=np.float32).reshape((height, width))
                z = z.flatten().view(dtype=np.float32).reshape((height, width))
                rgba = rgba.reshape((height, width, 4))

                return np.stack([x, y, z], axis=-1), rgba

            for img, joint_state, point_cloud in self._synced_data_list:
                # convert to numpy
                img_np = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
                joint_position = joint_state.position
                name = joint_state.name
                joint_position = [x for _, x in sorted(zip(name, joint_position))]
                joint_position_np = np.array(joint_position)
                xyz_np, rgba_np = point_cloud_to_numpy(point_cloud)

                # save
                stamp = (
                    f"{joint_state.header.stamp.sec}_{joint_state.header.stamp.nanosec}"
                )
                cv2.imwrite(
                    os.path.join(path, f"img_{stamp}.png"),
                    img_np,
                )
                np.save(
                    os.path.join(path, f"joint_state_{stamp}.npy"),
                    joint_position_np,
                )
                np.save(
                    os.path.join(path, f"xyz_{stamp}.npy"),
                    xyz_np,
                )
                np.save(
                    os.path.join(path, f"rgba_{stamp}.npy"),
                    rgba_np,
                )
            self._synced_data_list.clear()

        if path.exists():
            try:
                write_synced_data()
            except Exception as e:
                response.success = False
                response.message = f"Could not write data to {path.absolute()}."
                self.get_logger().error(response.message)
                self.get_logger().error(e)
                return response
            response.success = True
            response.message = f"Wrote data to {path.absolute()}."
            return response

        if request.mkdir:
            path.mkdir(parents=True, exist_ok=True)
            try:
                write_synced_data()
            except Exception as e:
                response.success = False
                response.message = f"Could not write data to {path.absolute()}."
                self.get_logger().error(response.message)
                self.get_logger().error(e)
                return response
            response.success = True
            response.message = (
                f"Created directory {path.absolute()} and wrote data to it."
            )
            return response

        response.success = False
        response.message = (
            f"Path {path.absolute()} does not exist and was not created as per request."
        )
        return response
