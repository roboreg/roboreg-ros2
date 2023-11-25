import copy
import os
import pathlib
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import cv_bridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_srvs.srv import Trigger

from ros2_roboreg_msgs.srv import CollectData, SaveSyncedData


@dataclass
class SyncedData:
    left_image: Image
    right_image: Image
    left_camera_info: CameraInfo
    right_camera_info: CameraInfo
    joint_state: JointState
    point_cloud: PointCloud2

    def __init__(self) -> None:
        self.left_image = None
        self.right_image = None
        self.left_camera_info = None
        self.right_camera_info = None
        self.joint_state = None
        self.point_cloud = None

    def clear(self) -> None:
        self.left_image = None
        self.right_image = None
        self.left_camera_info = None
        self.right_camera_info = None
        self.joint_state = None
        self.point_cloud = None


class RoboregServer(Node):
    def __init__(self, node_name: str = "roboreg") -> None:
        super().__init__(node_name)

        # data collection
        self._synced_data = SyncedData()
        self._synced_data_list: List[SyncedData] = []

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
        if not self.has_parameter("min_joint_position_change"):
            self.declare_parameter("min_joint_position_change", 0.001)
        if not self.has_parameter("max_joint_velocity"):
            self.declare_parameter("max_joint_velocity", 0.01)
        if not self.has_parameter("left_image_topic"):
            self.declare_parameter("left_image_topic", "/left/image_rect_color")
        if not self.has_parameter("right_image_topic"):
            self.declare_parameter("right_image_topic", "/right/image_rect_color")
        if not self.has_parameter("left_camera_info_topic"):
            self.declare_parameter("left_camera_info_topic", "/left/camera_info")
        if not self.has_parameter("right_camera_info_topic"):
            self.declare_parameter("right_camera_info_topic", "/right/camera_info")
        if not self.has_parameter("joint_states_topic"):
            self.declare_parameter("joint_states_topic", "/joint_states")
        if not self.has_parameter("point_cloud_topic"):
            self.declare_parameter("point_cloud_topic", "/point_cloud/cloud_registered")

    def _get_parameters(self) -> None:
        self._sync_accuracy = (
            self.get_parameter("sync_accuracy").get_parameter_value().double_value
        )
        self._max_joint_velocity = (
            self.get_parameter("max_joint_velocity").get_parameter_value().double_value
        )
        self._min_joint_position_change = (
            self.get_parameter("min_joint_position_change")
            .get_parameter_value()
            .double_value
        )
        self._left_image_topic = (
            self.get_parameter("left_image_topic").get_parameter_value().string_value
        )
        if "left" not in self._left_image_topic:
            self.get_logger().warn(
                f"Left image topic does not contain 'left' but '{self._left_image_topic}'."
            )
        self._right_image_topic = (
            self.get_parameter("right_image_topic").get_parameter_value().string_value
        )
        if "right" not in self._right_image_topic:
            self.get_logger().warn(
                f"Right image topic does not contain 'right' but '{self._right_image_topic}'."
            )
        self._left_camera_info_topic = (
            self.get_parameter("left_camera_info_topic")
            .get_parameter_value()
            .string_value
        )
        if "left" not in self._left_camera_info_topic:
            self.get_logger().warn(
                f"Left camera info topic does not contain 'left' but '{self._left_camera_info_topic}'."
            )
        self._right_camera_info_topic = (
            self.get_parameter("right_camera_info_topic")
            .get_parameter_value()
            .string_value
        )
        if "right" not in self._right_camera_info_topic:
            self.get_logger().warn(
                f"Right camera info topic does not contain 'right' but '{self._right_camera_info_topic}'."
            )
        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._point_cloud_topic = (
            self.get_parameter("point_cloud_topic").get_parameter_value().string_value
        )

    def _create_services(self) -> None:
        # callback group
        callback_group = MutuallyExclusiveCallbackGroup()

        self.collect_service = self.create_service(
            CollectData,
            "~/collect_data",
            self._on_collect,
            callback_group=callback_group,
        )
        self.register_service = self.create_service(
            Trigger, "~/register", self._on_register
        )
        self.save_synced_data_service = self.create_service(
            SaveSyncedData,
            "~/save_synced_data",
            self._on_save_synced_data,
            callback_group=callback_group,
        )

    def _create_subscriptions(self) -> None:
        self._left_image_sub = Subscriber(self, Image, self._left_image_topic)
        self._right_image_sub = Subscriber(self, Image, self._right_image_topic)
        self._left_camera_info_sub = Subscriber(
            self, CameraInfo, self._left_camera_info_topic
        )
        self._right_camera_info_sub = Subscriber(
            self, CameraInfo, self._right_camera_info_topic
        )
        self._joint_state_sub = Subscriber(self, JointState, self._joint_states_topic)
        self._point_cloud_sub = Subscriber(self, PointCloud2, self._point_cloud_topic)

        self._approximate_time_sync = ApproximateTimeSynchronizer(
            [
                self._left_image_sub,
                self._right_image_sub,
                self._left_camera_info_sub,
                self._right_camera_info_sub,
                self._joint_state_sub,
                self._point_cloud_sub,
            ],
            queue_size=1,
            slop=self._sync_accuracy,
        )
        self._approximate_time_sync.registerCallback(self._on_sync)

    def _on_sync(
        self,
        left_image: Image,
        right_image: Image,
        left_camera_info: CameraInfo,
        right_camera_info: CameraInfo,
        joint_state: JointState,
        point_cloud: PointCloud2,
    ):
        self._synced_data.left_image = left_image
        self._synced_data.right_image = right_image
        self._synced_data.left_camera_info = left_camera_info
        self._synced_data.right_camera_info = right_camera_info
        self._synced_data.joint_state = joint_state
        self._synced_data.point_cloud = point_cloud

    def _on_collect(
        self, request: CollectData.Request, response: CollectData.Response
    ) -> CollectData.Response:
        if (
            self._synced_data.right_image is None
            or self._synced_data.left_image is None
            or self._synced_data.left_camera_info is None
            or self._synced_data.right_camera_info is None
            or self._synced_data.joint_state is None
            or self._synced_data.point_cloud is None
        ):
            response.success = False
            response.n_collected = len(self._synced_data_list)
            response.message = f"No data available yet. Data might not be synchronized. Synchronization accuracy: {self._sync_accuracy}s."
            self.get_logger().warn(response.message)
            return response

        # check if joint states changed from last data
        if len(self._synced_data_list) > 1:
            if np.isclose(
                self._synced_data_list[-1].joint_state.position,
                self._synced_data.joint_state.position,
                atol=self._min_joint_position_change,
            ).all():
                response.success = False
                response.n_collected = len(self._synced_data_list)
                response.message = f"Joint states did not change. Minimum joint position change: {self._min_joint_position_change} rad. Skipping data collection."
                self.get_logger().warn(response.message)
                return response

        # only allow joint states velocities close to zero
        if not np.isclose(
            self._synced_data.joint_state.velocity,
            np.zeros_like(self._synced_data.joint_state.velocity),
            atol=self._max_joint_velocity,
        ).all():
            response.success = False
            response.n_collected = len(self._synced_data_list)
            response.message = f"Joint states velocity greater zero. Maximum joint velocity: {self._max_joint_velocity} rad/s. This may cause un-correlated data. Skipping data collection."
            self.get_logger().warn(response.message)
            return response

        # add data
        self._synced_data_list.append(copy.deepcopy(self._synced_data))
        response.success = True
        response.n_collected = len(self._synced_data_list)
        response.message = (
            f"Added data with time stamp: {self._synced_data.joint_state.header.stamp}"
        )
        self.get_logger().info(response.message)
        self._synced_data.clear()
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
        self.get_logger().info(f"Saving data to {path.absolute()}.")

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

            def write_camera_info_to_yaml(camera_info_msg: CameraInfo, path: str):
                import yaml

                camera_info = {
                    "width": camera_info_msg.width,
                    "height": camera_info_msg.height,
                    "frame_id": camera_info_msg.header.frame_id,
                    "camera_matrix": {
                        "rows": 3,
                        "cols": 3,
                        "data": camera_info_msg.k.tolist(),
                    },
                    "distortion_model": camera_info_msg.distortion_model,
                    "distortion_coefficients": {
                        "rows": 1,
                        "cols": 5,
                        "data": camera_info_msg.d.tolist(),
                    },
                    "rectification_matrix": {
                        "rows": 3,
                        "cols": 3,
                        "data": camera_info_msg.r.tolist(),
                    },
                    "projection_matrix": {
                        "rows": 3,
                        "cols": 4,
                        "data": camera_info_msg.p.tolist(),
                    },
                }

                with open(path, "w") as f:
                    yaml.dump(camera_info, f)

            # save camera info
            write_camera_info_to_yaml(
                self._synced_data_list[0].left_camera_info,
                os.path.join(path, "left_camera_info.yaml"),
            )
            write_camera_info_to_yaml(
                self._synced_data_list[0].right_camera_info,
                os.path.join(path, "right_camera_info.yaml"),
            )

            # log time stamps to csv
            with open(os.path.join(path, "time_stamps.csv"), "w") as f:
                f.write("idx,sec,nanosec\n")

                for idx, synced_data in enumerate(self._synced_data_list):
                    # log time stamps
                    f.write(
                        f"{idx},{synced_data.joint_state.header.stamp.sec},{synced_data.joint_state.header.stamp.nanosec}\n"
                    )

                    # convert to numpy
                    left_img_np = bridge.imgmsg_to_cv2(
                        synced_data.left_image, desired_encoding="passthrough"
                    )
                    right_img_np = bridge.imgmsg_to_cv2(
                        synced_data.right_image, desired_encoding="passthrough"
                    )
                    joint_position = synced_data.joint_state.position
                    name = synced_data.joint_state.name
                    joint_position = [x for _, x in sorted(zip(name, joint_position))]
                    joint_position_np = np.array(joint_position)
                    xyz_np, rgba_np = point_cloud_to_numpy(synced_data.point_cloud)

                    # save
                    cv2.imwrite(
                        os.path.join(path, f"left_img_{idx}.png"),
                        left_img_np,
                    )
                    cv2.imwrite(
                        os.path.join(path, f"right_img_{idx}.png"),
                        right_img_np,
                    )
                    np.save(
                        os.path.join(path, f"joint_state_{idx}.npy"),
                        joint_position_np,
                    )
                    np.save(
                        os.path.join(path, f"xyz_{idx}.npy"),
                        xyz_np,
                    )
                    np.save(
                        os.path.join(path, f"rgba_{idx}.npy"),
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
