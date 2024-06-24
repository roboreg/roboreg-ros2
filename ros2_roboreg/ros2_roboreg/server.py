import copy
import os
import pathlib
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import cv_bridge
import numpy as np
import torch
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_system_default
from roboreg.detector import OpenCVDetector
from roboreg.hydra_icp import hydra_centroid_alignment, hydra_robust_icp
from roboreg.o3d_robot import O3DRobot
from roboreg.segmentor import SamSegmentor
from roboreg.util import clean_xyz, mask_boundary
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_roboreg.broadcaster import StaticTFBroadcaster
from ros2_roboreg_idl.srv import CollectSample, Export, Import


@dataclass
class SyncedSample:
    image: Image
    camera_info: CameraInfo
    joint_state: JointState
    point_cloud: PointCloud2

    def __init__(self) -> None:
        self.image = None
        self.camera_info = None
        self.joint_state = None
        self.point_cloud = None

    def clear(self) -> None:
        self.image = None
        self.camera_info = None
        self.joint_state = None
        self.point_cloud = None


@dataclass
class ServerParams:
    class _Filters:
        sync_accuracy: float
        min_joint_position_change: float
        max_joint_velocity: float

        def __init__(self) -> None:
            self.sync_accuracy = 0.01
            self.min_joint_position_change = 0.001
            self.max_joint_velocity = 0.01

    @dataclass
    class _TopicParam:
        name: str
        qos_reliability: str

        def __init__(self) -> None:
            self.name = ""
            self.qos_reliability = ""

    @dataclass
    class _SegmentationParams:
        buffer_size: int
        model_type: str
        device: str
        sam_checkpoint_path: str

        def __init__(self) -> None:
            self.buffer_size = 5
            self.model_type = "vit_h"
            self.device = "cuda"
            self.sam_checkpoint_path = ""

    @dataclass
    class _RegistrationParams:
        erosion_kernel_size: int
        convex_hull: bool
        number_of_points: int
        device: str
        max_distance: float
        outer_max_iter: int
        inner_max_iter: int
        rmse_change: float

        def __init__(self) -> None:
            self.erosion_kernel_size = 10
            self.convex_hull = False
            self.number_of_points = 5000
            self.device = "cuda"
            self.max_distance = 0.1
            self.outer_max_iter = 100
            self.inner_max_iter = 3
            self.rmse_change = 1.0e-6
            self.sam_checkpoint_path = ""

    def __init__(self) -> None:
        self.filters = self._Filters()
        self.camera_info_topic = self._TopicParam()
        self.image_topic = self._TopicParam()
        self.joint_state_topic = self._TopicParam()
        self.point_cloud_topic = self._TopicParam()
        self.robot_description_topic = self._TopicParam()
        self.segmentation = self._SegmentationParams()
        self.registration = self._RegistrationParams()
        self.tf_broadcaster = self.TFParams()

    @dataclass
    class TFParams:
        parent_frame: str
        child_frame: str
        target_child_frame: str

        def __init__(self) -> None:
            self.parent_frame = "world"
            self.child_frame = ""
            self.target_child_frame = ""


class RoboregServer(Node):
    def __init__(self, node_name: str = "roboreg") -> None:
        super().__init__(node_name)

        # data collection
        self._synced_sample = SyncedSample()
        self._synced_samples: List[SyncedSample] = []

        # opencv bridge
        self._bridge = cv_bridge.CvBridge()

        # tf broadcaster
        self._HT = np.eye(4)
        self._tf_broadcaster = StaticTFBroadcaster(self)

        # parameters
        self._params = ServerParams()
        self._declare_parameters()
        self._get_parameters()
        self._log_parameters()

        # parameter callback
        self.add_on_set_parameters_callback(self._on_set_parameter)

        # subscriptions
        self._camera_info_sub = None
        self._image_sub = None
        self._joint_state_sub = None
        self._point_cloud_sub = None
        self._approximate_time_sync = None
        self._create_subscriptions()

        # services
        self._create_services()

    def _declare_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("filters.sync_accuracy", 0.01),
                ("filters.min_joint_position_change", 0.001),
                ("filters.max_joint_velocity", 0.01),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.image.name", "left/image_rect_color"),
                ("topics.image.qos_reliability", "RELIABLE"),
                ("topics.camera_info.name", "left/camera_info"),
                ("topics.camera_info.qos_reliability", "RELIABLE"),
                ("topics.joint_state.name", "joint_state"),
                ("topics.joint_state.qos_reliability", "RELIABLE"),
                ("topics.point_cloud.name", "point_cloud/cloud_registered"),
                ("topics.point_cloud.qos_reliability", "RELIABLE"),
                ("topics.robot_description.name", "robot_description"),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("segmentation.buffer_size", 5),
                ("segmentation.model_type", "vit_h"),
                ("segmentation.device", "cuda"),
                ("segmentation.sam_checkpoint_path", ""),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("registration.erosion_kernel_size", 10),
                ("registration.convex_hull", False),
                ("registration.number_of_points", 5000),
                ("registration.device", "cuda"),
                ("registration.max_distance", 0.1),
                ("registration.outer_max_iter", 100),
                ("registration.inner_max_iter", 3),
                ("registration.rmse_change", 1.0e-6),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("tf_broadcaster.parent_frame", "world"),
                ("tf_broadcaster.child_frame", ""),
                ("tf_broadcaster.target_child_frame", ""),
            ],
        )

    def _get_parameters(self) -> None:
        # filter parameters
        self._params.filters.sync_accuracy = (
            self.get_parameter("filters.sync_accuracy")
            .get_parameter_value()
            .double_value
        )
        self._params.filters.max_joint_velocity = (
            self.get_parameter("filters.max_joint_velocity")
            .get_parameter_value()
            .double_value
        )

        self._params.filters.min_joint_position_change = (
            self.get_parameter("filters.min_joint_position_change")
            .get_parameter_value()
            .double_value
        )

        # topic parameters
        self._params.image_topic.name = (
            self.get_parameter("topics.image.name").get_parameter_value().string_value
        )
        self._params.image_topic.qos_reliability = (
            self.get_parameter("topics.image.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.camera_info_topic.name = (
            self.get_parameter("topics.camera_info.name")
            .get_parameter_value()
            .string_value
        )
        self._params.camera_info_topic.qos_reliability = (
            self.get_parameter("topics.camera_info.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.joint_state_topic.name = (
            self.get_parameter("topics.joint_state.name")
            .get_parameter_value()
            .string_value
        )
        self._params.joint_state_topic.qos_reliability = (
            self.get_parameter("topics.joint_state.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.point_cloud_topic.name = (
            self.get_parameter("topics.point_cloud.name")
            .get_parameter_value()
            .string_value
        )

        self._params.point_cloud_topic.qos_reliability = (
            self.get_parameter("topics.point_cloud.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.robot_description_topic.name = (
            self.get_parameter("topics.robot_description.name")
            .get_parameter_value()
            .string_value
        )

        # segmentation parameters
        self._params.segmentation.buffer_size = (
            self.get_parameter("segmentation.buffer_size")
            .get_parameter_value()
            .integer_value
        )
        self._params.segmentation.model_type = (
            self.get_parameter("segmentation.model_type")
            .get_parameter_value()
            .string_value
        )
        self._params.segmentation.device = (
            self.get_parameter("segmentation.device").get_parameter_value().string_value
        )
        self._params.segmentation.sam_checkpoint_path = (
            self.get_parameter("segmentation.sam_checkpoint_path")
            .get_parameter_value()
            .string_value
        )
        if not os.path.exists(self._params.segmentation.sam_checkpoint_path):
            self.get_logger().error(
                f"Path to SAM checkpoint does not exist: {self._params.segmentation.sam_checkpoint_path}"
            )

        # registration parameters
        self._params.registration.erosion_kernel_size = (
            self.get_parameter("registration.erosion_kernel_size")
            .get_parameter_value()
            .integer_value
        )
        self._params.registration.convex_hull = (
            self.get_parameter("registration.convex_hull")
            .get_parameter_value()
            .bool_value
        )
        self._params.registration.number_of_points = (
            self.get_parameter("registration.number_of_points")
            .get_parameter_value()
            .integer_value
        )
        self._params.registration.device = (
            self.get_parameter("registration.device").get_parameter_value().string_value
        )
        self._params.registration.max_distance = (
            self.get_parameter("registration.max_distance")
            .get_parameter_value()
            .double_value
        )
        self._params.registration.outer_max_iter = (
            self.get_parameter("registration.outer_max_iter")
            .get_parameter_value()
            .integer_value
        )
        self._params.registration.inner_max_iter = (
            self.get_parameter("registration.inner_max_iter")
            .get_parameter_value()
            .integer_value
        )
        self._params.registration.rmse_change = (
            self.get_parameter("registration.rmse_change")
            .get_parameter_value()
            .double_value
        )
        self._params.tf_broadcaster.parent_frame = (
            self.get_parameter("tf_broadcaster.parent_frame")
            .get_parameter_value()
            .string_value
        )
        self._params.tf_broadcaster.child_frame = (
            self.get_parameter("tf_broadcaster.child_frame")
            .get_parameter_value()
            .string_value
        )
        self._params.tf_broadcaster.target_child_frame = (
            self.get_parameter("tf_broadcaster.target_child_frame")
            .get_parameter_value()
            .string_value
        )

    def _log_parameters(self) -> None:
        self.get_logger().info(f"*** Parameters:")
        self.get_logger().info(f"*{' '*5}Filters:")
        self.get_logger().info(
            f"*{' '*7}Sync accuracy: {self._params.filters.sync_accuracy} s"
        )
        self.get_logger().info(
            f"*{' '*7}Max joint velocity: {self._params.filters.max_joint_velocity} rad/s"
        )
        self.get_logger().info(
            f"*{' '*7}Min joint position change: {self._params.filters.min_joint_position_change} rad"
        )
        self.get_logger().info(f"*{' '*5}Topics:")
        self.get_logger().info(f"*{' '*7}Image:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.image_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.image_topic.qos_reliability}."
        )
        self.get_logger().info(f"*{' '*7}Camera info:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.camera_info_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.camera_info_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Joint states:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.joint_state_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.joint_state_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Point cloud:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.point_cloud_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.point_cloud_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Robot description:")
        self.get_logger().info(
            f"*{' '*9}Name: {self._params.robot_description_topic.name}"
        )
        self.get_logger().info(f"*{' '*5}Segmentation:")
        self.get_logger().info(
            f"*{' '*7}Buffer size: {self._params.segmentation.buffer_size}"
        )
        self.get_logger().info(
            f"*{' '*7}Model type: {self._params.segmentation.model_type}"
        )
        self.get_logger().info(f"*{' '*7}Device: {self._params.segmentation.device}")
        self.get_logger().info(
            f"*{' '*7}SAM checkpoint path: {self._params.segmentation.sam_checkpoint_path}"
        )
        self.get_logger().info(f"*{' '*5}Registration:")
        self.get_logger().info(
            f"*{' '*7}Erosion kernel size: {self._params.registration.erosion_kernel_size}"
        )
        self.get_logger().info(
            f"*{' '*7}Convex hull: {self._params.registration.convex_hull}"
        )
        self.get_logger().info(
            f"*{' '*7}Number of points: {self._params.registration.number_of_points}"
        )
        self.get_logger().info(f"*{' '*7}Device: {self._params.registration.device}")
        self.get_logger().info(
            f"*{' '*7}Max distance: {self._params.registration.max_distance}"
        )
        self.get_logger().info(
            f"*{' '*7}Outer max iterations: {self._params.registration.outer_max_iter}"
        )
        self.get_logger().info(
            f"*{' '*7}Inner max iterations: {self._params.registration.inner_max_iter}"
        )
        self.get_logger().info(
            f"*{' '*7}RMSE change: {self._params.registration.rmse_change}"
        )
        self.get_logger().info(f"*{' '*5}TF broadcaster:")
        self.get_logger().info(
            f"*{' '*7}Parent frame: {self._params.tf_broadcaster.parent_frame}"
        )
        self.get_logger().info(
            f"*{' '*7}Child frame: {self._params.tf_broadcaster.child_frame}"
        )
        self.get_logger().info(
            f"*{' '*7}Target child frame: {self._params.tf_broadcaster.target_child_frame}"
        )
        self.get_logger().info("***")

    def _on_set_parameter(self, parameters: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            if parameter.name == "topics.camera_info.name":
                self.get_logger().info(
                    f"Setting camera info topic to {parameter.value}"
                )
                self._params.camera_info_topic.name = parameter.value
                self._create_camera_info_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.image.name":
                self.get_logger().info(f"Setting image topic to {parameter.value}")
                self._params.image_topic.name = parameter.value
                self._create_image_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.joint_state.name":
                self.get_logger().info(
                    f"Setting joint state topic to {parameter.value}"
                )
                self._params.joint_state_topic.name = parameter.value
                self._create_joint_state_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.point_cloud.name":
                self.get_logger().info(
                    f"Setting point cloud topic to {parameter.value}"
                )
                self._params.point_cloud_topic.name = parameter.value
                self._create_point_cloud_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.robot_description.name":
                self.get_logger().info(
                    f"Setting robot description topic to {parameter.value}"
                )
                self._params.robot_description_topic.name = parameter.value
                self._create_robot_description_subscription()
            else:
                continue
        return result

    def _create_services(self) -> None:
        # callback group
        callback_group = MutuallyExclusiveCallbackGroup()

        self._collect_sample_service = self.create_service(
            CollectSample,
            "~/collect_sample",
            self._on_collect_sample,
            callback_group=callback_group,
        )
        self._clear_samples_service = self.create_service(
            Trigger, "~/clear_samples", self._on_clear_samples
        )
        self._register_service = self.create_service(
            Trigger, "~/register", self._on_register
        )
        self._export_samples_service = self.create_service(
            Export,
            "~/export/samples",
            self._on_export_samples,
            callback_group=callback_group,
        )
        self._export_transform_service = self.create_service(
            Export,
            "~/export/transform",
            self._on_export_transform,
            callback_group=callback_group,
        )
        self._import_transform_service = self.create_service(
            Import,
            "~/import/transform",
            self._on_import_transform,
            callback_group=callback_group,
        )
        self._transform_service = self.create_service(
            Trigger,
            "~/broadcast_transform",
            self._on_broadcast_tf,
            callback_group=callback_group,
        )

    def _create_camera_info_subscription(self) -> None:
        if self._camera_info_sub is not None:
            self.destroy_subscription(self._camera_info_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.camera_info_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._camera_info_sub = Subscriber(
            self,
            CameraInfo,
            self._params.camera_info_topic.name,
            qos_profile=qos_profile,
        )

    def _create_image_subscription(self) -> None:
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.image_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._image_sub = Subscriber(
            self,
            Image,
            self._params.image_topic.name,
            qos_profile=qos_profile,
        )

    def _create_joint_state_subscription(self) -> None:
        if self._joint_state_sub is not None:
            self.destroy_subscription(self._joint_state_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.joint_state_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._joint_state_sub = Subscriber(
            self,
            JointState,
            self._params.joint_state_topic.name,
            qos_profile=qos_profile,
        )

    def _create_point_cloud_subscription(self) -> None:
        if self._point_cloud_sub is not None:
            self.destroy_subscription(self._point_cloud_sub)
        qos_profile = qos_profile_system_default  # careful, this creates a copy of the system default qos profile
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.point_cloud_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._point_cloud_sub = Subscriber(
            self,
            PointCloud2,
            self._params.point_cloud_topic.name,
            qos_profile=qos_profile,
        )

    def _create_approximate_time_sync(self) -> None:
        if self._approximate_time_sync is not None:
            self._approximate_time_sync = None
        self._approximate_time_sync = ApproximateTimeSynchronizer(
            [
                self._image_sub,
                self._camera_info_sub,
                self._joint_state_sub,
                self._point_cloud_sub,
            ],
            queue_size=1,
            slop=self._params.filters.sync_accuracy,
        )
        self._approximate_time_sync.registerCallback(self._on_sync)

    def _create_robot_description_subscription(self) -> None:
        qos_profile = qos_profile_system_default
        self._robot_description = None
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._robot_description_sub = self.create_subscription(
            String,
            self._params.robot_description_topic.name,
            self._on_robot_description,
            qos_profile=qos_profile,
        )

    def _create_subscriptions(self) -> None:
        self._create_camera_info_subscription()
        self._create_image_subscription()
        self._create_joint_state_subscription()
        self._create_point_cloud_subscription()

        # topic synchronizer
        self._create_approximate_time_sync()

        # robot description
        self._create_robot_description_subscription()

    def _on_sync(
        self,
        image: Image,
        camera_info: CameraInfo,
        joint_state: JointState,
        point_cloud: PointCloud2,
    ):
        self._synced_sample.image = image
        self._synced_sample.camera_info = camera_info
        self._synced_sample.joint_state = joint_state
        self._synced_sample.point_cloud = point_cloud

    def _on_robot_description(self, msg: String) -> None:
        self.get_logger().info("Received robot description.")
        self._robot_description = msg.data
        self._o3d_robot = O3DRobot(
            self._robot_description, self._params.registration.convex_hull
        )

    def _on_collect_sample(
        self, request: CollectSample.Request, response: CollectSample.Response
    ) -> CollectSample.Response:
        try:
            if (
                self._synced_sample.image is None
                or self._synced_sample.camera_info is None
                or self._synced_sample.joint_state is None
                or self._synced_sample.point_cloud is None
            ):
                response.success = False
                response.n_collected = len(self._synced_samples)
                response.message = f"No data available yet. Topics might be wrongly configured. Data might not be synchronized, accuracy: {self._params.filters.sync_accuracy} s."
                self.get_logger().warn(response.message)
                return response

            # check if joint states changed from last data
            if len(self._synced_samples) > 1:
                if np.isclose(
                    self._synced_samples[-1].joint_state.position,
                    self._synced_sample.joint_state.position,
                    atol=self._params.filters.min_joint_position_change,
                ).all():
                    response.success = False
                    response.n_collected = len(self._synced_samples)
                    response.message = f"Joint states did not change. Minimum joint position change: {self._params.filters.min_joint_position_change} rad. Skipping data collection."
                    self.get_logger().warn(response.message)
                    return response

            # only allow joint states velocities close to zero
            if not np.isclose(
                self._synced_sample.joint_state.velocity,
                np.zeros_like(self._synced_sample.joint_state.velocity),
                atol=self._params.filters.max_joint_velocity,
            ).all():
                response.success = False
                response.n_collected = len(self._synced_samples)
                response.message = f"Joint states velocity greater zero. Maximum joint velocity: {self._params.filters.max_joint_velocity} rad/s. This may cause un-correlated data. Skipping data collection."
                self.get_logger().warn(response.message)
                return response

            # add data
            self._synced_samples.append(copy.deepcopy(self._synced_sample))
            response.success = True
            response.n_collected = len(self._synced_samples)
            response.message = f"Added data with time stamp: {self._synced_sample.joint_state.header.stamp}"
            self.get_logger().info(response.message)
            self._synced_sample.clear()
        except Exception as e:
            response.success = False
            response.n_collected = len(self._synced_samples)
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_clear_samples(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._synced_samples.clear()
        response.success = True
        response.message = "Cleared all samples"
        self.get_logger().info(response.message)
        return response

    def _on_register(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if len(self._synced_samples) == 0:
            response.success = False
            response.message = "No data collected yet"
            return response
        if not self._robot_description:
            response.success = False
            response.message = "No robot description available"
            return response
        if not os.path.exists(self._params.segmentation.sam_checkpoint_path):
            response.success = False
            response.message = f"Path to SAM checkpoint does not exist: {self._params.segmentation.sam_checkpoint_path}"
            self.get_logger().error(response.message)
            return response

        try:

            def detect_and_segment():
                masks = []

                # segment the images
                detector = OpenCVDetector(
                    buffer_size=self._params.segmentation.buffer_size
                )
                self.get_logger().info(
                    f"Loading SAM model from {self._params.segmentation.sam_checkpoint_path}"
                )
                segmentor = SamSegmentor(
                    sam_checkpoint=self._params.segmentation.sam_checkpoint_path,
                    model_type=self._params.segmentation.model_type,
                    device=self._params.segmentation.device,
                )
                self.get_logger().info("Segmentation model loaded")
                self.get_logger().info("Segmenting robot...")
                for idx, synced_data in enumerate(self._synced_samples):
                    image = self._bridge.imgmsg_to_cv2(
                        synced_data.image, desired_encoding="bgr8"
                    )
                    points, labels = detector.detect(image)
                    detector.clear()
                    self.get_logger().info(
                        f"Annotated [{idx+1}/{len(self._synced_samples)}] images"
                    )
                    mask = segmentor(image, np.array(points), np.array(labels))
                    masks.append((mask * 255.0).astype(np.uint8))
                self.get_logger().info("Segmentation done")

                # delete model from gpu
                del segmentor
                torch.cuda.empty_cache()
                return masks

            # remove segmentor from gpu
            masks = detect_and_segment()

            # prepare point clouds
            masks = [
                mask_boundary(
                    mask,
                    np.ones(
                        [
                            self._params.registration.erosion_kernel_size,
                            self._params.registration.erosion_kernel_size,
                        ]
                    ),
                )
                for mask in masks
            ]
            self.get_logger().info("Preparing point clouds and meshes...")
            observed_xyzs = []
            mesh_xyzs = []
            mesh_xyzs_normals = []
            for synced_data, mask in zip(self._synced_samples, masks):
                # extract xyz from point cloud and clean data
                observed_xyz, _ = self._point_cloud_to_numpy(synced_data.point_cloud)
                observed_xyzs.append(clean_xyz(observed_xyz, mask))

                # transform mesh according to joint state
                mesh_xyz = None
                mesh_xyz_normals = None

                self._o3d_robot.set_joint_positions(
                    np.array(synced_data.joint_state.position)
                )
                pcds = self._o3d_robot.sample_point_clouds_equally(
                    number_of_points=self._params.registration.number_of_points
                )
                mesh_xyz = np.concatenate([np.array(pcd.points) for pcd in pcds])
                mesh_xyz_normals = np.concatenate(
                    [np.array(pcd.normals) for pcd in pcds]
                )
                mesh_xyzs.append(mesh_xyz)
                mesh_xyzs_normals.append(mesh_xyz_normals)

            # delete masks from gpu
            del masks
            torch.cuda.empty_cache()

            # to torch
            for i in range(len(observed_xyzs)):
                observed_xyzs[i] = torch.from_numpy(observed_xyzs[i]).to(
                    dtype=torch.float32, device=self._params.registration.device
                )
                mesh_xyzs[i] = torch.from_numpy(mesh_xyzs[i]).to(
                    dtype=torch.float32, device=self._params.registration.device
                )
                mesh_xyzs_normals[i] = torch.from_numpy(mesh_xyzs_normals[i]).to(
                    dtype=torch.float32, device=self._params.registration.device
                )

            # registration
            HT_init = hydra_centroid_alignment(observed_xyzs, mesh_xyzs)
            HT = hydra_robust_icp(
                HT_init,
                observed_xyzs,
                mesh_xyzs,
                mesh_xyzs_normals,
                max_distance=self._params.registration.max_distance,
                outer_max_iter=self._params.registration.outer_max_iter,
                inner_max_iter=self._params.registration.inner_max_iter,
            )
            self._HT = HT.cpu().numpy()
            response.success = True
            response.message = "Registration successful"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)

        return response

    def _point_cloud_to_numpy(
        self,
        point_cloud: PointCloud2,
    ) -> Tuple[np.ndarray, np.ndarray]:
        self.get_logger().info("Converting point cloud to numpy")
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

    def _on_export_samples(
        self, request: Export.Request, response: Export.Response
    ) -> Export.Response:
        try:
            if len(self._synced_samples) == 0:
                response.success = False
                response.message = "No data collected yet"
                return response

            path = pathlib.Path(request.path)
            self.get_logger().info(f"Saving data to {path.absolute()}")

            def write_synced_data():
                def write_camera_info_to_yaml(camera_info_msg: CameraInfo, path: str):
                    import yaml

                    camera_info_dict = {
                        "frame_id": camera_info_msg.header.frame_id,
                        "height": camera_info_msg.height,
                        "width": camera_info_msg.width,
                        "distortion_model": camera_info_msg.distortion_model,
                        "d": camera_info_msg.d.tolist(),
                        "k": camera_info_msg.k.tolist(),
                        "r": camera_info_msg.r.tolist(),
                        "p": camera_info_msg.p.tolist(),
                        "binning_x": camera_info_msg.binning_x,
                        "binning_y": camera_info_msg.binning_y,
                        "roi": {
                            "x_offset": camera_info_msg.roi.x_offset,
                            "y_offset": camera_info_msg.roi.y_offset,
                            "height": camera_info_msg.roi.height,
                            "width": camera_info_msg.roi.width,
                            "do_rectify": camera_info_msg.roi.do_rectify,
                        },
                    }
                    with open(path, "w") as file:
                        yaml.dump(camera_info_dict, file)

                # save camera info
                write_camera_info_to_yaml(
                    self._synced_samples[0].camera_info,
                    os.path.join(path, "camera_info.yaml"),
                )

                # log time stamps to csv
                with open(os.path.join(path, "time_stamps.csv"), "w") as f:
                    f.write("idx,sec,nanosec\n")

                    for idx, synced_data in enumerate(self._synced_samples):
                        # log time stamps
                        f.write(
                            f"{idx},{synced_data.joint_state.header.stamp.sec},{synced_data.joint_state.header.stamp.nanosec}\n"
                        )

                        # convert to numpy
                        image_np = self._bridge.imgmsg_to_cv2(
                            synced_data.image, desired_encoding="passthrough"
                        )
                        joint_position = synced_data.joint_state.position
                        name = synced_data.joint_state.name
                        joint_position = [
                            x for _, x in sorted(zip(name, joint_position))
                        ]
                        joint_position_np = np.array(joint_position)
                        xyz_np, rgba_np = self._point_cloud_to_numpy(
                            synced_data.point_cloud
                        )

                        # save
                        cv2.imwrite(
                            os.path.join(path, f"image_{idx}.png"),
                            image_np,
                        )
                        np.save(
                            os.path.join(path, f"joint_states_{idx}.npy"),
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
                self._synced_samples.clear()

            if path.exists():
                try:
                    write_synced_data()
                except Exception as e:
                    response.success = False
                    response.message = f"Could not write data to {path.absolute()}"
                    self.get_logger().error(response.message)
                    self.get_logger().error(e)
                    return response
                response.success = True
                response.message = f"Wrote data to {path.absolute()}"
                return response

            if request.mkdir:
                path.mkdir(parents=True, exist_ok=True)
                try:
                    write_synced_data()
                except Exception as e:
                    response.success = False
                    response.message = f"Could not write data to {path.absolute()}"
                    self.get_logger().error(response.message)
                    self.get_logger().error(e)
                    return response
                response.success = True
                response.message = (
                    f"Created directory {path.absolute()} and wrote data to it"
                )
                return response

            response.success = False
            response.message = f"Path {path.absolute()} does not exist and was not created as per request"
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_export_transform(
        self, request: Export.Request, response: Export.Response
    ) -> Export.Response:
        path = pathlib.Path(request.path)
        self.get_logger().info(f"Saving transform to {path.absolute()}")
        response.success = True
        try:
            if not path.parent.exists():
                if request.mkdir:
                    path.parent.mkdir(parents=True, exist_ok=True)
                else:
                    response.success = False
                    response.message = f"Path {path.parent.absolute()} does not exist"
                    self.get_logger().error(response.message)
                    return response
            np.savetxt(path, self._HT)
            response.message = f"Saved transform to {path.absolute()}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_import_transform(
        self, request: Import.Request, response: Import.Response
    ) -> Import.Response:
        response.success = True
        path = pathlib.Path(request.path)
        try:
            if not path.exists():
                response.success = False
                response.message = f"Path {path.absolute()} does not exist"
                self.get_logger().error(response.message)
                return response
            self.get_logger().info(f"Loading transform from {request.path}")
            HT = np.loadtxt(path.absolute())
            if HT.shape != (4, 4):
                response.success = False
                response.message = f"Transform has wrong shape: {HT.shape}"
                self.get_logger().error(response.message)
                return response
            self._HT = copy.deepcopy(HT)
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_broadcast_tf(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        try:
            self._tf_broadcaster.broadcast_tf(
                self._HT,
                parent=self._params.tf_broadcaster.parent_frame,
                child=self._params.tf_broadcaster.child_frame,
                target_child=self._params.tf_broadcaster.target_child_frame,
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response
