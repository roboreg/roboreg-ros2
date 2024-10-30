import copy
import os
import pathlib
from typing import List

import cv2
import cv_bridge
import numpy as np
import torch
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_system_default
from roboreg import differentiable as rrd
from roboreg.detector import OpenCVDetector
from roboreg.hydra_icp import hydra_centroid_alignment, hydra_robust_icp
from roboreg.io import URDFParser
from roboreg.segmentor import Sam2Segmentor
from roboreg.util import (
    clean_xyz,
    compute_vertex_normals,
    depth_to_xyz,
    from_homogeneous,
    generate_ht_optical,
    mask_boundary,
    to_homogeneous,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_roboreg.broadcaster import StaticTFBroadcaster
from ros2_roboreg.structs import ServerParams, SyncedSample
from ros2_roboreg_idl.srv import CollectSample, Export, Import, RegHydraICP, RegStereoDR


class RoboregServer(Node):
    def __init__(self, node_name: str = "roboreg") -> None:
        super().__init__(node_name)

        # data collection
        self._synced_sample = SyncedSample()
        self._synced_samples: List[SyncedSample] = []

        # segmentation
        self._left_detector = None
        self._right_detector = None
        self._left_segmentations = []
        self._right_segmentations = []

        # point cloud
        self._pcls = []

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

        # robot model
        self._urdf_parser = None
        self._kinematics = None
        self._meshes = None

        # subscriptions
        self._left_camera_info_sub = None
        self._left_image_sub = None
        self._right_camera_info_sub = None
        self._right_image_sub = None
        self._joint_state_sub = None
        self._depth_sub = None
        self._approximate_time_sync = None
        self._create_subscriptions()

        # services
        self._create_services()

    def _instantiate_robot_model(self, batch_size: int) -> None:
        if not self._urdf_parser:
            raise ValueError("No robot description available")

        if (  # already instantiated and correctly setup
            self._kinematics is not None
            and self._meshes is not None
            and batch_size == self._meshes.batch_size
            and self._kinematics.device == self._params.robot_model.device
            and self._meshes.device == self._params.robot_model.device
        ):
            return

        if self._params.robot_model.root_link_name == "":
            self._params.robot_model.root_link_name = (
                self._urdf_parser.link_names_with_meshes[0]
            )
            self.get_logger().info(
                f"Root link name not provided. Using the first link with mesh: '{self._params.robot_model.root_link_name}'."
            )
        if self._params.robot_model.end_link_name == "":
            self._params.robot_model.end_link_name = (
                self._urdf_parser.link_names_with_meshes[-1]
            )
            self.get_logger().info(
                f"End link name not provided. Using the last link with mesh: '{self._params.robot_model.end_link_name}'."
            )

        # instantiate kinematics
        self._kinematics = rrd.TorchKinematics(
            urdf_parser=self._urdf_parser,
            root_link_name=self._params.robot_model.root_link_name,
            end_link_name=self._params.robot_model.end_link_name,
            device=self._params.robot_model.device,
        )

        # instantiate meshes
        self._meshes = rrd.TorchMeshContainer(
            self._urdf_parser.ros_package_mesh_paths(
                root_link_name=self._params.robot_model.root_link_name,
                end_link_name=self._params.robot_model.end_link_name,
            ),
            batch_size=batch_size,
            device=self._params.robot_model.device,
        )

    def _detect_and_segment(self):
        left_segment_idx = len(self._left_segmentations)
        right_segment_idx = len(self._right_segmentations)
        if left_segment_idx >= len(self._synced_samples) and right_segment_idx >= len(
            self._synced_samples
        ):
            self.get_logger().info("Segmentation already done")
            return

        # segment the images
        self._left_detector = OpenCVDetector(
            n_positive_samples=self._params.segmentation.n_positive_samples,
            n_negative_samples=self._params.segmentation.n_negative_samples,
            window_name="Left Image Detector",
        )
        self._right_detector = OpenCVDetector(
            n_positive_samples=self._params.segmentation.n_positive_samples,
            n_negative_samples=self._params.segmentation.n_negative_samples,
            window_name="Right Image Detector",
        )
        self.get_logger().info(
            f"Loading SAM 2 model with ID: '{self._params.segmentation.model_id}'"
        )

        segmentor = Sam2Segmentor(
            model_id=self._params.segmentation.model_id,
            pth=self._params.segmentation.pth,
            device=self._params.segmentation.device,
        )
        self.get_logger().info("Segmentation model loaded")
        self.get_logger().info("Segmenting robot...")
        while left_segment_idx < len(self._synced_samples):
            # segment left images
            synced_sample = self._synced_samples[left_segment_idx]
            left_image = self._bridge.imgmsg_to_cv2(
                synced_sample.left_image, desired_encoding="bgr8"
            )
            samples, labels = self._left_detector.detect(left_image)
            probability = segmentor(left_image, np.array(samples), np.array(labels))
            mask = np.where(probability > segmentor.pth, 255, 0).astype(np.uint8)
            self._left_segmentations.append((mask * 255.0).astype(np.uint8))
            left_segment_idx += 1
            self.get_logger().info(
                f"Annotated [{left_segment_idx}/{len(self._synced_samples)}] left images"
            )
        while right_segment_idx < len(self._synced_samples):
            # segment right images
            synced_sample = self._synced_samples[right_segment_idx]
            right_image = self._bridge.imgmsg_to_cv2(
                synced_sample.right_image, desired_encoding="bgr8"
            )
            samples, labels = self._right_detector.detect(right_image)
            probability = segmentor(right_image, np.array(samples), np.array(labels))
            mask = np.where(probability > segmentor.pth, 255, 0).astype(np.uint8)
            self._right_segmentations.append((mask * 255.0).astype(np.uint8))
            right_segment_idx += 1
            self.get_logger().info(
                f"Annotated [{right_segment_idx}/{len(self._synced_samples)}] right images"
            )
        self.get_logger().info("Segmentation done")

        # delete model from gpu
        del segmentor
        torch.cuda.empty_cache()

    def _obtain_pcl_from_depth(self) -> None:
        if len(self._pcls) == len(self._synced_samples):
            self.get_logger().info("Point clouds already computed")
            return
        if len(self._synced_samples) == 0:
            self.get_logger().warn("No data available")
            return
        depths = [
            self._bridge.imgmsg_to_cv2(synced_sample.depth)
            for synced_sample in self._synced_samples
        ]
        depths = torch.tensor(
            np.array(depths),
            dtype=torch.float32,
            device=self._params.robot_model.device,
        )
        intrinsics = [
            np.array(synced_sample.left_camera_info.k).reshape(3, 3)
            for synced_sample in self._synced_samples
        ]
        intrinsics = torch.tensor(
            np.array(intrinsics),
            dtype=torch.float32,
            device=self._params.robot_model.device,
        )
        pcls = depth_to_xyz(
            depths,
            intrinsics,
            z_min=self._params.filters.min_depth,
            z_max=self._params.filters.max_depth,
        )

        # transform into desired frame
        height, width = (
            self._synced_samples[0].left_camera_info.height,
            self._synced_samples[0].left_camera_info.width,
        )
        # flatten BxHxWx3 -> Bx(H*W)x3
        pcls = pcls.view(-1, height * width, 3)
        pcls = to_homogeneous(pcls)
        ht_optical = generate_ht_optical(
            pcls.shape[0], dtype=torch.float32, device=self._params.robot_model.device
        )
        pcls = torch.matmul(pcls, ht_optical.transpose(-1, -2))
        pcls = from_homogeneous(pcls)

        # unflatten
        pcls = pcls.view(-1, height, width, 3)

        # turn pcls into list of numpy arrays
        self._pcls = [pcl.cpu().numpy() for pcl in pcls]

    def _declare_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("filters.sync_accuracy", 0.01),
                ("filters.min_joint_position_change", 0.001),
                ("filters.max_joint_velocity", 0.01),
                ("filters.min_depth", 0.01),
                ("filters.max_depth", 4.0),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.image.left.name", "left/image_rect_color"),
                ("topics.image.left.qos_reliability", "RELIABLE"),
                ("topics.camera_info.left.name", "left/camera_info"),
                ("topics.camera_info.left.qos_reliability", "RELIABLE"),
                ("topics.image.right.name", "right/image_rect_color"),
                ("topics.image.right.qos_reliability", "RELIABLE"),
                ("topics.camera_info.right.name", "right/camera_info"),
                ("topics.camera_info.right.qos_reliability", "RELIABLE"),
                ("topics.joint_state.name", "joint_state"),
                ("topics.joint_state.qos_reliability", "RELIABLE"),
                ("topics.depth.name", "depth/registered"),
                ("topics.depth.qos_reliability", "RELIABLE"),
                ("topics.robot_description.name", "robot_description"),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("segmentation.device", "cuda" if torch.cuda.is_available() else "cpu"),
                ("segmentation.n_positive_samples", 5),
                ("segmentation.n_negative_samples", 5),
                ("segmentation.model_id", "facebook/sam2-hiera-large"),
                ("segmentation.pth", 0.5),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_model.device", "cuda" if torch.cuda.is_available() else "cpu"),
                ("robot_model.root_link_name", ""),
                ("robot_model.end_link_name", ""),
                ("robot_model.visual_meshes", False),
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
        self._params.filters.min_depth = (
            self.get_parameter("filters.min_depth").get_parameter_value().double_value
        )
        self._params.filters.max_depth = (
            self.get_parameter("filters.max_depth").get_parameter_value().double_value
        )

        # topic parameters
        self._params.left_image_topic.name = (
            self.get_parameter("topics.image.left.name")
            .get_parameter_value()
            .string_value
        )
        self._params.left_image_topic.qos_reliability = (
            self.get_parameter("topics.image.left.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.left_camera_info_topic.name = (
            self.get_parameter("topics.camera_info.left.name")
            .get_parameter_value()
            .string_value
        )
        self._params.left_camera_info_topic.qos_reliability = (
            self.get_parameter("topics.camera_info.left.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.right_image_topic.name = (
            self.get_parameter("topics.image.right.name")
            .get_parameter_value()
            .string_value
        )
        self._params.right_image_topic.qos_reliability = (
            self.get_parameter("topics.image.right.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.right_camera_info_topic.name = (
            self.get_parameter("topics.camera_info.right.name")
            .get_parameter_value()
            .string_value
        )
        self._params.right_camera_info_topic.qos_reliability = (
            self.get_parameter("topics.camera_info.right.qos_reliability")
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
        self._params.depth_topic.name = (
            self.get_parameter("topics.depth.name").get_parameter_value().string_value
        )

        self._params.depth_topic.qos_reliability = (
            self.get_parameter("topics.depth.qos_reliability")
            .get_parameter_value()
            .string_value
        )
        self._params.robot_description_topic.name = (
            self.get_parameter("topics.robot_description.name")
            .get_parameter_value()
            .string_value
        )

        # segmentation parameters
        self._params.segmentation.device = (
            self.get_parameter("segmentation.device").get_parameter_value().string_value
        )
        self._params.segmentation.n_positive_samples = (
            self.get_parameter("segmentation.n_positive_samples")
            .get_parameter_value()
            .integer_value
        )
        self._params.segmentation.n_negative_samples = (
            self.get_parameter("segmentation.n_negative_samples")
            .get_parameter_value()
            .integer_value
        )
        self._params.segmentation.model_id = (
            self.get_parameter("segmentation.model_id")
            .get_parameter_value()
            .string_value
        )
        self._params.segmentation.pth = (
            self.get_parameter("segmentation.pth").get_parameter_value().double_value
        )

        # robot model parameters
        self._params.robot_model.device = (
            self.get_parameter("robot_model.device").get_parameter_value().string_value
        )
        self._params.robot_model.root_link_name = (
            self.get_parameter("robot_model.root_link_name")
            .get_parameter_value()
            .string_value
        )
        self._params.robot_model.end_link_name = (
            self.get_parameter("robot_model.end_link_name")
            .get_parameter_value()
            .string_value
        )
        self._params.robot_model.visual_meshes = (
            self.get_parameter("robot_model.visual_meshes")
            .get_parameter_value()
            .bool_value
        )

        # tf broadcaster parameters
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
        self.get_logger().info(f"*{' '*7}Min depth: {self._params.filters.min_depth} m")
        self.get_logger().info(f"*{' '*7}Max depth: {self._params.filters.max_depth} m")
        self.get_logger().info(f"*{' '*5}Topics:")
        self.get_logger().info(f"*{' '*7}Left image:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.left_image_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.left_image_topic.qos_reliability}."
        )
        self.get_logger().info(f"*{' '*7}Left camera info:")
        self.get_logger().info(
            f"*{' '*9}Name: {self._params.left_camera_info_topic.name}"
        )
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.left_camera_info_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Right image:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.right_image_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.right_image_topic.qos_reliability}."
        )
        self.get_logger().info(f"*{' '*7}Right camera info:")
        self.get_logger().info(
            f"*{' '*9}Name: {self._params.right_camera_info_topic.name}"
        )
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.right_camera_info_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Joint states:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.joint_state_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.joint_state_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Depth:")
        self.get_logger().info(f"*{' '*9}Name: {self._params.depth_topic.name}")
        self.get_logger().info(
            f"*{' '*9}QoS reliability: {self._params.depth_topic.qos_reliability}"
        )
        self.get_logger().info(f"*{' '*7}Robot description:")
        self.get_logger().info(
            f"*{' '*9}Name: {self._params.robot_description_topic.name}"
        )
        self.get_logger().info(f"*{' '*5}Segmentation:")
        self.get_logger().info(f"*{' '*7}Device: {self._params.segmentation.device}")
        self.get_logger().info(
            f"*{' '*7}N positive samples: {self._params.segmentation.n_positive_samples}"
        )
        self.get_logger().info(
            f"*{' '*7}N negative samples: {self._params.segmentation.n_negative_samples}"
        )
        self.get_logger().info(
            f"*{' '*7}Model ID: '{self._params.segmentation.model_id}'"
        )
        self.get_logger().info(
            f"*{' '*7}Probability threshold: {self._params.segmentation.pth}"
        )
        self.get_logger().info(f"*{' '*5}Robot model:")
        self.get_logger().info(f"*{' '*7}Device: {self._params.robot_model.device}")
        self.get_logger().info(
            f"*{' '*7}Root link name: '{self._params.robot_model.root_link_name}'"
        )
        self.get_logger().info(
            f"*{' '*7}End link name: '{self._params.robot_model.end_link_name}'"
        )
        self.get_logger().info(
            f"*{' '*7}Visual meshes: {self._params.robot_model.visual_meshes}"
        )
        self.get_logger().info(f"*{' '*5}TF broadcaster:")
        self.get_logger().info(
            f"*{' '*7}Parent frame: '{self._params.tf_broadcaster.parent_frame}'"
        )
        self.get_logger().info(
            f"*{' '*7}Child frame: '{self._params.tf_broadcaster.child_frame}'"
        )
        self.get_logger().info(
            f"*{' '*7}Target child frame: '{self._params.tf_broadcaster.target_child_frame}'"
        )
        self.get_logger().info("***")

    def _on_set_parameter(self, parameters: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            if parameter.name == "topics.camera_info.left.name":
                self.get_logger().info(
                    f"Setting left camera info topic to {parameter.value}"
                )
                self._params.left_camera_info_topic.name = parameter.value
                self._create_left_camera_info_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.camera_info.right.name":
                self.get_logger().info(
                    f"Setting right camera info topic to {parameter.value}"
                )
                self._params.right_camera_info_topic.name = parameter.value
                self._create_right_camera_info_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.image.left.name":
                self.get_logger().info(f"Setting left image topic to {parameter.value}")
                self._params.left_image_topic.name = parameter.value
                self._create_left_image_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.image.right.name":
                self.get_logger().info(
                    f"Setting right image topic to {parameter.value}"
                )
                self._params.right_image_topic.name = parameter.value
                self._create_right_image_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.joint_state.name":
                self.get_logger().info(
                    f"Setting joint state topic to {parameter.value}"
                )
                self._params.joint_state_topic.name = parameter.value
                self._create_joint_state_subscription()
                self._create_approximate_time_sync()
            elif parameter.name == "topics.depth.name":
                self.get_logger().info(f"Setting depth topic to {parameter.value}")
                self._params.depth_topic.name = parameter.value
                self._create_depth_subscription()
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
        self._hydra_icp_register_service = self.create_service(
            RegHydraICP, "~/register/hydra_icp", self._on_register_hydra_icp
        )
        self._stereo_dr_register_service = self.create_service(
            RegStereoDR, "~/register/stereo_dr", self._on_register_stereo_dr
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

    def _create_left_camera_info_subscription(self) -> None:
        if self._left_camera_info_sub is not None:
            self.destroy_subscription(self._left_camera_info_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.left_camera_info_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._left_camera_info_sub = Subscriber(
            self,
            CameraInfo,
            self._params.left_camera_info_topic.name,
            qos_profile=qos_profile,
        )

    def _create_right_camera_info_subscription(self) -> None:
        if self._right_camera_info_sub is not None:
            self.destroy_subscription(self._right_camera_info_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.right_camera_info_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._right_camera_info_sub = Subscriber(
            self,
            CameraInfo,
            self._params.right_camera_info_topic.name,
            qos_profile=qos_profile,
        )

    def _create_left_image_subscription(self) -> None:
        if self._left_image_sub is not None:
            self.destroy_subscription(self._left_image_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.left_image_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._left_image_sub = Subscriber(
            self,
            Image,
            self._params.left_image_topic.name,
            qos_profile=qos_profile,
        )

    def _create_right_image_subscription(self) -> None:
        if self._right_image_sub is not None:
            self.destroy_subscription(self._right_image_sub)
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.right_image_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._right_image_sub = Subscriber(
            self,
            Image,
            self._params.right_image_topic.name,
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

    def _create_depth_subscription(self) -> None:
        if self._depth_sub is not None:
            self.destroy_subscription(self._depth_sub)
        qos_profile = qos_profile_system_default  # careful, this creates a copy of the system default qos profile
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._params.depth_topic.qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._depth_sub = Subscriber(
            self,
            Image,
            self._params.depth_topic.name,
            qos_profile=qos_profile,
        )

    def _create_approximate_time_sync(self) -> None:
        if self._approximate_time_sync is not None:
            self._approximate_time_sync = None
        self._approximate_time_sync = ApproximateTimeSynchronizer(
            [
                self._left_image_sub,
                self._left_camera_info_sub,
                self._right_image_sub,
                self._right_camera_info_sub,
                self._joint_state_sub,
                self._depth_sub,
            ],
            queue_size=1,
            slop=self._params.filters.sync_accuracy,
        )
        self._approximate_time_sync.registerCallback(self._on_sync)

    def _create_robot_description_subscription(self) -> None:
        qos_profile = qos_profile_system_default
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._robot_description_sub = self.create_subscription(
            String,
            self._params.robot_description_topic.name,
            self._on_robot_description,
            qos_profile=qos_profile,
        )

    def _create_subscriptions(self) -> None:
        self._create_left_camera_info_subscription()
        self._create_left_image_subscription()
        self._create_right_camera_info_subscription()
        self._create_right_image_subscription()
        self._create_joint_state_subscription()
        self._create_depth_subscription()

        # topic synchronizer
        self._create_approximate_time_sync()

        # robot description
        self._create_robot_description_subscription()

    def _on_sync(
        self,
        left_image: Image,
        left_camera_info: CameraInfo,
        right_image: Image,
        right_camera_info: CameraInfo,
        joint_state: JointState,
        depth: Image,
    ):
        self._synced_sample.left_image = left_image
        self._synced_sample.left_camera_info = left_camera_info
        self._synced_sample.right_image = right_image
        self._synced_sample.right_camera_info = right_camera_info
        self._synced_sample.joint_state = joint_state
        self._synced_sample.depth = depth

    def _on_robot_description(self, msg: String) -> None:
        self.get_logger().info("Received robot description.")
        # instantiate urdf parser
        self._urdf_parser = URDFParser()
        self._urdf_parser.from_urdf(msg.data)

    def _on_collect_sample(
        self, request: CollectSample.Request, response: CollectSample.Response
    ) -> CollectSample.Response:
        try:
            if (
                self._synced_sample.left_image is None
                or self._synced_sample.right_image is None
                or self._synced_sample.left_camera_info is None
                or self._synced_sample.right_camera_info is None
                or self._synced_sample.joint_state is None
                or self._synced_sample.depth is None
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
        self._left_detector.clear()
        self._right_detector.clear()
        self._left_segmentations.clear()
        self._right_segmentations.clear()
        self._pcls.clear()
        response.success = True
        response.message = "Cleared all samples"
        self.get_logger().info(response.message)
        return response

    def _hydra_icp_impl(self, request: RegHydraICP.Request) -> torch.Tensor:
        # process data
        mesh_vertices = self._meshes.vertices.clone()
        name_idx_map = np.argsort(np.array(self._synced_samples[0].joint_state.name))
        joint_states = [
            synced_sample.joint_state.position[name_idx_map]
            for synced_sample in self._synced_samples
        ]
        joint_states = torch.tensor(
            np.array(joint_states),
            dtype=torch.float32,
            device=self._params.robot_model.device,
        )
        ht_lookup = self._kinematics.mesh_forward_kinematics(joint_states)
        for link_name, ht in ht_lookup.items():
            mesh_vertices[
                :,
                self._meshes.lower_vertex_index_lookup[
                    link_name
                ] : self._meshes.upper_vertex_index_lookup[link_name],
            ] = torch.matmul(
                mesh_vertices[
                    :,
                    self._meshes.lower_vertex_index_lookup[
                        link_name
                    ] : self._meshes.upper_vertex_index_lookup[link_name],
                ],
                ht.transpose(-1, -2),
            )

        # mesh vertices to list
        if self._meshes.batch_size != len(self._synced_samples):
            raise ValueError("Batch size mismatch.")
        batch_size = self._meshes.batch_size
        mesh_vertices = from_homogeneous(mesh_vertices)
        mesh_vertices = [mesh_vertices[i].contiguous() for i in range(batch_size)]
        mesh_normals = []
        for i in range(batch_size):
            mesh_normals.append(
                compute_vertex_normals(
                    vertices=mesh_vertices[i], faces=self._meshes.faces
                )
            )

        # clean observed vertices and turn into tensor
        observed_vertices = [
            torch.tensor(
                clean_xyz(
                    xyz=xyz,
                    mask=(
                        mask_boundary(
                            mask,
                            erosion_kernel=np.ones(
                                [
                                    request.erosion_kernel_size,
                                    request.erosion_kernel_size,
                                ]
                            ),
                        )
                        if request.with_erosion
                        else mask
                    ),
                ),
                dtype=torch.float32,
                device=self._params.robot_model.device,
            )
            for xyz, mask in zip(self._pcls, self._left_segmentations)
        ]

        # sample N points per mesh
        for i in range(batch_size):
            idx = torch.randperm(mesh_vertices[i].shape[0])[: request.number_of_points]
            mesh_vertices[i] = mesh_vertices[i][idx]
            mesh_normals[i] = mesh_normals[i][idx]

        HT_init = hydra_centroid_alignment(observed_vertices, mesh_vertices)
        HT = hydra_robust_icp(
            HT_init,
            observed_vertices,
            mesh_vertices,
            mesh_normals,
            max_distance=request.max_distance,
            outer_max_iter=request.outer_max_iter,
            inner_max_iter=request.inner_max_iter,
        )
        return HT

    def _stereo_dr_impl(self, request: RegStereoDR.Request) -> None:
        raise NotImplementedError("Stereo DR registration not implemented yet")

    def _on_register_hydra_icp(
        self, request: RegHydraICP.Request, response: RegHydraICP.Response
    ) -> Trigger.Response:
        if len(self._synced_samples) == 0:
            response.success = False
            response.message = "No data collected yet"
            return response
        self._instantiate_robot_model(batch_size=len(self._synced_samples))
        if not self._kinematics:
            response.success = False
            response.message = "No kinematics available"
            return response
        if not self._meshes:
            response.success = False
            response.message = "No meshes available"
            return response
        try:
            # generate segmentation masks
            self._detect_and_segment()
            if len(self._left_segmentations) != len(self._synced_samples):
                raise ValueError(
                    "Segmentation masks not generated for all left images."
                )
            if len(self._right_segmentations) != len(self._synced_samples):
                raise ValueError(
                    "Segmentation masks not generated for all right images."
                )
            self._obtain_pcl_from_depth()
            if len(self._pcls) != len(self._synced_samples):
                raise ValueError("Point clouds not generated for all depth images.")
            if len(self._pcls) != len(self._left_segmentations):
                raise ValueError("Point clouds and segmentations do not match.")
            self._hydra_icp_impl(request)
            response.success = True
            response.message = "Registration successful"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_register_stereo_dr(
        self, request: RegStereoDR.Request, response: RegStereoDR.Response
    ) -> Trigger.Response:
        if len(self._synced_samples) == 0:
            response.success = False
            response.message = "No data collected yet"
            return response
        self._instantiate_robot_model(batch_size=len(self._synced_samples))
        if not self._kinematics:
            response.success = False
            response.message = "No kinematics available"
            return response
        if not self._meshes:
            response.success = False
            response.message = "No meshes available"
            return response
        try:
            # generate segmentation masks
            self._detect_and_segment()
            if len(self._left_segmentations) != len(self._synced_samples):
                raise ValueError(
                    "Segmentation masks not generated for all left images."
                )
            if len(self._right_segmentations) != len(self._synced_samples):
                raise ValueError(
                    "Segmentation masks not generated for all right images."
                )
            self._stereo_dr_impl(request)
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response

    def _on_export_samples(
        self, request: Export.Request, response: Export.Response
    ) -> Export.Response:
        try:
            if len(self._synced_samples) == 0:
                response.success = False
                response.message = "No data collected yet"
                return response

            if len(self._pcls) != len(self._synced_samples):
                self._obtain_pcl_from_depth()  # generate point clouds in case
            if len(self._pcls) != len(self._left_segmentations):
                raise ValueError("Point clouds and segmentations do not match.")

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

                # save camera infos
                write_camera_info_to_yaml(
                    self._synced_samples[0].left_camera_info,
                    os.path.join(path, "left_camera_info.yaml"),
                )
                write_camera_info_to_yaml(
                    self._synced_samples[0].right_camera_info,
                    os.path.join(path, "right_camera_info.yaml"),
                )

                # save segmentation labels
                self._left_detector.write(
                    path=os.path.join(path, "sam2_left_labels.csv"),
                    samples=self._left_detector.samples,
                    labels=self._left_detector.labels,
                )
                self._right_detector.write(
                    path=os.path.join(path, "sam2_right_labels.csv"),
                    samples=self._right_detector.samples,
                    labels=self._right_detector.labels,
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
                        left_image_np = self._bridge.imgmsg_to_cv2(
                            synced_data.left_image, desired_encoding="passthrough"
                        )
                        right_image_np = self._bridge.imgmsg_to_cv2(
                            synced_data.right_image, desired_encoding="passthrough"
                        )
                        name_idx_map = np.argsort(
                            np.array(synced_data.joint_state.name)
                        )
                        joint_position = synced_data.joint_state.position[name_idx_map]
                        joint_position_np = np.array(joint_position)
                        depth_np = self._bridge.imgmsg_to_cv2(
                            synced_data.depth, desired_encoding="passthrough"
                        )

                        # save
                        cv2.imwrite(
                            os.path.join(path, f"left_image_{idx}.png"),
                            left_image_np,
                        )
                        cv2.imwrite(
                            os.path.join(path, f"right_image_{idx}.png"),
                            right_image_np,
                        )
                        np.save(
                            os.path.join(path, f"joint_states_{idx}.npy"),
                            joint_position_np,
                        )
                        np.save(
                            os.path.join(path, f"depth_{idx}.npy"),
                            depth_np,
                        )
                        np.save(
                            os.path.join(path, f"xyz_{idx}.npy"),
                            self._pcls[idx],
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
