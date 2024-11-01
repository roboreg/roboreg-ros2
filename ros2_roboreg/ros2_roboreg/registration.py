from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

import numpy as np
import torch
from message_filters import Subscriber
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_system_default
from roboreg import differentiable as rrd
from roboreg.detector import OpenCVDetector
from roboreg.io import URDFParser
from roboreg.segmentor import Sam2Segmentor
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

from .broadcaster import StaticTFBroadcaster
from .data.server import Server

# from .plugins.hydra_icp import ...


class HandEyeCalibrationBase(Node, ABC):
    @dataclass
    class FilterParams:
        sync_accuracy: float
        min_depth: float
        max_depth: float

    @dataclass
    class _SegmentationParams:
        device: str
        n_positive_samples: int
        n_negative_samples: int
        model_id: str
        pth: float
        window_name: str

    @dataclass
    class _RobotModelParams:
        device: str
        root_link_name: str
        end_link_name: str
        visual_meshes: bool

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # common parameters
        self._declare_common_parameters()
        self._get_common_parameters()

        # extra parameters
        self._declare_extra_parameters()
        self._get_extra_parameters()

        # data collections
        self._data_server = Server(node=self)
        self._register_synced_subscribers()

        # results broadcasting
        self._tf_broadcaster = StaticTFBroadcaster(node=self)

        # common registration utilities
        self._segmentor = Sam2Segmentor(
            model_id=self._segmentation_params.model_id,
            pth=self._segmentation_params.pth,
            device=self._segmentation_params.device,
        )
        self._detector = OpenCVDetector(
            n_positive_samples=self._segmentation_params.n_positive_samples,
            n_negative_samples=self._segmentation_params.n_negative_samples,
            window_name=self._segmentation_params.window_name,
        )
        self._kinematics: rrd.TorchKinematics = None
        self._meshes: rrd.TorchMeshContainer = (
            None  # requires configuration based on batch size (number of synced samples)
        )
        self._urdf_parser: URDFParser = None
        self._robot_description_sub = self.create_subscription(
            String, "robot_description", self._on_robot_description, 1
        )

    def initialize(self) -> None:
        self._data_server.initialize(accuracy=self._filter_params.sync_accuracy)

    def _on_robot_description(self, msg: String) -> None:
        self._urdf_parser.from_urdf(msg.data)
        if self._robot_model_params.root_link_name == "":
            self._robot_model_params.root_link_name = (
                self._urdf_parser.link_names_with_meshes(
                    self._robot_model_params.visual_meshes
                )[0]
            )
            self.get_logger().info(
                f"No root link name specified. Using first link with mesh: {self._robot_model_params.root_link_name}"
            )
        if self._robot_model_params.end_link_name == "":
            self._robot_model_params.end_link_name = (
                self._urdf_parser.link_names_with_meshes(
                    self._robot_model_params.visual_meshes
                )[-1]
            )
            self.get_logger().info(
                f"No end link name specified. Using last link with mesh: {self._robot_model_params.end_link_name}"
            )

        self.get_logger().info("Instantiating kinematics on robot description.")
        self._kinematics = rrd.TorchKinematics(
            urdf_parser=self._urdf_parser,
            root_link_name=self._robot_model_params.root_link_name,
            end_link_name=self._robot_model_params.end_link_name,
            device=self._robot_model_params.device,
        )

    def _declare_common_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("filters.sync_accuracy", 1.0),
                ("filters.min_depth", 0.01),
                ("filters.max_depth", 4.0),
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
                ("segmentation.window_name", "segmentation"),
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
            namespace="tf_broadcaster",
            parameters=[
                ("parent_frame", "world"),
                ("child_frame", "camera_frame"),
                ("target_child_frame", "camera_link"),
            ],
        )

    def _get_common_parameters(self) -> None:
        self._filter_params = self.FilterParams(
            sync_accuracy=self.get_parameter("filters.sync_accuracy")
            .get_parameter_value()
            .double_value,
            min_depth=self.get_parameter("filters.min_depth")
            .get_parameter_value()
            .double_value,
            max_depth=self.get_parameter("filters.max_depth")
            .get_parameter_value()
            .double_value,
        )
        self._segmentation_params = self._SegmentationParams(
            device=self.get_parameter("segmentation.device")
            .get_parameter_value()
            .string_value,
            n_positive_samples=self.get_parameter("segmentation.n_positive_samples")
            .get_parameter_value()
            .integer_value,
            n_negative_samples=self.get_parameter("segmentation.n_negative_samples")
            .get_parameter_value()
            .integer_value,
            model_id=self.get_parameter("segmentation.model_id")
            .get_parameter_value()
            .string_value,
            pth=self.get_parameter("segmentation.pth")
            .get_parameter_value()
            .double_value,
            window_name=self.get_parameter("segmentation.window_name")
            .get_parameter_value()
            .string_value,
        )
        self._robot_model_params = self._RobotModelParams(
            device=self.get_parameter("robot_model.device")
            .get_parameter_value()
            .string_value,
            root_link_name=self.get_parameter("robot_model.root_link_name")
            .get_parameter_value()
            .string_value,
            end_link_name=self.get_parameter("robot_model.end_link_name")
            .get_parameter_value()
            .string_value,
            visual_meshes=self.get_parameter("robot_model.visual_meshes")
            .get_parameter_value()
            .bool_value,
        )

    def _segment_impl(self, images: List[np.ndarray]) -> List[np.ndarray]:
        pass

    @abstractmethod
    def _register_synced_subscribers(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _segment(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _declare_extra_parameters(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _get_extra_parameters(self) -> None:
        raise NotImplementedError


class MonocularDepthHEIC(HandEyeCalibrationBase):
    @dataclass
    class _ExtraParams:
        image_topic: str
        image_qos_reliability: str
        camera_info_topic: str
        camera_info_qos_reliability: str
        depth_topic: str
        depth_qos_reliability: str
        depth_camera_info_topic: str
        depth_camera_info_qos_reliability: str
        joint_state_topic: str
        joint_state_qos_reliability: str

    def _register_synced_subscribers(self):
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.joint_state_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["joint_states"] = Subscriber(
            self,
            JointState,
            self._extra_params.joint_state_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.image"] = Subscriber(
            self,
            Image,
            self._extra_params.image_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.camera_info_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.camera_info_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.depth_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.depth"] = Subscriber(
            self,
            Image,
            self._extra_params.depth_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.depth_camera_info_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.depth.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.depth_camera_info_topic,
            qos_profile=qos_profile,
        )

    def _segment(self):
        pass

    def _declare_extra_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.image.name", "camera/image_rect_color"),
                ("topics.image.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.image.camera_info.name",
                    "camera/image_rect_color/camera_info",
                ),
                ("topics.image.camera_info.qos_reliability", "BEST_EFFORT"),
                ("topics.depth.name", "camera/depth_registered"),
                ("topics.depth.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.depth.camera_info.name",
                    "camera/depth_registered/camera_info",
                ),
                ("topics.depth.camera_info.qos_reliability", "BEST_EFFORT"),
                ("topics.joint_state.name", "joint_states"),
                ("topics.joint_state.qos_reliability", "BEST_EFFORT"),
            ],
        )

    def _get_extra_parameters(self):
        self._extra_params = self._ExtraParams(
            image_topic=self.get_parameter("topics.image.name")
            .get_parameter_value()
            .string_value,
            image_qos_reliability=self.get_parameter("topics.image.qos_reliability")
            .get_parameter_value()
            .string_value,
            camera_info_topic=self.get_parameter("topics.image.camera_info.name")
            .get_parameter_value()
            .string_value,
            camera_info_qos_reliability=self.get_parameter(
                "topics.image.camera_info.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            depth_topic=self.get_parameter("topics.depth.name")
            .get_parameter_value()
            .string_value,
            depth_qos_reliability=self.get_parameter("topics.depth.qos_reliability")
            .get_parameter_value()
            .string_value,
            depth_camera_info_topic=self.get_parameter("topics.depth.camera_info.name")
            .get_parameter_value()
            .string_value,
            depth_camera_info_qos_reliability=self.get_parameter(
                "topics.depth.camera_info.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            joint_state_topic=self.get_parameter("topics.joint_state.name")
            .get_parameter_value()
            .string_value,
            joint_state_qos_reliability=self.get_parameter(
                "topics.joint_state.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
        )


class StereoDepthHEIC(HandEyeCalibrationBase):
    @dataclass
    class _ExtraParams:
        left_image_topic: str
        left_image_qos_reliability: str
        left_camera_info_topic: str
        left_camera_info_qos_reliability: str
        right_image_topic: str
        right_image_qos_reliability: str
        right_camera_info_topic: str
        right_camera_info_qos_reliability: str
        depth_topic: str
        depth_qos_reliability: str
        depth_camera_info_topic: str
        depth_camera_info_qos_reliability: str
        joint_state_topic: str
        joint_state_qos_reliability: str

    def _register_synced_subscribers(self):
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.joint_state_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["joint_states"] = Subscriber(
            self,
            JointState,
            self._extra_params.joint_state_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.left_image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.left.image"] = Subscriber(
            self,
            Image,
            self._extra_params.left_image_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.left_camera_info_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.left.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.left_camera_info_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.right_image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.right.image"] = Subscriber(
            self,
            Image,
            self._extra_params.right_image_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.right_image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.right.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.right_camera_info_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.depth_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.depth"] = Subscriber(
            self,
            Image,
            self._extra_params.depth_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.depth_camera_info_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.depth.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.depth_camera_info_topic,
            qos_profile=qos_profile,
        )

    def _segment(self):
        pass

    def _declare_extra_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.left_image.name", "camera/left/image_rect_color"),
                ("topics.left_image.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.left_image.camera_info.name",
                    "camera/left/image_rect_color/camera_info",
                ),
                ("topics.left_image.camera_info.qos_reliability", "BEST_EFFORT"),
                ("topics.right_image.name", "camera/right/image_rect_color"),
                ("topics.right_image.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.right_image.camera_info.name",
                    "camera/right/image_rect_color/camera_info",
                ),
                ("topics.right_image.camera_info.qos_reliability", "BEST_EFFORT"),
                ("topics.depth.name", "camera/depth_registered"),
                ("topics.depth.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.depth.camera_info.name",
                    "camera/depth_registered/camera_info",
                ),
                ("topics.depth.camera_info.qos_reliability", "BEST_EFFORT"),
                ("topics.joint_state.name", "joint_states"),
                ("topics.joint_state.qos_reliability", "BEST_EFFORT"),
            ],
        )

    def _get_extra_parameters(self):
        self._extra_params = self._ExtraParams(
            left_image_topic=self.get_parameter("topics.left_image.name")
            .get_parameter_value()
            .string_value,
            left_image_qos_reliability=self.get_parameter(
                "topics.left_image.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            left_camera_info_topic=self.get_parameter(
                "topics.left_image.camera_info.name"
            )
            .get_parameter_value()
            .string_value,
            left_camera_info_qos_reliability=self.get_parameter(
                "topics.left_image.camera_info.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            right_image_topic=self.get_parameter("topics.right_image.name")
            .get_parameter_value()
            .string_value,
            right_image_qos_reliability=self.get_parameter(
                "topics.right_image.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            right_camera_info_topic=self.get_parameter(
                "topics.right_image.camera_info.name"
            )
            .get_parameter_value()
            .string_value,
            right_camera_info_qos_reliability=self.get_parameter(
                "topics.right_image.camera_info.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            depth_topic=self.get_parameter("topics.depth.name")
            .get_parameter_value()
            .string_value,
            depth_qos_reliability=self.get_parameter("topics.depth.qos_reliability")
            .get_parameter_value()
            .string_value,
            depth_camera_info_topic=self.get_parameter("topics.depth.camera_info.name")
            .get_parameter_value()
            .string_value,
            depth_camera_info_qos_reliability=self.get_parameter(
                "topics.depth.camera_info.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            joint_state_topic=self.get_parameter("topics.joint_state.name")
            .get_parameter_value()
            .string_value,
            joint_state_qos_reliability=self.get_parameter(
                "topics.joint_state.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
        )
