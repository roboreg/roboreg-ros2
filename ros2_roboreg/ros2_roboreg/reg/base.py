from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

import numpy as np
import torch
from rclpy.node import Node
from roboreg import differentiable as rrd
from roboreg.detector import OpenCVDetector
from roboreg.io import URDFParser
from roboreg.segmentor import Sam2Segmentor
from std_msgs.msg import String

from ..broadcaster import StaticTFBroadcaster
from ..data.server import Server


class Eye2HandRegistrationBase(Node, ABC):
    @dataclass
    class _FilterParams:
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
        self._ht = np.eye(4)
        self._tf_broadcaster = StaticTFBroadcaster(node=self)

        # common registration utilities
        self._segmentor = Sam2Segmentor(
            model_id=self._segmentation_params.model_id,
            pth=self._segmentation_params.pth,
            device=self._segmentation_params.device,
        )
        self._segmentations = []
        self._kinematics: rrd.TorchKinematics = None
        self._meshes: rrd.TorchMeshContainer = (
            None  # requires configuration based on batch size (number of synced samples)
        )
        self._urdf_parser: URDFParser = URDFParser()
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

    def _instantiate_meshes(self, batch_size: int) -> None:
        if self._kinematics is None:
            raise ValueError("Kinematics not instantiated.")
        if self._urdf_parser is None:
            raise ValueError("URDF parser not instantiated.")
        if self._meshes is not None:
            if self._meshes.batch_size == batch_size:
                return
        self._meshes = rrd.TorchMeshContainer(
            mesh_paths=self._urdf_parser.ros_package_mesh_paths(
                root_link_name=self._robot_model_params.root_link_name,
                end_link_name=self._robot_model_params.end_link_name,
                visual=self._robot_model_params.visual_meshes,
            ),
            batch_size=batch_size,
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
        self._filter_params = self._FilterParams(
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
        segmentations = []
        for image in images:
            detector = OpenCVDetector(
                n_positive_samples=self._segmentation_params.n_positive_samples,
                n_negative_samples=self._segmentation_params.n_negative_samples,
                window_name=self._segmentation_params.window_name,
            )
            samples, labels = detector.detect(image)
            probability = self._segmentor(image, np.array(samples), np.array(labels))
            segmentations.append(
                np.where(probability > self._segmentor.pth, 255, 0).astype(np.uint8)
            )
        return segmentations

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
