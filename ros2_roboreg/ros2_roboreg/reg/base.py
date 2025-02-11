import copy
import pathlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional

import cv_bridge
import numpy as np
import torch
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from roboreg import differentiable as rrd
from roboreg.detector import OpenCVDetector
from roboreg.io import URDFParser
from roboreg.segmentor import Sam2Segmentor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_roboreg_idl.srv import Export, Import

from ..broadcaster import StaticTFBroadcaster
from ..data.server import Server
from ..util import QoSParams, TopicParams, qos_profile_factory


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

    @dataclass
    class _RendererParams:
        enabled: bool
        color: str
        rate: float

    @dataclass
    class _TFBroadcasterParams:
        parent_frame: str
        child_frame: str
        target_child_frame: str

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        # common parameters
        self._declare_common_parameters()
        self._get_common_parameters()

        # extra parameters
        self._declare_extra_parameters()
        self._get_extra_parameters()

        # parameter callbacks
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # data collections
        self._data_server = Server(node=self)
        self._reload_synced_subscribers()

        # publishers
        self._render_timer: Timer = None
        self._render_pub: Publisher = None
        if self._renderer_params.enabled:
            self._render_timer = self.create_timer(
                1.0 / self._renderer_params.rate, self._on_render_timer
            )
            self._instantiate_render_publisher()

        # results broadcasting
        self._extrinsics = np.eye(4)
        self._tf_broadcaster = StaticTFBroadcaster(node=self)
        self._tf_broadcast_srv = self.create_service(
            Trigger, "broadcast_transform", self._on_tf_broadcast
        )
        self._tf_export_srv = self.create_service(
            Export, "export/transform", self._on_export_transform
        )
        self._tf_import_srv = self.create_service(
            Import, "import/transform", self._on_import_transform
        )

        # common registration utilities
        self.get_logger().info(
            "Instantiating segmentation model. This may take a while..."
        )
        self._segmentor = Sam2Segmentor(
            model_id=self._segmentation_params.model_id,
            pth=self._segmentation_params.pth,
            device=self._segmentation_params.device,
        )
        self.get_logger().info("Segmentation model instantiated.")
        self._segmentations = []
        self._urdf_parser: URDFParser = URDFParser()
        self._kinematics: rrd.TorchKinematics = None
        self._meshes: rrd.TorchMeshContainer = (
            None  # requires configuration based on batch size (number of synced samples)
        )
        self._cv_bridge = cv_bridge.CvBridge()
        self._render_meshes: rrd.TorchMeshContainer = None
        self._renderer: rrd.NVDiffRastRenderer = None
        self._virtual_camera: rrd.VirtualCamera = None
        if self._renderer_params.enabled:
            self._renderer = rrd.NVDiffRastRenderer()
        self._robot_description_sub = None
        self._create_robot_description_sub()

    def _create_robot_description_sub(self) -> None:
        if self._robot_description_sub is not None:
            self.destroy_subscription(self._robot_description_sub)
        qos_profile = qos_profile_factory(self._robot_description_topic.qos)
        self._robot_description_sub = self.create_subscription(
            String,
            self._robot_description_topic.name,
            self._on_robot_description,
            qos_profile,
        )

    def _on_tf_broadcast(self, _, res: Trigger.Response) -> Trigger.Response:
        try:
            self._tf_broadcaster.broadcast_tf(
                ht=self._extrinsics,
                parent=self._tf_broadcaster_params.parent_frame,
                child=self._tf_broadcaster_params.child_frame,
                target_child=self._tf_broadcaster_params.target_child_frame,
            )
            res.success = True
            res.message = "Broadcasted transform."
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
        return res

    def _on_export_transform(
        self, req: Export.Request, res: Export.Response
    ) -> Export.Response:
        path = pathlib.Path(req.path)
        self.get_logger().info(f"Saving transform to {path.absolute()}")
        res.success = True
        try:
            if not path.parent.exists():
                if req.mkdir:
                    path.parent.mkdir(parents=True, exist_ok=True)
                else:
                    res.success = False
                    res.message = f"Path {path.parent.absolute()} does not exist"
                    self.get_logger().error(res.message)
                    return res
            np.save(path, self._extrinsics)
            res.message = f"Saved transform to {path.absolute()}"
            self.get_logger().info(res.message)
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
        return res

    def _on_import_transform(
        self, req: Import.Request, res: Import.Response
    ) -> Import.Response:
        res.success = True
        path = pathlib.Path(req.path)
        try:
            if not path.exists():
                res.success = False
                res.message = f"Path {path.absolute()} does not exist"
                self.get_logger().error(res.message)
                return res
            self.get_logger().info(f"Loading transform from {req.path}")
            ht = np.load(path.absolute())
            if ht.shape != (4, 4):
                res.success = False
                res.message = f"Transform has wrong shape: {ht.shape}"
                self.get_logger().error(res.message)
                return res
            self._extrinsics = copy.deepcopy(ht)
        except Exception as e:
            res.success = False
            res.message = f"Failed service call with: {e}"
            self.get_logger().error(res.message)
        return res

    def _on_robot_description(self, msg: String) -> None:
        try:
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
            self.get_logger().info("Kinematics instantiated.")

            if self._renderer_params.enabled:
                self.get_logger().info(
                    "Instantiating render meshes on robot description."
                )
                self._render_meshes = self._meshes_factory(
                    batch_size=1, meshes=self._render_meshes
                )
                self.get_logger().info("Render meshes instantiated.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")

    def _meshes_factory(
        self, batch_size: int, meshes: Optional[rrd.TorchMeshContainer] = None
    ) -> rrd.TorchMeshContainer:
        if self._kinematics is None:
            raise ValueError("Kinematics not instantiated.")
        if self._urdf_parser is None:
            raise ValueError("URDF parser not instantiated.")
        if meshes is not None:
            if meshes.batch_size == batch_size:
                return meshes
        return rrd.TorchMeshContainer(
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
                ("topics.robot_description.name", "/robot_description"),
                ("topics.robot_description.qos.reliability", "RELIABLE"),
                ("topics.robot_description.qos.durability", "TRANSIENT_LOCAL"),
            ],
        )
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
            namespace="",
            parameters=[
                ("renderer.enabled", False),
                ("renderer.color", "b"),
                ("renderer.rate", 10.0),
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("tf_broadcaster.parent_frame", "world"),
                ("tf_broadcaster.child_frame", "camera_frame"),
                ("tf_broadcaster.target_child_frame", "camera_link"),
            ],
        )

    def _get_common_parameters(self) -> None:
        self._robot_description_topic = TopicParams(
            name=self.get_parameter("topics.robot_description.name")
            .get_parameter_value()
            .string_value,
            qos=QoSParams(
                reliability=self.get_parameter(
                    "topics.robot_description.qos.reliability"
                )
                .get_parameter_value()
                .string_value,
                durability=self.get_parameter("topics.robot_description.qos.durability")
                .get_parameter_value()
                .string_value,
            ),
        )
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
        self._renderer_params = self._RendererParams(
            enabled=self.get_parameter("renderer.enabled")
            .get_parameter_value()
            .bool_value,
            color=self.get_parameter("renderer.color")
            .get_parameter_value()
            .string_value,
            rate=self.get_parameter("renderer.rate").get_parameter_value().double_value,
        )
        if not torch.cuda.is_available():  # renderer only runs on GPU
            self.get_logger().info("CUDA not available. Renderer will be disabled.")
            self._renderer_params.enabled = False
        self._tf_broadcaster_params = self._TFBroadcasterParams(
            parent_frame=self.get_parameter("tf_broadcaster.parent_frame")
            .get_parameter_value()
            .string_value,
            child_frame=self.get_parameter("tf_broadcaster.child_frame")
            .get_parameter_value()
            .string_value,
            target_child_frame=self.get_parameter("tf_broadcaster.target_child_frame")
            .get_parameter_value()
            .string_value,
        )

    def _on_set_common_parameters_impl(
        self, paramaters: List[Parameter]
    ) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        for parameter in paramaters:
            if parameter.name == "topics.robot_description.name":
                self.get_logger().info(
                    f"Setting robot description topic to {parameter.value}"
                )
                self._robot_description_topic.name = parameter.value
                self._create_robot_description_sub()
            else:
                continue
        return result

    def _on_set_parameters(self, paramaters: List[Parameter]) -> SetParametersResult:
        result = self._on_set_common_parameters_impl(paramaters)
        if not result.successful:
            return result
        result = self._on_set_extra_parameters_impl(paramaters)
        return result

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

    def _reload_synced_subscribers(self) -> None:
        self._data_server.subscribers = {}
        self._register_synced_subscribers()
        self._data_server.initialize(accuracy=self._filter_params.sync_accuracy)

    @abstractmethod
    def _register_synced_subscribers(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _instantiate_render_publisher(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _on_set_extra_parameters_impl(
        self, paramaters: List[Parameter]
    ) -> SetParametersResult:
        raise NotImplementedError

    @abstractmethod
    def _on_render_timer(self) -> None:
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
