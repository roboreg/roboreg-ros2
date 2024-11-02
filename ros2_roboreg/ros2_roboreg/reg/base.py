import copy
import pathlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

import numpy as np
import torch
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node
from roboreg import differentiable as rrd
from roboreg.detector import OpenCVDetector
from roboreg.io import URDFParser
from roboreg.segmentor import Sam2Segmentor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_roboreg_idl.srv import Export, Import

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

    @dataclass
    class TFBroadcasterParams:
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

        # results broadcasting
        self._ht = np.eye(4)
        self._tf_broadcaster = StaticTFBroadcaster(node=self)
        self._tf_broadcast_srv = self.create_service(
            Trigger, "~/broadcast_transform", self._on_tf_broadcast
        )
        self._tf_export_srv = self.create_service(
            Export, "~/export/transform", self._on_export_transform
        )
        self._tf_import_srv = self.create_service(
            Import, "~/import/transform", self._on_import_transform
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
        self._kinematics: rrd.TorchKinematics = None
        self._meshes: rrd.TorchMeshContainer = (
            None  # requires configuration based on batch size (number of synced samples)
        )
        self._urdf_parser: URDFParser = URDFParser()
        self._robot_description_sub = None
        self._create_robot_description_sub()

    def _create_robot_description_sub(self) -> None:
        if self._robot_description_sub is not None:
            self.destroy_subscription(self._robot_description_sub)
        self._robot_description_sub = self.create_subscription(
            String, self._robot_description_topic, self._on_robot_description, 1
        )

    def _on_tf_broadcast(self, _, res: Trigger.Response) -> Trigger.Response:
        try:
            self._tf_broadcaster.broadcast_tf(
                ht=self._ht,
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
            np.save(path, self._ht)
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
            self._ht = copy.deepcopy(ht)
        except Exception as e:
            res.success = False
            res.message = f"Failed service call with: {e}"
            self.get_logger().error(res.message)
        return res

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
                ("topics.robot_description.name", "/robot_description"),
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
            namespace="tf_broadcaster",
            parameters=[
                ("parent_frame", "world"),
                ("child_frame", "camera_frame"),
                ("target_child_frame", "camera_link"),
            ],
        )

    def _get_common_parameters(self) -> None:
        self._robot_description_topic = (
            self.get_parameter("topics.robot_description.name")
            .get_parameter_value()
            .string_value
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
        self._tf_broadcaster_params = self.TFBroadcasterParams(
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
                self._robot_description_topic = parameter.value
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
    def _on_set_extra_parameters_impl(
        self, paramaters: List[Parameter]
    ) -> SetParametersResult:
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
