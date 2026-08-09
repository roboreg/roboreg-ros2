import copy
import pathlib
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List

import cv_bridge
import numpy as np
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node
from roboreg.core.robot import RobotData
from roboreg.io import URDFParser, apply_mesh_origins, load_meshes, simplify_meshes
from std_msgs.msg import String
from std_srvs.srv import Trigger

from roboreg_idl.srv import Export, Import

from .broadcaster import StaticTFBroadcaster
from .data.server import Server
from .util import QoSParams, TopicParams, qos_profile_factory


class RoboregNode(Node, ABC):
    @dataclass
    class _FilterParams:
        sync_accuracy: float

    @dataclass
    class _RobotDataParams:
        root_link_name: str
        end_link_name: str
        collision_meshes: bool
        target_reduction: float

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
        self._robot_data: RobotData = None
        self._cv_bridge = cv_bridge.CvBridge()
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
            urdf_parser = URDFParser(msg.data)
            if self._robot_data_params.root_link_name == "":
                self._robot_data_params.root_link_name = (
                    urdf_parser.link_names_with_meshes(
                        collision=self._robot_data_params.collision_meshes
                    )[0]
                )
                self.get_logger().info(
                    f"No root link name specified. Using first link with mesh: {self._robot_data_params.root_link_name}"
                )
            if self._robot_data_params.end_link_name == "":
                self._robot_data_params.end_link_name = (
                    urdf_parser.link_names_with_meshes(
                        collision=self._robot_data_params.collision_meshes
                    )[-1]
                )
                self.get_logger().info(
                    f"No end link name specified. Using last link with mesh: {self._robot_data_params.end_link_name}"
                )

            # parse data from URDF
            mesh_paths = urdf_parser.mesh_paths_from_ros_registry(
                root_link_name=self._robot_data_params.root_link_name,
                end_link_name=self._robot_data_params.end_link_name,
                collision=self._robot_data_params.collision_meshes,
            )

            mesh_origins = urdf_parser.mesh_origins(
                root_link_name=self._robot_data_params.root_link_name,
                end_link_name=self._robot_data_params.end_link_name,
                collision=self._robot_data_params.collision_meshes,
            )

            # load and preprocess meshes
            meshes = load_meshes(paths=mesh_paths)
            meshes = simplify_meshes(
                meshes=meshes,
                target_reduction=self._robot_data_params.target_reduction,
            )
            meshes = apply_mesh_origins(meshes=meshes, origins=mesh_origins)

            # instantiate robot data
            self._robot_data = RobotData(
                meshes=meshes,
                urdf=urdf_parser.urdf,
                root_link_name=self._robot_data_params.root_link_name,
                end_link_name=self._robot_data_params.end_link_name,
            )

        except Exception as e:
            self.get_logger().error(f"Failed to instantiate robot data: {e}")

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
            ],
        )
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_data.root_link_name", ""),
                ("robot_data.end_link_name", ""),
                ("robot_data.collision_meshes", False),
                ("robot_data.target_reduction", 0.0),
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
        )
        self._robot_data_params = self._RobotDataParams(
            root_link_name=self.get_parameter("robot_data.root_link_name")
            .get_parameter_value()
            .string_value,
            end_link_name=self.get_parameter("robot_data.end_link_name")
            .get_parameter_value()
            .string_value,
            collision_meshes=self.get_parameter("robot_data.collision_meshes")
            .get_parameter_value()
            .bool_value,
            target_reduction=self.get_parameter("robot_data.target_reduction")
            .get_parameter_value()
            .double_value,
        )
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
    def _declare_extra_parameters(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _get_extra_parameters(self) -> None:
        raise NotImplementedError
