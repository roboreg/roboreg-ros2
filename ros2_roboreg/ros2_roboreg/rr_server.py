import copy
import pathlib
from abc import ABC, abstractmethod
from typing import List, Tuple

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
from roboreg.io import URDFParser
from roboreg.segmentor import Sam2Segmentor
from roboreg.util import (
    depth_to_xyz,
    from_homogeneous,
    generate_ht_optical,
    to_homogeneous,
)
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_roboreg.broadcaster import StaticTFBroadcaster
from ros2_roboreg.structs import ServerParams, SyncedSample
from ros2_roboreg_idl.srv import CollectSample, Export, Import


class RoboregServer(Node, ABC):
    def __init__(self, node_name: str = "roboreg") -> None:
        super().__init__(node_name)

        # synced samples
        self._instantiate_synced_samples()

        # segmentation
        self._detector: OpenCVDetector = None
        self._segmentor: Sam2Segmentor = None

        # point cloud as obtained from depth
        self._pcls = []

        # opencv bridge
        self._bridge = cv_bridge.CvBridge()

        # tf broadcaster
        self._ht = np.eye(4)
        self._tf_broadcaster = StaticTFBroadcaster(self)

        # parameters
        self._instantiate_server_params()

        # node parameters
        self._declare_node_parameters()
        self._get_node_parameters()
        self._log_node_parameters()

        # parameter callback
        self.add_on_set_parameters_callback(self._on_set_parameter)

        # robot model
        self._urdf_parser: URDFParser = None
        self._kinematics: rrd.TorchKinematics = None
        self._meshes: rrd.TorchMeshContainer = None

        # subscriptions
        self._joint_state_sub: Subscriber = None
        self._depth_sub: Subscriber = None
        self._approximate_time_sync: ApproximateTimeSynchronizer = None
        self._create_subscriptions()

        # services
        self._create_services()

    def _instantiate_synced_samples(self) -> Tuple[SyncedSample, List[SyncedSample]]:
        self._synced_sample: SyncedSample = None
        self._synced_samples: List[SyncedSample] = None

    def _instantiate_server_params(self) -> None:
        self._params: ServerParams = ServerParams()

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
        segment_idx = len(self._segmentations)
        if segment_idx >= len(self._synced_samples):
            self.get_logger().info("Segmentation already done")
            return

        # segment the images
        self._detector = OpenCVDetector(
            n_positive_samples=self._params.segmentation.n_positive_samples,
            n_negative_samples=self._params.segmentation.n_negative_samples,
            window_name="Image Detector",
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
        while segment_idx < len(self._synced_samples):
            synced_sample = self._synced_samples[segment_idx]
            for image in synced_sample.images:  # TODO: make this batch-wise....
                image = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                samples, labels = self._detector.detect(image)
                probability = segmentor(image, np.array(samples), np.array(labels))
                mask = np.where(probability > segmentor.pth, 255, 0).astype(np.uint8)
                self._segmentations.append((mask * 255.0).astype(np.uint8))
            segment_idx += 1
            self.get_logger().info(
                f"Annotated [{segment_idx}/{len(self._synced_samples)}] image pairs"
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
            self._bridge.imgmsg_to_cv2(synced_sample.depth.image)
            for synced_sample in self._synced_samples
        ]
        depths = torch.tensor(
            np.array(depths),
            dtype=torch.float32,
            device=self._params.robot_model.device,
        )
        intrinsics = [
            np.array(synced_sample.depth.camera_info.k).reshape(3, 3)
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
            self._synced_samples[0].depth.camera_info.height,
            self._synced_samples[0].depth.camera_info.width,
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

        # turn pcls back into list of numpy arrays
        self._pcls = [pcl.cpu().numpy() for pcl in pcls]

    def _declare_filter_node_parameters(self) -> None:
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

    @abstractmethod
    def _declare_camera_topic_node_parameters(self) -> None:
        raise NotImplementedError

    def _declare_topic_node_parameters(self) -> None:
        self._declare_camera_topic_node_parameters()
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.joint_state.name", "joint_state"),
                ("topics.joint_state.qos_reliability", "RELIABLE"),
                ("topics.depth.name", "camera/depth/registered"),
                ("topics.depth.qos_reliability", "RELIABLE"),
                (
                    "topics.depth.camera_info.name",
                    "camera/depth/registered/camera_info",
                ),
                ("topics.depth.camera_info.qos_reliability", "RELIABLE"),
                ("topics.robot_description.name", "robot_description"),
            ],
        )

    def _declare_segmentation_node_parameters(self) -> None:
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

    def _declare_robot_model_node_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("robot_model.device", "cuda" if torch.cuda.is_available() else "cpu"),
                ("robot_model.root_link_name", ""),
                ("robot_model.end_link_name", ""),
                ("robot_model.visual_meshes", False),
            ],
        )

    def _declare_tf_broadcaster_node_parameters(self) -> None:
        self.declare_parameters(
            namespace="",
            parameters=[
                ("tf_broadcaster.parent_frame", "world"),
                ("tf_broadcaster.child_frame", ""),
                ("tf_broadcaster.target_child_frame", ""),
            ],
        )

    def _declare_node_parameters(self) -> None:
        self._declare_filter_node_parameters()
        self._declare_topic_node_parameters()
        self._declare_segmentation_node_parameters()
        self._declare_robot_model_node_parameters()
        self._declare_tf_broadcaster_node_parameters()

    def _get_filter_node_parameters(self) -> None:
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

    @abstractmethod
    def _get_camera_topic_node_parameters(self) -> None:
        raise NotImplementedError

    def _get_topic_node_parameters(self) -> None:
        self._get_camera_topic_node_parameters()
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

    def _get_segmentation_node_parameters(self) -> None:
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

    def _get_robot_model_node_parameters(self) -> None:
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

    def _get_tf_broadcaster_node_parameters(self) -> None:
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

    def _get_node_parameters(self) -> None:
        self._get_filter_node_parameters()
        self._get_topic_node_parameters()
        self._get_segmentation_node_parameters()
        self._get_robot_model_node_parameters()
        self._get_tf_broadcaster_node_parameters()

    def _log_filter_node_parameters(self) -> None:
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

    @abstractmethod
    def _log_camera_topic_node_parameters(self) -> None:
        raise NotImplementedError

    def _log_topic_node_parameters(self) -> None:
        self.get_logger().info(f"*{' '*5}Topics:")
        self._log_camera_topic_node_parameters()
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

    def _log_segmentation_node_parameters(self) -> None:
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

    def _log_robot_model_node_parameters(self) -> None:
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

    def _log_tf_broadcaster_node_parameters(self) -> None:
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

    def _log_node_parameters(self) -> None:
        self.get_logger().info(f"*** Parameters:")
        self._log_filter_node_parameters()
        self._log_topic_node_parameters()
        self._log_segmentation_node_parameters()
        self._log_robot_model_node_parameters()
        self._log_tf_broadcaster_node_parameters()
        self.get_logger().info("***")

    @abstractmethod
    def _on_set_camera_topic_parameter(
        self, parameter: Parameter, result: SetParametersResult
    ) -> None:
        raise NotImplementedError

    def _on_set_parameter(self, parameters: List[Parameter]) -> SetParametersResult:
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            self._on_set_camera_topic_parameter(parameter, result)
            if parameter.name == "topics.joint_state.name":
                self.get_logger().info(
                    f"Setting joint state topic to {parameter.value}"
                )
                self._params.joint_state_topic.name = parameter.value
                self._create_joint_state_subscription()
                self._create_approximate_time_sync()
                self._register_on_sync()
            elif parameter.name == "topics.depth.name":
                self.get_logger().info(f"Setting depth topic to {parameter.value}")
                self._params.depth_topic.name = parameter.value
                self._create_depth_subscription()
                self._create_approximate_time_sync()
                self._register_on_sync()
            elif parameter.name == "topics.robot_description.name":
                self.get_logger().info(
                    f"Setting robot description topic to {parameter.value}"
                )
                self._params.robot_description_topic.name = parameter.value
                self._create_robot_description_subscription()
            else:
                continue
        return result

    @abstractmethod
    def _create_registration_services(self) -> None:
        raise NotImplementedError

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
        self._create_registration_services()
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

    @abstractmethod
    def _create_camera_subscriptions(self) -> None:
        raise NotImplementedError

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

    @abstractmethod
    def _create_approximate_time_sync(self) -> None:
        raise NotImplementedError

    def _register_on_sync(self) -> None:
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
        self._create_camera_subscriptions()
        self._create_joint_state_subscription()
        self._create_depth_subscription()

        # topic synchronizer
        self._create_approximate_time_sync()
        self._register_on_sync()

        # robot description
        self._create_robot_description_subscription()

    @abstractmethod
    def _on_sync(
        self,
    ) -> None:
        raise NotImplementedError

    def _on_robot_description(self, msg: String) -> None:
        self.get_logger().info("Received robot description.")
        # instantiate urdf parser
        self._urdf_parser = URDFParser()
        self._urdf_parser.from_urdf(msg.data)

    @abstractmethod
    def _on_collect_sample(
        self, request: CollectSample.Request, response: CollectSample.Response
    ) -> CollectSample.Response:
        raise NotImplementedError

    def _on_clear_samples(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self._synced_samples.clear()
        self._detector.clear()
        self._pcls.clear()
        response.success = True
        response.message = "Cleared all samples"
        self.get_logger().info(response.message)
        return response

    @abstractmethod
    def _on_export_samples(
        self, request: Export.Request, response: Export.Response
    ) -> Export.Response:
        raise NotImplementedError

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
            np.savetxt(path, self._ht)
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
            self._ht = copy.deepcopy(HT)
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
                self._ht,
                parent=self._params.tf_broadcaster.parent_frame,
                child=self._params.tf_broadcaster.child_frame,
                target_child=self._params.tf_broadcaster.target_child_frame,
            )
        except Exception as e:
            response.success = False
            response.message = f"Failed service call with: {e}"
            self.get_logger().error(response.message)
        return response
