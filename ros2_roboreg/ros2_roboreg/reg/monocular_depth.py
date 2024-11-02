from dataclasses import dataclass
from typing import List

from message_filters import Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_system_default
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from ros2_roboreg_idl.srv import RegHydraICP

from ..plugins.hydra_icp import HydraICP
from .base import Eye2HandRegistrationBase


class MonocularDepth(Eye2HandRegistrationBase, HydraICP):
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

    def __init__(self, node_name: str = "eye_to_hand_calibration") -> None:
        super().__init__(node_name)

        self._hydra_icp_srv = self.create_service(
            RegHydraICP, "~/register/hydra_icp", self._on_hydra_icp
        )

    def _on_hydra_icp(
        self, req: RegHydraICP.Request, res: RegHydraICP.Response
    ) -> RegHydraICP.Response:
        try:
            batch_size = len(self._data_server._collectables_history)
            self._instantiate_meshes(batch_size=batch_size)
            self._segment()
            depths = [
                collectables["camera.depth"].to_numpy()
                for collectables in self._data_server._collectables_history
            ]
            intrinsics = self._data_server._collectables_history[0][
                "camera.depth.camera_info"
            ].to_numpy()
            pcls = self._depths_to_pcls(
                depths=depths,
                intrinsics=intrinsics,
                z_min=self._filter_params.min_depth,
                z_max=self._filter_params.max_depth,
                device=self._meshes.device,
            )
            pcls = self._process_pcls(
                pcls=pcls,
                params=self._ProcessParams(
                    with_erosion=req.with_erosion,
                    erosion_kernel_size=req.erosion_kernel_size,
                ),
                masks=self._segmentations,
                device=self._meshes.device,
            )
            self._ht = self._register_hydra_icp(
                meshes=self._meshes,
                kinematics=self._kinematics,
                joint_states=self._data_server._collectables_history[0][
                    "joint_states"
                ].to_numpy(),
                pcls=pcls,
                params=self._RegistrationParams(
                    number_of_points=req.number_of_points,
                    max_distance=req.max_distance,
                    outer_max_iter=req.outer_max_iter,
                    inner_max_iter=req.inner_max_iter,
                    rmse_change=req.rmse_change,
                ),
            )
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
            return res
        return res

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
        if "compressed" in self._extra_params.image_topic:
            self._data_server.subscribers["camera.image"] = Subscriber(
                self,
                CompressedImage,
                self._extra_params.image_topic,
                qos_profile=qos_profile,
            )
        else:
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
        if "compressed" in self._extra_params.depth_topic:
            self._data_server.subscribers["camera.depth"] = Subscriber(
                self,
                CompressedImage,
                self._extra_params.depth_topic,
                qos_profile=qos_profile,
            )
        else:
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

    def _segment(self) -> None:
        images = [
            collectables["camera.image"].to_numpy()
            for collectables in self._data_server._collectables_history
        ]
        self._segmentations = self._segment_impl(images)

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

    def _on_set_extra_parameters_impl(
        self, paramaters: List[Parameter]
    ) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        for parameter in paramaters:
            if parameter.name == "topics.joint_state.name":
                self.get_logger().info(
                    f"Setting joint state topic to {parameter.value}"
                )
                self._extra_params.joint_state_topic = parameter.value
                self._reload_synced_subscribers()
            else:
                continue
        return result
