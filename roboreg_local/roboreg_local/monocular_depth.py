from dataclasses import dataclass
from typing import List

import numpy as np
from message_filters import Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from roboreg.registration.point_cloud.config import (
    DepthToPointCloudConfig,
    HydraConfig,
    HydraRobustICPConfig,
)
from roboreg.registration.point_cloud.request import HydraObservations, HydraRequest
from roboreg.registration.point_cloud.solver import HydraRobustICP
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from roboreg_base.base import Eye2HandRegistrationBase
from roboreg_base.util import QoSParams, TopicParams, qos_profile_factory
from roboreg_idl.srv import RegHydraRobustICP


class MonocularDepth(Eye2HandRegistrationBase):
    @dataclass
    class _ExtraParams:
        image_topic: TopicParams
        camera_info_topic: TopicParams
        depth_topic: TopicParams
        depth_camera_info_topic: TopicParams
        joint_state_topic: TopicParams

    def __init__(self, node_name: str = "eye_to_hand_calibration") -> None:
        super().__init__(node_name)

        self._hydra_icp_srv = self.create_service(
            RegHydraRobustICP, "register/hydra_robust_icp", self._on_hydra_icp
        )

    def _on_hydra_icp(
        self, req: RegHydraRobustICP.Request, res: RegHydraRobustICP.Response
    ) -> RegHydraRobustICP.Response:
        res.success = True
        try:
            self._segment()
            depths = [
                collectables["camera.depth"].to_numpy()
                for collectables in self._data_server.collectables_history
            ]
            intrinsics = self._data_server.collectables_history[0][
                "camera.depth.camera_info"
            ].to_numpy()
            joint_states = [
                collectables["joint_states"].to_numpy()
                for collectables in self._data_server.collectables_history
            ]
            registration = HydraRobustICP(
                config=HydraRobustICPConfig(
                    hydra=HydraConfig(
                        reference_points_per_mesh=req.reference_points_per_mesh,
                        depth_to_point_cloud=DepthToPointCloudConfig(
                            z_min=req.z_min,
                            z_max=req.z_max,
                            use_mask_boundary=req.use_mask_boundary,
                            dilation_kernel_size=req.dilation_kernel_size,
                            erosion_kernel_size=req.erosion_kernel_size,
                        ),
                        max_correspondence_distance=req.max_correspondence_distance,
                        rmse_change_tolerance=req.rmse_change_tolerance,
                    ),
                    max_outer_iterations=req.max_outer_iterations,
                    max_inner_iterations=req.max_inner_iterations,
                ),
            )
            result = registration(
                request=HydraRequest(
                    intrinsics=intrinsics,
                    robot_data=self._robot_data,
                    observations=HydraObservations(
                        joint_states=joint_states,
                        masks=self._segmentations,
                        depths=depths,
                    ),
                )
            )
            extrinsics = result.extrinsics.cpu().numpy()
            if np.isnan(extrinsics).any():
                raise ValueError("Registration failed: extrinsics contain NaN values.")
            self._extrinsics = extrinsics
            res.message = f"Optimization terminated after {result.iterations} iterations with status '{result.termination_reason}'."
        except Exception as e:
            res.success = False
            res.message = str(e)
            self.get_logger().error(res.message)
            return res
        return res

    def _register_synced_subscribers(self):
        qos_profile = qos_profile_factory(self._extra_params.image_topic.qos)
        self._data_server.subscribers["camera.image"] = Subscriber(
            self,
            (
                CompressedImage
                if "compressed" in self._extra_params.image_topic.name
                else Image
            ),
            self._extra_params.image_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.camera_info_topic.qos)
        self._data_server.subscribers["camera.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.camera_info_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.depth_topic.qos)
        self._data_server.subscribers["camera.depth"] = Subscriber(
            self,
            (
                CompressedImage
                if "compressed" in self._extra_params.depth_topic.name
                else Image
            ),
            self._extra_params.depth_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(
            self._extra_params.depth_camera_info_topic.qos
        )
        self._data_server.subscribers["camera.depth.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.depth_camera_info_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.joint_state_topic.qos)
        self._data_server.subscribers["joint_states"] = Subscriber(
            self,
            JointState,
            self._extra_params.joint_state_topic.name,
            qos_profile=qos_profile,
        )

    def _segment(self) -> None:
        images = [
            collectables["camera.image"].to_numpy()
            for collectables in self._data_server.collectables_history
        ]
        self._segmentations = self._segment_impl(images)

    def _declare_extra_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.image.name", "/camera/image_rect_color"),
                ("topics.image.qos.reliability", "BEST_EFFORT"),
                ("topics.image.qos.durability", "VOLATILE"),
                (
                    "topics.image.camera_info.name",
                    "/camera/image_rect_color/camera_info",
                ),
                ("topics.image.camera_info.qos.reliability", "BEST_EFFORT"),
                ("topics.image.camera_info.qos.durability", "VOLATILE"),
                ("topics.depth.name", "/camera/depth_registered"),
                ("topics.depth.qos.reliability", "BEST_EFFORT"),
                ("topics.depth.qos.durability", "VOLATILE"),
                (
                    "topics.depth.camera_info.name",
                    "/camera/depth_registered/camera_info",
                ),
                ("topics.depth.camera_info.qos.reliability", "BEST_EFFORT"),
                ("topics.depth.camera_info.qos.durability", "VOLATILE"),
                ("topics.joint_state.name", "/joint_states"),
                ("topics.joint_state.qos.reliability", "BEST_EFFORT"),
                ("topics.joint_state.qos.durability", "VOLATILE"),
            ],
        )

    def _get_extra_parameters(self):
        self._extra_params = self._ExtraParams(
            image_topic=TopicParams(
                name=self.get_parameter("topics.image.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter("topics.image.qos.reliability")
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter("topics.image.qos.durability")
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            camera_info_topic=TopicParams(
                name=self.get_parameter("topics.image.camera_info.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter(
                        "topics.image.camera_info.qos.reliability"
                    )
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter(
                        "topics.image.camera_info.qos.durability"
                    )
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            depth_topic=TopicParams(
                name=self.get_parameter("topics.depth.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter("topics.depth.qos.reliability")
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter("topics.depth.qos.durability")
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            depth_camera_info_topic=TopicParams(
                name=self.get_parameter("topics.depth.camera_info.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter(
                        "topics.depth.camera_info.qos.reliability"
                    )
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter(
                        "topics.depth.camera_info.qos.durability"
                    )
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            joint_state_topic=TopicParams(
                name=self.get_parameter("topics.joint_state.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter("topics.joint_state.qos.reliability")
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter("topics.joint_state.qos.durability")
                    .get_parameter_value()
                    .string_value,
                ),
            ),
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
                self._extra_params.joint_state_topic.name = parameter.value
                self._reload_synced_subscribers()
            else:
                continue
        return result
