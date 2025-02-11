from dataclasses import dataclass
from typing import List, Union

import numpy as np
import torch
from message_filters import Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from roboreg import differentiable as rrd
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from ros2_roboreg_idl.srv import RegHydraICP

from ..data.collectables import (
    CameraInfoCollectable,
    CompressedImageCollectable,
    ImageCollectable,
)
from ..plugins.hydra_icp import HydraICPPlugin
from ..plugins.render import RenderPlugin
from ..util import QoSParams, TopicParams, qos_profile_factory
from .base import Eye2HandRegistrationBase


class MonocularDepth(Eye2HandRegistrationBase, HydraICPPlugin):
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
            RegHydraICP, "register/hydra_icp", self._on_hydra_icp
        )

    def _on_hydra_icp(
        self, req: RegHydraICP.Request, res: RegHydraICP.Response
    ) -> RegHydraICP.Response:
        res.success = True
        try:
            batch_size = len(self._data_server.collectables_history)
            self._meshes = self._meshes_factory(
                batch_size=batch_size, meshes=self._meshes
            )
            self._segment()
            depths = [
                collectables["camera.depth"].to_numpy()
                for collectables in self._data_server.collectables_history
            ]
            intrinsics = self._data_server.collectables_history[0][
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
                    with_boundary=req.with_boundary,
                    dilation_kernel_size=req.dilation_kernel_size,
                    erosion_kernel_size=req.erosion_kernel_size,
                ),
                masks=self._segmentations,
                device=self._meshes.device,
            )
            joint_states = [
                collectables["joint_states"].to_numpy()
                for collectables in self._data_server.collectables_history
            ]
            self._extrinsics = self._register_hydra_icp(
                meshes=self._meshes,
                kinematics=self._kinematics,
                joint_states=joint_states,
                pcls=pcls,
                params=self._RegistrationParams(
                    number_of_points=req.number_of_points,
                    max_distance=req.max_distance,
                    outer_max_iter=req.outer_max_iter,
                    inner_max_iter=req.inner_max_iter,
                    rmse_change=req.rmse_change,
                ),
            )
            res.message = "Registration successful"
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

    def _instantiate_render_publisher(self) -> None:
        self._render_pub = self.create_publisher(
            Image,
            self._extra_params.image_topic.name + "/render",
            qos_profile_factory(
                QoSParams(reliability="BEST_EFFORT", durability="VOLATILE")
            ),
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

    def _on_render_timer(self) -> None:
        if not self._kinematics:
            return
        if not self._render_meshes:
            return
        if not self._renderer:
            return

        # try to get most recent data
        try:
            joint_states = self._data_server.collectables["joint_states"].to_numpy()
            image_collectable: Union[ImageCollectable, CompressedImageCollectable] = (
                self._data_server.collectables["camera.image"]
            )
            camera_info_collectable: CameraInfoCollectable = (
                self._data_server.collectables["camera.image.camera_info"]
            )
        except KeyError:  # collectables not available
            return
        except TypeError:  # NoneType not subscriptable
            return

        # establish scene
        self._virtual_camera = rrd.VirtualCamera(
            (camera_info_collectable.msg.height, camera_info_collectable.msg.width),
            camera_info_collectable.to_numpy(),
            self._extrinsics,
        )

        # render
        mesh_render = RenderPlugin.render_meshes(
            meshes=self._render_meshes,
            kinematics=self._kinematics,
            camera=self._virtual_camera,
            renderer=self._renderer,
            joint_states=torch.tensor(
                joint_states, dtype=torch.float32, device=self._render_meshes.device
            ),
        )

        # overlay render
        render_overlay = RenderPlugin.overlay_render(
            image=image_collectable.to_numpy(),
            render=(mesh_render.squeeze().cpu().numpy() * 255.0).astype(np.uint8),
            color=self._renderer_params.color,
        )

        # publish
        render_overlay_msg = self._cv_bridge.cv2_to_imgmsg(
            render_overlay, encoding="bgr8", header=image_collectable.msg.header
        )
        self._render_pub.publish(render_overlay_msg)
