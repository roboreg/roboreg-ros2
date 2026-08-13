from dataclasses import dataclass

from message_filters import Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from .parameters import QoSParams, TopicParams
from .qos_profile_factory import qos_profile_factory
from .roboreg_node import RoboregNode


class MonocularDepthNode(RoboregNode):
    @dataclass
    class _ExtraParams:
        image_topic: TopicParams
        camera_info_topic: TopicParams
        depth_topic: TopicParams
        depth_camera_info_topic: TopicParams
        joint_state_topic: TopicParams

    def _register_synced_subscribers(self):
        qos_profile = qos_profile_factory(self._extra_params.image_topic.qos)
        self._data_collector.subscribers["camera.image"] = Subscriber(
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
        self._data_collector.subscribers["camera.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.camera_info_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.depth_topic.qos)
        self._data_collector.subscribers["camera.depth"] = Subscriber(
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
        self._data_collector.subscribers["camera.depth.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.depth_camera_info_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.joint_state_topic.qos)
        self._data_collector.subscribers["joint_states"] = Subscriber(
            self,
            JointState,
            self._extra_params.joint_state_topic.name,
            qos_profile=qos_profile,
        )

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
        self, paramaters: list[Parameter]
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
