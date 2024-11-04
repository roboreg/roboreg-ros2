from dataclasses import dataclass

from message_filters import Subscriber
from sensor_msgs.msg import CameraInfo, Image, JointState

from ..util import QoSParams, TopicParams, qos_profile_factory
from .base import Eye2HandRegistrationBase


class StereoDepth(Eye2HandRegistrationBase):
    @dataclass
    class _ExtraParams:
        left_image_topic: TopicParams
        left_camera_info_topic: TopicParams
        right_image_topic: TopicParams
        right_camera_info_topic: TopicParams
        depth_topic: TopicParams
        depth_camera_info_topic: TopicParams
        joint_state_topic: TopicParams

    def _register_synced_subscribers(self):
        qos_profile = qos_profile_factory(self._extra_params.left_image_topic.qos)
        self._data_server.subscribers["camera.left.image"] = Subscriber(
            self,
            Image,
            self._extra_params.left_image_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.left_camera_info_topic.qos)
        self._data_server.subscribers["camera.left.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.left_camera_info_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.right_image_topic.qos)
        self._data_server.subscribers["camera.right.image"] = Subscriber(
            self,
            Image,
            self._extra_params.right_image_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(
            self._extra_params.right_camera_info_topic.qos
        )
        self._data_server.subscribers["camera.right.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.right_camera_info_topic.name,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_factory(self._extra_params.depth_topic.qos)
        self._data_server.subscribers["camera.depth"] = Subscriber(
            self,
            Image,
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
        super()._segment()

    def _declare_extra_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.left_image.name", "camera/left/image_rect_color"),
                ("topics.left_image.qos.reliability", "BEST_EFFORT"),
                ("topics.left_image.qos.durability", "VOLATILE"),
                (
                    "topics.left_image.camera_info.name",
                    "camera/left/image_rect_color/camera_info",
                ),
                ("topics.left_image.camera_info.qos.reliability", "BEST_EFFORT"),
                ("topics.left_image.camera_info.qos.durability", "VOLATILE"),
                ("topics.right_image.name", "camera/right/image_rect_color"),
                ("topics.right_image.qos.reliability", "BEST_EFFORT"),
                ("topics.right_image.qos.durability", "VOLATILE"),
                (
                    "topics.right_image.camera_info.name",
                    "camera/right/image_rect_color/camera_info",
                ),
                ("topics.right_image.camera_info.qos.reliability", "BEST_EFFORT"),
                ("topics.right_image.camera_info.qos.durability", "VOLATILE"),
                ("topics.depth.name", "camera/depth_registered"),
                ("topics.depth.qos.reliability", "BEST_EFFORT"),
                ("topics.depth.qos.durability", "VOLATILE"),
                (
                    "topics.depth.camera_info.name",
                    "camera/depth_registered/camera_info",
                ),
                ("topics.depth.camera_info.qos.reliability", "BEST_EFFORT"),
                ("topics.depth.camera_info.qos.durability", "VOLATILE"),
                ("topics.joint_state.name", "joint_states"),
                ("topics.joint_state.qos.reliability", "BEST_EFFORT"),
                ("topics.joint_state.qos.durability", "VOLATILE"),
            ],
        )

    def _get_extra_parameters(self):
        self._extra_params = self._ExtraParams(
            left_image_topic=TopicParams(
                name=self.get_parameter("topics.left_image.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter("topics.left_image.qos.reliability")
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter("topics.left_image.qos.durability")
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            left_camera_info_topic=TopicParams(
                name=self.get_parameter("topics.left_image.camera_info.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter(
                        "topics.left_image.camera_info.qos.reliability"
                    )
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter(
                        "topics.left_image.camera_info.qos.durability"
                    )
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            right_image_topic=TopicParams(
                name=self.get_parameter("topics.right_image.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter("topics.right_image.qos.reliability")
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter("topics.right_image.qos.durability")
                    .get_parameter_value()
                    .string_value,
                ),
            ),
            right_camera_info_topic=TopicParams(
                name=self.get_parameter("topics.right_image.camera_info.name")
                .get_parameter_value()
                .string_value,
                qos=QoSParams(
                    reliability=self.get_parameter(
                        "topics.right_image.camera_info.qos.reliability"
                    )
                    .get_parameter_value()
                    .string_value,
                    durability=self.get_parameter(
                        "topics.right_image.camera_info.qos.durability"
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
