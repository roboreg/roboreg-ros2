from dataclasses import dataclass

from message_filters import Subscriber
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy, qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image, JointState

from .base import Eye2HandRegistrationBase


class StereoDepth(Eye2HandRegistrationBase):
    @dataclass
    class _ExtraParams:
        left_image_topic: str
        left_image_qos_reliability: str
        left_camera_info_topic: str
        left_camera_info_qos_reliability: str
        right_image_topic: str
        right_image_qos_reliability: str
        right_camera_info_topic: str
        right_camera_info_qos_reliability: str
        depth_topic: str
        depth_qos_reliability: str
        depth_camera_info_topic: str
        depth_camera_info_qos_reliability: str
        joint_state_topic: str
        joint_state_qos_reliability: str

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
            ReliabilityPolicy, self._extra_params.left_image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.left.image"] = Subscriber(
            self,
            Image,
            self._extra_params.left_image_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.left_camera_info_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.left.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.left_camera_info_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.right_image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.right.image"] = Subscriber(
            self,
            Image,
            self._extra_params.right_image_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.right_image_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
        self._data_server.subscribers["camera.right.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.right_camera_info_topic,
            qos_profile=qos_profile,
        )
        qos_profile = qos_profile_system_default
        qos_profile.reliability = getattr(
            ReliabilityPolicy, self._extra_params.depth_qos_reliability
        )
        qos_profile.durability = DurabilityPolicy.VOLATILE
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
        super()._segment()

    def _declare_extra_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.left_image.name", "camera/left/image_rect_color"),
                ("topics.left_image.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.left_image.camera_info.name",
                    "camera/left/image_rect_color/camera_info",
                ),
                ("topics.left_image.camera_info.qos_reliability", "BEST_EFFORT"),
                ("topics.right_image.name", "camera/right/image_rect_color"),
                ("topics.right_image.qos_reliability", "BEST_EFFORT"),
                (
                    "topics.right_image.camera_info.name",
                    "camera/right/image_rect_color/camera_info",
                ),
                ("topics.right_image.camera_info.qos_reliability", "BEST_EFFORT"),
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
            left_image_topic=self.get_parameter("topics.left_image.name")
            .get_parameter_value()
            .string_value,
            left_image_qos_reliability=self.get_parameter(
                "topics.left_image.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            left_camera_info_topic=self.get_parameter(
                "topics.left_image.camera_info.name"
            )
            .get_parameter_value()
            .string_value,
            left_camera_info_qos_reliability=self.get_parameter(
                "topics.left_image.camera_info.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            right_image_topic=self.get_parameter("topics.right_image.name")
            .get_parameter_value()
            .string_value,
            right_image_qos_reliability=self.get_parameter(
                "topics.right_image.qos_reliability"
            )
            .get_parameter_value()
            .string_value,
            right_camera_info_topic=self.get_parameter(
                "topics.right_image.camera_info.name"
            )
            .get_parameter_value()
            .string_value,
            right_camera_info_qos_reliability=self.get_parameter(
                "topics.right_image.camera_info.qos_reliability"
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
