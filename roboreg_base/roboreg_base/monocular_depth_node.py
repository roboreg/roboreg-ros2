from dataclasses import dataclass

from message_filters import Subscriber
from rcl_interfaces.msg import Parameter, SetParametersResult
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from .qos_profiles import JOINT_STATES_QOS, SENSOR_QOS
from .roboreg_node import RoboregNode


class MonocularDepthNode(RoboregNode):
    @dataclass
    class _ExtraParams:
        image_topic: str
        camera_info_topic: str
        depth_topic: str
        depth_camera_info_topic: str
        joint_state_topic: str

    def _register_synced_subscribers(self):
        self._data_collector.subscribers["camera.image"] = Subscriber(
            self,
            (
                CompressedImage
                if "compressed" in self._extra_params.image_topic
                else Image
            ),
            self._extra_params.image_topic,
            qos_profile=SENSOR_QOS,
        )
        self._data_collector.subscribers["camera.image.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.camera_info_topic,
            qos_profile=SENSOR_QOS,
        )
        self._data_collector.subscribers["camera.depth"] = Subscriber(
            self,
            (
                CompressedImage
                if "compressed" in self._extra_params.depth_topic
                else Image
            ),
            self._extra_params.depth_topic,
            qos_profile=SENSOR_QOS,
        )
        self._data_collector.subscribers["camera.depth.camera_info"] = Subscriber(
            self,
            CameraInfo,
            self._extra_params.depth_camera_info_topic,
            qos_profile=SENSOR_QOS,
        )
        self._data_collector.subscribers["joint_states"] = Subscriber(
            self,
            JointState,
            self._extra_params.joint_state_topic,
            qos_profile=JOINT_STATES_QOS,
        )

    def _declare_extra_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("topics.image", "/camera/image_rect_color"),
                ("topics.camera_info", "/camera/image_rect_color/camera_info"),
                ("topics.depth", "/camera/depth_registered"),
                ("topics.depth_camera_info", "/camera/depth_registered/camera_info"),
                ("topics.joint_state", "/joint_states"),
            ],
        )

    def _get_extra_parameters(self):
        self._extra_params = self._ExtraParams(
            image_topic=self.get_parameter("topics.image")
            .get_parameter_value()
            .string_value,
            camera_info_topic=self.get_parameter("topics.camera_info")
            .get_parameter_value()
            .string_value,
            depth_topic=self.get_parameter("topics.depth")
            .get_parameter_value()
            .string_value,
            depth_camera_info_topic=self.get_parameter("topics.depth_camera_info")
            .get_parameter_value()
            .string_value,
            joint_state_topic=self.get_parameter("topics.joint_state")
            .get_parameter_value()
            .string_value,
        )

    def _on_set_extra_parameters_impl(
        self, paramaters: list[Parameter]
    ) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        for parameter in paramaters:
            if parameter.name == "topics.joint_state":
                self.get_logger().info(
                    f"Setting joint state topic to {parameter.value}"
                )
                self._extra_params.joint_state_topic = parameter.value
                self._reload_synced_subscribers()
            else:
                continue
        return result
