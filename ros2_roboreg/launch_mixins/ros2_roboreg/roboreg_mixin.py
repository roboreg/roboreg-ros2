from typing import Dict

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class RoboregMixin:
    @staticmethod
    def arg_config_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "config_pkg",
            default_value="ros2_roboreg",
            description="Package name containing the configuration file.",
        )

    @staticmethod
    def arg_config_path() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "config_path",
            default_value="config/roboreg.yaml",
            description="Configuration file path.",
        )

    @staticmethod
    def arg_sync_accuracy() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "sync_accuracy",
            default_value="1.0",
            description="Allowed synchronization accuracy for images, point clouds, joint states in seconds.",
        )

    @staticmethod
    def arg_min_joint_position_change() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "min_joint_position_change",
            default_value="0.001",
            description="Discard new data if joint position change is smaller than this value.",
        )

    @staticmethod
    def arg_max_joint_velocity() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "max_joint_velocity",
            default_value="0.01",
            description="Discard new data if joint velocity is larger than this value.",
        )

    @staticmethod
    def arg_left_image_topic() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "left_image_topic",
            default_value="/left/image_rect_color",
            description="Left rectified color image topic.",
        )

    @staticmethod
    def arg_right_image_topic() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "right_image_topic",
            default_value="/right/image_rect_color",
            description="Right rectified color image topic.",
        )

    @staticmethod
    def arg_left_camera_info_topic() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "left_camera_info_topic",
            default_value="/left/camera_info",
            description="Left camera info topic.",
        )

    @staticmethod
    def arg_right_camera_info_topic() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "right_camera_info_topic",
            default_value="/right/camera_info",
            description="Right camera info topic.",
        )

    @staticmethod
    def arg_joint_states_topic() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="Joint states topic.",
        )

    @staticmethod
    def arg_point_cloud_topic() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "point_cloud_topic",
            default_value="/point_cloud/cloud_registered",
            description="Point cloud topic.",
        )

    @staticmethod
    def param_sync_accuracy() -> Dict[str, LaunchConfiguration]:
        return {"sync_accuracy": LaunchConfiguration("sync_accuracy")}

    @staticmethod
    def param_min_joint_position_change() -> Dict[str, LaunchConfiguration]:
        return {
            "min_joint_position_change": LaunchConfiguration(
                "min_joint_position_change"
            )
        }

    @staticmethod
    def param_max_joint_velocity() -> Dict[str, LaunchConfiguration]:
        return {"max_joint_velocity": LaunchConfiguration("max_joint_velocity")}

    @staticmethod
    def param_left_image_topic() -> Dict[str, LaunchConfiguration]:
        return {"left_image_topic": LaunchConfiguration("left_image_topic")}

    @staticmethod
    def param_right_image_topic() -> Dict[str, LaunchConfiguration]:
        return {"right_image_topic": LaunchConfiguration("right_image_topic")}

    @staticmethod
    def param_left_camera_info_topic() -> Dict[str, LaunchConfiguration]:
        return {"left_camera_info_topic": LaunchConfiguration("left_camera_info_topic")}

    @staticmethod
    def param_right_camera_info_topic() -> Dict[str, LaunchConfiguration]:
        return {
            "right_camera_info_topic": LaunchConfiguration("right_camera_info_topic")
        }

    @staticmethod
    def param_joint_states_topic() -> Dict[str, LaunchConfiguration]:
        return {"joint_states_topic": LaunchConfiguration("joint_states_topic")}

    @staticmethod
    def param_point_cloud_topic() -> Dict[str, LaunchConfiguration]:
        return {"point_cloud_topic": LaunchConfiguration("point_cloud_topic")}

    @staticmethod
    def config_file_path_roboreg() -> PathJoinSubstitution:
        return PathJoinSubstitution(
            [
                FindPackageShare(LaunchConfiguration("config_pkg")),
                LaunchConfiguration("config_path"),
            ]
        )

    @staticmethod
    def node_roboreg(**kwargs) -> Node:
        return Node(
            package="ros2_roboreg",
            executable="roboreg",
            name="roboreg",
            output="screen",
            **kwargs
        )
