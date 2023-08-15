from typing import Dict

from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


class MeshregMixin:
    @staticmethod
    def arg_sync_accuracy() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            "sync_accuracy",
            default_value="0.1",
            description="Allowed synchronization accuracy for images, point clouds, joint states in seconds.",
        )

    @staticmethod
    def param_sync_accuracy() -> Dict[str, LaunchConfiguration]:
        return {"sync_accuracy": LaunchConfiguration("sync_accuracy")}

    @staticmethod
    def node_meshreg(**kwargs) -> Node:
        return Node(
            package="ros2_meshreg",
            executable="meshreg",
            name="meshreg",
            output="screen",
            **kwargs
        )
