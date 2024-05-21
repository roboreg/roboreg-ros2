from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "config_pkg",
            default_value="ros2_roboreg",
            description="Package name containing the configuration file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "config_path",
            default_value="config/roboreg.yaml",
            description="Configuration file path.",
        )
    )
    ld.add_action(
        Node(
            package="ros2_roboreg",
            executable="roboreg",
            name="roboreg",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare(LaunchConfiguration("config_pkg")),
                        LaunchConfiguration("config_path"),
                    ]
                )
            ],
        )
    )
    return ld
