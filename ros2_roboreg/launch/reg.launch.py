from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "mode",
            default_value="monocular_depth",
            choices=["monocular_depth", "stereo_depth"],
            description="Mode to launch.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cfg_pkg",
            default_value="ros2_roboreg",
            description="Package name containing the configuration file.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cfg_path",
            default_value="config/monocular_depth.yaml",
            description="Configuration file path relative to cfg_pkg.",
        )
    )
    ld.add_action(
        Node(
            package="ros2_roboreg",
            executable=LaunchConfiguration("mode"),
            name="eye_to_hand_calibration",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare(LaunchConfiguration("cfg_pkg")),
                        LaunchConfiguration("cfg_path"),
                    ]
                )
            ],
            emulate_tty=True,
        )
    )
    return ld
