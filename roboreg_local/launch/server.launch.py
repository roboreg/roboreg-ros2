from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="monocular_depth",
                choices=["monocular_depth", "stereo_depth"],
                description="Mode to launch.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for the roboreg node.",
            ),
            DeclareLaunchArgument(
                "cfg_pkg",
                default_value="roboreg_local",
                description="Package name containing the configuration file.",
            ),
            DeclareLaunchArgument(
                "cfg_path",
                default_value="config/monocular_depth.yaml",
                description="Configuration file path relative to cfg_pkg.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) time.",
            ),
            Node(
                package="roboreg_local",
                executable=LaunchConfiguration("mode"),
                name="roboreg",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare(LaunchConfiguration("cfg_pkg")),
                            LaunchConfiguration("cfg_path"),
                        ]
                    ),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                namespace=LaunchConfiguration("namespace"),
                emulate_tty=True,
            ),
        ]
    )
