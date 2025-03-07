from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
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
            "namespace",
            default_value="",
            description="Namespace for the roboreg node.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cfg_pkg",
            default_value="roboreg_nodes",
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
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation (Gazebo) time.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "max_jobs",
            default_value="2",
            description="Maximum number of concurrent jobs for nvdiffrast. Limits compilation RAM usage on first run. Relevant for rendering and differentiable rendering.",
        )
    )
    ld.add_action(
        SetEnvironmentVariable(
            name="MAX_JOBS",
            value=LaunchConfiguration("max_jobs"),
        )
    )
    ld.add_action(
        Node(
            package="roboreg_nodes",
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
        )
    )
    return ld
