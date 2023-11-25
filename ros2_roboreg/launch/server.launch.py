from launch import LaunchDescription

from ros2_roboreg import RoboregMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(RoboregMixin.arg_config_pkg())
    ld.add_action(RoboregMixin.arg_config_path())
    ld.add_action(
        RoboregMixin.node_roboreg(
            parameters=[RoboregMixin.config_file_path_roboreg()],
        )
    )
    return ld
