from launch import LaunchDescription
from ros2_roboreg import RoboregMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(RoboregMixin.arg_sync_accuracy())
    ld.add_action(
        RoboregMixin.node_roboreg(parameters=[RoboregMixin.param_sync_accuracy()])
    )
    return ld
