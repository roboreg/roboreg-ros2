from launch import LaunchDescription
from ros2_meshreg import MeshregMixin


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    ld.add_action(MeshregMixin.arg_sync_accuracy())
    ld.add_action(
        MeshregMixin.node_meshreg(parameters=[MeshregMixin.param_sync_accuracy()])
    )
    return ld
