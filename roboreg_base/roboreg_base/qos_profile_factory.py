from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_system_default,
)

from .parameters import QoSParams


def qos_profile_factory(qos: QoSParams) -> QoSProfile:
    qos_profile = qos_profile_system_default
    qos_profile.reliability = getattr(ReliabilityPolicy, qos.reliability)
    qos_profile.durability = getattr(DurabilityPolicy, qos.durability)
    return qos_profile
