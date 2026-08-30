from dataclasses import dataclass


@dataclass
class FilterParams:
    sync_accuracy: float = 0.01

    def __post_init__(self):
        if self.sync_accuracy <= 0:
            raise ValueError("sync_accuracy must be greater than 0.")


@dataclass
class RobotDataParams:
    root_link_name: str = ""
    end_link_name: str = ""
    collision_meshes: bool = False


@dataclass
class TFBroadcasterParams:
    parent_frame: str
    child_frame: str
    target_child_frame: str

    def __post_init__(self):
        if not self.parent_frame:
            raise ValueError("parent_frame must not be empty.")
        if not self.child_frame:
            raise ValueError("child_frame must not be empty.")
        if not self.target_child_frame:
            raise ValueError("target_child_frame must not be empty.")
