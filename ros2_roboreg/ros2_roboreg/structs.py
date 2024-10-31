from dataclasses import dataclass

import torch
from sensor_msgs.msg import Image, JointState, CameraInfo


class Camera:
    def __init__(self) -> None:
        self.camera_info: CameraInfo = None
        self.image: Image = None

    def clear(self) -> None:
        self.camera_info = None
        self.image = None


class SyncedSample:
    def __init__(self) -> None:
        self.joint_state: JointState = None
        self.depth: Camera = None

    def clear(self) -> None:
        self.joint_state = None
        self.depth.clear()


class MonocularDepthSample(SyncedSample):
    def __init__(self):
        super().__init__()
        self.camera: Camera = None

    def clear(self) -> None:
        super().clear()
        self.camera.clear()


class StereoDepthSample(SyncedSample):
    def __init__(self):
        super().__init__()
        self.left_camera: Camera = None
        self.right_camera: Camera = None

    def clear(self) -> None:
        super().clear()
        self.left_camera.clear()
        self.right_camera.clear()


@dataclass
class FilterParams:
    sync_accuracy: float = 0.01
    min_joint_position_change: float = 0.001
    max_joint_velocity: float = 0.01
    min_depth: float = 0.01
    max_depth: float = 4.0


@dataclass
class TopicParams:
    name: str = ""
    qos_reliability: str = ""


@dataclass
class SegmentationParams:
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    n_positive_samples: int = 5
    n_negative_samples: int = 5
    model_id: str = "facebook/sam2-hiera-large"
    pth: float = 0.5


@dataclass
class RobotModel:
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    root_link_name: str = ""
    end_link_name: str = ""
    visual_meshes: bool = False


@dataclass
class TFParams:
    parent_frame: str = "world"
    child_frame: str = ""
    target_child_frame: str = ""


class ServerParams:
    def __init__(self) -> None:
        self.filters: FilterParams = FilterParams()
        self.joint_state_topic: TopicParams = TopicParams()
        self.depth_topic: TopicParams = TopicParams()
        self.robot_description_topic: TopicParams = TopicParams()
        self.segmentation: SegmentationParams = SegmentationParams()
        self.robot_model: RobotModel = RobotModel()
        self.tf_broadcaster: TFParams = TFParams()


class MonocularDepthParams(ServerParams):
    def __init__(self) -> None:
        super().__init__()
        self.camera_info_topic: TopicParams = TopicParams()
        self.image_topic: TopicParams = TopicParams()


class StereoDepthParams(ServerParams):
    def __init__(self) -> None:
        super().__init__()
        self.left_camera_info_topic: TopicParams = TopicParams()
        self.left_image_topic: TopicParams = TopicParams()
        self.right_camera_info_topic: TopicParams = TopicParams()
        self.right_image_topic: TopicParams = TopicParams()
