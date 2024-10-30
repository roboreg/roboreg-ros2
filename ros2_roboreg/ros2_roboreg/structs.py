from dataclasses import dataclass

import torch
from sensor_msgs.msg import CameraInfo, Image, JointState


@dataclass
class SyncedSample:
    left_image: Image
    left_camera_info: CameraInfo
    right_image: Image
    right_camera_info: CameraInfo
    joint_state: JointState
    depth: Image

    def __init__(self) -> None:
        self.left_image = None
        self.left_camera_info = None
        self.right_image = None
        self.right_camera_info = None
        self.joint_state = None
        self.depth = None

    def clear(self) -> None:
        self.left_image = None
        self.left_camera_info = None
        self.right_image = None
        self.right_camera_info = None
        self.joint_state = None
        self.depth = None


@dataclass
class ServerParams:
    class _Filters:
        sync_accuracy: float
        min_joint_position_change: float
        max_joint_velocity: float

        def __init__(self) -> None:
            self.sync_accuracy = 0.01
            self.min_joint_position_change = 0.001
            self.max_joint_velocity = 0.01

    @dataclass
    class _TopicParam:
        name: str
        qos_reliability: str

        def __init__(self) -> None:
            self.name = ""
            self.qos_reliability = ""

    @dataclass
    class _SegmentationParams:
        device: str
        n_positive_samples: int
        n_negative_samples: int
        model_id: str
        pth: float

        def __init__(self) -> None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.n_positive_samples = 5
            self.n_negative_samples = 5
            self.model_id = "facebook/sam2-hiera-large"
            self.pth = 0.5

    @dataclass
    class _RobotModel:
        device: str
        root_link_name: str
        end_link_name: str
        visual_meshes: bool

        def __init__(self) -> None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
            self.root_link_name = ""
            self.end_link_name = ""
            self.visual_meshes = False

    @dataclass
    class _RegistrationParams:
        @dataclass
        class _HydraICPParams:
            erosion_kernel_size: int
            number_of_points: int
            max_distance: float
            outer_max_iter: int
            inner_max_iter: int
            rmse_change: float

            def __init__(self) -> None:
                self.erosion_kernel_size = 10
                self.number_of_points = 5000
                self.max_distance = 0.1
                self.outer_max_iter = 100
                self.inner_max_iter = 3
                self.rmse_change = 1.0e-6

        @dataclass
        class _StereoDRParams:
            optimizer: str
            lr: float
            epochs: int
            step_size: int
            gamma: float

            def __init__(self) -> None:
                self.optimizer = "SGD"
                self.lr = 0.001
                self.epochs = 100
                self.step_size = 100
                self.gamma = 1.0

        hydra_icp = _HydraICPParams()
        stereo_dr = _StereoDRParams()

    def __init__(self) -> None:
        self.filters = self._Filters()
        self.left_camera_info_topic = self._TopicParam()
        self.right_camera_info_topic = self._TopicParam()
        self.left_image_topic = self._TopicParam()
        self.right_image_topic = self._TopicParam()
        self.joint_state_topic = self._TopicParam()
        self.depth_topic = self._TopicParam()
        self.robot_description_topic = self._TopicParam()
        self.segmentation = self._SegmentationParams()
        self.robot_model = self._RobotModel()
        self.registration = self._RegistrationParams()
        self.tf_broadcaster = self.TFParams()

    @dataclass
    class TFParams:
        parent_frame: str
        child_frame: str
        target_child_frame: str

        def __init__(self) -> None:
            self.parent_frame = "world"
            self.child_frame = ""
            self.target_child_frame = ""
