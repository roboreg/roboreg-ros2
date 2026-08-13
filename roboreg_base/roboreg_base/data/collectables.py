import pathlib
from abc import ABC, abstractmethod
from typing import Generic, TypeVar, get_args

import cv2
import cv_bridge
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState

from .decode import decode_depth

T = TypeVar("T")


class Collectable(ABC, Generic[T]):
    _collectable_map: dict[type[T], type["Collectable"]] = {}

    def __init__(self, msg: T):
        self._msg: T = msg

    @abstractmethod
    def to_numpy(self) -> np.ndarray:
        raise NotImplementedError

    @abstractmethod
    def to_disk(self, path: pathlib.Path, filename: str, mkdir: bool = True):
        if not path.exists() and mkdir:
            path.mkdir(parents=True)

    @property
    def msg(self) -> T:
        return self._msg

    @classmethod
    def from_message(cls, msg: T) -> "Collectable[T]" | None:
        msg_type = type(msg)
        collectable_cls = cls._collectable_map.get(msg_type)
        if collectable_cls:
            return collectable_cls(msg)
        raise ValueError(
            f"No Collectable registered for message type {msg_type.__name__}"
        )

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if cls is not Collectable:  # Avoid registering the base class itself
            # Extract the type argument T from Collectable[T]
            type_args = get_args(cls.__orig_bases__[0])
            if type_args:
                msg_type = type_args[0]
                Collectable._collectable_map[msg_type] = cls  # Register the subclass


class ImageCollectable(Collectable[Image]):
    def __init__(self, msg: Image):
        super().__init__(msg)
        self._cv_bridge = cv_bridge.CvBridge()

    def to_numpy(self) -> np.ndarray:
        # two conventions:
        #   - any color to bgr8
        #   - any depth to 32FC1 (meters)
        encoding = self._msg.encoding
        if (
            encoding == "rgb8"
            or encoding == "bgr8"
            or encoding == "rgba8"
            or encoding == "bgra8"
        ):  # color image
            return self._cv_bridge.imgmsg_to_cv2(self._msg, desired_encoding="bgr8")
        elif encoding == "16UC1":  # depth image in mm
            return (
                self._cv_bridge.imgmsg_to_cv2(self._msg, desired_encoding="32FC1")
                / 1000.0
            )
        elif encoding == "32FC1":  # depth image in meters
            return self._cv_bridge.imgmsg_to_cv2(self._msg, desired_encoding="32FC1")
        else:
            raise ValueError(f"Unsupported encoding {self._msg.encoding}")

    def to_disk(self, path: pathlib.Path, filename: str, mkdir: bool = True):
        super().to_disk(path, filename=filename, mkdir=mkdir)
        if (
            self._msg.encoding == "rgb8"
            or self._msg.encoding == "bgr8"
            or self._msg.encoding == "rgba8"
            or self._msg.encoding == "bgra8"
        ):  # save color image as png
            cv2.imwrite(str(path / (filename + ".png")), self.to_numpy())
        elif (
            self._msg.encoding == "16UC1" or self._msg.encoding == "32FC1"
        ):  # save depth image as numpy array
            np.save(path / (filename + ".npy"), self.to_numpy())
        else:
            raise ValueError(f"Unsupported encoding {self._msg.encoding}")


class CompressedImageCollectable(Collectable[CompressedImage]):
    def __init__(self, msg: CompressedImage):
        super().__init__(msg)
        self._cv_bridge = cv_bridge.CvBridge()

    def to_numpy(self) -> np.ndarray:
        _, compr_type = self._msg.format.split(";")
        if compr_type.strip() == "compressedDepth":
            return decode_depth(
                self._msg
            )  # converts to depth in meters of type float32
        elif (
            "rgb8" in compr_type
            or "bgr8" in compr_type
            or "rgba8" in compr_type
            or "bgra8" in compr_type
        ):
            return self._cv_bridge.compressed_imgmsg_to_cv2(
                self._msg, desired_encoding="bgr8"
            )
        else:
            raise ValueError(f"Unsupported format {self._msg.format}")

    def to_disk(self, path: pathlib.Path, filename: str, mkdir: bool = True):
        super().to_disk(path, filename=filename, mkdir=mkdir)
        if "16UC1" in self._msg.format or "32FC1" in self._msg.format:
            np.save(path / (filename + ".npy"), self.to_numpy())
        elif (
            "rgb8" in self._msg.format
            or "bgr8" in self._msg.format
            or "rgba8" in self._msg.format
            or "bgra8" in self._msg.format
        ):
            cv2.imwrite(str(path / (filename + ".png")), self.to_numpy())
        else:
            raise ValueError(f"Unsupported format {self._msg.format}")


class JointStateCollectable(Collectable[JointState]):
    def __init__(self, msg: JointState):
        super().__init__(msg)

    def to_numpy(self) -> np.ndarray:
        return np.array(self._msg.position)[np.argsort(self._msg.name)]

    def to_disk(self, path: pathlib.Path, filename: str, mkdir: bool = True):
        super().to_disk(path, filename=filename, mkdir=mkdir)
        np.save(path / (filename + ".npy"), self.to_numpy())


class CameraInfoCollectable(Collectable[CameraInfo]):
    def __init__(self, msg: CameraInfo):
        super().__init__(msg)

    def to_numpy(self) -> np.ndarray:
        return np.array(self._msg.k).reshape(3, 3)

    def to_disk(self, path: pathlib.Path, filename: str, mkdir: bool = True):
        super().to_disk(path, filename=filename, mkdir=mkdir)
        camera_info_dict = {
            "frame_id": self._msg.header.frame_id,
            "height": self._msg.height,
            "width": self._msg.width,
            "distortion_model": self._msg.distortion_model,
            "d": self._msg.d.tolist(),
            "k": self._msg.k.tolist(),
            "r": self._msg.r.tolist(),
            "p": self._msg.p.tolist(),
            "binning_x": self._msg.binning_x,
            "binning_y": self._msg.binning_y,
            "roi": {
                "x_offset": self._msg.roi.x_offset,
                "y_offset": self._msg.roi.y_offset,
                "height": self._msg.roi.height,
                "width": self._msg.roi.width,
                "do_rectify": self._msg.roi.do_rectify,
            },
        }
        with open(str(path / (filename + ".yaml")), "w") as f:
            yaml.dump(camera_info_dict, f)
