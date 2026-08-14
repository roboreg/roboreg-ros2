import csv
import io
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, NamedTuple

import cv2
import numpy as np
from rclpy.node import Node


class Annotation(NamedTuple):
    x: int
    y: int
    label: int  # 1 = positive, 0 = negative


def annotations_to_csv(annotations: List[Annotation]) -> str:
    r"""Serialize annotations to the 'x,y,label' CSV format expected by build_archive.

    Args:
        annotations (List[Annotation]): Annotations to serialize.

    Returns:
        str: CSV text with header row, ready to pass as an image_annotations entry.
    """
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["x", "y", "label"])
    writer.writerows(annotations)
    return buf.getvalue()


class Annotator(ABC):
    @dataclass(frozen=True)
    class _Params:
        n_positive: int = 3
        n_negative: int = 3

    def __init__(self, node: Node) -> None:
        super().__init__()
        self._annotations: List[Annotation] = []
        self._node = node
        self._declare_annotation_parameters()
        self._params = self._get_annotation_params()

    def clear(self) -> None:
        self._annotations = []

    @abstractmethod
    def annotate(self, img: np.ndarray) -> List[Annotation]:
        raise NotImplementedError

    @property
    def annotations(self) -> List[Annotation]:
        return self._annotations

    @property
    def positive_annotations(self) -> List[Annotation]:
        return [annotation for annotation in self._annotations if annotation.label == 1]

    @property
    def negative_annotations(self) -> List[Annotation]:
        return [annotation for annotation in self._annotations if annotation.label == 0]

    def _declare_annotation_parameters(self) -> None:
        self._node.declare_parameter("annotation.n_positive", 3)
        self._node.declare_parameter("annotation.n_negative", 3)

    def _get_annotation_params(self) -> _Params:
        return self._Params(
            n_positive=self._node.get_parameter("annotation.n_positive")
            .get_parameter_value()
            .integer_value,
            n_negative=self._node.get_parameter("annotation.n_negative")
            .get_parameter_value()
            .integer_value,
        )


class OpenCVAnnotator(Annotator):
    def __init__(
        self,
        node: Node,
        window_name="Annotate: left click for positive, CTRL + left click for negative samples",
    ) -> None:
        super().__init__(node=node)
        self._window_name = window_name

    def _on_mouse(self, event, x, y, flags, param):
        if (
            event == cv2.EVENT_LBUTTONDOWN and flags & cv2.EVENT_FLAG_CTRLKEY
        ):  # bitwise and for flags: https://stackoverflow.com/questions/32210066/mouse-callback-event-flags-in-python-opencv-osx
            if len(self.negative_annotations) >= self._params.n_negative:
                self._node.get_logger().info(
                    f"Already added {len(self.negative_annotations)} of {self._params.n_negative}  negative samples."
                )
                return
            self._annotations.append(Annotation(x=x, y=y, label=0))
            self._node.get_logger().info(
                f"Negative samples: {len(self.negative_annotations)} of {self._params.n_negative}. Coordinates {x}, {y}."
            )
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.positive_annotations) >= self._params.n_positive:
                self._node.get_logger().info(
                    f"Already added {len(self.positive_annotations)} of {self._params.n_positive} positive samples. Use CTRL + Left Click to add negative samples."
                )
                return
            self._annotations.append(Annotation(x=x, y=y, label=1))
            self._node.get_logger().info(
                f"Positive samples: {len(self.positive_annotations)} of {self._params.n_positive}. Coordinates: {x}, {y}."
            )
            return

    def annotate(self, img: np.ndarray) -> List[Annotation]:
        cv2.namedWindow(self._window_name)
        cv2.setMouseCallback(self._window_name, self._on_mouse)
        img_cpy = img.copy()
        while (
            len(self.positive_annotations) < self._params.n_positive
            or len(self.negative_annotations) < self._params.n_negative
        ):
            try:
                cv2.imshow(self._window_name, img_cpy)
                cv2.waitKey(10)

                # draw samples
                positive_annotations = self.positive_annotations
                if positive_annotations:
                    last = positive_annotations[-1]
                    cv2.circle(img_cpy, (last.x, last.y), 5, (255, 255, 0), -1)
                negative_annotations = self.negative_annotations
                if negative_annotations:
                    last = negative_annotations[-1]
                    cv2.circle(img_cpy, (last.x, last.y), 5, (0, 255, 255), -1)
            except KeyboardInterrupt:
                break
        cv2.destroyAllWindows()
        return self._annotations
