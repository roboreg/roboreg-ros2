from dataclasses import dataclass

import numpy as np
import torch
from rclpy.node import Node
from roboreg.annotator import OpenCVAnnotator, annotations_to_arrays
from roboreg.segmentor import Sam2Segmentor


class InteractiveSegmentation:
    @dataclass
    class _AnnotationParams:
        n_positive: int = 3
        n_negative: int = 3

    @dataclass
    class _SegmentationParams:
        device: str
        model_id: str
        pth: float

    def __init__(self, node: Node) -> None:
        self._node = node
        self._declare_annotation_parameters()
        self._declare_segmentation_parameters()
        self._annotation_params = self._get_annotation_params()
        self._segmentation_params = self._get_segmentation_params()
        self._annotator = OpenCVAnnotator(
            n_positive=self._annotation_params.n_positive,
            n_negative=self._annotation_params.n_negative,
            window_name="Annotate: left click for positive, CTRL + left click for negative samples",
        )
        self._node.get_logger().info(
            f"Instantiating segmentation model on '{self._segmentation_params.device}' device. "
            "This may take a while..."
        )
        self._segmentor = Sam2Segmentor(
            model_id=self._segmentation_params.model_id,
            device=self._segmentation_params.device,
        )
        self._node.get_logger().info("Segmentation model instantiated.")

    def segment(self, images: list[np.ndarray]) -> list[np.ndarray]:
        segmentations = []
        for image in images:
            annotations = self._annotator.annotate(image)
            self._annotator.clear()
            samples, labels = annotations_to_arrays(annotations)
            probability = self._segmentor(
                image, np.asarray(samples), np.asarray(labels)
            )
            segmentations.append(
                np.where(
                    self._segmentor.threshold(
                        probability=probability, pth=self._segmentation_params.pth
                    ),
                    255,
                    0,
                ).astype(np.uint8)
            )
        return segmentations

    def _declare_annotation_parameters(self) -> None:
        self._node.declare_parameters(
            namespace="",
            parameters=[
                ("annotation.n_positive", 3),
                ("annotation.n_negative", 3),
            ],
        )

    def _declare_segmentation_parameters(self) -> None:
        self._node.declare_parameters(
            namespace="",
            parameters=[
                ("segmentation.device", "cuda" if torch.cuda.is_available() else "cpu"),
                ("segmentation.model_id", "facebook/sam2-hiera-large"),
                ("segmentation.pth", 0.5),
            ],
        )

    def _get_annotation_params(self) -> _AnnotationParams:
        return self._AnnotationParams(
            n_positive=self._node.get_parameter("annotation.n_positive")
            .get_parameter_value()
            .integer_value,
            n_negative=self._node.get_parameter("annotation.n_negative")
            .get_parameter_value()
            .integer_value,
        )

    def _get_segmentation_params(self) -> _SegmentationParams:
        return self._SegmentationParams(
            device=self._node.get_parameter("segmentation.device")
            .get_parameter_value()
            .string_value,
            model_id=self._node.get_parameter("segmentation.model_id")
            .get_parameter_value()
            .string_value,
            pth=self._node.get_parameter("segmentation.pth")
            .get_parameter_value()
            .double_value,
        )
