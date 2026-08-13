from dataclasses import dataclass

import numpy as np
import torch
from rclpy.node import Node
from roboreg.annotator import OpenCVAnnotator, annotations_to_arrays
from roboreg.segmentor import Sam2Segmentor


class InteractiveSegmentation:
    @dataclass
    class _Params:
        device: str
        n_positive_samples: int
        n_negative_samples: int
        model_id: str
        pth: float

    def __init__(self, node: Node) -> None:
        self._node = node
        self._declare_segmentation_parameters()
        self._params = self._get_segmentation_params()
        self._annotator = OpenCVAnnotator(
            n_positive=self._params.n_positive_samples,
            n_negative=self._params.n_negative_samples,
            window_name="Annotate: left click for positive, CTRL + left click for negative samples",
        )
        self._node.get_logger().info(
            f"Instantiating segmentation model on '{self._params.device}' device. "
            "This may take a while..."
        )
        self._segmentor = Sam2Segmentor(
            model_id=self._params.model_id,
            device=self._params.device,
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
                        probability=probability, pth=self._params.pth
                    ),
                    255,
                    0,
                ).astype(np.uint8)
            )
        return segmentations

    def _declare_segmentation_parameters(self) -> None:
        self._node.declare_parameters(
            namespace="",
            parameters=[
                ("segmentation.device", "cuda" if torch.cuda.is_available() else "cpu"),
                ("segmentation.n_positive_samples", 5),
                ("segmentation.n_negative_samples", 5),
                ("segmentation.model_id", "facebook/sam2-hiera-large"),
                ("segmentation.pth", 0.5),
            ],
        )

    def _get_segmentation_params(self) -> _Params:
        return self._Params(
            device=self._node.get_parameter("segmentation.device")
            .get_parameter_value()
            .string_value,
            n_positive_samples=self._node.get_parameter(
                "segmentation.n_positive_samples"
            )
            .get_parameter_value()
            .integer_value,
            n_negative_samples=self._node.get_parameter(
                "segmentation.n_negative_samples"
            )
            .get_parameter_value()
            .integer_value,
            model_id=self._node.get_parameter("segmentation.model_id")
            .get_parameter_value()
            .string_value,
            pth=self._node.get_parameter("segmentation.pth")
            .get_parameter_value()
            .double_value,
        )
