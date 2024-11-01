from roboreg.detector import OpenCVDetector
from roboreg.segmentor import Sam2Segmentor


class Segmentation:
    def __init__(self) -> None:
        self._segmentor = Sam2Segmentor()
        self._detector = OpenCVDetector()

    @property
    def segmentor(self) -> Sam2Segmentor:
        return self._segmentor

    @property
    def detector(self) -> OpenCVDetector:
        return self._detector
