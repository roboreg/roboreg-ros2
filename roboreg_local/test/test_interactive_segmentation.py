import importlib.util
import sys
import types
import uuid


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


class _FakeParameterValue:
    def __init__(self, value):
        self.integer_value = value if isinstance(value, int) else 0
        self.double_value = value if isinstance(value, float) else 0.0
        self.string_value = value if isinstance(value, str) else ""


class _FakeParameter:
    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return _FakeParameterValue(self._value)


class _FakeNode:
    def __init__(self, parameters=None):
        self._parameters = dict(parameters or {})
        self._logger = _FakeLogger()

    def declare_parameters(self, namespace, parameters):
        del namespace
        for name, value in parameters:
            self._parameters.setdefault(name, value)

    def get_parameter(self, name):
        return _FakeParameter(self._parameters[name])

    def get_logger(self):
        return self._logger


class _FakeOpenCVAnnotator:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def annotate(self, image):
        del image
        return []

    def clear(self):
        return None


def _load_interactive_segmentation_module(monkeypatch, cuda_available):
    numpy_module = types.ModuleType("numpy")
    numpy_module.ndarray = object
    numpy_module.uint8 = "uint8"
    numpy_module.asarray = lambda value: value
    numpy_module.where = lambda condition, if_true, if_false: (
        condition,
        if_true,
        if_false,
    )
    monkeypatch.setitem(sys.modules, "numpy", numpy_module)

    torch_module = types.ModuleType("torch")
    torch_module.cuda = types.SimpleNamespace(is_available=lambda: cuda_available)
    monkeypatch.setitem(sys.modules, "torch", torch_module)

    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_node_module.Node = object
    monkeypatch.setitem(sys.modules, "rclpy", rclpy_module)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node_module)

    annotator_module = types.ModuleType("roboreg.annotator")
    annotator_module.OpenCVAnnotator = _FakeOpenCVAnnotator
    annotator_module.annotations_to_arrays = lambda annotations: ([], [])
    monkeypatch.setitem(sys.modules, "roboreg.annotator", annotator_module)

    segmentor_module = types.ModuleType("roboreg.segmentor")

    class _FakeSam2Segmentor:
        instances = []

        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.instances.append(self)

        def __call__(self, image, samples, labels):
            del image, samples, labels
            return None

        def threshold(self, probability, pth):
            del probability, pth
            return []

    segmentor_module.Sam2Segmentor = _FakeSam2Segmentor
    monkeypatch.setitem(sys.modules, "roboreg.segmentor", segmentor_module)

    module_name = f"interactive_segmentation_test_{uuid.uuid4().hex}"
    module_path = "/home/runner/work/roboreg-ros2/roboreg-ros2/roboreg_local/roboreg_local/util/interactive_segmentation.py"
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, module_name, module)
    spec.loader.exec_module(module)
    return module, _FakeSam2Segmentor


def test_interactive_segmentation_falls_back_to_cpu_without_cuda(monkeypatch):
    module, segmentor = _load_interactive_segmentation_module(
        monkeypatch, cuda_available=False
    )

    node = _FakeNode({"segmentation.device": "cuda"})
    interactive_segmentation = module.InteractiveSegmentation(node)

    assert interactive_segmentation._segmentation_params.device == "cpu"
    assert segmentor.instances[0].kwargs["device"] == "cpu"
    assert node.get_logger().warnings == [
        "CUDA is not available. Falling back to 'cpu' instead of requested 'cuda' device."
    ]


def test_interactive_segmentation_keeps_cuda_when_available(monkeypatch):
    module, segmentor = _load_interactive_segmentation_module(
        monkeypatch, cuda_available=True
    )

    node = _FakeNode({"segmentation.device": "cuda"})
    interactive_segmentation = module.InteractiveSegmentation(node)

    assert interactive_segmentation._segmentation_params.device == "cuda"
    assert segmentor.instances[0].kwargs["device"] == "cuda"
    assert node.get_logger().warnings == []
