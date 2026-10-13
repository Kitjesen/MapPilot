"""Crop preprocessing and numerical output contracts for DINOv3."""

import ctypes
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from perception.encoding.dino import DinoEncoder


class FakeFunction:
    def __init__(self, implementation):
        self.implementation = implementation
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.implementation(*args)


class FakeTensorRTLibrary:
    def __init__(self):
        self.created = []
        self.destroyed = []
        self.calls = 0
        self.dino_create = FakeFunction(self._create)
        self.dino_encode = FakeFunction(self._encode)
        self.dino_destroy = FakeFunction(self._destroy)

    def _create(self, path):
        self.created.append(path)
        return 41

    def _encode(self, handle, pixels, descriptor):
        assert handle == 41
        array = np.ctypeslib.as_array(pixels, shape=(3 * 336 * 336,))
        assert array[0] > 2.0
        output = np.ctypeslib.as_array(descriptor, shape=(384,))
        output.fill(0.0)
        output[self.calls] = 2.0
        self.calls += 1
        return 0

    def _destroy(self, handle):
        self.destroyed.append(handle)


class Session:
    def get_inputs(self):
        return [type("Input", (), {"name": "input"})()]

    def run(self, names, feeds):
        pixels = feeds["input"]
        assert pixels.shape == (1, 3, 336, 336)
        assert pixels[0, 0, 0, 0] > 2.0
        assert pixels[0, 2, 0, 0] < -1.0
        return [np.ones((1, 384), dtype=np.float32)]


def test_bgr_crop_is_rgb_normalized_and_descriptor_has_unit_norm():
    encoder = DinoEncoder("model.onnx")
    encoder._session = Session()
    crop = np.zeros((16, 16, 3), np.uint8)
    crop[:, :, 2] = 255
    feature = encoder.encode(crop)
    assert feature.shape == (384,)
    assert np.linalg.norm(feature) == pytest.approx(1.0)
    assert not hasattr(encoder, "encode_text")


def test_invalid_descriptor_is_not_accepted_as_a_fast_inference_result():
    encoder = DinoEncoder("model.onnx")

    class Invalid(Session):
        def run(self, names, feeds):
            return [np.full((1, 384), np.nan, np.float32)]

    encoder._session = Invalid()
    with pytest.raises(RuntimeError, match="invalid crop descriptor"):
        encoder.encode(np.zeros((16, 16, 3), np.uint8))


def test_wrong_descriptor_shape_uses_the_public_runtime_error():
    encoder = DinoEncoder("model.onnx")

    class WrongShape(Session):
        def run(self, names, feeds):
            return [np.ones((1, 10), np.float32)]

    encoder._session = WrongShape()
    with pytest.raises(RuntimeError, match="invalid crop descriptor"):
        encoder.encode(np.zeros((16, 16, 3), np.uint8))


def test_onnx_batch_keeps_fixed_batch_one_and_output_order():
    class FixedBatchSession(Session):
        def __init__(self):
            self.runs = 0

        def run(self, names, feeds):
            pixels = feeds["input"]
            assert pixels.shape == (1, 3, 336, 336)
            feature = np.zeros((1, 384), dtype=np.float32)
            feature[0, self.runs] = 1.0
            self.runs += 1
            return [feature]

    session = FixedBatchSession()
    encoder = DinoEncoder("model.onnx")
    encoder._session = session

    features = encoder.encode_batch(
        [
            np.zeros((16, 16, 3), np.uint8),
            np.full((20, 12, 3), 127, np.uint8),
        ]
    )

    assert session.runs == 2
    assert len(features) == 2
    assert np.array_equal(features[0][:2], [1.0, 0.0])
    assert np.array_equal(features[1][:2], [0.0, 1.0])


def test_tensorrt_engine_uses_sibling_library_and_preserves_crop_order(
    monkeypatch,
    tmp_path,
):
    engine = tmp_path / "dinov3.engine"
    library_path = tmp_path / "liblingtu_dino_trt.so"
    engine.write_bytes(b"engine")
    library_path.write_bytes(b"library")
    library = FakeTensorRTLibrary()
    monkeypatch.setattr(ctypes, "CDLL", lambda path: library)

    encoder = DinoEncoder(str(engine))
    features = encoder.encode_batch(
        [
            np.full((16, 16, 3), [0, 0, 255], dtype=np.uint8),
            np.full((20, 12, 3), [0, 0, 255], dtype=np.uint8),
        ]
    )

    assert library.created == [str(engine).encode()]
    assert library.calls == 2
    assert encoder.backend == "TensorRT"
    assert features[0][0] == pytest.approx(1.0)
    assert features[1][1] == pytest.approx(1.0)
    encoder.shutdown()
    assert library.destroyed == [41]


def test_tensorrt_inference_error_uses_public_runtime_error(monkeypatch, tmp_path):
    engine = tmp_path / "dinov3.engine"
    library_path = tmp_path / "liblingtu_dino_trt.so"
    engine.write_bytes(b"engine")
    library_path.write_bytes(b"library")
    library = FakeTensorRTLibrary()
    library.dino_encode = FakeFunction(lambda *_args: 3)
    monkeypatch.setattr(ctypes, "CDLL", lambda path: library)

    encoder = DinoEncoder(str(engine))
    with pytest.raises(RuntimeError, match="TensorRT inference failed: 3"):
        encoder.encode(np.zeros((16, 16, 3), np.uint8))
    encoder.shutdown()


def test_tensorrt_engine_requires_sibling_runtime_library(tmp_path):
    engine = tmp_path / "dinov3.engine"
    engine.write_bytes(b"engine")

    with pytest.raises(RuntimeError, match="TensorRT library is missing"):
        DinoEncoder(str(engine)).load_model()


def test_empty_batch_returns_without_loading_model():
    encoder = DinoEncoder("missing.onnx")
    assert encoder.encode_batch([]) == []


@pytest.mark.parametrize("device", ["cuda", "cuda:0"])
def test_explicit_cuda_requires_a_cuda_execution_provider(monkeypatch, device):
    monkeypatch.setitem(
        sys.modules,
        "onnxruntime",
        SimpleNamespace(
            get_available_providers=lambda: ["CPUExecutionProvider"],
        ),
    )
    with pytest.raises(RuntimeError, match="CUDA execution provider is unavailable"):
        DinoEncoder("model.onnx", device=device).load_model()


@pytest.mark.parametrize("configured_threads, expected_threads", [(None, 4), (8, 8)])
def test_cpu_thread_budget_reaches_onnx_session_without_changing_default(
    monkeypatch,
    configured_threads,
    expected_threads,
):
    sessions = []

    class CpuSession:
        def __init__(self, path, options, providers):
            sessions.append((options.intra_op_num_threads, providers))

        def get_inputs(self):
            return [SimpleNamespace(shape=(1, 3, 336, 336))]

        def get_providers(self):
            return ["CPUExecutionProvider"]

    monkeypatch.setitem(
        sys.modules,
        "onnxruntime",
        SimpleNamespace(
            get_available_providers=lambda: ["CPUExecutionProvider"],
            SessionOptions=SimpleNamespace,
            InferenceSession=CpuSession,
        ),
    )
    kwargs = {} if configured_threads is None else {"cpu_threads": configured_threads}
    encoder = DinoEncoder("model.onnx", device="cpu", **kwargs)
    encoder.load_model()

    assert sessions == [(expected_threads, ["CPUExecutionProvider"])]
    assert encoder.cpu_threads == expected_threads
    assert encoder.input_size == 336
    assert encoder.backend == "CPUExecutionProvider"


def test_transformers_auto_device_uses_cuda_when_available(monkeypatch):
    loaded = []

    class Periods:
        def __init__(self):
            self.conversions = []

        def to(self, dtype):
            self.conversions.append(dtype)
            return self

    class Model:
        def __init__(self):
            self.timm_model = SimpleNamespace(rope=SimpleNamespace(periods=Periods()))

        def to(self, device):
            self.device = device
            return self

        def eval(self):
            return self

    class AutoModel:
        @staticmethod
        def from_pretrained(model_path, *, dtype):
            loaded.append((model_path, dtype))
            return Model()

    monkeypatch.setitem(
        sys.modules,
        "torch",
        SimpleNamespace(
            bfloat16="bfloat16",
            cuda=SimpleNamespace(is_available=lambda: True),
            float32="float32",
        ),
    )
    monkeypatch.setitem(sys.modules, "transformers", SimpleNamespace(AutoModel=AutoModel))

    encoder = DinoEncoder(DinoEncoder.TIMM_MODEL, device="auto")
    encoder.load_model()

    assert loaded == [(DinoEncoder.TIMM_MODEL, "float32")]
    assert encoder._model.device == "cuda"
    assert encoder.backend == "cuda"
    assert encoder._model.timm_model.rope.periods.conversions == ["bfloat16", "float32"]


def test_transformers_timm_wrapper_pooler_output_is_a_descriptor(monkeypatch):
    class Tensor:
        def __init__(self, values):
            self.values = np.asarray(values, dtype=np.float32)

        def to(self, device):
            return self

        def __getitem__(self, index):
            return Tensor(self.values[index])

        def float(self):
            return self

        def cpu(self):
            return self

        def numpy(self):
            return self.values

    class InferenceMode:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, traceback):
            return False

    class Model:
        device = "cuda"

        def __call__(self, *, pixel_values):
            assert pixel_values.values.shape == (1, 3, 336, 336)
            return SimpleNamespace(
                last_hidden_state=None,
                pooler_output=Tensor(np.ones((1, 384), dtype=np.float32)),
            )

    monkeypatch.setitem(
        sys.modules,
        "torch",
        SimpleNamespace(
            from_numpy=Tensor,
            inference_mode=InferenceMode,
        ),
    )
    encoder = DinoEncoder(DinoEncoder.TIMM_MODEL, device="cuda")
    encoder._model = Model()

    feature = encoder.encode(np.zeros((16, 16, 3), np.uint8))

    assert feature.shape == (384,)
    assert np.linalg.norm(feature) == pytest.approx(1.0)


def test_transformers_batch_uses_one_forward_and_one_cpu_transfer(monkeypatch):
    transfers = []

    class Tensor:
        def __init__(self, values):
            self.values = np.asarray(values, dtype=np.float32)

        def to(self, device):
            return self

        def __getitem__(self, index):
            return Tensor(self.values[index])

        def float(self):
            return self

        def cpu(self):
            transfers.append(self.values.shape)
            return self

        def numpy(self):
            return self.values

    class InferenceMode:
        def __enter__(self):
            return None

        def __exit__(self, exc_type, exc, traceback):
            return False

    class Model:
        device = "cuda"

        def __init__(self):
            self.calls = 0

        def __call__(self, *, pixel_values):
            self.calls += 1
            assert pixel_values.values.shape == (2, 3, 336, 336)
            descriptors = np.zeros((2, 384), dtype=np.float32)
            descriptors[0, 3] = 2.0
            descriptors[1, 7] = 4.0
            return SimpleNamespace(
                last_hidden_state=Tensor(descriptors[:, None, :]),
                pooler_output=None,
            )

    monkeypatch.setitem(
        sys.modules,
        "torch",
        SimpleNamespace(
            from_numpy=Tensor,
            inference_mode=InferenceMode,
        ),
    )
    encoder = DinoEncoder(DinoEncoder.TIMM_MODEL, device="cuda")
    encoder._model = Model()

    features = encoder.encode_batch(
        [
            np.zeros((16, 16, 3), np.uint8),
            np.full((20, 12, 3), 127, np.uint8),
        ]
    )

    assert encoder._model.calls == 1
    assert transfers == [(2, 384)]
    assert len(features) == 2
    assert features[0][3] == pytest.approx(1.0)
    assert features[1][7] == pytest.approx(1.0)
