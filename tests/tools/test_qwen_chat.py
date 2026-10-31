"""The local model adapter must preserve real crop colors and message order."""

import base64
import io
import sys
from contextlib import nullcontext
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from tools.datasets.qwen_chat import (
    QwenChat,
    _stream_context,
    _stream_synchronize,
    model_messages,
)


def test_text_and_inline_crop_messages_keep_roles_and_rgb_colors():
    image = Image.new("RGB", (4, 3), (220, 20, 10))
    output = io.BytesIO()
    image.save(output, format="PNG")
    url = "data:image/png;base64," + base64.b64encode(output.getvalue()).decode()
    messages = [
        {"role": "system", "content": "Parse a follow task"},
        {"role": "user", "content": [
            {"type": "text", "text": "Person 1:"},
            {"type": "image_url", "image_url": {"url": url}},
        ]},
    ]
    converted = model_messages(messages)
    assert converted[0] == {"role": "system", "content": [{"type": "text", "text": "Parse a follow task"}]}
    assert converted[1]["content"][0]["text"] == "Person 1:"
    crop = converted[1]["content"][1]["image"]
    assert crop.mode == "RGB"
    assert crop.size == (4, 3)
    assert crop.getpixel((0, 0)) == (220, 20, 10)


def test_external_image_is_not_silently_dropped_from_selection_inputs():
    with pytest.raises(ValueError, match="inline data URLs"):
        model_messages([{"role": "user", "content": [
            {"type": "image_url", "image_url": {"url": "https://example.com/crop.png"}},
        ]}])


def test_qwen_uses_owned_cuda_stream_without_global_synchronize():
    events = []

    class Context:
        def __enter__(self):
            events.append("enter")

        def __exit__(self, *_args):
            events.append("exit")

    class Cuda:
        def stream(self, stream):
            events.append(("context", stream))
            return Context()

    class Torch:
        cuda = Cuda()

    class Stream:
        def synchronize(self):
            events.append("stream_sync")

    stream = Stream()
    with _stream_context(Torch(), stream):
        events.append("work")
    _stream_synchronize(stream)
    assert events == [
        ("context", stream), "enter", "work", "exit", "stream_sync",
    ]


def test_qwen_cpu_path_does_not_touch_cuda():
    class Torch:
        class Cuda:
            def stream(self, _stream):
                raise AssertionError("CPU path must not touch CUDA")

        cuda = Cuda()

    with _stream_context(Torch(), None):
        pass
    _stream_synchronize(None)


def test_decode_timing_preserves_all_reference_and_candidate_images():
    colors = [(i * 30, 20, 200 - i * 20) for i in range(6)]
    parts = []
    for index, color in enumerate(colors):
        output = io.BytesIO()
        Image.new("RGB", (4, 3), color).save(output, format="PNG")
        parts.extend([
            {"type": "text", "text": str(index)},
            {"type": "image_url", "image_url": {
                "url": "data:image/png;base64," + base64.b64encode(output.getvalue()).decode(),
            }},
        ])
    timing = {}
    converted = model_messages([{"role": "user", "content": parts}], timing)
    assert [part["image"].getpixel((0, 0)) for part in converted[0]["content"]
            if part["type"] == "image"] == colors
    assert set(timing) == {"base64_decode_s", "image_pil_decode_s"}


def test_generation_records_cpu_preparation_separately_from_input_transfer(monkeypatch):
    events = []
    clock = [0.0]

    def now():
        return clock[0]

    class Prepared(dict):
        def to(self, device):
            events.append(("transfer", device))
            clock[0] += 0.007
            return self

    class Processor:
        def apply_chat_template(self, converted, **_kwargs):
            events.append("cpu_prepare")
            assert converted[0]["content"][0]["text"] == "compare people"
            clock[0] += 0.011
            return Prepared(input_ids=np.zeros((1, 5), dtype=np.int64))

        def batch_decode(self, generated, **_kwargs):
            events.append("response_decode")
            assert generated.shape == (1, 2)
            clock[0] += 0.002
            return ["MATCH"]

    class Model:
        def generate(self, **kwargs):
            events.append("generate")
            assert kwargs["max_new_tokens"] == 512
            clock[0] += 0.013
            return np.zeros((1, 7), dtype=np.int64)

    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace(inference_mode=nullcontext))
    monkeypatch.setattr("tools.datasets.qwen_chat.time.perf_counter", now)
    chat = QwenChat.__new__(QwenChat)
    chat.model_path = "test-model"
    chat.device = "cpu"
    chat.calls = []
    chat._cuda_stream = None
    chat.processor = Processor()
    chat.model = Model()
    assert chat._generate([{"role": "user", "content": "compare people"}]) == "MATCH"
    assert events == ["cpu_prepare", ("transfer", "cpu"), "generate", "response_decode"]
    call = chat.calls[0]
    assert call["processor_cpu_s"] == pytest.approx(0.011)
    assert call["input_transfer_s"] == pytest.approx(0.007)
    assert call["processor_s"] == pytest.approx(0.018)
    assert call["generate_s"] == pytest.approx(0.013)
    assert call["input_tokens"] == 5
    assert call["output_tokens"] == 2
