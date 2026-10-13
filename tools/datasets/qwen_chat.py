"""Local Qwen3-VL chat callable for recorded-data experiments."""

from __future__ import annotations

import asyncio
import base64
import io
import time
from contextlib import nullcontext
from pathlib import Path

from PIL import Image


def _stream_context(torch, stream):
    return nullcontext() if stream is None else torch.cuda.stream(stream)


def _stream_synchronize(stream) -> None:
    if stream is not None:
        stream.synchronize()


def model_messages(messages: list[dict], timing: dict | None = None) -> list[dict]:
    """Convert the existing chat/crop message format to Transformers inputs."""
    converted = []
    base64_s = 0.0
    image_s = 0.0
    for message in messages:
        content = message["content"]
        if isinstance(content, str):
            content = [{"type": "text", "text": content}]
        parts = []
        for part in content:
            if part["type"] == "text":
                parts.append({"type": "text", "text": part["text"]})
            elif part["type"] == "image_url":
                url = part["image_url"]["url"]
                if not url.startswith("data:image/"):
                    raise ValueError("Recorded experiment images must be inline data URLs")
                decode_started = time.perf_counter()
                data = base64.b64decode(url.split(",", 1)[1])
                base64_s += time.perf_counter() - decode_started
                image_started = time.perf_counter()
                with Image.open(io.BytesIO(data)) as image:
                    parts.append({"type": "image", "image": image.convert("RGB")})
                image_s += time.perf_counter() - image_started
            else:
                raise ValueError(f"Unsupported chat part: {part['type']}")
        converted.append({"role": message["role"], "content": parts})
    if timing is not None:
        timing.update(base64_decode_s=base64_s, image_pil_decode_s=image_s)
    return converted


class QwenChat:
    """Run real text and image inference without an external API or mock."""

    def __init__(self, model_path: Path | str, device: str = "cuda") -> None:
        import torch
        from transformers import AutoProcessor, Qwen3VLForConditionalGeneration

        self.model_path = str(model_path)
        self.device = device
        self.calls: list[dict] = []
        self.model = Qwen3VLForConditionalGeneration.from_pretrained(
            self.model_path,
            dtype=torch.bfloat16 if device.startswith("cuda") else torch.float32,
            attn_implementation="sdpa",
        ).to(device).eval()
        self._cuda_stream = None
        if device.startswith("cuda") and torch.cuda.is_available():
            with torch.cuda.device(device):
                self._cuda_stream = torch.cuda.Stream()
                self._cuda_stream.wait_stream(torch.cuda.current_stream())
        self.processor = AutoProcessor.from_pretrained(
            self.model_path, min_pixels=64 * 64, max_pixels=256 * 256,
        )

    def _generate(self, messages: list[dict]) -> str:
        import torch

        start = time.perf_counter()
        converted_started = time.perf_counter()
        image_timing = {}
        converted = model_messages(messages, image_timing)
        converted_s = time.perf_counter() - converted_started
        processor_started = time.perf_counter()
        prepared = self.processor.apply_chat_template(
            converted, tokenize=True, add_generation_prompt=True,
            return_dict=True, return_tensors="pt",
        )
        processor_cpu_s = time.perf_counter() - processor_started
        transfer_started = time.perf_counter()
        with _stream_context(torch, self._cuda_stream):
            inputs = prepared.to(self.device)
        _stream_synchronize(self._cuda_stream)
        input_transfer_s = time.perf_counter() - transfer_started
        processor_s = time.perf_counter() - processor_started
        generate_started = time.perf_counter()
        with _stream_context(torch, self._cuda_stream), torch.inference_mode():
            output = self.model.generate(**inputs, max_new_tokens=512, do_sample=False)
        _stream_synchronize(self._cuda_stream)
        generate_s = time.perf_counter() - generate_started
        decode_started = time.perf_counter()
        with _stream_context(torch, self._cuda_stream):
            generated = output[:, inputs["input_ids"].shape[-1]:]
        _stream_synchronize(self._cuda_stream)
        response = self.processor.batch_decode(
            generated, skip_special_tokens=True,
            clean_up_tokenization_spaces=False,
        )[0]
        decode_s = time.perf_counter() - decode_started
        input_tokens = int(inputs["input_ids"].shape[-1])
        output_tokens = int(output.shape[-1] - input_tokens)
        self.calls.append({
            "model": "Qwen3-VL-2B-Instruct", "model_path": self.model_path,
            "messages": messages, "response": response,
            "elapsed_s": time.perf_counter() - start,
            "image_decode_s": converted_s,
            **image_timing,
            "processor_s": processor_s,
            "processor_cpu_s": processor_cpu_s,
            "input_transfer_s": input_transfer_s,
            "generate_s": generate_s,
            "response_decode_s": decode_s,
            "input_tokens": input_tokens,
            "output_tokens": output_tokens,
            "max_new_tokens": 512,
            "image_count": sum(
                part.get("type") == "image_url"
                for message in messages
                for part in (
                    message.get("content", [])
                    if isinstance(message.get("content"), list)
                    else []
                )
            ),
            "mock": False,
        })
        return response

    async def chat(self, messages: list[dict]) -> str:
        return await asyncio.to_thread(self._generate, messages)
