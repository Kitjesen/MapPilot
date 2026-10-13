"""DINOv3 crop descriptors for appearance association, separate from CLIP."""

from __future__ import annotations

import ctypes
from pathlib import Path

import numpy as np


class DinoEncoder:
    """Encode BGR person crops with the official ViT-S/16 weights.

    A local ONNX export can run in an existing ONNX Runtime environment. A
    Hugging Face checkpoint uses an existing Transformers environment. Neither
    backend provides text embeddings.
    """

    MODEL = "facebook/dinov3-vits16-pretrain-lvd1689m"
    TIMM_MODEL = "timm/vit_small_patch16_dinov3.lvd1689m"

    def __init__(
        self,
        model_path: str = "",
        device: str = "auto",
        input_size: int = 336,
        cpu_threads: int = 4,
    ):
        self.model_path = model_path or self.MODEL
        self.device = device
        self.input_size = input_size
        self.cpu_threads = cpu_threads
        self._session = None
        self._model = None
        self._trt_library = None
        self._trt_handle = None
        self.backend = "unloaded"

    def load_model(self) -> None:
        if self._session is not None or self._model is not None or self._trt_handle is not None:
            return
        model_path = Path(self.model_path)
        if model_path.suffix == ".engine":
            library_path = model_path.with_name("liblingtu_dino_trt.so")
            if not library_path.is_file():
                raise RuntimeError(f"DINO TensorRT library is missing: {library_path}")
            library = ctypes.CDLL(str(library_path))
            library.dino_create.argtypes = [ctypes.c_char_p]
            library.dino_create.restype = ctypes.c_void_p
            library.dino_encode.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_float),
                ctypes.POINTER(ctypes.c_float),
            ]
            library.dino_encode.restype = ctypes.c_int
            library.dino_destroy.argtypes = [ctypes.c_void_p]
            library.dino_destroy.restype = None
            handle = library.dino_create(str(model_path).encode())
            if not handle:
                raise RuntimeError("DINO TensorRT engine could not be loaded")
            self._trt_library = library
            self._trt_handle = handle
            self.input_size = 336
            self.backend = "TensorRT"
        elif model_path.suffix == ".onnx":
            import onnxruntime as ort

            available = ort.get_available_providers()
            if self.device.startswith("cuda") and "CUDAExecutionProvider" not in available:
                raise RuntimeError("DINO CUDA execution provider is unavailable")
            providers = ["CPUExecutionProvider"]
            if self.device != "cpu" and "CUDAExecutionProvider" in available:
                providers.insert(0, "CUDAExecutionProvider")
            options = ort.SessionOptions()
            options.intra_op_num_threads = self.cpu_threads
            self._session = ort.InferenceSession(self.model_path, options, providers=providers)
            shape = self._session.get_inputs()[0].shape
            self.input_size = int(shape[-1])
            self.backend = self._session.get_providers()[0]
        else:
            import torch
            from transformers import AutoModel

            device = self.device
            if device == "auto":
                device = "cuda" if torch.cuda.is_available() else "cpu"
            self._model = (
                AutoModel.from_pretrained(
                    self.model_path,
                    dtype=torch.float32,
                )
                .to(device)
                .eval()
            )
            timm_model = getattr(self._model, "timm_model", None)
            if timm_model is not None:
                # The public timm DINOv3 checkpoint records RoPE periods in
                # FP32. Its model card requires this round trip to reproduce
                # the original Meta BF16-period features.
                periods = timm_model.rope.periods
                timm_model.rope.periods = periods.to(torch.bfloat16).to(torch.float32)
            self.backend = str(device)

    def encode(self, crop_bgr: np.ndarray) -> np.ndarray:
        """Return a normalized 384-D CLS descriptor for one person crop."""
        return self.encode_batch([crop_bgr])[0]

    def encode_batch(self, crops_bgr: list[np.ndarray]) -> list[np.ndarray]:
        """Return normalized descriptors while batching the Transformers forward pass."""
        if not crops_bgr:
            return []
        if any(crop.ndim != 3 or crop.shape[2] != 3 or crop.size == 0 for crop in crops_bgr):
            raise ValueError("DINO requires a nonempty BGR crop")
        self.load_model()
        pixels = [self._preprocess(crop) for crop in crops_bgr]
        if self._trt_handle is not None:
            features = np.stack([self._encode_trt(item) for item in pixels])
        elif self._session is not None:
            features = np.stack(
                [
                    self._session.run(
                        ["descriptor"],
                        {self._session.get_inputs()[0].name: item},
                    )[0][0]
                    for item in pixels
                ]
            )
        else:
            import torch

            batch = np.concatenate(pixels, axis=0)
            with torch.inference_mode():
                output = self._model(pixel_values=torch.from_numpy(batch).to(self._model.device))
                sequence = getattr(output, "last_hidden_state", None)
                if sequence is not None:
                    descriptors = sequence[:, 0]
                else:
                    descriptors = getattr(output, "pooler_output", None)
                    if descriptors is None:
                        raise RuntimeError("DINO model returned no crop descriptor")
                features = descriptors.float().cpu().numpy()
        features = np.asarray(features, dtype=np.float32)
        if features.shape != (len(crops_bgr), 384):
            raise RuntimeError("DINO produced an invalid crop descriptor")
        normalized = []
        for feature in features:
            norm = float(np.linalg.norm(feature))
            if not np.isfinite(feature).all() or norm <= 0:
                raise RuntimeError("DINO produced an invalid crop descriptor")
            normalized.append(feature / norm)
        return normalized

    def _encode_trt(self, pixels: np.ndarray) -> np.ndarray:
        descriptor = np.empty(384, dtype=np.float32)
        status = self._trt_library.dino_encode(
            self._trt_handle,
            pixels.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            descriptor.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        )
        if status != 0:
            raise RuntimeError(f"DINO TensorRT inference failed: {status}")
        return descriptor

    def shutdown(self) -> None:
        if self._trt_handle is not None:
            self._trt_library.dino_destroy(self._trt_handle)
            self._trt_handle = None
        self._trt_library = None

    def _preprocess(self, crop_bgr: np.ndarray) -> np.ndarray:
        """Apply the existing BGR-to-normalized-tensor crop transform."""
        from PIL import Image

        rgb = Image.fromarray(crop_bgr[:, :, ::-1]).resize(
            (self.input_size, self.input_size), Image.Resampling.BILINEAR
        )
        pixels = np.asarray(rgb, dtype=np.float32) / 255.0
        pixels = (pixels - np.array([0.485, 0.456, 0.406], dtype=np.float32)) / np.array(
            [0.229, 0.224, 0.225], dtype=np.float32
        )
        return np.ascontiguousarray(pixels.transpose(2, 0, 1)[None])
