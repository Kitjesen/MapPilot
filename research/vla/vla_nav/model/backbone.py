"""
VLA Backbone — Qwen2.5-VL-3B with Dynamic FPS Sampling.

Reference: VLingNav (arXiv 2601.08665) Section 3.3.1
  - Dynamic FPS sampling based on Ebbinghaus forgetting curve
  - Grid pooling for historical frame token compression
  - Timestamp-aware indicator tokens via RoPE

Original VLingNav uses LLaVA-Video-7B; we adapt to Qwen2.5-VL-3B for
Jetson Orin NX 16 GB deployment (INT4 ≈ 2-3 GB).
"""

import math
import time
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Frame buffer entry
# ---------------------------------------------------------------------------
@dataclass
class FrameEntry:
    """A timestamped RGB frame stored in the rolling buffer."""
    image: np.ndarray          # (H, W, 3) uint8 BGR
    timestamp: float           # seconds (time.monotonic or ROS stamp)
    position: Optional[np.ndarray] = None   # robot (x, y, z) at capture time
    heading: float = 0.0       # robot yaw (radians)


# ---------------------------------------------------------------------------
# Dynamic FPS Sampler  (Ebbinghaus forgetting curve)
# ---------------------------------------------------------------------------
class DynamicFPSSampler:
    """
    Sample frames from a rolling buffer using the Ebbinghaus forgetting curve.

    Recent frames are sampled at high rate; older frames are sampled
    progressively less often.  This controls the total number of visual
    tokens sent to the VLM while preserving short-term detail.

    VLingNav Eq. (1):  f_s(i) = f_s_max * exp(-ΔT / s)
    """

    def __init__(
        self,
        fs_max: float = 10.0,
        stability: float = 5.0,
        max_frames: int = 32,
    ):
        self.fs_max = fs_max
        self.stability = stability
        self.max_frames = max_frames

    def sample(
        self,
        buffer: List[FrameEntry],
        current_time: float,
    ) -> List[FrameEntry]:
        """Return a deterministic subset of *buffer* following the forgetting curve."""
        if not buffer:
            return []

        scored: List[Tuple[float, FrameEntry]] = []
        for entry in buffer:
            dt = max(current_time - entry.timestamp, 1e-6)
            retention = math.exp(-dt / self.stability)
            scored.append((retention, entry))

        scored.sort(key=lambda x: x[0], reverse=True)

        # Always keep the most recent frame
        sampled = [scored[0][1]]

        # Greedy selection: accept if retention above adaptive threshold
        n_remaining = min(self.max_frames - 1, len(scored) - 1)
        if n_remaining > 0:
            threshold_step = 1.0 / (n_remaining + 1)
            for idx, (retention, entry) in enumerate(scored[1:], start=1):
                adaptive_thresh = 1.0 - idx * threshold_step
                if retention >= adaptive_thresh * scored[0][0]:
                    sampled.append(entry)
                if len(sampled) >= self.max_frames:
                    break

        sampled.sort(key=lambda e: e.timestamp)
        return sampled


# ---------------------------------------------------------------------------
# Grid Pooling  (VLingNav Eq. 2-3)
# ---------------------------------------------------------------------------
class GridPooler:
    """
    Downsample visual feature maps for older historical frames.

    VLingNav Eq. (2):  g(i) = floor(exp(-ΔT / g_param))
    VLingNav Eq. (3):  V'_i  = GridPool(V_i, g(i))

    Older frames get a larger pooling stride → fewer tokens.
    """

    def __init__(self, g_param: float = 3.0, base_stride: int = 2):
        self.g_param = g_param
        self.base_stride = base_stride

    def compute_stride(self, dt: float) -> int:
        """Compute grid-pooling stride for a frame at temporal distance *dt*."""
        raw = math.exp(-dt / self.g_param)
        # raw ∈ (0, 1]: 1 for recent frames → stride 1, small → larger stride
        stride = max(1, int(self.base_stride / max(raw, 0.01)))
        return stride

    @staticmethod
    def pool(features: torch.Tensor, stride: int) -> torch.Tensor:
        """
        Apply 2-D average pooling with *stride*.

        Args:
            features: (1, N_patches, C) or (1, H, W, C) tensor
            stride:   pooling stride (1 = no-op)

        Returns:
            Pooled tensor with reduced spatial dims.
        """
        if stride <= 1:
            return features

        if features.dim() == 3:
            # (1, N, C) — assume square spatial layout
            bsz, n_patches, c = features.shape
            h = w = int(math.sqrt(n_patches))
            if h * w != n_patches:
                # Non-square: fall back to 1-D strided selection
                return features[:, ::stride, :]
            features = features.view(bsz, h, w, c)

        # (B, H, W, C) → (B, C, H, W) for F.avg_pool2d
        x = features.permute(0, 3, 1, 2).contiguous()
        x = F.avg_pool2d(x, kernel_size=stride, stride=stride)
        # Back to (B, H', W', C)
        x = x.permute(0, 2, 3, 1).contiguous()
        # Flatten spatial → (B, N', C)
        bsz = x.shape[0]
        return x.view(bsz, -1, x.shape[-1])


# ---------------------------------------------------------------------------
# VLA Backbone
# ---------------------------------------------------------------------------
class VLABackbone(nn.Module):
    """
    Qwen2.5-VL-3B backbone with dynamic FPS sampling and grid pooling.

    Responsibilities:
      1. Manage a rolling frame buffer
      2. Sample frames via Ebbinghaus forgetting curve
      3. Encode sampled frames + instruction text through the VLM
      4. Return hidden states for downstream AdaCoT / ActionHead
    """

    def __init__(
        self,
        model_name: str = "Qwen/Qwen2.5-VL-3B-Instruct",
        device: str = "cuda",
        dtype: str = "float16",
        fs_max: float = 10.0,
        stability: float = 5.0,
        max_frames: int = 32,
        grid_pool_g: float = 3.0,
        grid_pool_base_stride: int = 2,
        max_buffer_seconds: float = 30.0,
    ):
        super().__init__()
        self.model_name = model_name
        self.device_name = device
        self.dtype_str = dtype
        self.max_buffer_seconds = max_buffer_seconds

        # Frame management
        self.fps_sampler = DynamicFPSSampler(fs_max, stability, max_frames)
        self.grid_pooler = GridPooler(grid_pool_g, grid_pool_base_stride)
        self._frame_buffer: List[FrameEntry] = []

        # Lazy-loaded model and processor
        self._model: Optional[nn.Module] = None
        self._processor = None
        self._hidden_dim: int = 2048   # Updated after model load

    # ---- model loading (lazy) ------------------------------------------------

    def load_model(self):
        """Load Qwen2.5-VL model and processor."""
        from transformers import Qwen2VLForConditionalGeneration, AutoProcessor

        dtype_map = {
            "float16": torch.float16,
            "bfloat16": torch.bfloat16,
            "float32": torch.float32,
        }
        torch_dtype = dtype_map.get(self.dtype_str, torch.float16)

        logger.info("Loading VLM backbone: %s (dtype=%s)", self.model_name, self.dtype_str)
        self._model = Qwen2VLForConditionalGeneration.from_pretrained(
            self.model_name,
            torch_dtype=torch_dtype,
            device_map=self.device_name,
            trust_remote_code=True,
        )
        self._model.eval()
        self._processor = AutoProcessor.from_pretrained(
            self.model_name,
            trust_remote_code=True,
        )

        # Infer hidden dimension from model config
        config = self._model.config
        self._hidden_dim = getattr(config, "hidden_size", 2048)
        logger.info("VLM loaded — hidden_dim=%d", self._hidden_dim)

    def load_quantized(self, path: str):
        """Load a pre-quantized (GPTQ INT4) checkpoint."""
        from transformers import Qwen2VLForConditionalGeneration, AutoProcessor

        logger.info("Loading quantized VLM from %s", path)
        self._model = Qwen2VLForConditionalGeneration.from_pretrained(
            path,
            device_map=self.device_name,
            trust_remote_code=True,
        )
        self._model.eval()
        self._processor = AutoProcessor.from_pretrained(
            path,
            trust_remote_code=True,
        )
        config = self._model.config
        self._hidden_dim = getattr(config, "hidden_size", 2048)
        logger.info("Quantized VLM loaded — hidden_dim=%d", self._hidden_dim)

    @property
    def hidden_dim(self) -> int:
        return self._hidden_dim

    @property
    def model(self):
        if self._model is None:
            raise RuntimeError("Model not loaded. Call load_model() or load_quantized() first.")
        return self._model

    @property
    def processor(self):
        if self._processor is None:
            raise RuntimeError("Processor not loaded.")
        return self._processor

    # ---- frame buffer --------------------------------------------------------

    def push_frame(self, frame: FrameEntry):
        """Append a new frame and evict stale entries."""
        self._frame_buffer.append(frame)
        cutoff = frame.timestamp - self.max_buffer_seconds
        self._frame_buffer = [f for f in self._frame_buffer if f.timestamp >= cutoff]

    def clear_buffer(self):
        self._frame_buffer.clear()

    def get_sampled_frames(self, current_time: Optional[float] = None) -> List[FrameEntry]:
        """Return dynamically sampled subset of the frame buffer."""
        t = current_time if current_time is not None else time.monotonic()
        return self.fps_sampler.sample(self._frame_buffer, t)

    # ---- encoding ------------------------------------------------------------

    @torch.no_grad()
    def encode(
        self,
        instruction: str,
        sampled_frames: Optional[List[FrameEntry]] = None,
        current_time: Optional[float] = None,
        memory_text: str = "",
    ) -> torch.Tensor:
        """
        Encode sampled video frames + instruction through the VLM backbone.

        Returns:
            hidden_states: (1, hidden_dim) — the last hidden state
                           used by AdaCoT trigger head and action head.
        """
        if sampled_frames is None:
            sampled_frames = self.get_sampled_frames(current_time)

        if not sampled_frames:
            return torch.zeros(1, self._hidden_dim, device=self.device_name)

        from PIL import Image as PILImage
        from qwen_vl_utils import process_vision_info

        # Build Qwen2.5-VL multi-image conversation
        t_now = sampled_frames[-1].timestamp
        image_entries = []
        for frame in sampled_frames:
            pil_img = PILImage.fromarray(frame.image[..., ::-1])  # BGR→RGB
            image_entries.append({"type": "image", "image": pil_img})

        # Compose prompt text
        prompt_parts = []
        if memory_text:
            prompt_parts.append(f"Memory: {memory_text}")
        prompt_parts.append(f"Instruction: {instruction}")
        prompt_parts.append("Decide action:")

        messages = [
            {
                "role": "user",
                "content": image_entries + [{"type": "text", "text": "\n".join(prompt_parts)}],
            }
        ]

        text = self._processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        image_inputs, video_inputs = process_vision_info(messages)

        inputs = self._processor(
            text=[text],
            images=image_inputs,
            videos=video_inputs,
            padding=True,
            return_tensors="pt",
        ).to(self.device_name)

        outputs = self._model(**inputs, output_hidden_states=True)
        # Last hidden state of the final token
        last_hidden = outputs.hidden_states[-1]  # (1, seq_len, hidden_dim)
        pooled = last_hidden[:, -1, :]            # (1, hidden_dim)
        return pooled

    @torch.no_grad()
    def generate_text(
        self,
        prompt: str,
        images: Optional[List[np.ndarray]] = None,
        max_new_tokens: int = 128,
        temperature: float = 0.3,
    ) -> str:
        """Generate free-form text (used for CoT reasoning)."""
        from PIL import Image as PILImage
        from qwen_vl_utils import process_vision_info

        content = []
        if images:
            for img in images:
                pil_img = PILImage.fromarray(img[..., ::-1])
                content.append({"type": "image", "image": pil_img})
        content.append({"type": "text", "text": prompt})

        messages = [{"role": "user", "content": content}]
        text = self._processor.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        image_inputs, video_inputs = process_vision_info(messages)

        inputs = self._processor(
            text=[text],
            images=image_inputs,
            videos=video_inputs,
            padding=True,
            return_tensors="pt",
        ).to(self.device_name)

        output_ids = self._model.generate(
            **inputs,
            max_new_tokens=max_new_tokens,
            temperature=temperature,
            do_sample=temperature > 0,
        )
        # Decode only newly generated tokens
        gen_ids = output_ids[:, inputs["input_ids"].shape[1]:]
        return self._processor.batch_decode(gen_ids, skip_special_tokens=True)[0].strip()
