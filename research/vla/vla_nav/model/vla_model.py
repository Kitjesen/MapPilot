"""
VLA Navigation Model — Unified forward pass.

Integrates all four components into a single model:
  1. VLABackbone   — Qwen2.5-VL-3B with dynamic FPS sampling
  2. AdaCoTModule  — Adaptive Chain-of-Thought trigger + generation
  3. VLingMemModule — Persistent cross-modal linguistic memory
  4. ActionHead    — MLP waypoint prediction

The forward pass mirrors VLingNav (arXiv 2601.08665):
  Frame buffer → Dynamic FPS → VLM encode → AdaCoT trigger
  → [if THINK: generate CoT → store in VLingMem]
  → Action Head → waypoints

Reference: VLingNav Section 3 (Methodology)
"""

import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn

from vla_nav.model.backbone import VLABackbone, FrameEntry
from vla_nav.model.adacot import AdaCoTModule, AdaCoTResult
from vla_nav.model.vlingmem import VLingMemModule
from vla_nav.model.action_head import ActionHead

logger = logging.getLogger(__name__)


@dataclass
class VLANavOutput:
    """Output of a single VLA inference step."""
    waypoints: torch.Tensor           # (1, horizon, 3) — (dx, dy, dtheta)
    world_waypoints: list             # List[(x, y, theta)] in world frame
    adacot_result: AdaCoTResult       # THINK/NO_THINK decision + CoT text
    memory_text: str                  # Retrieved memory context
    inference_time_ms: float          # Total inference time


class VLANavModel(nn.Module):
    """
    End-to-end VLA Navigation Model.

    This is the top-level module that orchestrates all components.
    It provides:
      - push_frame()       — feed new RGB frames
      - set_instruction()  — set the current navigation goal
      - step()             — run one inference cycle → waypoints
      - reset()            — clear all state for a new episode

    Integration with ROS2:
      The VLANavNode creates one VLANavModel instance and calls step()
      on each frame (or at the configured control_hz).
    """

    def __init__(
        self,
        backbone_name: str = "Qwen/Qwen2.5-VL-3B-Instruct",
        device: str = "cuda",
        dtype: str = "float16",
        # Dynamic FPS
        fs_max: float = 10.0,
        stability: float = 5.0,
        max_frames: int = 32,
        grid_pool_g: float = 3.0,
        grid_pool_base_stride: int = 2,
        max_buffer_seconds: float = 30.0,
        # AdaCoT
        adacot_enabled: bool = True,
        trigger_threshold: float = 0.5,
        max_cot_tokens: int = 128,
        cot_temperature: float = 0.3,
        # VLingMem
        mem_enabled: bool = True,
        mem_max_entries: int = 100,
        mem_top_k: int = 5,
        mem_eviction: str = "fifo",
        # Action Head
        action_horizon: int = 5,
        max_linear_step: float = 0.5,
        max_angular_step: float = 0.785,
    ):
        super().__init__()

        # ---- Backbone ----
        self.backbone = VLABackbone(
            model_name=backbone_name,
            device=device,
            dtype=dtype,
            fs_max=fs_max,
            stability=stability,
            max_frames=max_frames,
            grid_pool_g=grid_pool_g,
            grid_pool_base_stride=grid_pool_base_stride,
            max_buffer_seconds=max_buffer_seconds,
        )

        # Hidden dim placeholder — updated after model load
        hidden_dim = 2048

        # ---- AdaCoT ----
        self.adacot_enabled = adacot_enabled
        self.adacot = AdaCoTModule(
            hidden_dim=hidden_dim,
            trigger_threshold=trigger_threshold,
            max_cot_tokens=max_cot_tokens,
            cot_temperature=cot_temperature,
        )

        # ---- VLingMem ----
        self.mem_enabled = mem_enabled
        self.memory = VLingMemModule(
            hidden_dim=hidden_dim,
            max_entries=mem_max_entries,
            top_k=mem_top_k,
            eviction_strategy=mem_eviction,
        )

        # ---- Action Head ----
        self.action_head = ActionHead(
            hidden_dim=hidden_dim,
            action_dim=3,
            horizon=action_horizon,
            max_linear_step=max_linear_step,
            max_angular_step=max_angular_step,
        )

        # ---- State ----
        self._instruction: str = ""
        self._robot_position: np.ndarray = np.zeros(3)
        self._robot_heading: float = 0.0
        self._step_count: int = 0
        self._loaded: bool = False

    # ---- Loading -------------------------------------------------------------

    def load(self, quantized_path: Optional[str] = None):
        """Load the VLM backbone and move all heads to device."""
        if quantized_path:
            self.backbone.load_quantized(quantized_path)
        else:
            self.backbone.load_model()

        # Update hidden dim from actual model config
        hdim = self.backbone.hidden_dim
        self._reinit_heads(hdim)

        self._loaded = True
        logger.info("VLANavModel fully loaded (hidden_dim=%d)", hdim)

    def _reinit_heads(self, hidden_dim: int):
        """Re-initialise heads if hidden_dim differs from placeholder."""
        device = self.backbone.device_name

        self.adacot = AdaCoTModule(
            hidden_dim=hidden_dim,
            trigger_threshold=self.adacot.trigger_threshold,
            max_cot_tokens=self.adacot.cot_generator.max_tokens,
            cot_temperature=self.adacot.cot_generator.temperature,
        ).to(device)

        self.memory = VLingMemModule(
            hidden_dim=hidden_dim,
            max_entries=self.memory.max_entries,
            top_k=self.memory.top_k,
            eviction_strategy=self.memory.eviction_strategy,
        ).to(device)

        self.action_head = ActionHead(
            hidden_dim=hidden_dim,
            action_dim=self.action_head.action_dim,
            horizon=self.action_head.horizon,
            max_linear_step=self.action_head.max_linear_step,
            max_angular_step=self.action_head.max_angular_step,
        ).to(device)

    @classmethod
    def from_config(cls, config: dict) -> "VLANavModel":
        """Create from a flat config dictionary (e.g. from ROS2 params)."""
        model_cfg = config.get("model", {})
        fps_cfg = config.get("dynamic_fps", {})
        ada_cfg = config.get("adacot", {})
        mem_cfg = config.get("vlingmem", {})
        act_cfg = config.get("action_head", {})

        return cls(
            backbone_name=model_cfg.get("backbone", "Qwen/Qwen2.5-VL-3B-Instruct"),
            device=model_cfg.get("device", "cuda"),
            dtype=model_cfg.get("dtype", "float16"),
            fs_max=fps_cfg.get("fs_max", 10.0),
            stability=fps_cfg.get("stability", 5.0),
            max_frames=fps_cfg.get("max_frames", 32),
            grid_pool_g=fps_cfg.get("grid_pool_stability", 3.0),
            grid_pool_base_stride=fps_cfg.get("grid_pool_base_stride", 2),
            adacot_enabled=ada_cfg.get("enabled", True),
            trigger_threshold=ada_cfg.get("trigger_threshold", 0.5),
            max_cot_tokens=ada_cfg.get("max_cot_tokens", 128),
            cot_temperature=ada_cfg.get("temperature", 0.3),
            mem_enabled=mem_cfg.get("enabled", True),
            mem_max_entries=mem_cfg.get("max_entries", 100),
            mem_top_k=mem_cfg.get("top_k_retrieval", 5),
            mem_eviction=mem_cfg.get("eviction_strategy", "fifo"),
            action_horizon=act_cfg.get("horizon", 5),
            max_linear_step=act_cfg.get("max_linear_step", 0.5),
            max_angular_step=act_cfg.get("max_angular_step", 0.785),
        )

    # ---- State management ----------------------------------------------------

    def set_instruction(self, instruction: str):
        """Set the current navigation instruction."""
        self._instruction = instruction
        self._step_count = 0
        logger.info("New instruction: %s", instruction[:100])

    def update_robot_pose(self, x: float, y: float, z: float, heading: float):
        """Update the robot's current world-frame pose."""
        self._robot_position = np.array([x, y, z])
        self._robot_heading = heading

    def push_frame(self, image: np.ndarray, timestamp: Optional[float] = None):
        """Push a new RGB frame into the rolling buffer."""
        ts = timestamp if timestamp is not None else time.monotonic()
        entry = FrameEntry(
            image=image,
            timestamp=ts,
            position=self._robot_position.copy(),
            heading=self._robot_heading,
        )
        self.backbone.push_frame(entry)

    def reset(self):
        """Clear all state for a new navigation episode."""
        self.backbone.clear_buffer()
        self.memory.clear()
        self.adacot.reset_statistics()
        self._instruction = ""
        self._step_count = 0
        self._robot_position = np.zeros(3)
        self._robot_heading = 0.0
        logger.info("VLANavModel reset")

    # ---- Inference -----------------------------------------------------------

    @torch.no_grad()
    def step(
        self,
        force_think: bool = False,
        current_time: Optional[float] = None,
    ) -> VLANavOutput:
        """
        Run one complete inference step.

        Pipeline:
          1. Dynamic FPS sample → select frames
          2. Retrieve memories (if enabled)
          3. VLM encode (frames + instruction + memory text)
          4. AdaCoT trigger decision
          5. [If THINK]: generate CoT → store in memory
          6. Action Head → predict waypoints
          7. Convert to world coordinates

        Returns:
            VLANavOutput with waypoints, CoT result, timing, etc.
        """
        if not self._loaded:
            raise RuntimeError("Model not loaded. Call load() first.")

        t_start = time.monotonic()
        t_now = current_time if current_time is not None else t_start
        self._step_count += 1

        # 1. Sample frames
        sampled_frames = self.backbone.get_sampled_frames(t_now)
        if not sampled_frames:
            logger.warning("No frames in buffer — returning zero waypoints")
            return self._zero_output(t_start)

        # 2. Retrieve memories
        memory_text = ""
        if self.mem_enabled and self.memory.size > 0:
            # Use a dummy hidden state for first retrieval — will be refined
            dummy_h = torch.zeros(
                1, self.backbone.hidden_dim,
                device=self.backbone.device_name,
            )
            retrieved = self.memory.retrieve(dummy_h)
            memory_text = self.memory.format_memory_text(
                retrieved, self._robot_position
            )

        # 3. VLM encode
        hidden_state = self.backbone.encode(
            instruction=self._instruction,
            sampled_frames=sampled_frames,
            current_time=t_now,
            memory_text=memory_text,
        )

        # (Refine memory retrieval with actual hidden state)
        if self.mem_enabled and self.memory.size > 0:
            retrieved = self.memory.retrieve(hidden_state)
            memory_text = self.memory.format_memory_text(
                retrieved, self._robot_position
            )

        # 4. AdaCoT trigger
        if self.adacot_enabled:
            current_images = [f.image for f in sampled_frames[-3:]]
            obs_summary = self._build_observation_summary(sampled_frames)

            adacot_result = self.adacot.step(
                hidden_state=hidden_state,
                backbone=self.backbone,
                instruction=self._instruction,
                observation_summary=obs_summary,
                memory_summary=memory_text,
                current_images=current_images,
                force_think=force_think,
            )

            # 5. If THINK: store CoT in memory
            if adacot_result.should_think and self.mem_enabled:
                self.memory.store(
                    cot_text=adacot_result.cot_text,
                    cot_hidden_states=None,  # Not available without full forward
                    visual_feature=hidden_state,
                    position=self._robot_position,
                    heading=self._robot_heading,
                    timestamp=t_now,
                )
        else:
            from vla_nav.model.adacot import AdaCoTResult
            adacot_result = AdaCoTResult(
                should_think=False,
                trigger_prob=0.0,
                hidden_state=hidden_state,
            )

        # 6. Action Head → waypoints
        waypoints = self.action_head(hidden_state)

        # 7. Convert to world coordinates
        world_wp = self.action_head.waypoints_to_world(
            waypoints,
            robot_x=float(self._robot_position[0]),
            robot_y=float(self._robot_position[1]),
            robot_theta=self._robot_heading,
        )

        t_end = time.monotonic()
        inference_ms = (t_end - t_start) * 1000.0

        logger.debug(
            "VLA step #%d: %s, %.1f ms, wp0=(%.2f, %.2f, %.2f°)",
            self._step_count,
            "THINK" if adacot_result.should_think else "NO_THINK",
            inference_ms,
            world_wp[0][0],
            world_wp[0][1],
            np.degrees(world_wp[0][2]),
        )

        return VLANavOutput(
            waypoints=waypoints,
            world_waypoints=world_wp,
            adacot_result=adacot_result,
            memory_text=memory_text,
            inference_time_ms=inference_ms,
        )

    # ---- Helpers -------------------------------------------------------------

    def _build_observation_summary(self, frames: List[FrameEntry]) -> str:
        """Build a short text summary of the most recent observations."""
        if not frames:
            return "No observations yet."
        latest = frames[-1]
        pos = latest.position if latest.position is not None else np.zeros(3)
        return (
            f"At ({pos[0]:.1f}, {pos[1]:.1f}), step {self._step_count}, "
            f"{len(frames)} frames in view"
        )

    def _zero_output(self, t_start: float) -> VLANavOutput:
        from vla_nav.model.adacot import AdaCoTResult
        return VLANavOutput(
            waypoints=torch.zeros(1, self.action_head.horizon, 3),
            world_waypoints=[(0.0, 0.0, 0.0)] * self.action_head.horizon,
            adacot_result=AdaCoTResult(should_think=False, trigger_prob=0.0),
            memory_text="",
            inference_time_ms=(time.monotonic() - t_start) * 1000.0,
        )

    def get_statistics(self) -> dict:
        return {
            "step_count": self._step_count,
            "instruction": self._instruction[:50],
            "adacot": self.adacot.get_statistics(),
            "memory": self.memory.get_statistics(),
            "buffer_size": len(self.backbone._frame_buffer),
        }

    # ---- Save / Load trained heads -------------------------------------------

    def save_heads(self, directory: str):
        """Save only the trained heads (AdaCoT trigger, VLingMem encoders, ActionHead)."""
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)
        torch.save(self.adacot.state_dict(), path / "adacot.pt")
        torch.save(self.memory.state_dict(), path / "vlingmem.pt")
        torch.save(self.action_head.state_dict(), path / "action_head.pt")
        logger.info("Heads saved to %s", directory)

    def load_heads(self, directory: str):
        """Load pre-trained heads."""
        path = Path(directory)
        device = self.backbone.device_name

        adacot_path = path / "adacot.pt"
        if adacot_path.exists():
            self.adacot.load_state_dict(
                torch.load(adacot_path, map_location=device, weights_only=True)
            )
            logger.info("Loaded AdaCoT head from %s", adacot_path)

        mem_path = path / "vlingmem.pt"
        if mem_path.exists():
            self.memory.load_state_dict(
                torch.load(mem_path, map_location=device, weights_only=True)
            )
            logger.info("Loaded VLingMem from %s", mem_path)

        action_path = path / "action_head.pt"
        if action_path.exists():
            self.action_head.load_state_dict(
                torch.load(action_path, map_location=device, weights_only=True)
            )
            logger.info("Loaded ActionHead from %s", action_path)
