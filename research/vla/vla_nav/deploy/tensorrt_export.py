"""
TensorRT Export for Jetson Orin NX Deployment.

Converts quantized Qwen2.5-VL-3B + custom heads to TensorRT engines
for optimised inference on Jetson.

Strategy:
  - VLM backbone: Export via torch2trt or TensorRT-LLM (if compatible)
  - Custom heads (AdaCoT, ActionHead, VLingMem encoders): Export as ONNX → TensorRT
  - Fallback: Use PyTorch with CUDA and torch.compile for Jetson

Requirements:
  pip install tensorrt onnx onnxruntime-gpu
  (On Jetson: installed via JetPack SDK)

Usage:
  python -m vla_nav.deploy.tensorrt_export \
    --model-path checkpoints/quantized \
    --output-path checkpoints/trt \
    --device jetson
"""

import json
import logging
import os
import time
from pathlib import Path
from typing import Optional

import torch
import torch.nn as nn

logger = logging.getLogger(__name__)


class TensorRTExporter:
    """
    Export VLA model components to TensorRT for Jetson deployment.

    Due to the complexity of VLM models, we use a hybrid approach:
      - VLM backbone: Keep in PyTorch with torch.compile + CUDA graphs
      - Custom heads: Convert to ONNX → TensorRT for maximum speed
    """

    def __init__(
        self,
        model_path: str = "checkpoints/quantized",
        output_path: str = "checkpoints/trt",
        hidden_dim: int = 2048,
        action_horizon: int = 5,
        action_dim: int = 3,
        fp16: bool = True,
        max_workspace_gb: float = 2.0,
    ):
        self.model_path = Path(model_path)
        self.output_path = Path(output_path)
        self.output_path.mkdir(parents=True, exist_ok=True)

        self.hidden_dim = hidden_dim
        self.action_horizon = action_horizon
        self.action_dim = action_dim
        self.fp16 = fp16
        self.max_workspace_bytes = int(max_workspace_gb * (1024**3))

    def export_all(self):
        """Export all model components."""
        logger.info("Starting TensorRT export pipeline")
        t_start = time.time()

        # Export custom heads as ONNX
        self.export_action_head_onnx()
        self.export_adacot_trigger_onnx()

        # Convert ONNX to TensorRT (if available)
        if self._tensorrt_available():
            self.convert_onnx_to_trt("action_head.onnx", "action_head.trt")
            self.convert_onnx_to_trt("adacot_trigger.onnx", "adacot_trigger.trt")
        else:
            logger.warning(
                "TensorRT not available — ONNX files saved for later conversion on Jetson"
            )

        # torch.compile wrapper for backbone
        self.create_compile_wrapper()

        elapsed = time.time() - t_start
        logger.info("Export complete in %.1f sec", elapsed)

    def export_action_head_onnx(self):
        """Export ActionHead MLP to ONNX."""
        from vla_nav.model.action_head import ActionHead

        model = ActionHead(
            hidden_dim=self.hidden_dim,
            action_dim=self.action_dim,
            horizon=self.action_horizon,
        )

        # Load weights if available
        weights_path = self.model_path / "heads" / "action_head.pt"
        if weights_path.exists():
            model.load_state_dict(
                torch.load(weights_path, map_location="cpu", weights_only=True)
            )

        model.eval()
        dummy_input = torch.randn(1, self.hidden_dim)
        onnx_path = self.output_path / "action_head.onnx"

        torch.onnx.export(
            model,
            dummy_input,
            str(onnx_path),
            input_names=["hidden_state"],
            output_names=["waypoints"],
            dynamic_axes={
                "hidden_state": {0: "batch"},
                "waypoints": {0: "batch"},
            },
            opset_version=17,
        )

        logger.info("ActionHead exported to %s", onnx_path)
        self._verify_onnx(onnx_path, dummy_input, model)

    def export_adacot_trigger_onnx(self):
        """Export AdaCoT trigger head to ONNX."""
        from vla_nav.model.adacot import AdaCoTTriggerHead

        model = AdaCoTTriggerHead(hidden_dim=self.hidden_dim)

        weights_path = self.model_path / "heads" / "adacot.pt"
        if weights_path.exists():
            state = torch.load(weights_path, map_location="cpu", weights_only=True)
            trigger_state = {
                k.replace("trigger_head.", ""): v
                for k, v in state.items()
                if k.startswith("trigger_head.")
            }
            if trigger_state:
                model.load_state_dict(trigger_state)

        model.eval()
        dummy_input = torch.randn(1, self.hidden_dim)
        onnx_path = self.output_path / "adacot_trigger.onnx"

        torch.onnx.export(
            model,
            dummy_input,
            str(onnx_path),
            input_names=["hidden_state"],
            output_names=["trigger_logits"],
            dynamic_axes={
                "hidden_state": {0: "batch"},
                "trigger_logits": {0: "batch"},
            },
            opset_version=17,
        )

        logger.info("AdaCoT trigger exported to %s", onnx_path)

    def convert_onnx_to_trt(self, onnx_name: str, trt_name: str):
        """Convert ONNX model to TensorRT engine."""
        try:
            import tensorrt as trt
        except ImportError:
            logger.warning("TensorRT not available, skipping %s", onnx_name)
            return

        onnx_path = self.output_path / onnx_name
        trt_path = self.output_path / trt_name

        if not onnx_path.exists():
            logger.warning("ONNX file not found: %s", onnx_path)
            return

        logger.info("Converting %s → %s", onnx_name, trt_name)

        trt_logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(trt_logger)
        network = builder.create_network(
            1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        )
        parser = trt.OnnxParser(network, trt_logger)

        with open(onnx_path, "rb") as f:
            if not parser.parse(f.read()):
                for i in range(parser.num_errors):
                    logger.error("ONNX parse error: %s", parser.get_error(i))
                return

        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, self.max_workspace_bytes)

        if self.fp16:
            config.set_flag(trt.BuilderFlag.FP16)

        # Build engine
        engine = builder.build_serialized_network(network, config)
        if engine is None:
            logger.error("Failed to build TensorRT engine for %s", onnx_name)
            return

        with open(trt_path, "wb") as f:
            f.write(engine)

        logger.info("TensorRT engine saved: %s (%.1f KB)", trt_path, trt_path.stat().st_size / 1024)

    def create_compile_wrapper(self):
        """Create a torch.compile wrapper script for the backbone."""
        wrapper_code = '''"""
Auto-generated torch.compile wrapper for VLA backbone on Jetson.

Usage:
    from vla_nav.deploy.compiled_backbone import load_compiled_model
    model = load_compiled_model("checkpoints/quantized/model")
"""

import torch
from transformers import Qwen2VLForConditionalGeneration, AutoProcessor


def load_compiled_model(model_path: str, device: str = "cuda"):
    """Load and compile the quantized VLM backbone for Jetson inference."""
    model = Qwen2VLForConditionalGeneration.from_pretrained(
        model_path,
        device_map=device,
        trust_remote_code=True,
    )
    model.eval()

    # torch.compile with Jetson-friendly settings
    try:
        model = torch.compile(
            model,
            mode="reduce-overhead",  # Optimise for latency
            fullgraph=False,         # Allow graph breaks for complex VLM
        )
    except Exception as e:
        print(f"torch.compile not available or failed: {e}")
        print("Falling back to eager mode")

    processor = AutoProcessor.from_pretrained(
        model_path,
        trust_remote_code=True,
    )

    return model, processor
'''
        wrapper_path = self.output_path / "compiled_backbone.py"
        with open(wrapper_path, "w") as f:
            f.write(wrapper_code)
        logger.info("Compile wrapper saved to %s", wrapper_path)

    @staticmethod
    def _tensorrt_available() -> bool:
        try:
            import tensorrt
            return True
        except ImportError:
            return False

    @staticmethod
    def _verify_onnx(onnx_path, dummy_input, pytorch_model):
        """Verify ONNX export matches PyTorch output."""
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(str(onnx_path))
            onnx_output = session.run(
                None, {"hidden_state": dummy_input.numpy()}
            )[0]

            with torch.no_grad():
                pt_output = pytorch_model(dummy_input).numpy()

            max_diff = abs(onnx_output - pt_output).max()
            logger.info("ONNX verification: max diff = %.6f", max_diff)
            if max_diff > 0.01:
                logger.warning("ONNX output differs from PyTorch by %.6f!", max_diff)
        except ImportError:
            logger.info("onnxruntime not available — skipping verification")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Export VLA model to TensorRT")
    parser.add_argument("--model-path", type=str, default="checkpoints/quantized")
    parser.add_argument("--output-path", type=str, default="checkpoints/trt")
    parser.add_argument("--no-fp16", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    exporter = TensorRTExporter(
        model_path=args.model_path,
        output_path=args.output_path,
        fp16=not args.no_fp16,
    )
    exporter.export_all()
