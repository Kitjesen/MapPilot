"""
GPTQ INT4 Quantization for Jetson Deployment.

Quantizes the fine-tuned Qwen2.5-VL-3B model to INT4 using GPTQ,
reducing memory from ~7GB (BF16) to ~2-3GB (INT4).

Requirements:
  pip install auto-gptq optimum

Usage:
  python -m vla_nav.deploy.quantize \
    --model-path checkpoints/sft/best/backbone_lora \
    --output-path checkpoints/quantized \
    --bits 4

Reference: VLingNav deployed on real robots with quantized inference
"""

import json
import logging
import os
import shutil
import time
from pathlib import Path
from typing import Optional

import torch

logger = logging.getLogger(__name__)


class VLAQuantizer:
    """
    Quantize VLA model for edge deployment on Jetson Orin NX.

    Steps:
      1. Merge LoRA adapters into the base model
      2. Quantize with GPTQ (INT4 / INT8)
      3. Save quantized model + tokenizer
      4. Copy trained heads (AdaCoT, ActionHead, VLingMem)
    """

    def __init__(
        self,
        base_model_name: str = "Qwen/Qwen2.5-VL-3B-Instruct",
        lora_path: Optional[str] = None,
        heads_path: Optional[str] = None,
        output_path: str = "checkpoints/quantized",
        bits: int = 4,
        group_size: int = 128,
        desc_act: bool = True,
        dataset_for_calibration: str = "c4",
        num_calibration_samples: int = 128,
    ):
        self.base_model_name = base_model_name
        self.lora_path = lora_path
        self.heads_path = heads_path
        self.output_path = Path(output_path)
        self.bits = bits
        self.group_size = group_size
        self.desc_act = desc_act
        self.dataset_for_calibration = dataset_for_calibration
        self.num_calibration_samples = num_calibration_samples

    def quantize(self):
        """Run the full quantization pipeline."""
        self.output_path.mkdir(parents=True, exist_ok=True)

        logger.info("Starting quantization pipeline (bits=%d)", self.bits)
        t_start = time.time()

        # Step 1: Load and merge LoRA
        model, tokenizer = self._load_and_merge()

        # Step 2: Quantize
        quantized_model = self._quantize_gptq(model, tokenizer)

        # Step 3: Save
        self._save(quantized_model, tokenizer)

        # Step 4: Copy heads
        if self.heads_path:
            self._copy_heads()

        elapsed = time.time() - t_start
        logger.info("Quantization complete in %.1f min", elapsed / 60)

    def _load_and_merge(self):
        """Load base model and merge LoRA adapters if present."""
        from transformers import AutoTokenizer

        logger.info("Loading base model: %s", self.base_model_name)

        if self.lora_path and os.path.exists(self.lora_path):
            from peft import PeftModel
            from transformers import Qwen2VLForConditionalGeneration

            logger.info("Loading LoRA adapter from: %s", self.lora_path)
            base_model = Qwen2VLForConditionalGeneration.from_pretrained(
                self.base_model_name,
                torch_dtype=torch.float16,
                device_map="auto",
                trust_remote_code=True,
            )
            model = PeftModel.from_pretrained(base_model, self.lora_path)
            model = model.merge_and_unload()
            logger.info("LoRA merged into base model")
        else:
            from transformers import Qwen2VLForConditionalGeneration
            model = Qwen2VLForConditionalGeneration.from_pretrained(
                self.base_model_name,
                torch_dtype=torch.float16,
                device_map="auto",
                trust_remote_code=True,
            )
            logger.info("No LoRA — using base model directly")

        tokenizer = AutoTokenizer.from_pretrained(
            self.base_model_name, trust_remote_code=True
        )

        return model, tokenizer

    def _quantize_gptq(self, model, tokenizer):
        """Apply GPTQ quantization."""
        try:
            from optimum.gptq import GPTQQuantizer
        except ImportError:
            logger.warning(
                "optimum[gptq] not installed. Attempting auto-gptq fallback."
            )
            return self._quantize_auto_gptq(model, tokenizer)

        logger.info(
            "Running GPTQ quantization: bits=%d, group_size=%d, samples=%d",
            self.bits, self.group_size, self.num_calibration_samples,
        )

        quantizer = GPTQQuantizer(
            bits=self.bits,
            group_size=self.group_size,
            desc_act=self.desc_act,
            dataset=self.dataset_for_calibration,
            model_seqlen=2048,
        )

        quantized_model = quantizer.quantize_model(model, tokenizer)
        logger.info("GPTQ quantization complete")
        return quantized_model

    def _quantize_auto_gptq(self, model, tokenizer):
        """Fallback quantization using auto-gptq directly."""
        try:
            from auto_gptq import AutoGPTQForCausalLM, BaseQuantizeConfig
        except ImportError:
            raise ImportError(
                "Neither optimum[gptq] nor auto-gptq installed.\n"
                "  pip install optimum[gptq]   OR\n"
                "  pip install auto-gptq"
            )

        logger.info("Using auto-gptq for quantization")

        quantize_config = BaseQuantizeConfig(
            bits=self.bits,
            group_size=self.group_size,
            desc_act=self.desc_act,
        )

        # Save model temporarily for auto-gptq loading
        tmp_dir = self.output_path / "_tmp_merged"
        tmp_dir.mkdir(exist_ok=True)
        model.save_pretrained(tmp_dir)
        tokenizer.save_pretrained(tmp_dir)

        quantized_model = AutoGPTQForCausalLM.from_pretrained(
            str(tmp_dir),
            quantize_config=quantize_config,
            trust_remote_code=True,
        )

        # Prepare calibration data
        from datasets import load_dataset
        calibration_data = load_dataset(
            self.dataset_for_calibration,
            split=f"train[:{self.num_calibration_samples}]",
        )

        quantized_model.quantize(
            calibration_data,
            use_triton=False,
        )

        # Cleanup
        shutil.rmtree(tmp_dir, ignore_errors=True)

        logger.info("auto-gptq quantization complete")
        return quantized_model

    def _save(self, model, tokenizer):
        """Save the quantized model and tokenizer."""
        model_dir = self.output_path / "model"
        model_dir.mkdir(exist_ok=True)

        logger.info("Saving quantized model to %s", model_dir)
        model.save_pretrained(model_dir)
        tokenizer.save_pretrained(model_dir)

        # Save metadata
        meta = {
            "base_model": self.base_model_name,
            "lora_path": self.lora_path,
            "bits": self.bits,
            "group_size": self.group_size,
            "desc_act": self.desc_act,
        }
        with open(self.output_path / "quantization_meta.json", "w") as f:
            json.dump(meta, f, indent=2)

        # Measure size
        total_size = sum(
            f.stat().st_size for f in model_dir.rglob("*") if f.is_file()
        )
        logger.info(
            "Quantized model size: %.1f GB",
            total_size / (1024**3),
        )

    def _copy_heads(self):
        """Copy trained heads (AdaCoT, ActionHead, VLingMem) to output."""
        heads_src = Path(self.heads_path)
        heads_dst = self.output_path / "heads"
        heads_dst.mkdir(exist_ok=True)

        for name in ["adacot.pt", "action_head.pt", "vlingmem.pt"]:
            src = heads_src / name
            if src.exists():
                shutil.copy2(src, heads_dst / name)
                logger.info("Copied %s to quantized package", name)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Quantize VLA model for Jetson")
    parser.add_argument("--base-model", type=str, default="Qwen/Qwen2.5-VL-3B-Instruct")
    parser.add_argument("--lora-path", type=str, default=None)
    parser.add_argument("--heads-path", type=str, default=None)
    parser.add_argument("--output-path", type=str, default="checkpoints/quantized")
    parser.add_argument("--bits", type=int, default=4, choices=[4, 8])
    parser.add_argument("--group-size", type=int, default=128)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    quantizer = VLAQuantizer(
        base_model_name=args.base_model,
        lora_path=args.lora_path,
        heads_path=args.heads_path,
        output_path=args.output_path,
        bits=args.bits,
        group_size=args.group_size,
    )
    quantizer.quantize()
