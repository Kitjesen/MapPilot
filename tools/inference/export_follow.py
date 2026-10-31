"""Export the fixed-person YOLO-World detector used by online following."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

import torch
import torch.nn.functional as F


class FixedPersonContrastiveHead(torch.nn.Module):
    """Preserve source image normalization and fold one text prompt into a 1x1 convolution."""

    def __init__(self, source: torch.nn.Module, text_embedding: torch.Tensor) -> None:
        super().__init__()
        text = F.normalize(text_embedding.detach().float().reshape(1, -1), dim=-1, p=2)
        scale = source.logit_scale.detach().float().exp()
        weight = (text * scale).reshape(1, -1, 1, 1)
        bias = source.bias.detach().float().reshape(1)
        self.register_buffer("weight", weight)
        self.register_buffer("bias", bias)
        self.norm = getattr(source, "norm", None)

    def forward(self, image_features: torch.Tensor, _text: torch.Tensor) -> torch.Tensor:
        normalized = (
            self.norm(image_features)
            if self.norm is not None
            else F.normalize(image_features, dim=1, p=2)
        )
        return F.conv2d(
            normalized,
            self.weight.to(dtype=image_features.dtype),
            self.bias.to(dtype=image_features.dtype),
        )


def fix_person_contrastive_head(model: object) -> str:
    """Replace YOLO-World's fixed-person einsum with an equivalent convolution."""
    world_model = model.model
    text_features = world_model.txt_feats
    if tuple(text_features.shape[:2]) != (1, 1):
        raise RuntimeError(f"expected one fixed text class, got {tuple(text_features.shape)}")
    detection_head = world_model.model[-1]
    normalizations = set()
    for index, source in enumerate(detection_head.cv4):
        normalizations.add("batchnorm" if getattr(source, "norm", None) is not None else "l2")
        replacement = FixedPersonContrastiveHead(source, text_features[0, 0])
        probe = torch.randn(
            1,
            replacement.weight.shape[1],
            3,
            3,
            device=text_features.device,
            dtype=text_features.dtype,
        )
        with torch.inference_mode():
            expected = source(probe, text_features)
            actual = replacement(probe, text_features)
        # BatchNorm and the folded 1x1 path may differ by a few ulps on the
        # probe tensor; exported boxes and scores still require a separate
        # frame-level comparison.
        if not torch.allclose(expected, actual, rtol=5e-5, atol=5e-5):
            difference = float((expected - actual).abs().max())
            raise RuntimeError(f"fixed-person head differs from source at level {index}: {difference}")
        detection_head.cv4[index] = replacement
    if len(normalizations) != 1:
        raise RuntimeError(f"mixed contrastive normalization is unsupported: {sorted(normalizations)}")
    return f"fixed_person_{normalizations.pop()}_conv1x1"


def export_person_detector(model_path: Path, output: Path) -> dict:
    os.environ["ULTRALYTICS_AUTOINSTALL"] = "false"
    from ultralytics import YOLO

    output.parent.mkdir(parents=True, exist_ok=True)
    staged_model = output.with_suffix(".pt")
    if staged_model.resolve() != model_path.resolve():
        shutil.copy2(model_path, staged_model)
    model = YOLO(str(staged_model))
    model.set_classes(["person"])
    classification_head = fix_person_contrastive_head(model)
    exported = Path(
        model.export(
            format="onnx",
            imgsz=640,
            batch=1,
            opset=12,
            simplify=False,
            dynamic=False,
            device="cpu",
        )
    )
    if exported.resolve() != output.resolve():
        exported.replace(output)

    import onnxruntime as ort

    session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    contract = {
        "model": str(model_path),
        "output": str(output),
        "classes": ["person"],
        "image_size": 640,
        "batch": 1,
        "opset": 12,
        "dynamic": False,
        "simplify": False,
        "classification_head": classification_head,
        "inputs": [
            {"name": value.name, "shape": value.shape, "type": value.type}
            for value in session.get_inputs()
        ],
        "outputs": [
            {"name": value.name, "shape": value.shape, "type": value.type}
            for value in session.get_outputs()
        ],
    }
    output.with_suffix(".contract.json").write_text(
        json.dumps(contract, indent=2) + "\n", encoding="utf-8"
    )
    return contract


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(export_person_detector(args.model, args.output), separators=(",", ":")))


if __name__ == "__main__":
    main()
