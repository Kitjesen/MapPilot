#!/usr/bin/env python3
"""Run the registered YOLO-World detector on Habitat frames and record VL cues."""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from decision.frontiers.vlnav import cue_from_box, vl_score
from perception.backends import DetectorSpec
from runtime.registry import get


def annotate(image: Image.Image, detections: list[dict], angles: list[dict], prompt: str) -> Image.Image:
    width, height = image.size
    canvas = Image.new("RGB", (width, height + 150), "#14202a")
    canvas.paste(image, (0, 0))
    draw = ImageDraw.Draw(canvas)
    for item in detections:
        box = item["bbox_xyxy"]
        draw.rectangle(box, outline="#63ffaf", width=2)
        draw.text((box[0], max(0, box[1] - 13)),
                  f'{item["label"]} {item["confidence"]:.2f} / {item["bearing_degrees"]:+.1f} deg',
                  fill="#63ffaf", stroke_width=1, stroke_fill="black")
    draw.text((12, height + 8), f"Prompt: {prompt} | YOLO-World -> box -> Cue -> VL score", fill="white")
    draw.text((12, height + 25), "Positive angle = left. Direction scores only; no navigation.", fill="white")
    x0, x1, y0, y1 = 24, width - 24, height + 48, height + 122
    draw.line((x0, y1, x1, y1), fill="gray")
    points = [(x0 + i * (x1 - x0) / (len(angles) - 1), y1 - item["vl_score"] * (y1 - y0))
              for i, item in enumerate(angles)]
    draw.line(points, fill="#63ffaf", width=2)
    draw.text((x0, y1 + 5), f'left (+{angles[0]["bearing_degrees"]:.0f})', fill="white")
    draw.text((width / 2 - 25, y1 + 5), "0 deg", fill="white")
    draw.text((x1 - 75, y1 + 5), f'right ({angles[-1]["bearing_degrees"]:.0f})', fill="white")
    return canvas


def run(recording: Path, output: Path, *, prompt: str, weights: Path,
        device: str = "cpu", confidence: float = 0.25) -> dict:
    import torch
    import ultralytics

    torch.set_num_threads(4)
    recording = recording.resolve()
    weights = weights.resolve()
    metadata = json.loads((recording / "report.json").read_text(encoding="utf-8"))
    camera = metadata["camera"]
    hfov = math.radians(camera["hfov_degrees"])
    provider = get("detector", "yolo_world")
    detector = provider.create(DetectorSpec(
        name="yolo_world", model_path=str(weights), confidence=confidence, device=device,
    ))
    output.mkdir(parents=True, exist_ok=True)
    frames = []
    panels = []
    detector.load_model()
    try:
        for frame in metadata["frames"]:
            # habitat_probe records level cameras: OpenGL local +Y must stay world +Y.
            qx, _, qz, _ = frame["camera_rotation_xyzw"]
            if abs(qx) > 1e-6 or abs(qz) > 1e-6:
                raise ValueError("tilted camera requires calibrated ray-to-horizontal projection")
            rgb = Image.open(recording / frame["rgb"]).convert("RGB")
            if rgb.size != (camera["width"], camera["height"]):
                raise ValueError("recorded image dimensions do not match camera intrinsics")
            start = time.perf_counter()
            # The registered detector accepts BGR arrays; PNG input is RGB.
            boxes = detector.detect(np.asarray(rgb)[:, :, ::-1].copy(), prompt)
            elapsed = time.perf_counter() - start
            cues = [cue_from_box(tuple(map(float, item.bbox)), float(item.score),
                                 fx=camera["fx"], cx=camera["cx"]) for item in boxes]
            detections = []
            for item, cue in zip(boxes, cues):
                detections.append({
                    "label": item.label, "bbox_xyxy": list(map(float, item.bbox)),
                    **asdict(cue), "bearing_degrees": math.degrees(cue.bearing),
                    "vl_score_at_bearing": vl_score(cue.bearing, cues, hfov),
                })
            angles = [{"bearing_degrees": math.degrees(float(angle)),
                       "vl_score": vl_score(float(angle), cues, hfov)}
                      for angle in np.linspace(hfov / 2, -hfov / 2, 91)]
            panel = annotate(rgb, detections, angles, prompt)
            name = f'{frame["index"]:02d}.jpg'
            panel.save(output / name, quality=92)
            panels.append(panel)
            frames.append({
                "index": frame["index"], "rgb": str(recording / frame["rgb"]),
                "camera_position": frame["camera_position"],
                "camera_rotation_xyzw": frame["camera_rotation_xyzw"],
                "inference_seconds_including_prompt_encoding": elapsed,
                "detections": detections, "direction_scores": angles, "annotated": name,
            })
            print(json.dumps({"frame": frame["index"], "detections": len(boxes), "seconds": elapsed}), flush=True)
    finally:
        detector.shutdown()
    width, height = panels[0].size
    sheet = Image.new("RGB", (width * 4, height * math.ceil(len(panels) / 4)))
    for i, panel in enumerate(panels):
        sheet.paste(panel, ((i % 4) * width, (i // 4) * height))
    sheet.save(output / "views.jpg", quality=90)
    report = {
        "evidence": "habitat_image_detector_cue_scoring", "scene": metadata["scene"],
        "prompt": prompt, "weights": str(weights), "device": device,
        "ultralytics_version": ultralytics.__version__, "confidence_threshold": confidence,
        "camera": camera, "cue_source": "YOLO-World boxes and confidence; rectified pinhole projection",
        "bearing_convention": "camera-relative radians, positive left; level camera",
        "sigma_radians": 0.1, "frames": frames,
        "detector_executed": True, "llm_executed": False, "segmentation_executed": False,
        "depth_used": False, "dataset_semantic_labels_used": False,
        "frontier_selection_executed": False, "robot_motion_executed": False,
        "motion_authorization": False,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prompt", required=True, help="detector phrases, e.g. chair or water bottle")
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--confidence", type=float, default=0.25)
    args = parser.parse_args()
    run(args.recording, args.output, prompt=args.prompt, weights=args.weights,
        device=args.device, confidence=args.confidence)


if __name__ == "__main__":
    main()
