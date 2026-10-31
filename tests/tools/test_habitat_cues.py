"""Recorded-image adapter contracts; detector responses here are test fixtures."""

import json
import math
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image
from tools.datasets.habitat_cues import run

from perception.detection.detector_base import Detection2D
from runtime.registry import get


def test_recorded_rgb_reaches_detector_as_bgr_and_scores_actual_boxes(tmp_path, monkeypatch):
    recording = tmp_path / "recording"
    recording.mkdir()
    Image.new("RGB", (640, 480), (230, 40, 10)).save(recording / "00.png")
    Image.new("RGB", (640, 480), (230, 40, 10)).save(recording / "01.png")
    metadata = {
        "scene": "fixture", "camera": {"width": 640, "height": 480, "fx": 320, "cx": 320,
                                        "hfov_degrees": 90},
        "frames": [{"index": i, "rgb": f"{i:02d}.png", "camera_position": [0, 1.2, 0],
                    "camera_rotation_xyzw": [0, 0, 0, 1]} for i in range(2)],
    }
    (recording / "report.json").write_text(json.dumps(metadata))
    calls = []

    def detect(bgr, prompt):
        assert bgr[0, 0].tolist() == [10, 40, 230]
        assert prompt == "chair"
        calls.append(prompt)
        return [Detection2D(np.array([460, 100, 500, 160]), 0.8, "chair")] if len(calls) == 1 else []

    detector = SimpleNamespace(load_model=lambda: None, detect=detect, shutdown=lambda: None)
    monkeypatch.setattr(get("detector", "yolo_world"), "create", lambda spec: detector)
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(__version__="test-fixture"))
    result = run(recording, tmp_path / "result", prompt="chair", weights=tmp_path / "weights.pt")
    first, empty = result["frames"]
    assert first["detections"][0]["bearing"] == pytest.approx(-math.atan(0.5))
    assert first["detections"][0]["confidence"] == 0.8
    best = max(first["direction_scores"], key=lambda item: item["vl_score"])
    assert best["bearing_degrees"] < 0
    assert empty["detections"] == []
    assert all(item["vl_score"] == 0 for item in empty["direction_scores"])
    assert result["llm_executed"] is False
    assert result["frontier_selection_executed"] is False
    assert result["robot_motion_executed"] is False
