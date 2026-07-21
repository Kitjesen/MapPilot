from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODULES_ROOT = PROJECT_ROOT / "modules"
if str(MODULES_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULES_ROOT))

from vehicle_parking_detection.alarm_store import AlarmStore
from vehicle_parking_detection.config import RoiConfig
from vehicle_parking_detection.models import Detection
from vehicle_parking_detection.parking_engine import VehicleParkingEngine
from vehicle_parking_detection.scene_matching import SceneMatcher, frame_descriptor


def vehicle(track_id: int = 1, box=(10.0, 10.0, 40.0, 40.0)) -> Detection:
    return Detection(
        class_id=1,
        class_name="vehicle",
        confidence=0.9,
        box=box,
        track_id=track_id,
    )


class VehicleParkingDetectionTest(unittest.TestCase):
    def test_vehicle_inside_roi_triggers_after_dwell(self):
        roi = RoiConfig(
            id="no_parking_zone_01",
            name="禁停区01",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
        )
        engine = VehicleParkingEngine([roi], min_confidence=0.2)

        self.assertEqual([], engine.evaluate([vehicle()], now=1000.0))
        triggers = engine.evaluate([vehicle()], now=1005.0)

        self.assertEqual(1, len(triggers))
        self.assertEqual("车辆违停", triggers[0].event_name)
        self.assertEqual("禁停区01", triggers[0].location)
        self.assertEqual("no_parking_zone_01", triggers[0].roi_id)

    def test_vehicle_outside_roi_does_not_trigger(self):
        roi = RoiConfig(
            id="no_parking_zone_01",
            name="禁停区01",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
        )
        engine = VehicleParkingEngine([roi], min_confidence=0.2)
        outside = vehicle(box=(150.0, 10.0, 190.0, 40.0))

        engine.evaluate([outside], now=1000.0)
        self.assertEqual([], engine.evaluate([outside], now=1020.0))

    def test_cooldown_prevents_duplicate_alarm(self):
        roi = RoiConfig(
            id="no_parking_zone_01",
            name="禁停区01",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
        )
        engine = VehicleParkingEngine([roi], min_confidence=0.2)
        engine.evaluate([vehicle()], now=1000.0)
        self.assertEqual(1, len(engine.evaluate([vehicle()], now=1005.0)))
        self.assertEqual([], engine.evaluate([vehicle()], now=1020.0))

    def test_alarm_store_outputs_only_required_schema(self):
        roi = RoiConfig(
            id="no_parking_zone_01",
            name="禁停区01",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
        )
        engine = VehicleParkingEngine([roi], min_confidence=0.2)
        engine.evaluate([vehicle()], now=1000.0)
        trigger = engine.evaluate([vehicle()], now=1005.0)[0]

        with tempfile.TemporaryDirectory() as tmp:
            store = AlarmStore(tmp)
            payload = store.save(np.zeros((40, 60, 3), dtype=np.uint8), trigger)
            self.assertEqual({"keyframe_image", "time", "event_name", "location"}, set(payload))
            self.assertEqual("车辆违停", payload["event_name"])
            self.assertEqual("禁停区01", payload["location"])
            self.assertTrue((Path(tmp) / payload["keyframe_image"]).is_file())
            rows = (Path(tmp) / "alarm_events.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(payload, json.loads(rows[0]))

    def test_scene_matcher_selects_only_matching_scene_roi(self):
        scene_a = np.zeros((240, 320, 3), dtype=np.uint8)
        scene_b = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2 = __import__("cv2")
        cv2.rectangle(scene_a, (30, 40), (260, 160), (30, 180, 230), -1)
        cv2.line(scene_a, (0, 220), (319, 180), (255, 255, 255), 5)
        cv2.circle(scene_b, (220, 90), 55, (220, 50, 40), -1)
        cv2.line(scene_b, (20, 30), (300, 220), (0, 255, 0), 6)
        roi_a = RoiConfig(
            id="no_parking_zone_01",
            name="scene_a",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
            scene_id="scene_a",
            scene_anchor=frame_descriptor(scene_a),
        )
        roi_b = RoiConfig(
            id="no_parking_zone_02",
            name="scene_b",
            polygon=[(110, 0), (200, 0), (200, 100), (110, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
            scene_id="scene_b",
            scene_anchor=frame_descriptor(scene_b),
        )
        matcher = SceneMatcher([roi_a, roi_b], enabled=True, min_score=0.55)

        active_rois, match = matcher.select(scene_b)

        self.assertTrue(match.matched)
        self.assertEqual("scene_b", match.scene_id)
        self.assertEqual(["no_parking_zone_02"], [roi.id for roi in active_rois])

    def test_scene_matcher_holds_last_scene_temporarily(self):
        scene = np.zeros((240, 320, 3), dtype=np.uint8)
        mismatch = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2 = __import__("cv2")
        cv2.rectangle(scene, (30, 40), (260, 160), (30, 180, 230), -1)
        cv2.line(scene, (0, 220), (319, 180), (255, 255, 255), 5)
        roi = RoiConfig(
            id="no_parking_zone_01",
            name="scene_a",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
            scene_id="scene_a",
            scene_anchor=frame_descriptor(scene),
        )
        matcher = SceneMatcher([roi], enabled=True, min_score=0.70, hold_seconds=5.0)

        active_rois, match = matcher.select(scene, now=10.0)
        self.assertTrue(match.matched)
        self.assertEqual(["no_parking_zone_01"], [item.id for item in active_rois])

        active_rois, match = matcher.select(mismatch, now=12.0)
        self.assertTrue(match.matched)
        self.assertEqual("scene_a", match.scene_id)
        self.assertTrue(match.reason.startswith("held_after_match:"))
        self.assertEqual(["no_parking_zone_01"], [item.id for item in active_rois])

        active_rois, match = matcher.select(mismatch, now=16.0)
        self.assertFalse(match.matched)
        self.assertEqual([], active_rois)

    def test_scene_matcher_accepts_extra_scene_anchors_for_same_roi(self):
        primary = np.zeros((240, 320, 3), dtype=np.uint8)
        extra = np.zeros((240, 320, 3), dtype=np.uint8)
        cv2 = __import__("cv2")
        cv2.rectangle(primary, (20, 30), (180, 150), (30, 180, 230), -1)
        cv2.circle(extra, (220, 90), 55, (220, 50, 40), -1)
        cv2.line(extra, (20, 30), (300, 220), (0, 255, 0), 6)
        roi = RoiConfig(
            id="no_parking_zone_01",
            name="scene_a",
            polygon=[(0, 0), (100, 0), (100, 100), (0, 100)],
            dwell_seconds=5,
            cooldown_seconds=300,
            scene_id="scene_a",
            scene_anchor=frame_descriptor(primary),
            scene_anchors=[frame_descriptor(extra)],
        )
        matcher = SceneMatcher([roi], enabled=True, min_score=0.55)

        active_rois, match = matcher.select(extra)

        self.assertTrue(match.matched)
        self.assertEqual("scene_a", match.scene_id)
        self.assertEqual(["no_parking_zone_01"], [item.id for item in active_rois])


if __name__ == "__main__":
    unittest.main()
