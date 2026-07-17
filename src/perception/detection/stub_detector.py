"""
PC-friendly stub detector for testing BPUTracker without BPU hardware.

``StubDetector`` mimics ``BPUDetector.detect()`` by returning a list of
``Detection2D`` with moving, prompt-matched fake objects. Bounding boxes drift
smoothly across frames so that a tracker (BoT-SORT / ByteTrack / native) can
form stable persistent IDs. Optional elliptical instance masks are generated
inside each bbox.

Example::

    from perception.detection.stub_detector import StubDetector
    from perception.tracking.bpu_tracker import BPUTracker
    import numpy as np

    detector = StubDetector(num_objects=3, with_masks=True, seed=0)
    detector.load_model()
    tracker = BPUTracker(detector, tracker_type="native_bytetrack")

    for _ in range(30):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)   # BGR canvas
        tracked = tracker.track(frame, "person . chair")
        for t in tracked:
            print(t.track_id, t.label, t.bbox, t.score)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

# Reuse the canonical COCO label/id mapping so class_id stays consistent with
# the real BPU pipeline (e.g. person -> 0, required by PersonTrackCounter).
from .bpu_detector import _COCO_NAME_TO_ID
from .detector_base import Detection2D, DetectorBase

logger = logging.getLogger(__name__)

# Base class id offset for labels that are not part of the COCO-80 vocabulary.
_NON_COCO_BASE_ID = 1000


@dataclass
class _FakeObject:
    """A single virtual target with smooth linear motion and edge bouncing."""

    label: str
    class_id: int
    cx: float
    cy: float
    vx: float
    vy: float
    w: float
    h: float
    base_score: float

    def step(self, width: int, height: int, rng: np.random.Generator) -> None:
        """Advance one frame; bounce off image borders keeping the box inside."""
        self.cx += self.vx
        self.cy += self.vy

        half_w = self.w / 2.0
        half_h = self.h / 2.0

        if self.cx - half_w < 0.0:
            self.cx = half_w
            self.vx = abs(self.vx)
        elif self.cx + half_w > width:
            self.cx = width - half_w
            self.vx = -abs(self.vx)

        if self.cy - half_h < 0.0:
            self.cy = half_h
            self.vy = abs(self.vy)
        elif self.cy + half_h > height:
            self.cy = height - half_h
            self.vy = -abs(self.vy)

    def bbox_xyxy(self, rng: np.random.Generator, jitter_px: float) -> np.ndarray:
        """Return an [x1, y1, x2, y2] box with small per-frame jitter."""
        jx = rng.uniform(-jitter_px, jitter_px)
        jy = rng.uniform(-jitter_px, jitter_px)
        x1 = self.cx - self.w / 2.0 + jx
        y1 = self.cy - self.h / 2.0 + jy
        x2 = self.cx + self.w / 2.0 + jx
        y2 = self.cy + self.h / 2.0 + jy
        return np.array([x1, y1, x2, y2], dtype=np.float32)


class StubDetector(DetectorBase):
    """Fake detector producing moving, prompt-matched ``Detection2D`` objects.

    Args:
        confidence: Minimum score; produced scores are clamped to be >= this.
        num_objects: Number of virtual targets to spawn per requested label set.
        with_masks: If True, attach an elliptical HxW bool mask per detection.
        image_size: Fallback (height, width) when detect() receives no frame.
        bbox_jitter_px: Per-frame pixel jitter to emulate detector noise.
        drop_prob: Probability of dropping a detection on a given frame (to
            emulate missed detections and exercise the tracker's lost logic).
        seed: RNG seed for reproducible sequences.
    """

    def __init__(
        self,
        confidence: float = 0.25,
        num_objects: int = 3,
        with_masks: bool = True,
        image_size: tuple[int, int] = (480, 640),
        bbox_jitter_px: float = 1.5,
        drop_prob: float = 0.0,
        seed: int = 0,
    ) -> None:
        self._conf_thr = float(confidence)
        self._num_objects = max(int(num_objects), 0)
        self._with_masks = bool(with_masks)
        self._default_h, self._default_w = int(image_size[0]), int(image_size[1])
        self._bbox_jitter_px = max(0.0, float(bbox_jitter_px))
        self._drop_prob = min(max(float(drop_prob), 0.0), 1.0)
        self._seed = int(seed)

        self._rng = np.random.default_rng(self._seed)
        self._loaded = False
        self._objects: list[_FakeObject] = []
        self._spawn_key: tuple[str, ...] | None = None

    def load_model(self) -> None:
        """No real model to load; just mark the detector ready."""
        self._loaded = True
        logger.info(
            "StubDetector ready (num_objects=%d, masks=%s, seed=%d)",
            self._num_objects,
            self._with_masks,
            self._seed,
        )

    def shutdown(self) -> None:
        """Release virtual state."""
        self._objects = []
        self._spawn_key = None
        self._loaded = False

    def reset(self) -> None:
        """Re-seed and clear objects so sequences are reproducible."""
        self._rng = np.random.default_rng(self._seed)
        self._objects = []
        self._spawn_key = None

    def detect(self, rgb: np.ndarray, text_prompt: str) -> list[Detection2D]:
        """Advance virtual objects one frame and return matching detections."""
        if not self._loaded:
            return []

        if rgb is not None and getattr(rgb, "ndim", 0) >= 2:
            height, width = int(rgb.shape[0]), int(rgb.shape[1])
        else:
            height, width = self._default_h, self._default_w

        labels = self._parse_prompt(text_prompt)
        if not labels:
            labels = ["person"]

        self._maybe_spawn(labels, width, height)

        detections: list[Detection2D] = []
        for obj in self._objects:
            obj.step(width, height, self._rng)

            if self._drop_prob > 0.0 and self._rng.random() < self._drop_prob:
                continue

            bbox = obj.bbox_xyxy(self._rng, self._bbox_jitter_px)
            bbox = self._clip_bbox(bbox, width, height)

            score = float(
                np.clip(
                    obj.base_score + self._rng.uniform(-0.05, 0.05),
                    self._conf_thr,
                    0.99,
                )
            )

            mask = None
            if self._with_masks:
                mask = self._elliptical_mask(bbox, width, height)

            detections.append(
                Detection2D(
                    bbox=bbox,
                    score=score,
                    label=obj.label,
                    class_id=obj.class_id,
                    mask=mask,
                )
            )

        return detections

    # -- internals ---------------------------------------------------------

    @staticmethod
    def _parse_prompt(text_prompt: str) -> list[str]:
        """Split a '. '-separated prompt into unique, ordered labels."""
        if not text_prompt:
            return []
        seen: dict[str, None] = {}
        for part in text_prompt.split("."):
            label = part.strip().lower()
            if label and label not in seen:
                seen[label] = None
        return list(seen.keys())

    @staticmethod
    def _class_id_for(label: str, fallback_index: int) -> int:
        """Map a label to a COCO id, else a stable non-COCO id."""
        cid = _COCO_NAME_TO_ID.get(label)
        if cid is not None:
            return int(cid)
        return _NON_COCO_BASE_ID + fallback_index

    def _maybe_spawn(self, labels: list[str], width: int, height: int) -> None:
        """Spawn virtual objects once per unique label set."""
        key = tuple(labels)
        if self._spawn_key == key and self._objects:
            return

        self._spawn_key = key
        self._objects = []
        if self._num_objects == 0:
            return

        for i in range(self._num_objects):
            label = labels[i % len(labels)]
            class_id = self._class_id_for(label, i % len(labels))

            w = float(self._rng.uniform(40.0, min(120.0, width * 0.4)))
            h = float(self._rng.uniform(60.0, min(180.0, height * 0.5)))
            cx = float(self._rng.uniform(w / 2.0, width - w / 2.0))
            cy = float(self._rng.uniform(h / 2.0, height - h / 2.0))
            vx = float(self._rng.uniform(-6.0, 6.0))
            vy = float(self._rng.uniform(-6.0, 6.0))
            base_score = float(self._rng.uniform(0.55, 0.9))

            self._objects.append(
                _FakeObject(
                    label=label,
                    class_id=class_id,
                    cx=cx,
                    cy=cy,
                    vx=vx,
                    vy=vy,
                    w=w,
                    h=h,
                    base_score=base_score,
                )
            )

    @staticmethod
    def _clip_bbox(bbox: np.ndarray, width: int, height: int) -> np.ndarray:
        """Clamp a box to the image bounds keeping it valid."""
        x1 = float(np.clip(bbox[0], 0.0, width - 1.0))
        y1 = float(np.clip(bbox[1], 0.0, height - 1.0))
        x2 = float(np.clip(bbox[2], x1 + 1.0, width))
        y2 = float(np.clip(bbox[3], y1 + 1.0, height))
        return np.array([x1, y1, x2, y2], dtype=np.float32)

    @staticmethod
    def _elliptical_mask(bbox: np.ndarray, width: int, height: int) -> np.ndarray:
        """Build an HxW bool mask with a filled ellipse inside the bbox."""
        mask = np.zeros((height, width), dtype=bool)
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        rx = max((x2 - x1) / 2.0, 1.0)
        ry = max((y2 - y1) / 2.0, 1.0)

        ix1, iy1 = int(np.floor(x1)), int(np.floor(y1))
        ix2, iy2 = int(np.ceil(x2)), int(np.ceil(y2))
        ix1, iy1 = max(ix1, 0), max(iy1, 0)
        ix2, iy2 = min(ix2, width), min(iy2, height)
        if ix2 <= ix1 or iy2 <= iy1:
            return mask

        ys = np.arange(iy1, iy2)[:, None]
        xs = np.arange(ix1, ix2)[None, :]
        ellipse = ((xs - cx) / rx) ** 2 + ((ys - cy) / ry) ** 2 <= 1.0
        mask[iy1:iy2, ix1:ix2] = ellipse
        return mask
