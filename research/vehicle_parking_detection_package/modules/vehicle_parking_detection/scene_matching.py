from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class SceneMatch:
    scene_id: str
    score: float
    margin: float
    matched: bool
    reason: str


def scene_id_for_roi(roi: Any) -> str:
    if isinstance(roi, dict):
        return str(roi.get("scene_id") or roi.get("id") or "")
    return str(getattr(roi, "scene_id", "") or getattr(roi, "id", "") or "")


def scene_anchor_for_roi(roi: Any) -> dict[str, Any] | None:
    if isinstance(roi, dict):
        value = roi.get("scene_anchor")
    else:
        value = getattr(roi, "scene_anchor", None)
    return value if isinstance(value, dict) else None


def scene_anchors_for_roi(roi: Any) -> list[dict[str, Any]]:
    anchors = []
    primary = scene_anchor_for_roi(roi)
    if primary:
        anchors.append(primary)
    if isinstance(roi, dict):
        extra = roi.get("scene_anchors")
    else:
        extra = getattr(roi, "scene_anchors", None)
    if isinstance(extra, list):
        for item in extra:
            if isinstance(item, dict) and item not in anchors:
                anchors.append(item)
    return anchors


def frame_descriptor(image: np.ndarray, max_orb_descriptors: int = 96) -> dict[str, Any]:
    resized = _resize_for_descriptor(image)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    return {
        "schema_version": "vehicle-parking-scene-anchor/v1",
        "method": "phash_hsv_orb",
        "phash": _phash(gray),
        "hsv_hist": _hsv_hist(resized),
        "orb": _orb_descriptors(gray, max_orb_descriptors),
    }


def anchor_for_frame(
    image: np.ndarray,
    *,
    keyframe_image: str,
    frame_index: int,
    second: float,
    original_width: int,
    original_height: int,
) -> dict[str, Any]:
    desc = frame_descriptor(image)
    desc.update(
        {
            "keyframe_image": keyframe_image,
            "frame_index": int(frame_index),
            "second": float(round(second, 3)),
            "frame_width": int(original_width),
            "frame_height": int(original_height),
        }
    )
    return desc


def best_scene_match(image: np.ndarray, rois: list[Any], min_score: float = 0.65, min_margin: float = 0.08) -> SceneMatch:
    anchors = _anchors_by_scene(rois)
    if not anchors:
        return SceneMatch(scene_id="", score=1.0, margin=1.0, matched=True, reason="no_scene_anchors")

    current = frame_descriptor(image)
    best_id = ""
    best_score = -1.0
    second_score = -1.0
    for scene_id, scene_anchors in anchors.items():
        score = max(descriptor_score(current, anchor) for anchor in scene_anchors)
        if score > best_score:
            second_score = best_score
            best_id = scene_id
            best_score = score
        elif score > second_score:
            second_score = score

    margin = best_score - max(0.0, second_score)
    matched = best_score >= float(min_score) and margin >= float(min_margin)
    return SceneMatch(
        scene_id=best_id if matched else "",
        score=max(0.0, float(best_score)),
        margin=max(0.0, float(margin)),
        matched=matched,
        reason="matched" if matched else ("below_threshold" if best_score < float(min_score) else "ambiguous_scene"),
    )


def active_rois_for_scene(rois: list[Any], match: SceneMatch) -> list[Any]:
    has_anchors = any(scene_anchors_for_roi(roi) for roi in rois)
    if not has_anchors:
        return rois
    if not match.matched or not match.scene_id:
        return []
    return [roi for roi in rois if scene_id_for_roi(roi) == match.scene_id]


def descriptor_score(current: dict[str, Any], anchor: dict[str, Any]) -> float:
    phash_score = _phash_score(str(current.get("phash", "")), str(anchor.get("phash", "")))
    hist_score = _hist_score(current.get("hsv_hist"), anchor.get("hsv_hist"))
    orb_score = _orb_score(current.get("orb"), anchor.get("orb"))
    if orb_score > 0:
        return 0.45 * orb_score + 0.30 * phash_score + 0.25 * hist_score
    return 0.55 * phash_score + 0.45 * hist_score


def _resize_for_descriptor(image: np.ndarray) -> np.ndarray:
    height, width = image.shape[:2]
    if width <= 0 or height <= 0:
        return image
    scale = min(1.0, 320.0 / float(max(width, height)))
    if scale >= 0.999:
        return image.copy()
    return cv2.resize(image, (max(1, int(width * scale)), max(1, int(height * scale))), interpolation=cv2.INTER_AREA)


def _phash(gray: np.ndarray) -> str:
    small = cv2.resize(gray, (32, 32), interpolation=cv2.INTER_AREA).astype(np.float32)
    dct = cv2.dct(small)
    low = dct[:8, :8].copy()
    values = low.flatten()[1:]
    median = float(np.median(values))
    bits = (low.flatten() > median).astype(np.uint8)
    value = 0
    for bit in bits.tolist():
        value = (value << 1) | int(bit)
    return f"{value:016x}"


def _hsv_hist(image: np.ndarray) -> list[float]:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [8, 8], [0, 180, 0, 256]).astype(np.float32)
    total = float(hist.sum())
    if total <= 0:
        return [0.0] * 64
    return [round(float(value), 6) for value in (hist.flatten() / total)]


def _orb_descriptors(gray: np.ndarray, max_count: int) -> dict[str, Any]:
    orb = cv2.ORB_create(nfeatures=max_count, scaleFactor=1.2, nlevels=4, fastThreshold=12)
    _keypoints, descriptors = orb.detectAndCompute(gray, None)
    if descriptors is None or len(descriptors) == 0:
        return {"count": 0, "descriptors": []}
    descriptors = descriptors[:max_count]
    return {
        "count": int(len(descriptors)),
        "descriptors": [bytes(row.tolist()).hex() for row in descriptors],
    }


def _phash_score(left: str, right: str) -> float:
    if not left or not right:
        return 0.0
    try:
        diff = int(left, 16) ^ int(right, 16)
    except ValueError:
        return 0.0
    distance = diff.bit_count()
    return max(0.0, 1.0 - distance / 64.0)


def _hist_score(left: Any, right: Any) -> float:
    if not isinstance(left, list) or not isinstance(right, list) or not left or not right:
        return 0.0
    size = min(len(left), len(right))
    left_arr = np.array(left[:size], dtype=np.float32)
    right_arr = np.array(right[:size], dtype=np.float32)
    return float(np.minimum(left_arr, right_arr).sum())


def _orb_score(left: Any, right: Any) -> float:
    left_desc = _decode_orb(left)
    right_desc = _decode_orb(right)
    if left_desc is None or right_desc is None:
        return 0.0
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = matcher.knnMatch(left_desc, right_desc, k=2)
    good = 0
    for pair in matches:
        if len(pair) < 2:
            continue
        first, second = pair
        if first.distance <= 55 and first.distance < 0.78 * second.distance:
            good += 1
    return min(1.0, good / max(8.0, min(len(left_desc), len(right_desc)) * 0.35))


def _decode_orb(value: Any) -> np.ndarray | None:
    if not isinstance(value, dict):
        return None
    raw = value.get("descriptors")
    if not isinstance(raw, list) or not raw:
        return None
    rows = []
    for item in raw:
        try:
            row = np.frombuffer(bytes.fromhex(str(item)), dtype=np.uint8)
        except ValueError:
            continue
        if row.size == 32:
            rows.append(row)
    if not rows:
        return None
    return np.vstack(rows).astype(np.uint8)


def _anchors_by_scene(rois: list[Any]) -> dict[str, list[dict[str, Any]]]:
    anchors: dict[str, list[dict[str, Any]]] = {}
    for roi in rois:
        scene_id = scene_id_for_roi(roi)
        roi_anchors = scene_anchors_for_roi(roi)
        if roi_anchors and scene_id:
            anchors.setdefault(scene_id, []).extend(roi_anchors)
    return anchors


class SceneMatcher:
    def __init__(
        self,
        rois: list[Any],
        enabled: bool = True,
        min_score: float = 0.65,
        min_margin: float = 0.08,
        hold_seconds: float = 0.0,
    ):
        self.rois = rois
        self.enabled = bool(enabled)
        self.min_score = float(min_score)
        self.min_margin = float(min_margin)
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.has_anchors = any(scene_anchors_for_roi(roi) for roi in rois)
        self._held_scene_id = ""
        self._held_until = 0.0

    def select(self, image: np.ndarray, now: float | None = None) -> tuple[list[Any], SceneMatch]:
        if not self.enabled or not self.has_anchors:
            match = SceneMatch(scene_id="", score=1.0, margin=1.0, matched=True, reason="disabled_or_no_anchors")
            return self.rois, match

        current_time = time.monotonic() if now is None else float(now)
        match = best_scene_match(image, self.rois, self.min_score, self.min_margin)
        if match.matched and match.scene_id:
            self._held_scene_id = match.scene_id
            self._held_until = current_time + self.hold_seconds
            return active_rois_for_scene(self.rois, match), match

        if self._held_scene_id and current_time <= self._held_until:
            held_match = SceneMatch(
                scene_id=self._held_scene_id,
                score=match.score,
                margin=match.margin,
                matched=True,
                reason=f"held_after_match:{match.reason}",
            )
            return active_rois_for_scene(self.rois, held_match), held_match

        self._held_scene_id = ""
        self._held_until = 0.0
        return active_rois_for_scene(self.rois, match), match
