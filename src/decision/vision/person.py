"""Person tracking and re-identification helper."""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from runtime.msgs.numpy_compat import is_numpy_array, np

logger = logging.getLogger(__name__)

# OSNet Re-ID feature dimension
_OSNET_FEAT_DIM = 512

# Adaptive Re-ID thresholds based on crowd density
_REID_THRESHOLD_SPARSE = 0.55
_REID_THRESHOLD_DENSE = 0.70

# Weight of CLIP secondary signal in appearance fusion (0.0 = OSNet only)
_CLIP_FUSION_WEIGHT = 0.4

# Motion prediction horizon (seconds)
_MOTION_PREDICT_DT = 0.3


@dataclass
class TrackedPerson:
    """Tracked target person."""

    position: list[float]  # [x, y, z] EMA-smoothed, world frame
    # Last raw (unsmoothed) measurement, used for velocity finite-difference
    # so that the velocity estimator isn't doubly low-passed through the
    # position EMA.
    last_raw_pos: list[float] | None = None
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0])  # [vx, vy] m/s
    # Local arrival time is only for freshness and operator-visible status.
    last_seen: float = field(default_factory=time.time)
    # Perception source time orders observations and determines velocity dt.
    observation_ts: float | None = None
    confidence: float = 1.0
    # Appearance features for Re-ID after occlusion
    appearance: np.ndarray | None = None  # CLIP image feature, shape (D,)
    osnet_feat: np.ndarray | None = None  # OSNet 512-dim L2-normalised feature
    bbox: list[int] | None = None  # Most recent [x1, y1, x2, y2]
    obj_id: str | None = None  # Scene graph object ID (preferred for matching)


class PersonTracker:
    """Person Tracker."""

    LOST_TIMEOUT = 5.0
    REID_TIMEOUT = 15.0
    EMA_ALPHA = 0.4
    APPEARANCE_ALPHA = 0.1
    MATCH_DIST_THRESHOLD = 2.0
    APPEARANCE_THRESHOLD = 0.6
    REACQUIRE_CONFIRM_FRAMES = 3

    def __init__(self, follow_distance: float = 1.5, lost_timeout: float = 5.0):
        self.follow_distance = follow_distance
        self.lost_timeout = lost_timeout
        self._person: TrackedPerson | None = None
        self._description: str = ""  # user description e.g. "person in red"
        self._target_selected: bool = False  # whether VLM has selected a target
        self._vlm_selecting: bool = False  # VLM selection in progress (re-entry guard)
        self._clip_encoder = None  # CLIP encoder (externally injected)
        self._encoder_lock = threading.Lock()  # guards _clip_encoder injection/reads
        self._lock = threading.Lock()
        self._reacquire_id: str | None = None
        self._reacquire_count = 0

        # OSNet Re-ID encoder (W3-5): primary Re-ID signal.
        # Initialised lazily via set_osnet_encoder() or enable_osnet_reid().
        self._osnet_encoder = None

        # Observability counters for Re-ID signal usage.
        self._reid_stats: dict = {
            "osnet_match": 0,
            "clip_fallback": 0,
            "motion_dominant": 0,
            "lost": 0,
        }

    def set_clip_encoder(self, clip_encoder) -> None:
        """Inject CLIP encoder for secondary appearance feature extraction."""
        with self._encoder_lock:
            self._clip_encoder = clip_encoder

    @property
    def has_image_selector(self) -> bool:
        """Return whether the configured encoder can compare person crops."""
        with self._encoder_lock:
            return bool(
                self._clip_encoder is not None
                and callable(getattr(self._clip_encoder, "encode_text", None))
                and callable(getattr(self._clip_encoder, "encode_image", None))
            )

    def set_osnet_encoder(self, encoder) -> None:
        """Inject a pre-constructed OSNetReIDEncoder (primary Re-ID signal).

        Allows callers to share a single encoder instance across modules.
        """
        self._osnet_encoder = encoder
        logger.info("PersonTracker: OSNet Re-ID encoder injected (backend=%s)", getattr(encoder, "backend", "unknown"))

    def enable_osnet_reid(self) -> bool:
        """Enable osnet reid."""
        try:
            from .reid import OSNetReIDEncoder

            self._osnet_encoder = OSNetReIDEncoder()
            logger.info("PersonTracker: OSNet Re-ID enabled (backend=%s)", self._osnet_encoder.backend)
            return True
        except Exception as exc:
            logger.info("PersonTracker: OSNet Re-ID unavailable (%s), will use CLIP-only Re-ID", exc)
            return False

    _CLIP_SELECT_MIN_SIM = 0.2  # below this, CLIP is not confident enough to lock

    def _clip_prescore(self, description: str, person_crops: list[np.ndarray]) -> list[float]:
        """Cosine similarity of each crop against the description via CLIP.

        Single source of truth for both `select_by_clip` and the CLIP hint
        attached to the VLM prompt. Crops are expected **BGR** (CLIPEncoder /
        cv2 convention). Returns a list aligned with ``person_crops`` (0.0 where
        a crop fails to encode), or an empty list when no CLIP encoder is
        attached or the model can't encode images (e.g. mobileclip).
        """
        with self._encoder_lock:
            encoder = self._clip_encoder
        if encoder is None:
            return []
        try:
            text_feat = encoder.encode_text([description])
        except Exception as exc:
            logger.debug("CLIP text encode failed: %s", exc)
            return []
        if text_feat is None or len(text_feat) == 0:
            return []

        scores: list[float] = []
        for crop in person_crops:
            try:
                img_feat = encoder.encode_image(crop)
                if img_feat is not None and img_feat.size > 0:
                    sim = float(
                        np.dot(text_feat[0], img_feat)
                        / (np.linalg.norm(text_feat[0]) * np.linalg.norm(img_feat) + 1e-9)
                    )
                    scores.append(sim)
                else:
                    scores.append(0.0)
            except Exception:
                scores.append(0.0)
        return scores

    @staticmethod
    def _build_vlm_select_messages(text_prompt: str, person_crops: list[np.ndarray]) -> list[dict]:
        """OpenAI multimodal messages: instruction text + one image per person.

        Crops are expected **BGR** (cv2 convention; ``cv2.imencode`` assumes BGR).
        Falls back to text-only content when image encoding is unavailable, so a
        non-vision LLM still receives the numbered prompt + CLIP hints.
        """
        content: list[dict] = [{"type": "text", "text": text_prompt}]
        try:
            from .vlm_scene import encode_image_b64
        except Exception:
            encode_image_b64 = None  # type: ignore[assignment]

        if encode_image_b64 is not None:
            for i, crop in enumerate(person_crops):
                try:
                    b64 = encode_image_b64(crop)
                except Exception:
                    b64 = None
                if b64:
                    content.append({"type": "text", "text": f"Person {i + 1}:"})
                    content.append(
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{b64}",
                                "detail": "low",
                            },
                        }
                    )
        return [{"role": "user", "content": content}]

    async def choose_target_with_vlm(
        self,
        description: str,
        person_crops: list[np.ndarray],
        llm_chat_fn: Callable,
    ) -> int:
        """Return the best matching crop index without changing tracker state."""
        if not person_crops:
            return -1

        n = len(person_crops)

        clip_scores = self._clip_prescore(description, person_crops)

        text_prompt = (
            f"There are {n} detected people, numbered 1 to {n}. Each number is followed by its crop.\n"
            f"The user wants to follow: {description}\n"
            f"Reply with only one number from 1 to {n}, selecting the best matching person.\n"
            "Reply 0 if none of the people match."
        )
        if clip_scores:
            text_prompt += f"\nCLIP similarity hints: {[f'{s:.2f}' for s in clip_scores]}"

        messages = self._build_vlm_select_messages(text_prompt, person_crops)

        try:
            response = await llm_chat_fn(messages)
            import re

            numbers = re.findall(r"\d+", str(response))
            if numbers:
                idx = int(numbers[0])
                if 1 <= idx <= n:
                    selected = idx - 1  # 0-based
                    logger.info("VLM selected person #%d for '%s'", idx, description)
                    return selected
                elif idx == 0:
                    logger.info("VLM: no matching person for '%s'", description)
                    return -1
        except Exception as e:
            logger.error("VLM selection failed: %s", e)

        if clip_scores:
            best_idx = int(np.argmax(clip_scores))
            if clip_scores[best_idx] > self._CLIP_SELECT_MIN_SIM:
                logger.info(
                    "VLM failed, using CLIP fallback: person #%d (sim=%.2f)",
                    best_idx + 1,
                    clip_scores[best_idx],
                )
                return best_idx

        return -1

    async def select_target_with_vlm(
        self,
        description: str,
        person_crops: list[np.ndarray],
        person_objects: list[dict],
        llm_chat_fn: Callable,
    ) -> int:
        """Select and lock one target using a vision-language model."""
        selected = await self.choose_target_with_vlm(
            description,
            person_crops,
            llm_chat_fn,
        )
        if selected >= 0:
            self.lock_target(person_objects[selected], person_crops[selected])
        return selected

    def select_by_clip(
        self,
        description: str,
        person_crops: list[np.ndarray],
        person_objects: list[dict],
    ) -> int:
        """Select by clip."""
        with self._encoder_lock:
            encoder = self._clip_encoder
        if not person_crops or encoder is None:
            return -1

        self._description = description
        scores = self._clip_prescore(description, person_crops)
        if not scores:
            return -1

        best_idx = int(np.argmax(scores))
        if scores[best_idx] > self._CLIP_SELECT_MIN_SIM:
            logger.info(
                "CLIP selected person #%d for '%s' (sim=%.2f)",
                best_idx + 1,
                description,
                scores[best_idx],
            )
            self.lock_target(person_objects[best_idx], person_crops[best_idx])
            return best_idx
        return -1

    def lock_target(self, obj: dict, crop: np.ndarray | None = None) -> None:
        """Lock a target selected by a trusted caller."""
        pos = obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            pos = [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]

        appearance = None
        osnet_feat = None

        if crop is not None:
            # Extract CLIP feature for secondary Re-ID signal
            with self._encoder_lock:
                clip_encoder = self._clip_encoder
            if clip_encoder is not None:
                try:
                    feat = clip_encoder.encode_image(crop)
                    if feat is not None and feat.size > 0:
                        norm = np.linalg.norm(feat)
                        appearance = feat / norm if norm > 0 else feat
                except Exception:
                    pass

            # Extract OSNet feature as primary Re-ID signal
            if self._osnet_encoder is not None:
                try:
                    osnet_feat = self._osnet_encoder.encode(crop)
                except Exception as exc:
                    logger.debug("OSNet encode on lock_target failed: %s", exc)

        with self._lock:
            self._person = TrackedPerson(
                position=list(pos[:3]),
                last_raw_pos=list(pos[:3]),
                last_seen=time.time(),
                observation_ts=self._get_observation_ts(obj),
                confidence=obj.get("confidence", 1.0),
                appearance=appearance,
                osnet_feat=osnet_feat,
                bbox=obj.get("bbox"),
                obj_id=obj.get("id"),
            )
            self._target_selected = True
            self._reset_reacquire()

    def update(
        self,
        scene_objects: list[dict],
        rgb_frame: np.ndarray | None = None,
    ) -> bool:
        """Update."""
        persons = [
            o for o in scene_objects if o.get("label", "").lower() in ("person", "people", "human", "pedestrian")
        ]
        if not persons:
            with self._lock:
                self._reset_reacquire()
            return False

        with self._lock:
            if self._person is None:
                best = max(persons, key=lambda p: p.get("confidence", 0))
                self._init_person(best)
                return True

            matched = self._match_person(persons, rgb_frame)
            if matched is not None:
                return self._update_tracked(matched, rgb_frame)

            if not self._target_selected:
                nearest = min(
                    persons,
                    key=lambda p: math.hypot(
                        self._get_pos(p)[0] - self._person.position[0],
                        self._get_pos(p)[1] - self._person.position[1],
                    ),
                )
                return self._update_tracked(nearest, rgb_frame)

        return False

    def _adaptive_reid_threshold(self, n_candidates: int) -> float:
        """Adaptive reid threshold."""
        if n_candidates <= 2:
            return _REID_THRESHOLD_SPARSE
        if n_candidates >= 5:
            return _REID_THRESHOLD_DENSE
        # Linear ramp from 0.55 to 0.70 as n goes from 2 to 5
        t = (n_candidates - 2) / 3.0
        return _REID_THRESHOLD_SPARSE + t * (_REID_THRESHOLD_DENSE - _REID_THRESHOLD_SPARSE)

    def _predict_position(self, dt: float = _MOTION_PREDICT_DT) -> list[float]:
        """Linear motion prediction: project current position forward by dt seconds.

        Uses the EMA-smoothed velocity stored in _person.velocity.
        Returns the predicted [x, y, z] position.
        """
        if self._person is None:
            return [0.0, 0.0, 0.0]
        px = self._person.position[0] + self._person.velocity[0] * dt
        py = self._person.position[1] + self._person.velocity[1] * dt
        pz = self._person.position[2] if len(self._person.position) > 2 else 0.0
        return [px, py, pz]

    def _match_person(self, persons: list[dict], rgb_frame: np.ndarray | None) -> dict | None:
        """Match person."""
        if self._person is None:
            return None

        n_candidates = len(persons)

        # Strategy 1: obj_id exact match
        if self._person.obj_id:
            for p in persons:
                if p.get("id") == self._person.obj_id:
                    self._reset_reacquire()
                    return p

        # Strategy 2: motion match. A changed tracker ID is only a candidate;
        # it must remain the same for several frames before identity changes.
        predicted_pos = self._predict_position()
        motion_candidates: list[tuple[float, dict]] = []
        for p in persons:
            pos = self._get_pos(p)
            dist = math.hypot(
                pos[0] - predicted_pos[0],
                pos[1] - predicted_pos[1],
            )
            if dist < self.MATCH_DIST_THRESHOLD:
                motion_candidates.append((dist, p))
        motion_candidates.sort(key=lambda item: item[0])

        if not self._person.obj_id and motion_candidates:
            self._reset_reacquire()
            return motion_candidates[0][1]
        for _, candidate in motion_candidates:
            if not candidate.get("id"):
                self._reset_reacquire()
                return candidate

        # Strategy 3: appearance Re-ID across new tracker IDs.
        if rgb_frame is None:
            return self._motion_reacquire(motion_candidates)

        reid_threshold = self._adaptive_reid_threshold(n_candidates)

        best_reid_p = None
        best_reid_score = reid_threshold  # must exceed threshold to match

        # Capture encoder reference once for thread-safe use within the loop.
        with self._encoder_lock:
            clip_encoder = self._clip_encoder

        for p in persons:
            crop = self._crop_person(rgb_frame, p)
            if crop is None:
                continue

            score = 0.0
            osnet_sim: float | None = None
            clip_sim: float | None = None

            # OSNet primary signal
            if self._osnet_encoder is not None and self._person.osnet_feat is not None:
                try:
                    osnet_feat = self._osnet_encoder.encode(crop)
                    osnet_sim = float(np.dot(self._person.osnet_feat, osnet_feat))
                    score = osnet_sim
                except Exception as exc:
                    logger.debug("OSNet encode failed: %s", exc)

            if clip_encoder is not None and self._person.appearance is not None:
                try:
                    clip_feat = clip_encoder.encode_image(crop)
                    if clip_feat is not None and clip_feat.size > 0:
                        norm = np.linalg.norm(clip_feat)
                        if norm > 0:
                            clip_feat = clip_feat / norm
                        clip_sim = float(np.dot(self._person.appearance, clip_feat))
                        if osnet_sim is not None:
                            # Fuse: OSNet (60%) + CLIP (40%)
                            score = (1.0 - _CLIP_FUSION_WEIGHT) * osnet_sim + _CLIP_FUSION_WEIGHT * clip_sim
                        else:
                            # CLIP only (OSNet unavailable)
                            score = clip_sim
                except Exception:
                    pass

            if score > best_reid_score:
                best_reid_score = score
                best_reid_p = p

        if best_reid_p is not None:
            if not self._confirm_reacquire(best_reid_p):
                return None
            # Attribute the winning signal type to stats
            if self._osnet_encoder is not None and self._person.osnet_feat is not None:
                self._reid_stats["osnet_match"] += 1
            elif clip_encoder is not None:
                self._reid_stats["clip_fallback"] += 1
            else:
                self._reid_stats["motion_dominant"] += 1
            logger.info(
                "PersonTracker: Re-ID matched (score=%.3f, threshold=%.2f, candidates=%d)",
                best_reid_score,
                reid_threshold,
                n_candidates,
            )
            return best_reid_p

        return self._motion_reacquire(motion_candidates)

    def _motion_reacquire(self, candidates: list[tuple[float, dict]]) -> dict | None:
        """Use motion only when one unambiguous new track stays consistent."""
        if len(candidates) != 1:
            self._reset_reacquire()
            self._reid_stats["lost"] += 1
            return None
        candidate = candidates[0][1]
        if not self._confirm_reacquire(candidate):
            return None
        self._reid_stats["motion_dominant"] += 1
        return candidate

    def _confirm_reacquire(self, candidate: dict) -> bool:
        current_id = str(self._person.obj_id or "") if self._person is not None else ""
        candidate_id = str(candidate.get("id") or "")
        if not current_id or candidate_id == current_id:
            self._reset_reacquire()
            return True
        if not candidate_id:
            self._reset_reacquire()
            return False
        if candidate_id == self._reacquire_id:
            self._reacquire_count += 1
        else:
            self._reacquire_id = candidate_id
            self._reacquire_count = 1
        if self._reacquire_count < self.REACQUIRE_CONFIRM_FRAMES:
            return False
        logger.info(
            "PersonTracker: reacquired target as %s after %d frames",
            candidate_id,
            self._reacquire_count,
        )
        self._reset_reacquire()
        return True

    def _reset_reacquire(self) -> None:
        self._reacquire_id = None
        self._reacquire_count = 0

    def _update_tracked(self, obj: dict, rgb_frame: np.ndarray | None) -> bool:
        """Update position, velocity, and appearance features for the matched target."""
        new_pos = self._get_pos(obj)
        now = time.time()
        observation_ts = self._get_observation_ts(obj)
        previous_observation_ts = self._person.observation_ts

        if previous_observation_ts is not None and (
            observation_ts is None or observation_ts <= previous_observation_ts
        ):
            return False

        old_smoothed = self._person.position
        old_raw = self._person.last_raw_pos or old_smoothed

        if observation_ts is not None and previous_observation_ts is not None:
            dt = observation_ts - previous_observation_ts
            self._person.velocity = [
                (new_pos[0] - old_raw[0]) / dt * 0.3 + self._person.velocity[0] * 0.7,
                (new_pos[1] - old_raw[1]) / dt * 0.3 + self._person.velocity[1] * 0.7,
            ]

        # Position EMA
        a = self.EMA_ALPHA
        self._person.position = [
            a * new_pos[0] + (1 - a) * old_smoothed[0],
            a * new_pos[1] + (1 - a) * old_smoothed[1],
            a * new_pos[2] + (1 - a) * old_smoothed[2],
        ]
        self._person.last_raw_pos = list(new_pos[:3])
        self._person.last_seen = now
        self._person.observation_ts = observation_ts
        self._person.confidence = obj.get("confidence", 1.0)
        self._person.obj_id = obj.get("id", self._person.obj_id)
        self._person.bbox = obj.get("bbox", self._person.bbox)

        if rgb_frame is not None:
            crop = self._crop_person(rgb_frame, obj)
            if crop is not None:
                # CLIP appearance EMA update (secondary Re-ID signal)
                with self._encoder_lock:
                    clip_encoder = self._clip_encoder
                if clip_encoder is not None:
                    try:
                        feat = clip_encoder.encode_image(crop)
                        if feat is not None and feat.size > 0:
                            norm = np.linalg.norm(feat)
                            if norm > 0:
                                feat = feat / norm
                            if self._person.appearance is not None:
                                aa = self.APPEARANCE_ALPHA
                                self._person.appearance = aa * feat + (1 - aa) * self._person.appearance
                            else:
                                self._person.appearance = feat
                    except Exception:
                        pass

                # OSNet feature EMA update (primary Re-ID signal)
                if self._osnet_encoder is not None:
                    try:
                        osnet_feat = self._osnet_encoder.encode(crop)
                        if self._person.osnet_feat is not None:
                            aa = self.APPEARANCE_ALPHA
                            updated = aa * osnet_feat + (1 - aa) * self._person.osnet_feat
                            # Re-normalise after EMA blend (features drift off unit sphere)
                            norm = np.linalg.norm(updated)
                            self._person.osnet_feat = updated / norm if norm > 1e-9 else updated
                        else:
                            self._person.osnet_feat = osnet_feat
                    except Exception as exc:
                        logger.debug("OSNet update_tracked encode failed: %s", exc)

        return True

    def _init_person(self, obj: dict) -> None:
        """Init person."""
        pos = self._get_pos(obj)
        self._person = TrackedPerson(
            position=list(pos[:3]),
            last_raw_pos=list(pos[:3]),
            last_seen=time.time(),
            observation_ts=self._get_observation_ts(obj),
            confidence=obj.get("confidence", 1.0),
            obj_id=obj.get("id"),
            bbox=obj.get("bbox"),
        )

    @staticmethod
    def _get_pos(obj: dict) -> list[float]:
        """Get pos."""
        pos = obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            return [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]
        return list(pos[:3]) if len(pos) >= 3 else list(pos) + [0.0] * (3 - len(pos))

    @staticmethod
    def _get_observation_ts(obj: dict) -> float | None:
        value = obj.get("ts")
        return None if value is None else float(value)

    @staticmethod
    def _crop_person(rgb: np.ndarray, obj: dict) -> np.ndarray | None:
        """Crop person."""
        bbox = obj.get("bbox")
        if bbox is None:
            return None
        if is_numpy_array(bbox):
            bbox = bbox.astype(int).tolist()
        elif isinstance(bbox, dict):
            bbox = [int(bbox.get("x1", 0)), int(bbox.get("y1", 0)), int(bbox.get("x2", 0)), int(bbox.get("y2", 0))]

        h, w = rgb.shape[:2]
        x1 = max(0, int(bbox[0]))
        y1 = max(0, int(bbox[1]))
        x2 = min(w, int(bbox[2]))
        y2 = min(h, int(bbox[3]))
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None
        return rgb[y1:y2, x1:x2].copy()

    def get_follow_waypoint(
        self,
        robot_pos: list[float],
        predict_dt: float = 0.3,
    ) -> dict | None:
        """Get follow waypoint."""
        with self._lock:
            if self._person is None or self.is_lost():
                return None

            px = self._person.position[0] + self._person.velocity[0] * predict_dt
            py = self._person.position[1] + self._person.velocity[1] * predict_dt
            pz = self._person.position[2] if len(self._person.position) > 2 else 0.0

            dx = robot_pos[0] - px
            dy = robot_pos[1] - py
            dist = math.hypot(dx, dy)
            if dist <= self.follow_distance:
                robot_z = robot_pos[2] if len(robot_pos) > 2 else pz
                return {"x": robot_pos[0], "y": robot_pos[1], "z": robot_z}

            fx = px + (dx / dist) * self.follow_distance
            fy = py + (dy / dist) * self.follow_distance
            return {"x": fx, "y": fy, "z": pz}

    def is_lost(self) -> bool:
        if self._person is None:
            return True
        return (time.time() - self._person.last_seen) > self.lost_timeout

    def needs_vlm_reselect(self) -> bool:
        """Needs vlm reselect."""
        if self._person is None:
            return True
        elapsed = time.time() - self._person.last_seen
        return elapsed > self.REID_TIMEOUT

    def get_person_position(self) -> list[float] | None:
        with self._lock:
            if self._person and not self.is_lost():
                return list(self._person.position)
        return None

    def status(self) -> dict | None:
        """Return the current tracked person using JSON-ready values."""
        with self._lock:
            if self._person is None:
                return None
            return {
                "id": self._person.obj_id,
                "position": [float(value) for value in self._person.position],
                "velocity": [float(value) for value in self._person.velocity],
                "last_seen": float(self._person.last_seen),
                "confidence": float(self._person.confidence),
            }

    @property
    def target_selected(self) -> bool:
        return self._target_selected

    @property
    def description(self) -> str:
        return self._description

    def reset(self):
        with self._lock:
            self._person = None
            self._description = ""
            self._target_selected = False
            self._vlm_selecting = False
            self._reset_reacquire()
