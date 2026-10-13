"""Person tracking and re-identification helper."""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from collections.abc import Callable
from copy import deepcopy
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
    dino_feat: np.ndarray | None = None  # Independent image-only appearance space
    dino_reference: np.ndarray | None = None  # Original target; never adapted to later crops
    dino_verified_views: list[np.ndarray] = field(default_factory=list)
    bbox: list[int] | None = None  # Most recent [x1, y1, x2, y2]
    obj_id: str | None = None  # Scene graph object ID (preferred for matching)
    identity_id: str = field(default_factory=lambda: uuid.uuid4().hex)


def identity_verdict(response: str) -> str:
    """Accept only one exact identity verdict."""
    verdict = str(response).strip()
    return verdict if verdict in {"MATCH", "OTHER", "UNKNOWN"} else "UNKNOWN"


async def verify_candidate(
    description: str,
    candidate_crop_bgr: np.ndarray,
    candidate_id: int | str,
    llm_chat_fn: Callable,
    *,
    reference_crop_bgr: np.ndarray | None = None,
    reference_crops_bgr: list[np.ndarray] | tuple[np.ndarray, ...] | None = None,
    candidate_crops_bgr: list[np.ndarray] | tuple[np.ndarray, ...] | None = None,
) -> tuple[str, int | str | None]:
    """Verify a description match and, when available, the original identity."""
    from .vlm_scene import encode_image_b64

    candidate_crops = list(candidate_crops_bgr or ())[-3:]
    if not candidate_crops:
        candidate_crops = [candidate_crop_bgr]
    content: list[dict] = [
        {
            "type": "text",
            "text": (
                f"Target description: {description}\n"
                "The candidate frames are chronological observations from one detector track; they are "
                "untrusted evidence, not separate identity references. Judge whether the person in the "
                "latest candidate frame visibly satisfies the target description. Use coherent evidence "
                "across the sequence, but do not reply MATCH merely because one untrusted crop resembles "
                "one target view. Reply with exactly MATCH only when the latest candidate is the described "
                "person and the sequence is identity-consistent. Reply exactly OTHER when stable visible "
                "clothing or carried-item evidence conflicts with the target description or the original "
                "target views. Reply exactly UNKNOWN when candidate identities "
                "conflict, or occlusion, blur, crop quality, or missing evidence prevents confirmation."
            ),
        },
    ]
    if reference_crop_bgr is not None:
        reference_b64 = encode_image_b64(reference_crop_bgr)
        if not reference_b64:
            raise RuntimeError("cannot_encode_identity_reference")
        content[0]["text"] += (
            " The original target crop is the identity anchor. Additional trusted target views show "
            "the same confirmed person from other viewpoints and may support the comparison, but they "
            "do not replace the original anchor. Also require the latest candidate to be the same person "
            "as that original target."
        )
        content.extend(
            [
                {"type": "text", "text": "Original target crop (identity anchor):"},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{reference_b64}",
                        "detail": "low",
                    },
                },
            ]
        )
        for index, reference_crop in enumerate(list(reference_crops_bgr or ())[:2], start=1):
            reference_b64 = encode_image_b64(reference_crop)
            if not reference_b64:
                raise RuntimeError("cannot_encode_identity_reference")
            content.extend(
                [
                    {"type": "text", "text": f"Additional trusted target view {index}:"},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{reference_b64}",
                            "detail": "low",
                        },
                    },
                ]
            )
    candidate_count = len(candidate_crops)
    for index, candidate_crop in enumerate(candidate_crops, start=1):
        candidate_b64 = encode_image_b64(candidate_crop)
        if not candidate_b64:
            raise RuntimeError("cannot_encode_identity_candidate")
        if candidate_count == 1:
            label = "Current selected candidate crop:"
        elif index == 1:
            label = f"Candidate frame {index} of {candidate_count} (earliest):"
        elif index == candidate_count:
            label = (
                f"Candidate frame {index} of {candidate_count} "
                "(latest; judge this person):"
            )
        else:
            label = f"Candidate frame {index} of {candidate_count}:"
        content.extend(
            [
                {"type": "text", "text": label},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{candidate_b64}",
                        "detail": "low",
                    },
                },
            ]
        )
    verdict = identity_verdict(
        await llm_chat_fn([{"role": "user", "content": content}])
    )
    return verdict, candidate_id if verdict == "MATCH" else None


class PersonTracker:
    """Person Tracker."""

    LOST_TIMEOUT = 5.0
    REID_TIMEOUT = 15.0
    EMA_ALPHA = 0.4
    APPEARANCE_ALPHA = 0.1
    MATCH_DIST_THRESHOLD = 2.0
    APPEARANCE_THRESHOLD = 0.6
    REACQUIRE_CONFIRM_FRAMES = 3
    REACQUIRE_GAP = 1.0
    MAX_PERSON_SPEED = 3.0  # m/s, with measurement noise allowed separately
    POSITION_NOISE = 0.2  # m
    MAX_VERIFIED_VIEWS = 8
    MAX_REFERENCE_CROPS = 2
    IDENTITY_EVIDENCE_FRAMES = 3
    IDENTITY_RETRY_INTERVAL = 0.3  # source seconds between candidate image checks
    IDENTITY_OTHER_RETRY_MAX = 1.2
    IDENTITY_SCORE_IMPROVEMENT = 0.08
    REFERENCE_CROP_DUPLICATE_SIMILARITY = 0.98

    def __init__(
        self, follow_distance: float = 1.5, lost_timeout: float = 5.0,
        dino_min_similarity: float = 0.65, dino_margin: float = 0.03,
    ):
        self.follow_distance = follow_distance
        self.lost_timeout = lost_timeout
        self._person: TrackedPerson | None = None
        self._description: str = ""  # user description e.g. "person in red"
        self._target_selected: bool = False  # whether VLM has selected a target
        self._observation_accepted = False
        self._observation_reason = ""
        self._vlm_selecting: bool = False  # VLM selection in progress (re-entry guard)
        self._clip_encoder = None  # CLIP encoder (externally injected)
        self._encoder_lock = threading.Lock()  # guards _clip_encoder injection/reads
        self._lock = threading.Lock()
        self._reacquire_id: str | None = None
        self._reacquire_count = 0
        self._reacquire_ts: float | None = None
        self._reacquire_pos: list[float] | None = None
        self._reacquire_crops: list[np.ndarray] = []
        self._reacquire_crop_timestamps: list[float | None] = []

        # OSNet Re-ID encoder (W3-5): primary Re-ID signal.
        # Initialised lazily via set_osnet_encoder() or enable_osnet_reid().
        self._osnet_encoder = None
        self._dino_encoder = None
        self._dino_min_similarity = float(dino_min_similarity)
        self._dino_margin = float(dino_margin)
        self._dino_similarity: float | None = None
        self._dino_event = "disabled"
        self._frame_dino: dict[tuple, np.ndarray] = {}
        self._reference_crop: np.ndarray | None = None
        self._verified_reference_crops: list[np.ndarray] = []
        self._verified_reference_timestamps: list[float | None] = []
        self._verified_reference_features: list[np.ndarray] = []
        self._pending_identity: dict | None = None
        self._verified_identity: dict | None = None
        self._identity_retry_until: dict[str, float] = {}
        self._identity_retry_state: dict[str, dict] = {}

        # Observability counters for Re-ID signal usage.
        self._reid_stats: dict = {
            "osnet_match": 0,
            "clip_fallback": 0,
            "motion_dominant": 0,
            "lost": 0,
            "dino_match": 0,
            "dino_rejected": 0,
        }

    def set_dino_encoder(self, encoder) -> None:
        """Inject image-only appearance features without replacing CLIP."""
        with self._encoder_lock:
            self._dino_encoder = encoder
        self._dino_event = "ready"

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
    def _build_vlm_select_messages(
        text_prompt: str,
        person_crops: list[np.ndarray],
        reference_crop_bgr: np.ndarray | None = None,
    ) -> list[dict]:
        """OpenAI multimodal messages: instruction text + one image per person.

        Crops are expected **BGR** (cv2 convention; ``cv2.imencode`` assumes BGR).
        Falls back to text-only content when image encoding is unavailable, so a
        non-vision LLM still receives the numbered prompt + CLIP hints.
        """
        content: list[dict] = []
        try:
            from .vlm_scene import encode_image_b64
        except Exception:
            encode_image_b64 = None  # type: ignore[assignment]

        if reference_crop_bgr is not None:
            if encode_image_b64 is None:
                raise RuntimeError("cannot_encode_target_reference")
            reference_b64 = encode_image_b64(reference_crop_bgr)
            if not reference_b64:
                raise RuntimeError("cannot_encode_target_reference")
            content.extend(
                [
                    {
                        "type": "text",
                        "text": (
                            "Reference image of the originally selected target. It is not a numbered "
                            "candidate. Match stable clothing and carried items; ignore pose and image "
                            "position. If the target is absent or the evidence is ambiguous, reply 0. "
                            "Do not choose the closest-looking person."
                        ),
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{reference_b64}",
                            "detail": "low",
                        },
                    },
                    {"type": "text", "text": "Numbered current candidates follow:"},
                ]
            )
        content.append({"type": "text", "text": text_prompt})

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
        *,
        reference_crop_bgr: np.ndarray | None = None,
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

        messages = self._build_vlm_select_messages(
            text_prompt,
            person_crops,
            reference_crop_bgr,
        )

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
        dino_feat = self._provided_dino_feature(obj)

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
            if self._dino_encoder is not None and dino_feat is None:
                dino_feat = self._dino_encoder.encode(crop)

        with self._lock:
            self._person = TrackedPerson(
                position=list(pos[:3]),
                last_raw_pos=list(pos[:3]),
                last_seen=time.time(),
                observation_ts=self._get_observation_ts(obj),
                confidence=obj.get("confidence", 1.0),
                appearance=appearance,
                osnet_feat=osnet_feat,
                dino_feat=dino_feat,
                dino_reference=None if dino_feat is None else dino_feat.copy(),
                bbox=obj.get("bbox"),
                obj_id=obj.get("id"),
            )
            self._target_selected = True
            self._observation_accepted = True
            self._observation_reason = ""
            self._reference_crop = None if crop is None else crop.copy()
            self._verified_reference_crops.clear()
            self._verified_reference_timestamps.clear()
            self._verified_reference_features.clear()
            self._pending_identity = None
            self._verified_identity = None
            self._identity_retry_until.clear()
            self._identity_retry_state.clear()
            self._reset_reacquire()
            self._frame_dino.clear()
            self._dino_event = "locked" if dino_feat is not None else "disabled"
            self._dino_similarity = 1.0 if dino_feat is not None else None

    def update(
        self,
        scene_objects: list[dict],
        rgb_frame: np.ndarray | None = None,
    ) -> bool:
        """Update."""
        self._observation_accepted = False
        self._observation_reason = ""
        persons = [
            o for o in scene_objects if o.get("label", "").lower() in ("person", "people", "human", "pedestrian")
        ]
        if not persons:
            with self._lock:
                self._reset_reacquire()
                self._pending_identity = None
                self._verified_identity = None
                self._identity_retry_until = {
                    key: value for key, value in self._identity_retry_until.items()
                    if self._person is not None and key == str(self._person.obj_id or "")
                }
                self._identity_retry_state = {
                    key: value for key, value in self._identity_retry_state.items()
                    if self._person is not None and key == str(self._person.obj_id or "")
                }
                self._dino_similarity = None
                if self._dino_encoder is not None:
                    self._dino_event = "target_absent"
            return False

        with self._lock:
            self._frame_dino.clear()
            visible_ids = {str(obj.get("id") or "") for obj in persons}
            current_id = "" if self._person is None else str(self._person.obj_id or "")
            self._identity_retry_until = {
                key: value for key, value in self._identity_retry_until.items()
                if key in visible_ids or key == current_id
            }
            self._identity_retry_state = {
                key: value for key, value in self._identity_retry_state.items()
                if key in visible_ids or key == current_id
            }
            if self._pending_identity is not None and str(self._pending_identity["object"].get("id")) not in visible_ids:
                self._pending_identity = None
                self._reset_reacquire()
            if self._verified_identity is not None and str(self._verified_identity["object"].get("id")) not in visible_ids:
                self._verified_identity = None
            if self._person is None:
                best = max(persons, key=lambda p: p.get("confidence", 0))
                self._init_person(best)
                if self._dino_encoder is not None and rgb_frame is not None:
                    self._person.dino_feat = self._dino_feature(rgb_frame, best)
                    feature = self._person.dino_feat
                    self._person.dino_reference = None if feature is None else feature.copy()
                    self._reference_crop = self._crop_person(rgb_frame, best)
                self._observation_accepted = True
                return True

            matched = self._match_person(persons, rgb_frame)
            if matched is not None:
                self._observation_accepted = self._update_tracked(matched, rgb_frame)
                return self._observation_accepted

            if not self._target_selected and not self.has_identity_reference:
                nearest = min(
                    persons,
                    key=lambda p: math.hypot(
                        self._get_pos(p)[0] - self._person.position[0],
                        self._get_pos(p)[1] - self._person.position[1],
                    ),
                )
                self._observation_accepted = self._update_tracked(nearest, rgb_frame)
                return self._observation_accepted

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
        if self._dino_encoder is not None and self._person.dino_feat is not None:
            return self._match_dino(persons, rgb_frame)

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

    def _dino_feature(self, frame: np.ndarray, obj: dict) -> np.ndarray | None:
        bbox = obj.get("bbox", [])
        if isinstance(bbox, dict):
            bbox = [bbox.get(name, 0) for name in ("x1", "y1", "x2", "y2")]
        key = (str(obj.get("id", "")), tuple(np.asarray(bbox).reshape(-1)))
        if key not in self._frame_dino:
            feature = self._provided_dino_feature(obj)
            if feature is None:
                crop = self._crop_person(frame, obj)
                if crop is None:
                    return None
                feature = np.asarray(self._dino_encoder.encode(crop), dtype=np.float32)
            norm = float(np.linalg.norm(feature))
            if not np.isfinite(feature).all() or norm <= 0:
                raise ValueError("invalid DINO appearance feature")
            self._frame_dino[key] = feature / norm
        return self._frame_dino[key]

    @staticmethod
    def _provided_dino_feature(obj: dict) -> np.ndarray | None:
        value = obj.get("dino_feature")
        if value is None:
            return None
        feature = np.asarray(value, dtype=np.float32)
        if feature.size == 0:
            return None
        if feature.shape != (384,):
            raise ValueError("provided DINO appearance feature must have 384 values")
        norm = float(np.linalg.norm(feature))
        if not np.isfinite(feature).all() or norm <= 0:
            raise ValueError("invalid provided DINO appearance feature")
        return feature / norm

    def _retry_evidence_improved(
        self,
        candidate_id: str,
        score: float,
    ) -> bool:
        """Allow an early recheck only when the candidate evidence materially changed."""
        state = self._identity_retry_state.get(candidate_id)
        if state is None:
            return False
        previous_score = state.get("score")
        if previous_score is None:
            return False
        return (
            score >= previous_score + self.IDENTITY_SCORE_IMPROVEMENT
            or (previous_score < self._dino_min_similarity <= score)
        )

    def _match_dino(self, persons: list[dict], frame: np.ndarray | None) -> dict | None:
        """Keep the original identity and expand reachability with elapsed source time."""
        self._dino_similarity = None
        if frame is None:
            self._pending_identity = None
            self._verified_identity = None
            self._reset_reacquire()
            self._dino_event = "missing_image"
            return None
        if self._pending_identity is not None:
            self._dino_event = "awaiting_identity_verification"
            return None
        candidates: list[tuple[float, dict]] = []
        cooling_down = False
        current_id = str(self._person.obj_id or "")
        ordered = sorted(persons, key=lambda candidate: str(candidate.get("id") or "") != current_id)
        eligible = []
        for candidate in ordered:
            candidate_id = str(candidate.get("id") or "")
            timestamp = self._get_observation_ts(candidate)
            previous = self._person.observation_ts
            if timestamp is not None and previous is not None and timestamp <= previous:
                continue
            retry_until = self._identity_retry_until.get(candidate_id)
            gap = 0.0 if timestamp is None or previous is None else max(0.0, timestamp - previous)
            predicted = self._predict_position(min(gap, _MOTION_PREDICT_DT))
            radius = self.MATCH_DIST_THRESHOLD + self.MAX_PERSON_SPEED * max(0.0, gap - _MOTION_PREDICT_DT)
            position = self._get_pos(candidate)
            if math.hypot(position[0] - predicted[0], position[1] - predicted[1]) >= radius:
                continue
            eligible.append((candidate, candidate_id, timestamp, retry_until, gap, position))

        authorized = self._verified_identity
        encode_batch = getattr(self._dino_encoder, "encode_batch", None)
        if (
            authorized is None
            and len(eligible) > 1
            and all(candidate_id != current_id for _, candidate_id, *_ in eligible)
            and callable(encode_batch)
        ):
            batch_candidates = []
            batch_crops = []
            for candidate, *_ in eligible:
                try:
                    feature = self._provided_dino_feature(candidate)
                except ValueError:
                    continue
                if feature is not None:
                    bbox = candidate.get("bbox", [])
                    if isinstance(bbox, dict):
                        bbox = [bbox.get(name, 0) for name in ("x1", "y1", "x2", "y2")]
                    key = (str(candidate.get("id", "")), tuple(np.asarray(bbox).reshape(-1)))
                    self._frame_dino[key] = feature
                    continue
                crop = self._crop_person(frame, candidate)
                if crop is not None:
                    batch_candidates.append(candidate)
                    batch_crops.append(crop)
            if batch_crops:
                try:
                    features = encode_batch(batch_crops)
                except Exception as exc:
                    logger.debug("DINO person crop batch encode failed: %s", exc)
                else:
                    if len(features) == len(batch_candidates):
                        for candidate, feature in zip(batch_candidates, features, strict=True):
                            bbox = candidate.get("bbox", [])
                            if isinstance(bbox, dict):
                                bbox = [bbox.get(name, 0) for name in ("x1", "y1", "x2", "y2")]
                            key = (str(candidate.get("id", "")), tuple(np.asarray(bbox).reshape(-1)))
                            feature = np.asarray(feature, dtype=np.float32)
                            norm = float(np.linalg.norm(feature))
                            if np.isfinite(feature).all() and norm > 0:
                                self._frame_dino[key] = feature / norm

        for candidate, candidate_id, timestamp, retry_until, gap, position in eligible:
            try:
                feature = self._dino_feature(frame, candidate)
            except Exception as exc:
                logger.warning("DINO person crop encode failed: %s", exc)
                self._dino_event = "encode_failed"
                continue
            if feature is not None:
                reference = self._person.dino_reference
                references = [reference if reference is not None else self._person.dino_feat, *self._person.dino_verified_views]
                if authorized is not None and str(authorized["object"].get("id")) == candidate_id:
                    references.append(authorized["feature"])
                score = max(float(np.dot(view, feature)) for view in references)
                if (
                    timestamp is not None
                    and retry_until is not None
                    and timestamp < retry_until
                    and not self._retry_evidence_improved(candidate_id, score)
                ):
                    cooling_down = True
                    continue
                if authorized is not None and str(authorized["object"].get("id")) == candidate_id:
                    verified_ts = self._get_observation_ts(authorized["object"])
                    if timestamp is not None and verified_ts is not None:
                        dt = timestamp - verified_ts
                        if dt <= 0:
                            continue
                        if math.dist(position, self._get_pos(authorized["object"])) > self.MAX_PERSON_SPEED * dt + self.POSITION_NOISE:
                            self._verified_identity = None
                            self._reset_reacquire()
                            continue
                    if score >= self._dino_min_similarity:
                        self._dino_similarity = score
                        self._dino_event = "matched"
                        return candidate
                if (
                    str(candidate.get("id") or "") == current_id
                    and gap <= self.REACQUIRE_GAP
                    and retry_until is None
                    and score >= self._dino_min_similarity
                ):
                    self._reset_reacquire()
                    self._dino_similarity = score
                    self._dino_event = "matched"
                    self._reid_stats["dino_match"] += 1
                    return candidate
                candidates.append((score, candidate))
        candidates.sort(key=lambda item: item[0], reverse=True)
        if not candidates:
            self._reset_reacquire()
            if self._dino_event != "encode_failed":
                self._dino_event = "verification_backoff" if cooling_down else "no_candidate"
            return None
        score, best = candidates[0]
        self._dino_similarity = score
        previous = self._person.observation_ts
        timestamps = [self._get_observation_ts(candidate) for _, candidate in candidates]
        long_absence = previous is not None and any(value is not None and value - previous > self.REACQUIRE_GAP for value in timestamps)
        text_ranked = False
        if len(candidates) > 1 and long_absence and self._description and self.has_image_selector:
            crops = [self._crop_person(frame, candidate) for _, candidate in candidates]
            if all(crop is not None for crop in crops):
                text_scores = self._clip_prescore(self._description, crops)
                if text_scores and max(text_scores) > self._CLIP_SELECT_MIN_SIM:
                    score, best = candidates[int(np.argmax(text_scores))]
                    self._dino_similarity = score
                    text_ranked = True
        if not text_ranked and len(candidates) > 1 and score - candidates[1][0] < self._dino_margin:
            self._reset_reacquire()
            self._reid_stats["dino_rejected"] += 1
            self._dino_event = "ambiguous"
            return None
        crop = self._crop_person(frame, best)
        feature = self._dino_feature(frame, best)
        if crop is None or feature is None or self._reference_crop is None:
            self._dino_event = "missing_image"
            return None
        if not self._confirm_reacquire(best, force=True, crop=crop, feature=feature):
            self._dino_event = "confirming"
            return None
        crops = [value.copy() for value in self._reacquire_crops[-self.IDENTITY_EVIDENCE_FRAMES:]]
        crop_timestamps = list(self._reacquire_crop_timestamps[-self.IDENTITY_EVIDENCE_FRAMES:])
        self._pending_identity = {
            "object": deepcopy(best), "crop": crop,
            "reference_crop": self._reference_crop,
            "reference_crops": [value.copy() for value in self._verified_reference_crops],
            "reference_timestamps": list(self._verified_reference_timestamps),
            "crops": crops,
            "crop_timestamps": crop_timestamps,
            "feature": feature.copy(),
            "score": score,
        }
        self._reset_reacquire()
        self._dino_event = "awaiting_identity_verification"
        return None

    def identity_candidate(self) -> dict | None:
        """Return the stable proposal; it has not changed target identity or position."""
        with self._lock:
            if self._pending_identity is None:
                return None
            return {
                "object": deepcopy(self._pending_identity["object"]),
                "crop": self._pending_identity["crop"].copy(),
                "reference_crop": self._pending_identity["reference_crop"].copy(),
                "reference_crops": [value.copy() for value in self._pending_identity["reference_crops"]],
                "reference_timestamps": list(self._pending_identity["reference_timestamps"]),
                "crops": [value.copy() for value in self._pending_identity["crops"]],
                "crop_timestamps": list(self._pending_identity["crop_timestamps"]),
            }

    def invalidate_reacquisition(self) -> None:
        """Discard in-flight evidence after a perception interruption, retaining identity."""
        with self._lock:
            self._pending_identity = None
            self._verified_identity = None
            self._reset_reacquire()

    def resolve_identity(self, candidate_id: str, source_ts: float | None, verdict: str) -> bool:
        """Queue one verified view; a fresh observation must still pass geometry."""
        with self._lock:
            pending = self._pending_identity
            if pending is None or str(pending["object"].get("id")) != str(candidate_id) or self._get_observation_ts(pending["object"]) != source_ts:
                return False
            self._pending_identity = None
            self._reset_reacquire()
            if verdict == "MATCH":
                self._identity_retry_until.pop(str(candidate_id), None)
                self._identity_retry_state.pop(str(candidate_id), None)
                self._verified_identity = pending
                self._dino_event = "identity_verified"
                return True
            if source_ts is not None:
                previous = self._identity_retry_state.get(str(candidate_id), {})
                attempts = min(int(previous.get("attempts", 0)) + 1, 3)
                interval = min(
                    self.IDENTITY_RETRY_INTERVAL * (2 ** (attempts - 1)),
                    self.IDENTITY_OTHER_RETRY_MAX,
                )
                self._identity_retry_until[str(candidate_id)] = source_ts + interval
                self._identity_retry_state[str(candidate_id)] = {
                    "attempts": attempts,
                    "score": pending.get("score"),
                    "verdict": verdict,
                }
            self._dino_event = "identity_rejected" if verdict == "OTHER" else "identity_unknown"
            return False

    def _motion_reacquire(self, candidates: list[tuple[float, dict]]) -> dict | None:
        """Use motion only when one unambiguous new track stays consistent."""
        if len(candidates) != 1:
            self._reset_reacquire()
            self._reid_stats["lost"] += 1
            return None
        candidate = candidates[0][1]
        if not self._confirm_reacquire(candidate):
            return None
        self._reset_reacquire()
        self._reid_stats["motion_dominant"] += 1
        return candidate

    def _confirm_reacquire(
        self,
        candidate: dict,
        *,
        force: bool = False,
        crop: np.ndarray | None = None,
        feature: np.ndarray | None = None,
    ) -> bool:
        current_id = str(self._person.obj_id or "") if self._person is not None else ""
        candidate_id = str(candidate.get("id") or "")
        if not force and (not current_id or candidate_id == current_id):
            self._reset_reacquire()
            return True
        if not candidate_id:
            self._reset_reacquire()
            return False
        if candidate_id == self._reacquire_id:
            timestamp = self._get_observation_ts(candidate)
            if timestamp is not None and self._reacquire_ts is not None and timestamp <= self._reacquire_ts:
                return False
            if timestamp is not None and self._reacquire_ts is not None and self._reacquire_pos is not None:
                dt = timestamp - self._reacquire_ts
                if math.dist(self._get_pos(candidate), self._reacquire_pos) > self.MAX_PERSON_SPEED * dt + self.POSITION_NOISE:
                    self._reset_reacquire()
                    return False
            self._reacquire_count += 1
        else:
            self._reset_reacquire()
            self._reacquire_id = candidate_id
            self._reacquire_count = 1
        self._reacquire_ts = self._get_observation_ts(candidate)
        self._reacquire_pos = self._get_pos(candidate)
        if crop is not None and feature is not None:
            self._reacquire_crops.append(crop.copy())
            self._reacquire_crop_timestamps.append(self._reacquire_ts)
            self._reacquire_crops = self._reacquire_crops[-self.IDENTITY_EVIDENCE_FRAMES:]
            self._reacquire_crop_timestamps = self._reacquire_crop_timestamps[-self.IDENTITY_EVIDENCE_FRAMES:]
        if self._reacquire_count < self.REACQUIRE_CONFIRM_FRAMES:
            return False
        logger.info(
            "PersonTracker: reacquired target as %s after %d frames",
            candidate_id,
            self._reacquire_count,
        )
        return True

    def _reset_reacquire(self) -> None:
        self._reacquire_id = None
        self._reacquire_count = 0
        self._reacquire_ts = None
        self._reacquire_pos = None
        self._reacquire_crops.clear()
        self._reacquire_crop_timestamps.clear()

    def _retain_reference_crop(
        self,
        crop: np.ndarray,
        feature: np.ndarray,
        source_ts: float | None,
    ) -> None:
        """Keep the two trusted views that provide the widest appearance coverage."""
        def similarity(left: np.ndarray, right: np.ndarray) -> float:
            denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
            return float(np.dot(left, right) / denominator) if denominator > 0 else 1.0

        original = self._person.dino_reference
        known = [value for value in [original, *self._verified_reference_features] if value is not None]
        if any(
            similarity(feature, value) >= self.REFERENCE_CROP_DUPLICATE_SIMILARITY
            for value in known
        ):
            return

        records = list(zip(
            self._verified_reference_crops,
            self._verified_reference_timestamps,
            self._verified_reference_features,
            strict=True,
        ))
        records.append((crop.copy(), source_ts, feature.copy()))
        if len(records) > self.MAX_REFERENCE_CROPS:
            best_pair = (0, 1)
            best_score = -math.inf
            for left in range(len(records) - 1):
                for right in range(left + 1, len(records)):
                    left_feature = records[left][2]
                    right_feature = records[right][2]
                    score = 1.0 - similarity(left_feature, right_feature)
                    if original is not None:
                        score += 1.0 - similarity(original, left_feature)
                        score += 1.0 - similarity(original, right_feature)
                    if score > best_score:
                        best_score = score
                        best_pair = (left, right)
            records = [records[index] for index in best_pair]

        self._verified_reference_crops = [value[0] for value in records]
        self._verified_reference_timestamps = [value[1] for value in records]
        self._verified_reference_features = [value[2] for value in records]

    def _update_tracked(self, obj: dict, rgb_frame: np.ndarray | None) -> bool:
        """Update position, velocity, and appearance features for the matched target."""
        new_pos = self._get_pos(obj)
        now = time.time()
        observation_ts = self._get_observation_ts(obj)
        previous_observation_ts = self._person.observation_ts

        if previous_observation_ts is not None and (
            observation_ts is None or observation_ts <= previous_observation_ts
        ):
            self._observation_reason = "stale_observation"
            return False

        old_smoothed = self._person.position
        old_raw = self._person.last_raw_pos or old_smoothed
        after_gap = False

        if observation_ts is not None and previous_observation_ts is not None:
            dt = observation_ts - previous_observation_ts
            displacement = math.dist(new_pos, old_raw)
            if displacement > self.MAX_PERSON_SPEED * dt + self.POSITION_NOISE:
                self._observation_reason = "position_rejected"
                if self._dino_encoder is not None:
                    self._dino_event = "position_rejected"
                return False
            after_gap = dt > self.REACQUIRE_GAP
            self._person.velocity = [0.0, 0.0] if after_gap else [
                (new_pos[0] - old_raw[0]) / dt * 0.3 + self._person.velocity[0] * 0.7,
                (new_pos[1] - old_raw[1]) / dt * 0.3 + self._person.velocity[1] * 0.7,
            ]

        # Position EMA
        a = 1.0 if after_gap else self.EMA_ALPHA
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
        verified = self._verified_identity
        verified_commit = verified is not None and str(verified["object"].get("id")) == str(obj.get("id"))
        if verified_commit:
            self._person.dino_verified_views.append(verified["feature"].copy())
            self._person.dino_verified_views = self._person.dino_verified_views[-self.MAX_VERIFIED_VIEWS:]
            self._person.dino_feat = verified["feature"].copy()

        if rgb_frame is not None:
            crop = self._crop_person(rgb_frame, obj)
            if crop is not None:
                if verified_commit:
                    fresh_feature = self._dino_feature(rgb_frame, obj)
                    if fresh_feature is not None:
                        self._retain_reference_crop(crop, fresh_feature, observation_ts)
                dino_active = self._dino_encoder is not None and self._person.dino_feat is not None
                if not dino_active:
                    # Maintain fallback appearance only while it owns association.
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

                    if self._osnet_encoder is not None:
                        try:
                            osnet_feat = self._osnet_encoder.encode(crop)
                            if self._person.osnet_feat is not None:
                                aa = self.APPEARANCE_ALPHA
                                updated = aa * osnet_feat + (1 - aa) * self._person.osnet_feat
                                norm = np.linalg.norm(updated)
                                self._person.osnet_feat = updated / norm if norm > 1e-9 else updated
                            else:
                                self._person.osnet_feat = osnet_feat
                        except Exception as exc:
                            logger.debug("OSNet update_tracked encode failed: %s", exc)

                if self._dino_encoder is not None:
                    feature = self._dino_feature(rgb_frame, obj)
                    if feature is not None:
                        previous = self._person.dino_feat
                        updated = feature if previous is None else (
                            self.APPEARANCE_ALPHA * feature + (1 - self.APPEARANCE_ALPHA) * previous
                        )
                        self._person.dino_feat = updated / np.linalg.norm(updated)

        if verified_commit:
            self._verified_identity = None

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
                "native_track_id": self._person.obj_id,
                "identity_id": self._person.identity_id,
                "position": [float(value) for value in self._person.position],
                "velocity": [float(value) for value in self._person.velocity],
                "last_seen": float(self._person.last_seen),
                "confidence": float(self._person.confidence),
                "observation_accepted": self._observation_accepted,
                "observation_reason": self._observation_reason,
                "raw_position": list(self._person.last_raw_pos or self._person.position),
                "appearance_backend": "dino" if self._dino_encoder is not None else "clip_osnet",
                "dino_similarity": self._dino_similarity,
                "dino_event": self._dino_event,
                "identity_retry_until": dict(self._identity_retry_until),
                "reid_stats": dict(self._reid_stats),
            }

    @property
    def target_selected(self) -> bool:
        return self._target_selected

    @property
    def has_identity_reference(self) -> bool:
        """Whether a lost target can be compared with its original DINO appearance."""
        return bool(
            self._dino_encoder is not None
            and self._person is not None
            and self._person.dino_reference is not None
        )

    @property
    def description(self) -> str:
        return self._description

    def reset(self):
        with self._lock:
            self._person = None
            self._description = ""
            self._target_selected = False
            self._observation_accepted = False
            self._observation_reason = ""
            self._vlm_selecting = False
            self._reset_reacquire()
            self._frame_dino.clear()
            self._reference_crop = None
            self._verified_reference_crops.clear()
            self._verified_reference_timestamps.clear()
            self._verified_reference_features.clear()
            self._pending_identity = None
            self._verified_identity = None
            self._identity_retry_until.clear()
            self._identity_retry_state.clear()
            self._dino_similarity = None
            self._dino_event = "ready" if self._dino_encoder is not None else "disabled"
