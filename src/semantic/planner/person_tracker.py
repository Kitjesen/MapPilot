"""
PersonTracker -- VLM person selection + real-time visual tracking (person following core)

Pipeline:
  1. VLM one-shot person selection: "person in red" → YOLO crops → VLM selects index → lock target
  2. Frame-to-frame tracking: obj_id match → distance match → OSNet Re-ID (primary) + CLIP fuse
  3. Lost re-identification: appearance search → fallback VLM reselect

Re-ID strategy (W3-5):
  - Primary: OSNetReIDEncoder (512-dim, L2-normalised BPU or torchreid feature)
  - Adaptive threshold: 0.55 when ≤2 candidates, 0.70 when ≥5 candidates
  - Secondary fusion: CLIP (weight 0.4) when OSNet confidence is low
  - Motion prediction: linear Kalman-like (x, y, vx, vy) in world frame,
    predict location at +0.3s to disambiguate when appearance is weak

No Kalman filter library dependency — linear motion model only.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

from core.msgs.numpy_compat import is_numpy_array, np
logger = logging.getLogger(__name__)

# OSNet Re-ID feature dimension
_OSNET_FEAT_DIM = 512

# Adaptive Re-ID thresholds based on crowd density
_REID_THRESHOLD_SPARSE = 0.55   # ≤2 candidates: lower threshold (easier match)
_REID_THRESHOLD_DENSE = 0.70    # ≥5 candidates: higher threshold (stricter match)

# Weight of CLIP secondary signal in appearance fusion (0.0 = OSNet only)
_CLIP_FUSION_WEIGHT = 0.4

# Motion prediction horizon (seconds)
_MOTION_PREDICT_DT = 0.3


@dataclass
class TrackedPerson:
    """Tracked target person."""
    position: list[float]                          # [x, y, z] EMA-smoothed, world frame
    # Last raw (unsmoothed) measurement, used for velocity finite-difference
    # so that the velocity estimator isn't doubly low-passed through the
    # position EMA.
    last_raw_pos: list[float] | None = None
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0])  # [vx, vy] m/s
    last_seen: float = field(default_factory=time.time)
    confidence: float = 1.0
    # Appearance features for Re-ID after occlusion
    appearance: np.ndarray | None = None        # CLIP image feature, shape (D,)
    osnet_feat: np.ndarray | None = None        # OSNet 512-dim L2-normalised feature
    bbox: list[int] | None = None               # Most recent [x1, y1, x2, y2]
    obj_id: str | None = None                   # Scene graph object ID (preferred for matching)


class PersonTracker:
    """
    VLM 辅助选人 + 实时视觉跟踪。

    使用方式:
      1. set_target(description, llm_fn, clip_fn) — 用 VLM 从候选人中选定目标
      2. update(scene_objects, rgb_frame) — 每帧更新跟踪状态
      3. get_follow_waypoint(robot_pos) — 获取跟随航点
    """

    LOST_TIMEOUT = 5.0          # 秒: 丢失判定
    REID_TIMEOUT = 15.0         # 秒: 超过此时间放弃 Re-ID, 需 VLM 重选
    EMA_ALPHA = 0.4             # 位置平滑系数
    APPEARANCE_ALPHA = 0.1      # 外观特征 EMA 更新系数
    MATCH_DIST_THRESHOLD = 2.0  # 米: 距离匹配阈值
    APPEARANCE_THRESHOLD = 0.6  # 外观相似度阈值 (Re-ID)

    def __init__(self, follow_distance: float = 1.5, lost_timeout: float = 5.0):
        self.follow_distance = follow_distance
        self.lost_timeout = lost_timeout
        self._person: TrackedPerson | None = None
        self._description: str = ""                # user description e.g. "person in red"
        self._target_selected: bool = False        # whether VLM has selected a target
        self._vlm_selecting: bool = False          # VLM selection in progress (re-entry guard)
        self._clip_encoder = None                  # CLIP encoder (externally injected)
        self._lock = threading.Lock()
        # FusionMOT backend (optional, enabled via enable_fusion_tracking)
        self._fusion_tracker = None
        self._reid_extractor = None
        self._target_track_id: int | None = None
        self._following_selector = None  # PersonFollowingSelector (optional)

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
        self._clip_encoder = clip_encoder

    def set_osnet_encoder(self, encoder) -> None:
        """Inject a pre-constructed OSNetReIDEncoder (primary Re-ID signal).

        Allows callers to share a single encoder instance across modules.
        """
        self._osnet_encoder = encoder
        logger.info("PersonTracker: OSNet Re-ID encoder injected (backend=%s)",
                    getattr(encoder, "backend", "unknown"))

    def enable_osnet_reid(self) -> bool:
        """Attempt to construct an OSNetReIDEncoder and attach it.

        Returns True if OSNet loaded successfully, False if unavailable.
        Does NOT raise — caller decides whether absence is fatal.
        """
        try:
            from .osnet_reid import OSNetReIDEncoder
            self._osnet_encoder = OSNetReIDEncoder()
            logger.info("PersonTracker: OSNet Re-ID enabled (backend=%s)",
                        self._osnet_encoder.backend)
            return True
        except Exception as exc:
            logger.info("PersonTracker: OSNet Re-ID unavailable (%s), "
                        "will use CLIP-only Re-ID", exc)
            return False

    def enable_fusion_tracking(self) -> bool:
        """Enable FusionMOT + OSNet Re-ID backend for Kalman-smoothed tracking.

        When enabled, update() routes through FusionMOT which provides:
          - Kalman prediction between frames (smooth bbox trajectory)
          - OSNet Re-ID features (robust re-identification after occlusion)
          - Stable track IDs across frames

        Returns True if successfully initialized, False if qp_perception unavailable.
        """
        try:
            from qp_perception.reid.extractor import ReIDConfig, ReIDExtractor
            from qp_perception.tracking.fusion import FusionMOT, FusionMOTConfig

            reid_cfg = ReIDConfig(backbone="osnet_x1_0", device="")
            self._reid_extractor = ReIDExtractor(reid_cfg)

            mot_cfg = FusionMOTConfig()
            self._fusion_tracker = FusionMOT(
                config=mot_cfg,
                feature_dim=self._reid_extractor.feature_dim,
            )

            # PersonFollowingSelector: state machine for target lock lifecycle
            try:
                from qp_perception.selection.person_following import (
                    FollowingConfig,
                    PersonFollowingSelector,
                )
                self._following_selector = PersonFollowingSelector(
                    FollowingConfig(
                        search_timeout_s=self.REID_TIMEOUT,
                        auto_lock=True,
                        min_confidence=0.3,
                    )
                )
            except Exception:
                self._following_selector = None

            logger.info("PersonTracker: FusionMOT + OSNet Re-ID enabled")
            return True
        except Exception as e:
            logger.info(
                "PersonTracker: FusionMOT unavailable (%s), using classic tracking", e
            )
            self._fusion_tracker = None
            self._reid_extractor = None
            return False

    _CLIP_SELECT_MIN_SIM = 0.2  # below this, CLIP is not confident enough to lock

    def _clip_prescore(
        self, description: str, person_crops: list[np.ndarray]
    ) -> list[float]:
        """Cosine similarity of each crop against the description via CLIP.

        Single source of truth for both `select_by_clip` and the CLIP hint
        attached to the VLM prompt. Crops are expected **BGR** (CLIPEncoder /
        cv2 convention). Returns a list aligned with ``person_crops`` (0.0 where
        a crop fails to encode), or an empty list when no CLIP encoder is
        attached or the model can't encode images (e.g. mobileclip).
        """
        if self._clip_encoder is None:
            return []
        try:
            text_feat = self._clip_encoder.encode_text([description])
        except Exception as exc:
            logger.debug("CLIP text encode failed: %s", exc)
            return []
        if text_feat is None or len(text_feat) == 0:
            return []

        scores: list[float] = []
        for crop in person_crops:
            try:
                img_feat = self._clip_encoder.encode_image(crop)
                if img_feat is not None and img_feat.size > 0:
                    sim = float(np.dot(text_feat[0], img_feat) / (
                        np.linalg.norm(text_feat[0]) * np.linalg.norm(img_feat) + 1e-9
                    ))
                    scores.append(sim)
                else:
                    scores.append(0.0)
            except Exception:
                scores.append(0.0)
        return scores

    @staticmethod
    def _build_vlm_select_messages(
        text_prompt: str, person_crops: list[np.ndarray]
    ) -> list[dict]:
        """OpenAI multimodal messages: instruction text + one image per person.

        Crops are expected **BGR** (cv2 convention; ``cv2.imencode`` assumes BGR).
        Falls back to text-only content when image encoding is unavailable, so a
        non-vision LLM still receives the numbered prompt + CLIP hints.
        """
        content: list[dict] = [{"type": "text", "text": text_prompt}]
        try:
            from .vlm_scene_agent import encode_image_b64
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
                    content.append({
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{b64}",
                            "detail": "low",
                        },
                    })
        return [{"role": "user", "content": content}]

    async def select_target_with_vlm(
        self,
        description: str,
        person_crops: list[np.ndarray],
        person_objects: list[dict],
        llm_chat_fn: Callable,
    ) -> int:
        """
        用 VLM 从候选人中选定目标 (一次性调用, 多模态看图)。

        每个 person 的裁剪图会被编码成 base64 随 prompt 一起发给 VLM，让模型
        真正"看图选人"，而不是只靠文字盲猜。CLIP 分数 (若可用) 作为辅助提示 +
        VLM 失败时的降级依据。真机 mobileclip 无图像编码时，CLIP 提示为空，
        选人完全依赖 VLM 看图。

        Args:
            description: 用户描述, 如 "穿红衣服背包的人"
            person_crops: 每个 person 的裁剪图像列表 (BGR, 与 CLIP/cv2 一致)
            person_objects: 对应的场景图对象列表
            llm_chat_fn: async VLM 调用函数, 接受 OpenAI 多模态 messages
                         (list[dict], 含 text + image_url) 返回 str

        Returns:
            选中的 person 索引 (0-based), -1 表示失败
        """
        if not person_crops:
            return -1

        self._description = description
        n = len(person_crops)

        # CLIP 预打分: 既作为 VLM prompt 的辅助提示, 也是 VLM 失败时的降级依据。
        clip_scores = self._clip_prescore(description, person_crops)

        text_prompt = (
            f"画面中有 {n} 个人，编号 1 到 {n}（每个编号后附其裁剪图）。\n"
            f"用户想跟随的是: {description}\n"
            f"请只回答一个数字（1-{n}），表示最匹配的人的编号。\n"
            f"如果都不匹配，回答 0。"
        )
        if clip_scores:
            text_prompt += f"\nCLIP 相似度参考: {[f'{s:.2f}' for s in clip_scores]}"

        messages = self._build_vlm_select_messages(text_prompt, person_crops)

        # 调用 VLM (多模态)
        try:
            response = await llm_chat_fn(messages)
            import re
            numbers = re.findall(r'\d+', str(response))
            if numbers:
                idx = int(numbers[0])
                if 1 <= idx <= n:
                    selected = idx - 1  # 0-based
                    logger.info("VLM selected person #%d for '%s'", idx, description)
                    self._lock_target(person_objects[selected], person_crops[selected])
                    return selected
                elif idx == 0:
                    logger.info("VLM: no matching person for '%s'", description)
                    return -1
        except Exception as e:
            logger.error("VLM selection failed: %s", e)

        # VLM 失败时，用 CLIP 分数选 (降级)
        if clip_scores:
            best_idx = int(np.argmax(clip_scores))
            if clip_scores[best_idx] > self._CLIP_SELECT_MIN_SIM:
                logger.info(
                    "VLM failed, using CLIP fallback: person #%d (sim=%.2f)",
                    best_idx + 1, clip_scores[best_idx],
                )
                self._lock_target(person_objects[best_idx], person_crops[best_idx])
                return best_idx

        return -1

    def select_by_clip(
        self,
        description: str,
        person_crops: list[np.ndarray],
        person_objects: list[dict],
    ) -> int:
        """
        纯 CLIP 选人 (无 VLM 时的降级方案, 同步调用)。

        仅在注入的编码器支持图像编码时有效 (clip backend)。mobileclip 只做
        文本编码 → `_clip_prescore` 返回空 → 此处返回 -1, 调用方应转 VLM 选人。

        Returns:
            选中的 person 索引 (0-based), -1 表示失败
        """
        if not person_crops or self._clip_encoder is None:
            return -1

        self._description = description
        scores = self._clip_prescore(description, person_crops)
        if not scores:
            return -1

        best_idx = int(np.argmax(scores))
        if scores[best_idx] > self._CLIP_SELECT_MIN_SIM:
            logger.info(
                "CLIP selected person #%d for '%s' (sim=%.2f)",
                best_idx + 1, description, scores[best_idx],
            )
            self._lock_target(person_objects[best_idx], person_crops[best_idx])
            return best_idx
        return -1

    def _lock_target(self, obj: dict, crop: np.ndarray | None = None) -> None:
        """Lock target and store initial appearance features (CLIP + OSNet)."""
        pos = obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            pos = [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]

        appearance = None
        osnet_feat = None

        if crop is not None:
            # Extract CLIP feature for secondary Re-ID signal
            if self._clip_encoder is not None:
                try:
                    feat = self._clip_encoder.encode_image(crop)
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
                confidence=obj.get("confidence", 1.0),
                appearance=appearance,
                osnet_feat=osnet_feat,
                bbox=obj.get("bbox"),
                obj_id=obj.get("id"),
            )
            self._target_selected = True

    def update(
        self,
        scene_objects: list[dict],
        rgb_frame: np.ndarray | None = None,
    ) -> bool:
        """
        每帧更新跟踪状态。

        When FusionMOT is enabled (via enable_fusion_tracking), routes through
        Kalman-smoothed tracking with OSNet Re-ID. Otherwise uses classic matching:
          1. obj_id 匹配 (instance_tracker 已赋予唯一 ID)
          2. 距离最近 (< MATCH_DIST_THRESHOLD)
          3. 外观特征 Re-ID (遮挡后恢复)

        Args:
            scene_objects: 场景图 objects (含 label, position, confidence, id, bbox)
            rgb_frame: 当前 RGB 帧 (用于外观特征更新)

        Returns:
            True 如果成功匹配到目标
        """
        persons = [
            o for o in scene_objects
            if o.get("label", "").lower() in ("person", "people", "human", "pedestrian")
        ]
        if not persons:
            self._fusion_tick_empty()
            return False

        with self._lock:
            # FusionMOT path: Kalman + OSNet Re-ID
            if self._fusion_tracker is not None and rgb_frame is not None:
                return self._update_fusion(persons, rgb_frame)

            # ── Classic path (unchanged) ──
            if self._person is None:
                # 未选定目标时，退化为跟最高置信度的 (兼容旧行为)
                best = max(persons, key=lambda p: p.get("confidence", 0))
                self._init_person(best)
                return True

            matched = self._match_person(persons, rgb_frame)
            if matched is not None:
                self._update_tracked(matched, rgb_frame)
                return True

            # 未经 VLM 选定时，退化为跟最近的 person (兼容旧行为)
            if not self._target_selected:
                nearest = min(persons, key=lambda p: math.hypot(
                    self._get_pos(p)[0] - self._person.position[0],
                    self._get_pos(p)[1] - self._person.position[1],
                ))
                self._update_tracked(nearest, rgb_frame)
                return True

        return False

    # ── FusionMOT backend ─────────────────────────────────────────────────────

    def _update_fusion(self, persons: list[dict], rgb_frame: np.ndarray) -> bool:
        """Tracking via FusionMOT: Kalman smoothing + OSNet Re-ID.

        Uses Selective Re-ID (Fast-Deep-OC-SORT): only extracts appearance
        features for detections that lack a high-IoU geometric match,
        saving ~60-70% of Re-ID computation in steady-state tracking.

        Flow:
          1. Convert person bboxes → FusionMOT input (xywh)
          2. FusionMOT.update_selective() → IoU pre-match → extract Re-ID
             only for ambiguous detections → full matching
          3. Match target by track_id (O(1) lookup) or fall back to classic
        """
        timestamp = time.time()

        # Convert person detections to FusionMOT format
        bboxes, confs, valid_persons = [], [], []
        for p in persons:
            bbox = p.get("bbox")
            if not bbox or len(bbox) < 4:
                continue
            if is_numpy_array(bbox):
                bbox = bbox.tolist()
            x1, y1 = float(bbox[0]), float(bbox[1])
            x2, y2 = float(bbox[2]), float(bbox[3])
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue
            bboxes.append([x1, y1, w, h])
            confs.append(float(p.get("confidence", 0.5)))
            valid_persons.append(p)

        if not bboxes:
            self._fusion_tick_empty()
            return False

        bboxes_np = np.array(bboxes, dtype=np.float32)
        confs_np = np.array(confs, dtype=np.float32)

        # Selective Re-ID: only extract for ambiguous/unmatched detections
        tracks = self._fusion_tracker.update_selective(
            bboxes_np, confs_np, timestamp,
            reid_extractor=self._reid_extractor,
            frame=rgb_frame,
        )
        if not tracks:
            return False

        # Map track_id → person object (by bbox center proximity)
        track_map = self._map_tracks_to_persons(tracks, valid_persons)

        # ── PersonFollowingSelector path ──
        if self._following_selector is not None:
            track_objects = self._raw_to_tracks(tracks, timestamp)

            # If VLM selected a person but selector not yet locked, associate
            if (
                not self._following_selector.is_locked
                and self._person is not None
                and self._target_selected
            ):
                matched = self._match_person(list(track_map.values()), rgb_frame)
                if matched is not None:
                    for tid, p in track_map.items():
                        if p is matched:
                            self._following_selector.lock_track(tid, self._description)
                            self._target_track_id = tid
                            break

            obs = self._following_selector.select(track_objects, timestamp)
            if obs is not None:
                person = track_map.get(obs.track_id)
                if person is not None:
                    self._target_track_id = obs.track_id
                    self._update_tracked(person, rgb_frame)
                    return True
            return False

        # ── Fallback: manual track_id matching (no selector) ──
        if self._target_track_id is not None and self._target_track_id in track_map:
            self._update_tracked(track_map[self._target_track_id], rgb_frame)
            return True

        if self._person is not None:
            matched = self._match_person(list(track_map.values()), rgb_frame)
            if matched is not None:
                for tid, p in track_map.items():
                    if p is matched:
                        self._target_track_id = tid
                        break
                self._update_tracked(matched, rgb_frame)
                return True
            return False

        if not self._target_selected:
            best_tid = max(track_map, key=lambda t: track_map[t].get("confidence", 0))
            self._init_person(track_map[best_tid])
            self._target_track_id = best_tid
            return True

        return False

    def _fusion_tick_empty(self) -> None:
        """Maintain FusionMOT state with no detections (keeps lost-track timers)."""
        if self._fusion_tracker is None:
            return
        try:
            self._fusion_tracker.update(
                np.empty((0, 4)), np.array([]), None, time.time()
            )
        except Exception:
            pass

    @staticmethod
    def _raw_to_tracks(raw_tracks: list, timestamp: float) -> list:
        """Convert FusionMOT raw output to qp_perception Track objects."""
        from qp_perception.types import BoundingBox, Track
        result = []
        for track_id, bbox_arr, conf in raw_tracks:
            x, y, w, h = bbox_arr
            result.append(Track(
                track_id=int(track_id),
                bbox=BoundingBox(x=float(x), y=float(y), w=float(w), h=float(h)),
                confidence=float(conf),
                class_id="person",
                first_seen_ts=timestamp,
                last_seen_ts=timestamp,
            ))
        return result

    @staticmethod
    def _map_tracks_to_persons(
        tracks: list, persons: list[dict],
    ) -> dict[int, dict]:
        """Map FusionMOT track_id to nearest input person by bbox center distance."""
        result: dict[int, dict] = {}
        for track_id, bbox_arr, _conf in tracks:
            tx, ty, tw, th = bbox_arr
            tcx, tcy = tx + tw / 2, ty + th / 2
            best_p, best_d = None, float("inf")
            for p in persons:
                pb = p.get("bbox", [0, 0, 0, 0])
                if is_numpy_array(pb):
                    pb = pb.tolist()
                if len(pb) < 4:
                    continue
                pcx = (float(pb[0]) + float(pb[2])) / 2
                pcy = (float(pb[1]) + float(pb[3])) / 2
                d = (tcx - pcx) ** 2 + (tcy - pcy) ** 2
                if d < best_d:
                    best_d = d
                    best_p = p
            if best_p is not None:
                result[int(track_id)] = best_p
        return result

    def _adaptive_reid_threshold(self, n_candidates: int) -> float:
        """Return adaptive Re-ID cosine similarity threshold based on crowd density.

        Sparse scene (<=2 candidates): 0.55 — easier match, fewer impostors.
        Dense scene (>=5 candidates): 0.70 — stricter match, more discrimination needed.
        Linear interpolation for 3-4 candidates.
        """
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

    def _match_person(
        self, persons: list[dict], rgb_frame: np.ndarray | None
    ) -> dict | None:
        """Match the locked target among candidate persons.

        Matching priority:
          1. obj_id exact match (cheapest, most reliable)
          2. Distance match within MATCH_DIST_THRESHOLD
             — uses motion-predicted position to handle latency
          3. Appearance Re-ID when distance match fails:
             a. OSNet primary (512-dim cosine, adaptive threshold)
             b. CLIP secondary fusion (weight _CLIP_FUSION_WEIGHT) when OSNet
                feature confidence is below a good match
             c. Motion prediction tie-break when appearance scores are close

        Updates _reid_stats for observability.
        """
        if self._person is None:
            return None

        n_candidates = len(persons)

        # Strategy 1: obj_id exact match
        if self._person.obj_id:
            for p in persons:
                if p.get("id") == self._person.obj_id:
                    return p

        # Strategy 2: distance match using motion-predicted position
        predicted_pos = self._predict_position()
        best_dist_p = None
        best_dist = self.MATCH_DIST_THRESHOLD
        for p in persons:
            pos = self._get_pos(p)
            # Use predicted position to account for motion since last observation
            dist = math.hypot(
                pos[0] - predicted_pos[0],
                pos[1] - predicted_pos[1],
            )
            if dist < best_dist:
                best_dist = dist
                best_dist_p = p

        if best_dist_p is not None:
            return best_dist_p

        # Strategy 3: appearance Re-ID (distance match failed)
        if rgb_frame is None:
            self._reid_stats["lost"] += 1
            return None

        reid_threshold = self._adaptive_reid_threshold(n_candidates)

        best_reid_p = None
        best_reid_score = reid_threshold  # must exceed threshold to match

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

            # CLIP secondary signal — fuse when available
            if (
                self._clip_encoder is not None
                and self._person.appearance is not None
            ):
                try:
                    clip_feat = self._clip_encoder.encode_image(crop)
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
            # Attribute the winning signal type to stats
            if self._osnet_encoder is not None and self._person.osnet_feat is not None:
                self._reid_stats["osnet_match"] += 1
            elif self._clip_encoder is not None:
                self._reid_stats["clip_fallback"] += 1
            else:
                self._reid_stats["motion_dominant"] += 1
            logger.info("PersonTracker: Re-ID matched (score=%.3f, threshold=%.2f, "
                        "candidates=%d)", best_reid_score, reid_threshold, n_candidates)
            return best_reid_p

        self._reid_stats["lost"] += 1
        return None

    def _update_tracked(self, obj: dict, rgb_frame: np.ndarray | None) -> None:
        """Update position, velocity, and appearance features for the matched target."""
        new_pos = self._get_pos(obj)
        now = time.time()

        dt = now - self._person.last_seen
        old_smoothed = self._person.position
        old_raw = self._person.last_raw_pos or old_smoothed

        # Velocity EMA on the raw finite difference. Clamp dt so a long
        # occlusion gap (or stale timestamp after re-lock) can't collapse the
        # velocity to ~0. Skip the update if dt is pathologically small.
        if dt > 0.01:
            dt_v = min(dt, 0.2)
            self._person.velocity = [
                (new_pos[0] - old_raw[0]) / dt_v * 0.3 + self._person.velocity[0] * 0.7,
                (new_pos[1] - old_raw[1]) / dt_v * 0.3 + self._person.velocity[1] * 0.7,
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
        self._person.confidence = obj.get("confidence", 1.0)
        self._person.obj_id = obj.get("id", self._person.obj_id)
        self._person.bbox = obj.get("bbox", self._person.bbox)

        if rgb_frame is not None:
            crop = self._crop_person(rgb_frame, obj)
            if crop is not None:
                # CLIP appearance EMA update (secondary Re-ID signal)
                if self._clip_encoder is not None:
                    try:
                        feat = self._clip_encoder.encode_image(crop)
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

    def _init_person(self, obj: dict) -> None:
        """初始化目标 (未经 VLM 选定时的退化路径)。"""
        pos = self._get_pos(obj)
        self._person = TrackedPerson(
            position=list(pos[:3]),
            last_raw_pos=list(pos[:3]),
            last_seen=time.time(),
            confidence=obj.get("confidence", 1.0),
            obj_id=obj.get("id"),
            bbox=obj.get("bbox"),
        )

    @staticmethod
    def _get_pos(obj: dict) -> list[float]:
        """统一提取 position，兼容 list 和 dict 格式。"""
        pos = obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            return [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]
        return list(pos[:3]) if len(pos) >= 3 else list(pos) + [0.0] * (3 - len(pos))

    @staticmethod
    def _crop_person(rgb: np.ndarray, obj: dict) -> np.ndarray | None:
        """从 RGB 帧裁剪 person 区域。"""
        bbox = obj.get("bbox")
        if bbox is None:
            return None
        if is_numpy_array(bbox):
            bbox = bbox.astype(int).tolist()
        elif isinstance(bbox, dict):
            bbox = [int(bbox.get("x1", 0)), int(bbox.get("y1", 0)),
                    int(bbox.get("x2", 0)), int(bbox.get("y2", 0))]

        h, w = rgb.shape[:2]
        x1 = max(0, int(bbox[0]))
        y1 = max(0, int(bbox[1]))
        x2 = min(w, int(bbox[2]))
        y2 = min(h, int(bbox[3]))
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None
        return rgb[y1:y2, x1:x2].copy()

    # ── 导航接口 (保持与旧版兼容) ──

    def get_follow_waypoint(
        self, robot_pos: list[float], predict_dt: float = 0.3,
    ) -> dict | None:
        """返回跟随航点 (目标后方 follow_distance 处)。"""
        with self._lock:
            if self._person is None or self.is_lost():
                return None

            px = self._person.position[0] + self._person.velocity[0] * predict_dt
            py = self._person.position[1] + self._person.velocity[1] * predict_dt
            pz = self._person.position[2] if len(self._person.position) > 2 else 0.0

            dx = robot_pos[0] - px
            dy = robot_pos[1] - py
            dist = math.hypot(dx, dy)
            if dist < 0.01:
                return {"x": px, "y": py, "z": pz}

            fx = px + (dx / dist) * self.follow_distance
            fy = py + (dy / dist) * self.follow_distance
            return {"x": fx, "y": fy, "z": pz}

    def is_lost(self) -> bool:
        if self._person is None:
            return True
        return (time.time() - self._person.last_seen) > self.lost_timeout

    def needs_vlm_reselect(self) -> bool:
        """是否需要 VLM 重新选人 (丢失超过 REID_TIMEOUT)。"""
        if self._following_selector is not None:
            return self._following_selector.needs_reselect
        if self._person is None:
            return True
        elapsed = time.time() - self._person.last_seen
        return elapsed > self.REID_TIMEOUT

    def get_person_position(self) -> list[float] | None:
        with self._lock:
            if self._person and not self.is_lost():
                return list(self._person.position)
        return None

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
            self._target_track_id = None
            if self._following_selector is not None:
                self._following_selector.unlock()
