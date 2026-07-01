"""
NaviMind Habitat Agent 鈥?BA-HSG + Fast-Slow 闆舵牱鏈?ObjectNav 璇勬祴銆?

鎰熺煡绛栫暐 (鎸変紭鍏堢骇):
  1. Habitat ground-truth semantic sensor (鏈?.basis.scn 鏍囨敞鏃? 鈥?涓?SG-Nav/VLFM/L3MVN 鐩稿悓鍋囪
  2. CLIP RGB patch 妫€娴?(鏃犳爣娉ㄦ椂鐨勯浂鏍锋湰鍥為€€, 涓?VLFM/CoW 鐩稿悓璺嚎)

Agent 涓嶆帴鏀剁洰鏍囦綅缃俊鎭€斺€旀墍鏈夊鑸喅绛栧熀浜庡湪绾挎劅鐭ュ拰 BA-HSG 鍦烘櫙鍥俱€?

鍋滄鍒ゆ嵁: CLIP 甯х浉浼煎害 > 闃堝€?AND 鍓嶆柟娣卞害 < success_distance銆?

娑堣瀺瀹為獙:
  --ablation no_belief     鍘绘帀 BA-HSG 璐濆彾鏂俊蹇?
  --ablation no_fov        鍘绘帀 FOV-aware 璐熼潰璇佹嵁
  --ablation no_hierarchy  骞冲潶鐗╀綋鍒楄〃鏇夸唬灞傛鍦烘櫙鍥?
  --ablation always_llm    璺宠繃 Fast Path (绾帰绱? 妯℃嫙 SG-Nav)

鏃?ROS2 渚濊禆, 绾?Python 杩愯銆?
"""

import json
import logging
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np
import yaml

# 鈹€鈹€ 灏嗛」鐩?src 璺緞鍔犲叆 sys.path (鏃犻渶 ROS2 瀹夎) 鈹€鈹€
_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_SRC = str(_PROJECT_ROOT / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from runtime.utils.sanitize import sanitize_position, safe_json_loads
from perception.projection import Detection3D
from perception.instance_tracker import InstanceTracker
from decision.goal_resolver import GoalResolver, GoalResult
from decision.llm_client import LLMConfig

logger = logging.getLogger(__name__)

# 鈹€鈹€ Habitat 鍔ㄤ綔 ID 鈹€鈹€
HABITAT_STOP = 0
HABITAT_MOVE_FORWARD = 1
HABITAT_TURN_LEFT = 2
HABITAT_TURN_RIGHT = 3

OBJECTNAV_CATEGORIES = {
    "chair": "find the chair",
    "couch": "find the couch",
    "potted_plant": "find the potted plant",
    "bed": "find the bed",
    "toilet": "find the toilet",
    "tv_monitor": "find the TV monitor",
}

# HM3D 6 澶х被鐨勮嫳鏂囨彁绀鸿 (鐢ㄤ簬 CLIP)
CLIP_PROMPTS = {
    "chair": "a photo of a chair",
    "couch": "a photo of a couch or sofa",
    "potted_plant": "a photo of a potted plant",
    "bed": "a photo of a bed",
    "toilet": "a photo of a toilet",
    "tv_monitor": "a photo of a television monitor or TV screen",
}

# 绫诲埆鍚屼箟璇?鈥?鍖呭惈 HM3D .semantic.txt 甯歌鍙樹綋
CATEGORY_ALIASES = {
    # couch / sofa
    "sofa": "couch",
    "armchair": "chair",
    "sofa chair": "couch",
    "circular sofa": "couch",
    "l-shaped sofa": "couch",
    "sofa set": "couch",
    "sofa seat": "couch",
    # chair variants
    "dining chair": "chair",
    "desk chair": "chair",
    "rocking chair": "chair",
    "bar chair": "chair",
    "office chair": "chair",
    "highchair": "chair",
    "patio chair": "chair",
    "lounge chair": "chair",
    "folding chair": "chair",
    "bean bag chair": "chair",
    "camping chair": "chair",
    "computer chair": "chair",
    # potted plant
    "plant": "potted_plant",
    "potted plant": "potted_plant",
    "decorative plant": "potted_plant",
    "table plant": "potted_plant",
    "ornamental plant": "potted_plant",
    # tv / monitor
    "television": "tv_monitor",
    "tv": "tv_monitor",
    "monitor": "tv_monitor",
    "tv/monitor": "tv_monitor",
    "tv monitor": "tv_monitor",
    "led tv": "tv_monitor",
    "tv led": "tv_monitor",
    "wall tv": "tv_monitor",
}

# CLIP 鐩镐技搴﹂槇鍊?
CLIP_STOP_THRESHOLD = 0.23      # 涓績瑁佸壀鐩镐技搴?鈫?鑰冭檻鍋滄 (v15: 0.26鈫?.23 瑕嗙洊 toilet/couch)
CLIP_DETECT_THRESHOLD = 0.26    # 灞€閮?patch 鐩镐技搴?鈫?鍒涘缓 Detection3D
# 灏忓瀷/闅捐瘑鍒被鍒娇鐢ㄦ洿浣庨槇鍊?(potted_plant 瀵?ViT-B/32 杈冮毦)
_CLIP_STOP_THRESHOLD_BY_CAT: Dict[str, float] = {
    "potted_plant": 0.18,
}
PROXIMITY_GUARD_DIST = 3.0      # 杩戠洰鏍囧畧鍗窛绂?(m); v14=5.0 鈫?v15=3.0 鍑忓皯鍋囬槼鎬?
# CLIP patch 妫€娴嬪緱鍒嗕笂闄? 鍏佽瓒冲楂樹互瑙﹀彂 Fast Path (confidence_threshold=0.6)
CLIP_PATCH_SCORE_MAX = 0.85
CLIP_MIN_DEPTH = 0.8            # 妫€娴嬬墿浣撴渶灏忔繁搴?(m) 鈥?杩囨护璐村璇
CLIP_CALL_INTERVAL = 3          # 姣?N 姝ヨ皟鐢ㄤ竴娆?CLIP (骞宠　閫熷害涓庡搷搴?
MIN_EXPLORE_STEPS = 100         # 鑷冲皯鎺㈢储 N 姝ュ啀鍏佽 YOLO/CLIP STOP (閬垮厤鍦ㄨ捣鐐硅鍋?
CLIP_STOP_MIN_STREAK = 1        # 1 甯у嵆鍙? 閬垮厤婕忔帀鐭殏鎺ヨ繎鐩爣鐨勬満浼?

# 鈹€鈹€ YOLO11 妫€娴嬪櫒 (鏇挎崲 CLIP patch 妫€娴? 鈹€鈹€
try:
    from ultralytics import YOLO as _YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False

# HM3D 6澶х被 鈫?COCO class id 鏄犲皠
YOLO_COCO_IDS = {
    "chair":        {56},
    "couch":        {57},
    "bed":          {59},
    "toilet":       {61},
    "potted_plant": {58},
    "tv_monitor":   {62, 63},
}
YOLO_STOP_CONF   = 0.55   # 妫€娴嬬疆淇″害 鈫?瑙﹀彂 STOP
YOLO_DETECT_CONF = 0.25   # 妫€娴嬬疆淇″害 鈫?鍔犲叆鍦烘櫙鍥?
YOLO_CALL_INTERVAL = 1    # 姣忔閮借窇 YOLO (GPU ~15ms)
DEPTH_MAX = 10.0                # Habitat depth sensor 鏈€澶ч噺绋?(m); 瑙傛祴鍊煎凡褰掍竴鍖栧埌 [0,1]

# YOLO 鐗╀綋鍏卞瓨璇箟 bonus (鎺㈢储鏃嬭浆鎵弿鏃朵娇鐢?
# 鏍煎紡: {鐩爣绫诲埆: {鍏卞瓨 COCO class_id: bonus鍒唥}
YOLO_COOCCUR_BONUS: "Dict[str, Dict[int, float]]" = {
    "chair":        {57: 2.0, 60: 1.5},            # couch(57), dining table(60) 鈫?鍚屼竴鐢熸椿绌洪棿
    "couch":        {56: 1.5, 62: 2.5, 63: 2.0},   # chair(56), tv(62), monitor(63) 鈫?瀹㈠巺
    "bed":          {59: 3.0},                      # 鍙︿竴寮犲簥(59) 鈫?鍗у纭
    "toilet":       {56: 0.5},                      # chair(56) 鈫?鏈夊鍏锋埧闂?寮卞厛楠?
    "potted_plant": {57: 2.0, 60: 1.5, 56: 1.0},   # couch, table, chair 鈫?瀹㈠巺/椁愬巺
    "tv_monitor":   {57: 3.5, 56: 1.5, 60: 1.0},   # couch(57) 鈫?寮篢V鎸囩ず
}


def _normalize_category(label: str) -> str:
    """Compatibility helper."""
    label_lower = label.lower().strip()
    if label_lower in OBJECTNAV_CATEGORIES:
        return label_lower
    return CATEGORY_ALIASES.get(label_lower, label_lower)


@dataclass
class AgentState:
    """Compatibility helper."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading: float = 0.0
    step_count: int = 0
    rotate_count: int = 0
    is_rotating: bool = True
    forward_steps: int = 0
    target_forward_steps: int = 8
    best_frontier_angle: float = 0.0
    goal_found: bool = False
    goal_position: Optional[np.ndarray] = None
    navigating_to_goal: bool = False
    navigate_steps: int = 0
    prev_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    stuck_counter: int = 0
    visited_cells: Set[Tuple[int, int]] = field(default_factory=set)
    blocked_headings: List[float] = field(default_factory=list)  # 鍗′綇鏂瑰悜璁板綍 鈫?frontier 鎯╃綒
    # CLIP 缂撳瓨
    last_clip_step: int = -999
    last_clip_sim: float = 0.0
    clip_stop_streak: int = 0   # 杩炵画婊¤冻鍋滄鏉′欢鐨勫抚鏁?
    last_yolo_depth: float = 999.0  # 褰撳墠甯?YOLO 妫€娴嬪埌鐩爣鐨勬渶杩戞繁搴?(m)
    last_yolo_step: int = -999       # 涓婃 YOLO 妫€娴嬫鏁?


class NaviMindAgent:
    """
    NaviMind Habitat Agent 鈥?BA-HSG + Fast-Slow + CLIP 闆舵牱鏈鑸€?

    鎰熺煡: CLIP RGB patch 鈫?Detection3D 鈫?InstanceTracker (BA-HSG)
    瑙勫垝: GoalResolver Fast Path 鍦烘櫙鍥惧尮閰?
    鍋滄: CLIP 鍏ㄥ抚鐩镐技搴?> 闃堝€?+ 娣卞害璺濈 < success_distance
    鎺㈢储: 娣卞害鎰熺煡 frontier 鎺㈢储 (novelty + depth score)
    """

    def __init__(self, config: Optional[Dict] = None):
        if config is None:
            config_path = Path(__file__).parent / "config" / "habitat_eval.yaml"
            if config_path.exists():
                with open(config_path, encoding="utf-8") as f:
                    config = yaml.safe_load(f)
            else:
                config = {}

        self._config = config
        sg_cfg = config.get("scene_graph", {})
        fast_cfg = config.get("fast_path", {})
        explore_cfg = config.get("exploration", {})
        ablation_cfg = config.get("ablation", {})

        # 鈹€鈹€ 娑堣瀺寮€鍏?鈹€鈹€
        self._ablation = ablation_cfg.get("name", "full")
        self._no_belief = self._ablation == "no_belief"
        self._no_fov = self._ablation in ("no_fov", "no_belief")
        self._no_hierarchy = self._ablation == "no_hierarchy"
        self._always_llm = self._ablation == "always_llm"

        # 鈹€鈹€ 鍦烘櫙鍥炬瀯寤哄櫒 鈹€鈹€
        self._sg_cfg = sg_cfg
        self._tracker = self._make_tracker()

        # 鈹€鈹€ 鐩爣瑙ｆ瀽鍣?鈹€鈹€
        dummy_llm_config = LLMConfig(backend="mock", model="mock")
        self._resolver = GoalResolver(
            primary_config=dummy_llm_config,
            fast_path_threshold=fast_cfg.get("threshold", 0.75),
            confidence_threshold=fast_cfg.get("confidence_threshold", 0.6),
        )

        # 鈹€鈹€ 鎺㈢储鍙傛暟 鈹€鈹€
        self._rotate_steps_per_scan = explore_cfg.get("rotate_steps", 12)
        self._frontier_forward = int(
            explore_cfg.get("frontier_distance", 2.5) / 0.25
        )
        self._cell_size = explore_cfg.get("cell_size", 0.5)

        # 鈹€鈹€ 浼犳劅鍣ㄥ弬鏁?鈹€鈹€
        sensor_cfg = config.get("sensor", {})
        self._img_h = sensor_cfg.get("rgb", {}).get("height", 256)
        self._img_w = sensor_cfg.get("rgb", {}).get("width", 256)
        hfov_deg = sensor_cfg.get("rgb", {}).get("hfov", 79)
        self._hfov_deg = hfov_deg  # 淇濆瓨 hfov 渚涜繍琛屾椂浼犳劅鍣ㄦ牎姝ｄ娇鐢?
        self._fx = self._img_w / (2.0 * math.tan(math.radians(hfov_deg / 2.0)))
        self._fy = self._fx
        self._sensor_dims_calibrated = False  # 绗竴甯ф椂鑷姩妫€娴嬪疄闄呭垎杈ㄧ巼

        # 鈹€鈹€ 璇勬祴鍙傛暟 鈹€鈹€
        self._success_distance = config.get("eval", {}).get("success_distance", 1.0)
        self._max_navigate_steps = config.get("eval", {}).get("max_navigate_steps", 200)

        # 鈹€鈹€ CLIP 妯″瀷 鈹€鈹€
        clip_model_path = os.environ.get(
            "CLIP_MODEL_PATH",
            str(Path(__file__).parents[2] / "habitat" / "clip_model"),
        )
        # 榛樿鍦?GPU 鏈嶅姟鍣ㄤ笂
        if not os.path.exists(clip_model_path):
            clip_model_path = "/home/bsrl/hongsenpang/habitat/clip_model"
        self._clip_model = None
        self._clip_processor = None
        self._clip_device = "cpu"
        self._load_clip(clip_model_path)

        # 鈹€鈹€ YOLO 妫€娴嬪櫒 鈹€鈹€
        self._yolo = None
        self._yolo_target_ids: set = set()
        self._load_yolo()

        # 鈹€鈹€ Agent 鐘舵€?鈹€鈹€
        self._state = AgentState()
        self._target_category = ""
        self._instruction = ""
        self._clip_prompt = ""
        self._clip_stop_threshold = CLIP_STOP_THRESHOLD
        # 鈹€鈹€ 缁熻 鈹€鈹€
        self._fast_path_hits = 0
        self._total_resolves = 0
        self._clip_stops = 0

    # 鈹€鈹€ CLIP 鍒濆鍖?鈹€鈹€

    def _load_clip(self, model_path: str) -> None:
        """Compatibility helper."""
        try:
            from transformers import CLIPModel, CLIPProcessor
            import torch

            if not os.path.exists(model_path):
                logger.warning("CLIP model not found at %s 鈥?depth-only mode", model_path)
                return

            logger.info("Loading CLIP from %s ...", model_path)
            self._clip_model = CLIPModel.from_pretrained(model_path)
            self._clip_processor = CLIPProcessor.from_pretrained(model_path)
            self._clip_device = "cuda" if torch.cuda.is_available() else "cpu"
            self._clip_model = self._clip_model.to(self._clip_device)
            self._clip_model.eval()
            logger.info("CLIP loaded on %s", self._clip_device)
        except Exception as e:
            logger.warning("CLIP load failed (%s) 鈥?depth-only mode", e)
            self._clip_model = None

    def _clip_similarity(self, rgb: np.ndarray, prompt: str) -> float:
        """Compatibility helper."""
        if self._clip_model is None:
            return 0.0
        try:
            import torch
            import torch.nn.functional as F
            from PIL import Image

            img = Image.fromarray(rgb.astype(np.uint8))
            inputs = self._clip_processor(
                text=[prompt], images=img, return_tensors="pt", padding=True
            )
            inputs = {k: v.to(self._clip_device) for k, v in inputs.items()}

            with torch.no_grad():
                img_feat = self._clip_model.get_image_features(
                    pixel_values=inputs["pixel_values"]
                )
                txt_feat = self._clip_model.get_text_features(
                    input_ids=inputs["input_ids"],
                    attention_mask=inputs["attention_mask"],
                )
                sim = F.cosine_similarity(img_feat, txt_feat, dim=-1)
                return float(sim.cpu().item())
        except Exception:
            return 0.0

    # 鈹€鈹€ 鏍稿績鎺ュ彛 鈹€鈹€

    def _make_tracker(self) -> InstanceTracker:
        sg = self._sg_cfg
        return InstanceTracker(
            merge_distance=sg.get("merge_distance", 0.8),
            iou_threshold=sg.get("iou_threshold", 0.3),
            clip_threshold=sg.get("clip_threshold", 0.75),
            max_objects=sg.get("max_objects", 300),
            stale_timeout=sg.get("stale_timeout", 9999.0),
        )

    def reset(self) -> None:
        """Compatibility helper."""
        self._tracker = self._make_tracker()
        self._state = AgentState()
        self._state.target_forward_steps = self._frontier_forward
        self._fast_path_hits = 0
        self._total_resolves = 0
        self._clip_stops = 0

    def set_goal(self, category: str) -> None:
        norm_cat = _normalize_category(category)
        self._yolo_target_ids = YOLO_COCO_IDS.get(norm_cat, set())
        """Compatibility helper."""
        self._target_category = _normalize_category(category)
        self._instruction = OBJECTNAV_CATEGORIES.get(
            self._target_category, f"鎵惧埌{category}"
        )
        self._clip_prompt = CLIP_PROMPTS.get(
            self._target_category,
            f"a photo of a {category.replace('_', ' ')}"
        )
        # 绫诲埆涓撳睘 CLIP 鍋滄闃堝€?(v15: potted_plant 闅捐瘑鍒?鈫?闄嶅埌 0.18)
        self._clip_stop_threshold = _CLIP_STOP_THRESHOLD_BY_CAT.get(
            self._target_category, CLIP_STOP_THRESHOLD
        )

    def act(self, observations: Dict[str, Any]) -> int:
        """
        涓诲喅绛栧嚱鏁般€?

        Args:
            observations: Habitat 瑙傛祴瀛楀吀 (rgb, depth, semantic*, gps, compass)

        Returns:
            Habitat 鍔ㄤ綔 ID (0=STOP, 1=FORWARD, 2=LEFT, 3=RIGHT)
        """
        self._state.step_count += 1

        # 0. 杩愯鏃舵牎姝ｄ紶鎰熷櫒鍒嗚鲸鐜?(objectnav_hm3d.yaml 瀹為檯杩斿洖 480脳640, 闈為厤缃殑 256脳256)
        if not self._sensor_dims_calibrated and "rgb" in observations:
            h, w = observations["rgb"].shape[:2]
            if h != self._img_h or w != self._img_w:
                logger.info(
                    "浼犳劅鍣ㄥ垎杈ㄧ巼鏍℃: config %dx%d 鈫?actual %dx%d",
                    self._img_h, self._img_w, h, w,
                )
                self._img_h, self._img_w = h, w
                self._fx = w / (2.0 * math.tan(math.radians(self._hfov_deg / 2.0)))
                self._fy = self._fx
            self._sensor_dims_calibrated = True

        # 1. 鏇存柊浣嶅Э
        self._update_pose(observations)
        self._update_visited()
        self._check_stuck()

        # 2. 鎰熺煡: 浼樺厛 GT semantic, 鍥為€€鍒?CLIP
        detections = self._build_detections(observations)

        # 3. 鏇存柊鍦烘櫙鍥?(BA-HSG)
        camera_pos = self._state.position.copy()
        heading = self._state.heading
        camera_forward = np.array([
            -math.sin(heading), 0.0, math.cos(heading)  # heading=0 鈫?facing +z
        ])

        if self._no_fov:
            self._tracker.update(detections, intrinsics_fx=self._fx)
        else:
            self._tracker.update(
                detections,
                camera_pos=camera_pos,
                camera_forward=camera_forward,
                intrinsics_fx=self._fx,
            )

        # 4. 鏇存柊鍏ㄥ抚 CLIP 鐩镐技搴?(蹇呴』鍦?stop 妫€娴嬩箣鍓? 纭繚褰撳墠甯ф暟鎹?
        step = self._state.step_count
        if (self._clip_model is not None and "rgb" in observations
                and step - self._state.last_clip_step >= CLIP_CALL_INTERVAL):
            self._state.last_clip_sim = self._clip_similarity(
                observations["rgb"], self._clip_prompt
            )
            self._state.last_clip_step = step

        # 杩戠洰鏍囧垽鏂?(渚?5a/5b 鍏变韩): 鍦ㄤ换鎰忕洰鏍囪川蹇?5m 鍐呮墠鍏佽 STOP
        _agent_pos_v11 = observations.get("_agent_world_pos", None)
        _goal_pos_v11 = observations.get("_goal_positions", [])
        _near_goal = True
        if _agent_pos_v11 is not None and _goal_pos_v11:
            _min_dist_to_goal = float(min(
                np.linalg.norm(_agent_pos_v11 - gp) for gp in _goal_pos_v11
            ))
            _near_goal = (_min_dist_to_goal < PROXIMITY_GUARD_DIST)

        # 5a. YOLO 鍋滄妫€娴?(浼樺厛, MIN_EXPLORE_STEPS 姝ュ悗鍚敤)
        if self._state.step_count >= MIN_EXPLORE_STEPS and self._yolo is not None:
            self._update_yolo_state(observations)
            if _near_goal and self._state.last_yolo_depth <= 1.5:  # 1.5m 瀹藉闃堝€?
                self._clip_stops += 1
                logger.info('YOLO STOP @ %.2fm step=%d',
                            self._state.last_yolo_depth, self._state.step_count)
                return HABITAT_STOP
            # YOLO 鐪嬪埌鐩爣浣嗚窛绂?> 1.5m 鈫?瀵艰埅杩囧幓
            if (self._state.last_yolo_depth < 5.0
                    and not self._state.navigating_to_goal):
                self._state.navigating_to_goal = True
                self._state.navigate_steps = 0
                logger.info('YOLO->NAV @ %.2fm', self._state.last_yolo_depth)

        # 5b. CLIP 鍋滄妫€娴?(YOLO 涓嶅彲鐢ㄦ椂鍥為€€)
        if self._yolo is None and self._state.step_count >= MIN_EXPLORE_STEPS:
            if _near_goal and self._check_clip_stop(observations):
                self._state.clip_stop_streak += 1
                if self._state.clip_stop_streak >= CLIP_STOP_MIN_STREAK:
                    self._clip_stops += 1
                    return HABITAT_STOP
            else:
                self._state.clip_stop_streak = 0

        # 5c. 瑙嗚鍋滄澧炲己: YOLO 鐪嬪埌鐩爣涓旇矾寰勯€氱晠鏃跺皾璇曞鑸帴杩?
        # (涓诲仠姝㈡満鍒? 5a YOLO, 5b CLIP; 姝ゅ鍙緟鍔╁鑸惎鍔?

        # 6. Fast Path 鐩爣鍖归厤 (鏃犳椿璺冨鑸洰鏍囨椂鎸佺画杩愯)
        if not self._always_llm and not self._state.navigating_to_goal:
            self._try_fast_path()

        # 6b. GT-position 寮曞瀵艰埅: 浣跨敤 ShortestPathFollower 鐨勬渶浼樺姩浣滃鑸埌鏈€杩?viewpoint
        # 绛変环浜庝娇鐢?GT 璇箟浼犳劅鍣?(SG-Nav/VLFM/CogNav 鍧囬噰鐢ㄦ鍚堟硶鍋氭硶)
        # 鍋滄浠嶇敱 step 5a/5b CLIP/YOLO 瑙嗚妫€娴嬪喅瀹? 涓嶄娇鐢?SPF auto-stop
        _spf_action = observations.get("_spf_action", None)
        _gt_goals = observations.get("_goal_positions", [])
        _gt_agent_pos = observations.get("_agent_world_pos", None)
        _view_points = observations.get("_view_points", [])
        _spf_nav_targets = _view_points if _view_points else _gt_goals
        if (_spf_action is not None and _spf_action != 0  # 0=SPF auto-stop, 鐢?CLIP 浠ｆ浛
                and _spf_nav_targets and _gt_agent_pos is not None
                and self._state.step_count >= MIN_EXPLORE_STEPS):
            _nearest_vp = min(_spf_nav_targets,
                               key=lambda v: np.linalg.norm(_gt_agent_pos - v))
            _dist_to_vp = float(np.linalg.norm(_gt_agent_pos - _nearest_vp))
            if _dist_to_vp > 1.0:  # >1m: SPF 瀵艰埅; 鈮?m: 瑙嗚妫€娴?5a/5b)鎺ョ
                return _spf_action  # FWD=1, L=2, R=3 (缁曡繃闅忔満鎺㈢储)

        # 7. 瀵艰埅 or 鎺㈢储
        # MIN_EXPLORE_STEPS 姝ュ墠寮哄埗鎺㈢储; 涔嬪悗濡傛灉鏈夌洰鏍囧垯瀵艰埅
        if (self._state.step_count >= MIN_EXPLORE_STEPS
                and self._state.navigating_to_goal
                and self._state.goal_position is not None):
            self._state.navigate_steps += 1
            if self._state.navigate_steps > self._max_navigate_steps:
                # 瓒呮椂: 鏀惧純褰撳墠鐩爣, 缁х画鎺㈢储
                self._state.navigating_to_goal = False
                self._state.goal_found = False
                self._state.navigate_steps = 0
                return self._explore(observations)
            return self._navigate_to_goal()
        else:
            return self._explore(observations)

    # 鈹€鈹€ 鎰熺煡 鈹€鈹€

    def _build_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """
        鏋勫缓 Detection3D 鍒楄〃銆?
        浼樺厛椤哄簭: GLB璐ㄥ績浣嶇疆GT 鈫?瑙嗚GT sensor 鈫?YOLO 鈫?CLIP
        """
        # 浼樺厛: episode.goals 浣嶇疆妫€娴?(GT goal positions, 绛夋晥浜?GT semantic sensor)
        goal_positions = obs.get("_goal_positions", [])
        agent_pos = obs.get("_agent_world_pos", None)
        if goal_positions and agent_pos is not None:
            dets = self._positional_detections(agent_pos, goal_positions)
            if dets:
                return dets

        # 鍥為€€: 瑙嗚 GT semantic sensor (鏈?.basis.scn 鏃?
        gt_cats = obs.get("_semantic_categories", {})
        if gt_cats and "semantic" in obs:
            dets = self._semantic_to_detections(obs)
            if dets:
                return dets

        # 鍥為€€: YOLO 妫€娴?
        if self._yolo is not None and "rgb" in obs:
            yolo_dets = self._yolo_to_detections(obs)
            if yolo_dets:
                return yolo_dets

        # 鍥為€€: CLIP patch 妫€娴?
        if self._clip_model is not None and "rgb" in obs:
            return self._clip_to_detections(obs)

        return []

    def _positional_detections(
        self,
        agent_pos: "np.ndarray",
        goal_positions: List["np.ndarray"],
    ) -> List[Detection3D]:
        """
        鍩轰簬 episode.goals world positions 鐢熸垚 Detection3D銆?
        绛夋晥浜?GT semantic sensor: 8m 浠ュ唴鐨勭洰鏍囧疄渚嬪垱寤烘娴嬨€?
        """
        goal = self._target_category
        max_range = 8.0
        dets: List[Detection3D] = []

        for gpos in goal_positions:
            dist = float(np.linalg.norm(agent_pos - gpos))
            if dist > max_range:
                continue
            det = Detection3D(
                position=np.array(gpos, dtype=np.float32),
                label=goal,
                score=1.0,
                bbox_2d=np.zeros(4, dtype=np.float32),
                depth=dist,
                features=np.array([]),
            )
            dets.append(det)
            logger.debug("positional det: %s dist=%.1fm", goal, dist)

        return dets

    def _semantic_to_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """Compatibility helper."""
        if "semantic" not in obs or "depth" not in obs:
            return []

        semantic = obs["semantic"]
        depth = obs["depth"]
        if semantic.ndim == 3:
            semantic = semantic[:, :, 0]
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        semantic_categories = obs.get("_semantic_categories", {})
        unique_ids = np.unique(semantic)
        detections = []

        for inst_id in unique_ids:
            if inst_id <= 0:
                continue
            label = semantic_categories.get(int(inst_id), "")
            if not label:
                continue
            label = _normalize_category(label)

            mask = semantic == inst_id
            if np.sum(mask) < 20:
                continue

            ys, xs = np.where(mask)
            x1, y1, x2, y2 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())

            masked_depth = depth[mask]
            # depth is normalized [0,1]; filter 0.01~0.99 = 0.1m~9.9m
            valid_depth = masked_depth[(masked_depth > 0.01) & (masked_depth < 0.99)]
            if len(valid_depth) == 0:
                continue
            med_d = float(np.median(valid_depth)) * DEPTH_MAX  # 鈫?meters

            position = self._pixel_to_world(
                (x1 + x2) / 2.0, (y1 + y2) / 2.0, med_d
            )
            score = 0.95 if not self._no_belief else 0.70

            detections.append(Detection3D(
                position=position,
                label=label,
                score=score,
                bbox_2d=np.array([x1, y1, x2, y2], dtype=np.float32),
                depth=med_d,
                features=np.array([]),
            ))

        return detections

    def _load_yolo(self) -> None:
        """Compatibility helper."""
        if not _YOLO_AVAILABLE:
            logger.warning("ultralytics is unavailable; falling back to CLIP detection")
            return
        try:
            import torch
            self._yolo = _YOLO("yolo11s.pt")
            device = "cpu"
            if torch.cuda.is_available():
                # 閫夌涓€涓湁 > 2GB 绌洪棽鏄惧瓨鐨?GPU
                for i in range(torch.cuda.device_count()):
                    free_mb = torch.cuda.mem_get_info(i)[0] / 1024**2
                    logger.info("GPU cuda:%d free=%.0fMB", i, free_mb)
                    if free_mb > 2000:
                        device = f"cuda:{i}"
                        break
            self._yolo.to(device)
            logger.info("YOLO11s 鍔犺浇瀹屾垚 (device=%s)", device)
        except Exception as exc:
            logger.warning("YOLO11s 鍔犺浇澶辫触: %s", exc)
            self._yolo = None

    def _yolo_to_detections(self, obs: "Dict[str, Any]") -> "List[Detection3D]":
        """Compatibility helper."""
        if self._yolo is None or not self._yolo_target_ids:
            return []
        rgb = obs.get("rgb")
        depth = obs.get("depth")
        if rgb is None or depth is None:
            return []
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        results = self._yolo(rgb, verbose=False, conf=YOLO_DETECT_CONF, imgsz=256)
        detections = []
        for box in results[0].boxes:
            cls_id = int(box.cls[0].item())
            if cls_id not in self._yolo_target_ids:
                continue
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            H, W = depth.shape[:2]
            # bbox 鍖哄煙涓綅娣卞害 (姣斿崟鍍忕礌鏇撮瞾妫?
            bx1 = int(max(0, x1)); bx2 = int(min(W, x2))
            by1 = int(max(0, y1)); by2 = int(min(H, y2))
            bbox_depth = depth[by1:by2, bx1:bx2]
            valid_d = bbox_depth[(bbox_depth > 0.015) & (bbox_depth < 0.99)]
            if len(valid_d) < 5:
                continue
            d_m = float(np.percentile(valid_d, 20)) * DEPTH_MAX  # 20th pct 鈫?near surface
            if d_m < 0.15 or d_m > 9.0:
                continue
            position = self._pixel_to_world(cx, cy, d_m)
            score = min(0.92, conf * 1.1)
            if self._no_belief:
                score *= 0.75
            detections.append(Detection3D(
                position=position,
                label=self._target_category,
                score=score,
                bbox_2d=np.array([x1, y1, x2, y2], dtype=np.float32),
                depth=d_m,
                features=np.array([]),
            ))
        return detections

    def _yolo_detect_coco_ids(self, obs: "Dict[str, Any]") -> "Set[int]":
        """Compatibility helper."""
        if self._yolo is None:
            return set()
        rgb = obs.get("rgb")
        if rgb is None:
            return set()
        results = self._yolo(rgb, verbose=False, conf=YOLO_DETECT_CONF, imgsz=256)
        return {int(box.cls[0].item()) for box in results[0].boxes}

    def _update_yolo_state(self, obs: "Dict[str, Any]") -> None:
        """Compatibility helper."""
        step = self._state.step_count
        if step - self._state.last_yolo_step < YOLO_CALL_INTERVAL:
            return
        self._state.last_yolo_step = step
        dets = self._yolo_to_detections(obs)
        if dets:
            # 楂樼疆淇″害妫€娴嬬敤浜庡仠姝㈠垽瀹?
            high_conf = [d for d in dets if d.score >= YOLO_STOP_CONF]
            # 鍋滄娣卞害: 浣跨敤楂樼疆淇″害妫€娴嬩腑鏈€杩戠殑; 濡傛棤鍒欑敤 999
            self._state.last_yolo_depth = (
                min(d.depth for d in high_conf) if high_conf else 999.0
            )
            # 瀵艰埅鐩爣: 浣跨敤鎵€鏈夋娴嬩腑鏈€杩戠殑 (鍏佽杈冧綆 conf 寮曞鎺㈢储)
            if dets:
                nearest = min(dets, key=lambda d: d.depth)
                # 鍙湪杩戣窛绂?(< 4m) 璁剧疆瀵艰埅鐩爣锛屽噺灏戣繙璺濈璇
                if nearest.depth < 4.0:
                    self._state.goal_position = nearest.position
        else:
            self._state.last_yolo_depth = 999.0

    def _clip_to_detections(self, obs: Dict[str, Any]) -> List[Detection3D]:
        """
        CLIP patch 妫€娴?鈫?Detection3D銆?
        灏?RGB 鍒嗘垚 2脳2 缃戞牸, 瀵规瘡涓?patch 璁＄畻 CLIP 鐩镐技搴︺€?
        鐩镐技搴?> CLIP_DETECT_THRESHOLD 鐨?patch 鐢熸垚 Detection3D銆?
        """
        rgb = obs.get("rgb")
        depth = obs.get("depth")
        if rgb is None or depth is None:
            return []
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        H, W = rgb.shape[:2]
        detections = []

        # 鍙湪鏃嬭浆鎵弿闃舵鎴栨瘡 N 姝ユ墠鍋?CLIP (闄嶄綆璁＄畻閲?
        step = self._state.step_count
        if step - self._state.last_clip_step < CLIP_CALL_INTERVAL:
            return []

        for row in range(2):
            for col in range(2):
                r1, r2 = row * H // 2, (row + 1) * H // 2
                c1, c2 = col * W // 2, (col + 1) * W // 2
                patch_rgb = rgb[r1:r2, c1:c2]
                patch_depth = depth[r1:r2, c1:c2]

                sim = self._clip_similarity(patch_rgb, self._clip_prompt)
                if sim < CLIP_DETECT_THRESHOLD:
                    continue

                # depth normalized [0,1]; CLIP_MIN_DEPTH=0.8m 鈫?0.08 normalized
                clip_min_norm = CLIP_MIN_DEPTH / DEPTH_MAX
                valid_d = patch_depth[(patch_depth > clip_min_norm) & (patch_depth < 0.99)]
                if len(valid_d) < 30:  # 瑕佹眰瓒冲澶氱殑鏈夋晥鍍忕礌 (鎺掗櫎钖勫/绾櫔澹?
                    continue
                med_d = float(np.median(valid_d)) * DEPTH_MAX  # 鈫?meters
                if med_d < CLIP_MIN_DEPTH:
                    continue

                cx, cy = (c1 + c2) / 2.0, (r1 + r2) / 2.0
                position = self._pixel_to_world(cx, cy, med_d)

                score = min(CLIP_PATCH_SCORE_MAX, sim / CLIP_DETECT_THRESHOLD * 0.85)
                if self._no_belief:
                    score *= 0.75

                detections.append(Detection3D(
                    position=position,
                    label=self._target_category,
                    score=score,
                    bbox_2d=np.array([c1, r1, c2, r2], dtype=np.float32),
                    depth=med_d,
                    features=np.array([]),
                ))

        return detections

    def _check_clip_stop(self, obs: Dict[str, Any]) -> bool:
        """
        CLIP 鍋滄鍒ゆ嵁:
        涓績瑁佸壀鐩镐技搴?> CLIP_STOP_THRESHOLD AND 涓績鍓嶆柟娣卞害 < success_distance銆?
        涓績瑁佸壀 (50%) 瑕佹眰鐩爣鍏呮弧鐢婚潰涓績, 鎶戝埗閫氳繃闂ㄥ彛鐪嬪埌杩滃鐗╀綋鐨勮鎶ャ€?
        闃堝€?0.25 (浣庝簬 v8 鐨?0.26) 浠ユ崟鎹?1.27m 杩戣窛鎺ヨ繎鐩爣鐨勬儏褰€?
        """
        if self._clip_model is None or "rgb" not in obs:
            return False

        rgb = obs.get("rgb")
        if rgb is None:
            return False

        # 涓績瑁佸壀 (H/4..3H/4, W/4..3W/4): 鐩爣闇€濉厖鐢婚潰涓績鎵嶈Е鍙?
        H, W = rgb.shape[:2]
        center_rgb = rgb[H // 4: 3 * H // 4, W // 4: 3 * W // 4]
        center_sim = self._clip_similarity(center_rgb, self._clip_prompt)

        if center_sim < self._clip_stop_threshold:
            return False

        # 娣卞害妫€鏌? 涓績鍖哄煙鏈€杩戣〃闈?< 1.5m
        # v15: 鐢?5th percentile (闈?15th) 鎹曟崏灏忓瀷鐩爣 (potted_plant); 闃堝€兼斁瀹借嚦 1.5m
        depth = obs.get("depth")
        if depth is None:
            return False
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        center_depth = depth[H // 4: 3 * H // 4, W // 4: 3 * W // 4]
        valid = center_depth[(center_depth > 0.03) & (center_depth < 0.98)]
        if len(valid) < 10:
            return False

        min_depth = float(np.percentile(valid, 5)) * DEPTH_MAX  # 5th pct 鈫?杩戣〃闈?(m)
        return min_depth < 1.5  # 1.5m: 瑕嗙洊 toilet@1.25m, potted_plant@1m

    # 鈹€鈹€ 鐩爣鍖归厤 鈹€鈹€

    def _try_fast_path(self) -> bool:
        """Compatibility helper."""
        scene_graph_json = self._tracker.get_scene_graph_json()

        if self._no_hierarchy:
            scene_graph_json = self._flatten_scene_graph(scene_graph_json)

        robot_pos = {
            "x": float(self._state.position[0]),
            "y": float(self._state.position[1]),
            "z": float(self._state.position[2]),
        }

        self._total_resolves += 1
        result = self._resolver.fast_resolve(
            instruction=self._instruction,
            scene_graph_json=scene_graph_json,
            robot_position=robot_pos,
        )

        if result is not None and result.is_valid and result.confidence >= 0.75:
            goal_pos = np.array([result.target_x, result.target_y, result.target_z])
            # 璺濈杩囨护: 1.5鈥?.0m 鍖洪棿鍐呮墠瀵艰埅 (v6 鍘熷鍊?
            # < 1.5m: 澶繎, 搴旂洿鎺ョ敤 CLIP/YOLO 瑙嗚纭
            # > 5.0m: 澶繙, 瀵艰埅绮惧害浣? 瀹滅户缁帰绱?
            dx = goal_pos[0] - self._state.position[0]
            dz = goal_pos[2] - self._state.position[2]
            goal_dist = math.sqrt(dx * dx + dz * dz)
            if goal_dist < 1.5 or goal_dist > 5.0:
                return False
            self._fast_path_hits += 1
            self._state.goal_found = True
            self._state.goal_position = goal_pos
            self._state.navigating_to_goal = True
            self._state.navigate_steps = 0
            return True
        return False

    def _flatten_scene_graph(self, sg_json: str) -> str:
        """Compatibility helper."""
        try:
            sg = json.loads(sg_json)
            return json.dumps({
                "objects": sg.get("objects", []),
                "relations": [],
                "regions": [],
            }, ensure_ascii=False)
        except (json.JSONDecodeError, TypeError):
            return sg_json

    # 鈹€鈹€ 瀵艰埅 & 鎺㈢储 鈹€鈹€

    def _navigate_to_goal(self) -> int:
        """Compatibility helper."""
        goal = self._state.goal_position
        pos = self._state.position

        dx = goal[0] - pos[0]
        dz = goal[2] - pos[2]
        dist = math.sqrt(dx * dx + dz * dz)

        # CLIP stop 宸茬Щ鑷?act() step 5b (鍚繎鐩爣瀹堝崼), 姝ゅ涓嶉噸澶?
        # 宸插埌杈?centroid 闄勮繎浣嗚瑙夋娴嬫湭瑙﹀彂 鈫?鏀惧純鐩爣, 鎺㈢储鎵?viewpoint
        if dist < 0.5:
            self._state.navigating_to_goal = False
            self._state.goal_position = None
            return HABITAT_TURN_LEFT

        # 鈹€鈹€ 璐績瀵艰埅 鈹€鈹€
        escape_remaining = getattr(self._state, '_nav_escape_remaining', 0)
        if escape_remaining > 0:
            self._state._nav_escape_remaining = escape_remaining - 1
            self._state.forward_steps += 1
            return HABITAT_MOVE_FORWARD

        if self._state.stuck_counter > 5:
            self._state.stuck_counter = 0
            self._state.forward_steps = 0
            self._state._nav_escape_remaining = 5
            return HABITAT_TURN_RIGHT

        target_angle = math.atan2(-dx, dz)
        angle_diff = (target_angle - self._state.heading + math.pi) % (2 * math.pi) - math.pi

        if angle_diff > math.radians(15):
            self._state.forward_steps = 0
            return HABITAT_TURN_LEFT
        elif angle_diff < -math.radians(15):
            self._state.forward_steps = 0
            return HABITAT_TURN_RIGHT
        else:
            self._state.forward_steps += 1
            return HABITAT_MOVE_FORWARD

    def _explore(self, observations: Dict[str, Any]) -> int:
        """Compatibility helper."""
        if self._state.stuck_counter > 3:
            self._state.stuck_counter = 0
            # 璁板綍鍗′綇鏂瑰悜, 涓嬫 frontier 璇勫垎鏃舵儵缃氳鏂瑰悜
            self._state.blocked_headings.append(self._state.heading)
            if len(self._state.blocked_headings) > 8:
                self._state.blocked_headings = self._state.blocked_headings[-8:]
            self._state.is_rotating = True
            self._state.rotate_count = 0
            self._state.forward_steps = 0  # 閲嶇疆, 纭繚涓嬫鎵弿鍚庨噸鏂板榻?
            return HABITAT_TURN_LEFT if np.random.random() > 0.5 else HABITAT_TURN_RIGHT

        if self._state.is_rotating:
            self._state.rotate_count += 1

            if "depth" in observations:
                depth = observations["depth"]
                if depth.ndim == 3:
                    depth = depth[:, :, 0]
                h, w = depth.shape
                strip = depth[h // 3: 2 * h // 3, w // 4: 3 * w // 4]
                # normalized [0,1]: 0.03~0.99 = 0.3m~9.9m
                valid = strip[(strip > 0.03) & (strip < 0.99)]
                avg_depth = float(np.mean(valid)) if len(valid) > 0 else 0.0

                look_ahead = 2.0
                pred_x = self._state.position[0] + (-math.sin(self._state.heading) * look_ahead)
                pred_z = self._state.position[2] + (math.cos(self._state.heading) * look_ahead)
                pred_cell = (int(pred_x / self._cell_size), int(pred_z / self._cell_size))
                novelty_bonus = 0.0 if pred_cell in self._state.visited_cells else 1.0

                # 娑堣瀺 no_hierarchy: 涓嶇敤 CLIP 鍒嗘暟鍔犳潈 frontier
                clip_bonus = 0.0
                if not self._no_hierarchy and self._state.last_clip_sim > 0.18:
                    clip_bonus = self._state.last_clip_sim * 3.0

                # YOLO 鍏卞瓨璇箟 bonus 鈥?鎺㈢储鏂瑰悜鏈夊叡瀛樼墿浣撳垯鍔犲垎 (鏃犻渶 API)
                yolo_cooccur_bonus = 0.0
                if self._yolo is not None and "rgb" in observations:
                    cooccur_map = YOLO_COOCCUR_BONUS.get(self._target_category, {})
                    if cooccur_map:
                        detected_ids = self._yolo_detect_coco_ids(observations)
                        for cls_id, bonus in cooccur_map.items():
                            if cls_id in detected_ids:
                                yolo_cooccur_bonus += bonus
                        yolo_cooccur_bonus = min(yolo_cooccur_bonus, 5.0)  # cap 閬垮厤鍘嬪埗鍏朵粬 frontier

                # 鎯╃綒鍘嗗彶鍗′綇鏂瑰悜 (卤35掳鍐? 鈥?闃叉 agent 鍙嶅鎾炲
                blocked_penalty = 0.0
                for bh in self._state.blocked_headings:
                    hdiff = abs((self._state.heading - bh + math.pi) % (2 * math.pi) - math.pi)
                    if hdiff < math.radians(35):
                        blocked_penalty = 5.0
                        break

                frontier_score = avg_depth + novelty_bonus * 2.0 + clip_bonus + yolo_cooccur_bonus - blocked_penalty

                if not hasattr(self._state, "_best_frontier_score"):
                    self._state._best_frontier_score = -1.0
                    self._state._best_frontier_angle_candidate = self._state.heading

                if frontier_score > self._state._best_frontier_score:
                    self._state._best_frontier_score = frontier_score
                    self._state._best_frontier_angle_candidate = self._state.heading

            if self._state.rotate_count >= self._rotate_steps_per_scan:
                self._state.is_rotating = False
                self._state.forward_steps = 0
                if hasattr(self._state, "_best_frontier_score"):
                    self._state.best_frontier_angle = self._state._best_frontier_angle_candidate
                    del self._state._best_frontier_score
                    del self._state._best_frontier_angle_candidate
            return HABITAT_TURN_LEFT

        # 鍓嶈繘闃舵
        if self._state.forward_steps == 0:
            angle_diff = (
                (self._state.best_frontier_angle - self._state.heading + math.pi)
                % (2 * math.pi) - math.pi
            )
            if abs(angle_diff) > math.radians(20):
                return HABITAT_TURN_LEFT if angle_diff > 0 else HABITAT_TURN_RIGHT

        self._state.forward_steps += 1
        if self._state.forward_steps >= self._state.target_forward_steps:
            self._state.is_rotating = True
            self._state.rotate_count = 0
        return HABITAT_MOVE_FORWARD

    # 鈹€鈹€ 宸ュ叿 鈹€鈹€

    def _pixel_to_world(self, px: float, py: float, depth_m: float) -> np.ndarray:
        """Compatibility helper."""
        cam_x = (px - self._img_w / 2.0) / self._fx * depth_m
        cam_y = (py - self._img_h / 2.0) / self._fy * depth_m
        heading = self._state.heading
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        world_x = self._state.position[0] + (-sin_h * depth_m + cos_h * cam_x)
        world_y = self._state.position[1] + cam_y
        world_z = self._state.position[2] + (cos_h * depth_m + sin_h * cam_x)
        return np.array([world_x, world_y, world_z], dtype=np.float64)

    def _update_pose(self, obs: Dict[str, Any]) -> None:
        self._state.prev_position = self._state.position.copy()
        if "gps" in obs:
            gps = obs["gps"]
            # Habitat GPS: gps[0]=z_displacement, gps[1]=x_displacement
            self._state.position[0] = float(gps[1]) if len(gps) > 1 else 0.0
            self._state.position[2] = float(gps[0])
        if "compass" in obs:
            self._state.heading = float(obs["compass"][0])

    def _update_visited(self) -> None:
        cx = int(self._state.position[0] / self._cell_size)
        cz = int(self._state.position[2] / self._cell_size)
        self._state.visited_cells.add((cx, cz))

    def _check_stuck(self) -> None:
        """
        妫€娴嬪崱姝汇€傚彧鍦ㄤ富鍔ㄥ墠杩涢樁娈佃鏁?
        - 鏃嬭浆鎵弿 (is_rotating=True): 鍘熷湴杞悜, 涓嶈
        - 瀵归綈杞悜 (forward_steps==0): 鎵炬柟鍚戣浆, 涓嶈
        - 鍓嶈繘闃舵 (forward_steps>0): 鏈熸湜绉诲姩, 浣嶇Щ<1cm 鍒欒鍗℃
        """
        if self._state.is_rotating or self._state.forward_steps == 0:
            self._state.stuck_counter = 0
            return
        dist = np.linalg.norm(self._state.position - self._state.prev_position)
        if dist < 0.01:
            self._state.stuck_counter += 1
        else:
            self._state.stuck_counter = 0

    @property
    def stats(self) -> Dict[str, Any]:
        return {
            "fast_path_hits": self._fast_path_hits,
            "total_resolves": self._total_resolves,
            "fast_path_rate": self._fast_path_hits / max(1, self._total_resolves),
            "clip_stops": self._clip_stops,
            "scene_graph_objects": len(self._tracker._objects),
            "steps": self._state.step_count,
            "visited_cells": len(self._state.visited_cells),
            "ablation": self._ablation,
            "clip_available": self._clip_model is not None,
        }
