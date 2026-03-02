"""
隐式 FSM 策略 (LOVON 风格)。

核心思想:
  状态转移不通过 if/else 硬编码, 而是由参数化模型预测:
    s_{t+1} = f_theta(obs_t, s_t, instruction)

本实现提供轻量级线性网络后端 (NumPy), 支持:
  - 使用默认参数直接推理
  - 从 .npz 权重文件加载参数
"""

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import numpy as np

logger = logging.getLogger(__name__)


MISSION_STATES = ["success", "searching_1", "searching_0", "running"]
SEARCH_STATES = ["had_searching_1", "had_searching_0"]


@dataclass
class ImplicitFSMObservation:
    """隐式 FSM 输入观测。"""

    mission_instruction_0: str
    mission_instruction_1: str
    mission_object_1: str
    predicted_object: str
    confidence: float
    object_xyn: np.ndarray
    object_whn: np.ndarray
    mission_state_in: str
    search_state_in: str


@dataclass
class ImplicitFSMPrediction:
    """隐式 FSM 输出。"""

    mission_state_out: str
    search_state_out: str
    motion_vector: np.ndarray
    state_prob: np.ndarray
    search_prob: np.ndarray


class _LinearFSMModel:
    """轻量线性模型: x -> state/search logits + motion。"""

    def __init__(
        self,
        w_state: np.ndarray,
        b_state: np.ndarray,
        w_search: np.ndarray,
        b_search: np.ndarray,
        w_motion: np.ndarray,
        b_motion: np.ndarray,
    ):
        self.w_state = w_state
        self.b_state = b_state
        self.w_search = w_search
        self.b_search = b_search
        self.w_motion = w_motion
        self.b_motion = b_motion

    @classmethod
    def from_npz(cls, path: str) -> "_LinearFSMModel":
        arr = np.load(path)
        return cls(
            w_state=np.asarray(arr["w_state"], dtype=np.float64),
            b_state=np.asarray(arr["b_state"], dtype=np.float64),
            w_search=np.asarray(arr["w_search"], dtype=np.float64),
            b_search=np.asarray(arr["b_search"], dtype=np.float64),
            w_motion=np.asarray(arr["w_motion"], dtype=np.float64),
            b_motion=np.asarray(arr["b_motion"], dtype=np.float64),
        )

    @classmethod
    def default(cls) -> "_LinearFSMModel":
        # 特征维度: 12
        # [conf, centered, size, matched, missing, instr_changed,
        #  s_success, s_s1, s_s0, s_run, h_s1, h_s0]
        w_state = np.array([
            # success, searching_1, searching_0, running
            [1.4, -0.4, -0.4, 0.9],    # conf
            [0.6, -0.2, -0.2, 0.5],    # centered
            [1.0, -0.3, -0.3, -0.4],   # size
            [2.0, -0.4, -0.4, 0.8],    # matched
            [-2.4, 1.6, 1.6, -1.0],    # missing
            [-1.2, 1.2, 1.2, -0.8],    # instruction changed
            [0.8, -0.6, -0.6, -0.2],   # state success
            [-0.2, 1.0, -0.6, -0.1],   # state searching_1
            [-0.2, -0.6, 1.0, -0.1],   # state searching_0
            [0.2, 0.6, 0.6, 0.9],      # state running
            [0.1, -0.4, 0.9, 0.1],     # search had_s1
            [0.1, 0.9, -0.4, 0.1],     # search had_s0
        ], dtype=np.float64)
        b_state = np.array([-0.2, 0.2, 0.2, 0.1], dtype=np.float64)

        w_search = np.array([
            # had_s1, had_s0
            [0.0, 0.0],      # conf
            [0.0, 0.0],      # centered
            [0.0, 0.0],      # size
            [0.0, 0.0],      # matched
            [0.0, 0.0],      # missing
            [0.4, 0.4],      # instruction changed
            [0.0, 0.0],      # state success
            [0.8, -0.6],     # state searching_1
            [-0.6, 0.8],     # state searching_0
            [0.0, 0.0],      # state running
            [1.2, -0.2],     # had_s1
            [-0.2, 1.2],     # had_s0
        ], dtype=np.float64)
        b_search = np.array([0.0, 0.0], dtype=np.float64)

        w_motion = np.array([
            # vx, vy, wz
            [0.8, 0.0, 0.0],     # conf
            [0.2, 0.0, 0.0],     # centered
            [0.3, 0.0, 0.0],     # size
            [0.9, 0.0, 0.0],     # matched
            [-0.6, 0.0, 0.5],    # missing
            [-0.3, 0.0, 0.0],    # instruction changed
            [0.0, 0.0, 0.0],     # state success
            [0.0, 0.0, 0.9],     # state searching_1
            [0.0, 0.0, -0.9],    # state searching_0
            [0.2, 0.0, 0.0],     # state running
            [0.0, 0.0, 0.3],     # had_s1
            [0.0, 0.0, -0.3],    # had_s0
        ], dtype=np.float64)
        b_motion = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        return cls(w_state, b_state, w_search, b_search, w_motion, b_motion)

    def forward(self, x: np.ndarray):
        state_logits = x @ self.w_state + self.b_state
        search_logits = x @ self.w_search + self.b_search
        motion = np.tanh(x @ self.w_motion + self.b_motion)
        return state_logits, search_logits, motion


class ImplicitFSMPolicy:
    """隐式 FSM 策略封装。"""

    def __init__(
        self,
        weights_path: str = "",
        strict: bool = False,
    ):
        self.strict = strict
        self.weights_path = weights_path
        self._ready = False

        self._model = None
        if weights_path:
            p = Path(weights_path)
            if p.exists():
                try:
                    self._model = _LinearFSMModel.from_npz(str(p))
                    self._ready = True
                except (OSError, ValueError, KeyError) as e:
                    logger.warning("Failed to load FSM weights from %s: %s", p, e)
                    if strict:
                        raise

        if self._model is None:
            self._model = _LinearFSMModel.default()
            self._ready = True

    @property
    def is_ready(self) -> bool:
        return self._ready

    def predict(self, obs: ImplicitFSMObservation) -> Optional[ImplicitFSMPrediction]:
        if not self._ready or self._model is None:
            return None

        x = self._encode_features(obs)
        state_logits, search_logits, motion = self._model.forward(x)

        state_prob = self._softmax(state_logits)
        search_prob = self._softmax(search_logits)

        state_idx = int(np.argmax(state_prob))
        search_idx = int(np.argmax(search_prob))

        return ImplicitFSMPrediction(
            mission_state_out=MISSION_STATES[state_idx],
            search_state_out=SEARCH_STATES[search_idx],
            motion_vector=np.asarray(motion, dtype=np.float64),
            state_prob=np.asarray(state_prob, dtype=np.float64),
            search_prob=np.asarray(search_prob, dtype=np.float64),
        )

    @staticmethod
    def _softmax(x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        x = x - np.max(x)
        exp_x = np.exp(x)
        return exp_x / np.sum(exp_x)

    @staticmethod
    def _is_match(predicted_object: str, mission_object: str) -> bool:
        p = (predicted_object or "").strip().lower()
        m = (mission_object or "").strip().lower()
        if not p or not m:
            return False
        if p == "null":
            return False
        return p in m or m in p

    @staticmethod
    def _state_one_hot(value: str, categories) -> np.ndarray:
        out = np.zeros(len(categories), dtype=np.float64)
        try:
            idx = categories.index(value)
            out[idx] = 1.0
        except ValueError:
            pass
        return out

    def _encode_features(self, obs: ImplicitFSMObservation) -> np.ndarray:
        conf = float(np.clip(obs.confidence, 0.0, 1.0))
        xyn = np.asarray(obs.object_xyn, dtype=np.float64)
        whn = np.asarray(obs.object_whn, dtype=np.float64)

        cx = float(np.clip(xyn[0] if xyn.size > 0 else 0.5, 0.0, 1.0))
        centered = 1.0 - min(1.0, abs(cx - 0.5) * 2.0)

        w = float(np.clip(whn[0] if whn.size > 0 else 0.0, 0.0, 1.0))
        h = float(np.clip(whn[1] if whn.size > 1 else 0.0, 0.0, 1.0))
        size_score = max(w, h)

        matched = 1.0 if self._is_match(obs.predicted_object, obs.mission_object_1) else 0.0
        missing = 1.0 if (obs.predicted_object or "").strip().lower() in ("", "null") else 0.0
        instruction_changed = 1.0 if obs.mission_instruction_0 != obs.mission_instruction_1 else 0.0

        mission_one_hot = self._state_one_hot(obs.mission_state_in, MISSION_STATES)
        search_one_hot = self._state_one_hot(obs.search_state_in, SEARCH_STATES)

        return np.array([
            conf,
            centered,
            size_score,
            matched,
            missing,
            instruction_changed,
            mission_one_hot[0],
            mission_one_hot[1],
            mission_one_hot[2],
            mission_one_hot[3],
            search_one_hot[0],
            search_one_hot[1],
        ], dtype=np.float64)

    @staticmethod
    def export_default_weights(path: str):
        """导出默认参数为 .npz，便于离线微调后替换。"""
        model = _LinearFSMModel.default()
        np.savez(
            path,
            w_state=model.w_state,
            b_state=model.b_state,
            w_search=model.w_search,
            b_search=model.b_search,
            w_motion=model.w_motion,
            b_motion=model.b_motion,
        )
