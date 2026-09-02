# ruff: noqa: F401
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pytest

from perception.tracking.instance_tracker import (
    BELIEF_FRESHNESS_TAU,
    InstanceTracker,
    TrackedObject,
)
from perception.tracking.projection import Detection3D
from decision.goals.resolver import (
    GoalResolver,
    GoalResult,
    TargetBeliefManager,
    TargetHypothesis,
)
from decision.tasks.decomposition import TaskDecomposer, SubGoalAction, SubGoalStatus
from memory.scheduling.voi_scheduler import SchedulerAction, SchedulerState, VoIConfig, VoIScheduler

def make_office_corridor_scene() -> dict:
    """办公走廊场景: 3 间办公室 + 1 条走廊, 约 30 个物体。"""
    objects = [
        # 走廊物体
        {"id": 0, "label": "door", "position": {"x": 3.5, "y": 1.2, "z": 1.0},
         "score": 0.92, "detection_count": 15, "room": "corridor",
         "belief": {"P_exist": 0.91, "sigma_pos": 0.05, "credibility": 0.88}},
        {"id": 1, "label": "fire extinguisher", "position": {"x": 4.0, "y": 1.0, "z": 0.8},
         "score": 0.88, "detection_count": 8, "room": "corridor",
         "belief": {"P_exist": 0.85, "sigma_pos": 0.08, "credibility": 0.82}},
        {"id": 2, "label": "sign", "position": {"x": 6.0, "y": -0.5, "z": 1.5},
         "score": 0.85, "detection_count": 12, "room": "corridor",
         "belief": {"P_exist": 0.88, "sigma_pos": 0.06, "credibility": 0.80}},
        {"id": 3, "label": "trash can", "position": {"x": 2.5, "y": 4.0, "z": 0.0},
         "score": 0.78, "detection_count": 5, "room": "corridor",
         "belief": {"P_exist": 0.75, "sigma_pos": 0.12, "credibility": 0.70}},
        {"id": 4, "label": "door", "position": {"x": 8.0, "y": 0.5, "z": 1.0},
         "score": 0.90, "detection_count": 10, "room": "corridor",
         "belief": {"P_exist": 0.90, "sigma_pos": 0.05, "credibility": 0.86}},
        {"id": 5, "label": "exit door", "position": {"x": 14.0, "y": 0.0, "z": 1.0},
         "score": 0.82, "detection_count": 3, "room": "corridor",
         "belief": {"P_exist": 0.72, "sigma_pos": 0.15, "credibility": 0.65}},
        {"id": 6, "label": "stairs", "position": {"x": 12.0, "y": -1.0, "z": 0.0},
         "score": 0.87, "detection_count": 7, "room": "corridor",
         "belief": {"P_exist": 0.86, "sigma_pos": 0.10, "credibility": 0.78}},
        {"id": 7, "label": "sign", "position": {"x": 11.5, "y": -0.5, "z": 1.5},
         "score": 0.80, "detection_count": 6, "room": "corridor",
         "belief": {"P_exist": 0.80, "sigma_pos": 0.10, "credibility": 0.72}},
        {"id": 8, "label": "elevator", "position": {"x": 15.0, "y": 0.0, "z": 0.0},
         "score": 0.75, "detection_count": 2, "room": "corridor",
         "belief": {"P_exist": 0.68, "sigma_pos": 0.20, "credibility": 0.60}},
        {"id": 30, "label": "fire extinguisher", "position": {"x": 8.0, "y": 0.5, "z": 0.8},
         "score": 0.82, "detection_count": 4, "room": "corridor",
         "belief": {"P_exist": 0.78, "sigma_pos": 0.12, "credibility": 0.72}},
        {"id": 31, "label": "fire extinguisher", "position": {"x": 12.0, "y": -0.5, "z": 0.8},
         "score": 0.79, "detection_count": 3, "room": "corridor",
         "belief": {"P_exist": 0.72, "sigma_pos": 0.15, "credibility": 0.65}},
        {"id": 32, "label": "door", "position": {"x": 12.0, "y": 0.5, "z": 1.0},
         "score": 0.86, "detection_count": 6, "room": "corridor",
         "belief": {"P_exist": 0.84, "sigma_pos": 0.08, "credibility": 0.78}},
        {"id": 33, "label": "button", "position": {"x": 14.5, "y": 0.5, "z": 1.0},
         "score": 0.70, "detection_count": 2, "room": "corridor",
         "belief": {"P_exist": 0.65, "sigma_pos": 0.18, "credibility": 0.55}},

        # 办公室 A 物体
        {"id": 10, "label": "desk", "position": {"x": 4.0, "y": 3.5, "z": 0.7},
         "score": 0.91, "detection_count": 18, "room": "office",
         "belief": {"P_exist": 0.93, "sigma_pos": 0.04, "credibility": 0.90}},
        {"id": 11, "label": "chair", "position": {"x": 5.0, "y": 2.0, "z": 0.4},
         "score": 0.89, "detection_count": 20, "room": "office",
         "belief": {"P_exist": 0.92, "sigma_pos": 0.04, "credibility": 0.89}},
        {"id": 12, "label": "monitor", "position": {"x": 4.5, "y": 3.0, "z": 0.8},
         "score": 0.86, "detection_count": 14, "room": "office",
         "belief": {"P_exist": 0.90, "sigma_pos": 0.05, "credibility": 0.85}},
        {"id": 13, "label": "computer", "position": {"x": 5.0, "y": 3.0, "z": 0.7},
         "score": 0.84, "detection_count": 10, "room": "office",
         "belief": {"P_exist": 0.87, "sigma_pos": 0.07, "credibility": 0.80}},
        {"id": 14, "label": "keyboard", "position": {"x": 4.2, "y": 3.2, "z": 0.75},
         "score": 0.80, "detection_count": 8, "room": "office",
         "belief": {"P_exist": 0.82, "sigma_pos": 0.08, "credibility": 0.76}},
        {"id": 15, "label": "lamp", "position": {"x": 4.0, "y": 2.5, "z": 1.2},
         "score": 0.77, "detection_count": 6, "room": "office",
         "belief": {"P_exist": 0.78, "sigma_pos": 0.10, "credibility": 0.72}},
        {"id": 16, "label": "trash can", "position": {"x": 4.2, "y": 3.8, "z": 0.0},
         "score": 0.75, "detection_count": 4, "room": "office",
         "belief": {"P_exist": 0.73, "sigma_pos": 0.12, "credibility": 0.68}},
        {"id": 17, "label": "bottle", "position": {"x": 5.0, "y": 3.0, "z": 0.85},
         "score": 0.72, "detection_count": 3, "room": "office",
         "belief": {"P_exist": 0.70, "sigma_pos": 0.15, "credibility": 0.62}},
        {"id": 18, "label": "window", "position": {"x": 3.0, "y": 6.0, "z": 1.0},
         "score": 0.83, "detection_count": 9, "room": "office",
         "belief": {"P_exist": 0.85, "sigma_pos": 0.07, "credibility": 0.79}},
        {"id": 19, "label": "chair", "position": {"x": 3.5, "y": 5.5, "z": 0.4},
         "score": 0.85, "detection_count": 11, "room": "office",
         "belief": {"P_exist": 0.87, "sigma_pos": 0.06, "credibility": 0.82}},

        # 休息区物体
        {"id": 20, "label": "sofa", "position": {"x": 7.0, "y": 5.0, "z": 0.4},
         "score": 0.90, "detection_count": 12, "room": "lounge",
         "belief": {"P_exist": 0.91, "sigma_pos": 0.05, "credibility": 0.87}},
        {"id": 21, "label": "table", "position": {"x": 6.5, "y": 4.5, "z": 0.4},
         "score": 0.85, "detection_count": 8, "room": "lounge",
         "belief": {"P_exist": 0.83, "sigma_pos": 0.08, "credibility": 0.78}},
        {"id": 22, "label": "tv", "position": {"x": 7.0, "y": 5.5, "z": 1.0},
         "score": 0.80, "detection_count": 5, "room": "lounge",
         "belief": {"P_exist": 0.79, "sigma_pos": 0.10, "credibility": 0.72}},
        {"id": 23, "label": "person", "position": {"x": 6.0, "y": 2.0, "z": 0.0},
         "score": 0.76, "detection_count": 2, "room": "lounge",
         "belief": {"P_exist": 0.65, "sigma_pos": 0.25, "credibility": 0.55}},

        # 茶水间物体
        {"id": 24, "label": "refrigerator", "position": {"x": 10.0, "y": 6.0, "z": 0.0},
         "score": 0.88, "detection_count": 7, "room": "kitchen",
         "belief": {"P_exist": 0.86, "sigma_pos": 0.08, "credibility": 0.80}},
        {"id": 25, "label": "shelf", "position": {"x": 9.0, "y": 4.0, "z": 0.0},
         "score": 0.82, "detection_count": 5, "room": "storage",
         "belief": {"P_exist": 0.80, "sigma_pos": 0.10, "credibility": 0.74}},
        {"id": 26, "label": "cabinet", "position": {"x": 8.0, "y": 3.0, "z": 0.0},
         "score": 0.79, "detection_count": 4, "room": "storage",
         "belief": {"P_exist": 0.76, "sigma_pos": 0.12, "credibility": 0.70}},
        {"id": 27, "label": "trash can", "position": {"x": 10.5, "y": 5.5, "z": 0.0},
         "score": 0.73, "detection_count": 3, "room": "kitchen",
         "belief": {"P_exist": 0.70, "sigma_pos": 0.15, "credibility": 0.63}},
    ]

    relations = [
        {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 0.6},
        {"subject_id": 1, "relation": "near", "object_id": 2, "distance": 2.1},
        {"subject_id": 6, "relation": "near", "object_id": 7, "distance": 0.7},
        {"subject_id": 8, "relation": "near", "object_id": 33, "distance": 0.7},
        {"subject_id": 10, "relation": "near", "object_id": 12, "distance": 0.7},
        {"subject_id": 12, "relation": "on", "object_id": 10, "distance": 0.1},
        {"subject_id": 11, "relation": "near", "object_id": 10, "distance": 1.8},
        {"subject_id": 13, "relation": "near", "object_id": 10, "distance": 1.0},
        {"subject_id": 16, "relation": "near", "object_id": 10, "distance": 0.4},
        {"subject_id": 18, "relation": "near", "object_id": 19, "distance": 0.7},
        {"subject_id": 20, "relation": "near", "object_id": 21, "distance": 0.7},
        {"subject_id": 21, "relation": "in_front_of", "object_id": 20, "distance": 0.7},
        {"subject_id": 24, "relation": "near", "object_id": 27, "distance": 0.7},
        {"subject_id": 4, "relation": "near", "object_id": 30, "distance": 0.5},
        {"subject_id": 6, "relation": "near", "object_id": 31, "distance": 0.7},
        {"subject_id": 32, "relation": "near", "object_id": 31, "distance": 0.5},
    ]

    rooms = [
        {"id": "room_0", "name": "corridor", "center": {"x": 8.0, "y": 0.0},
         "object_ids": [0, 1, 2, 3, 4, 5, 6, 7, 8, 30, 31, 32, 33]},
        {"id": "room_1", "name": "office", "center": {"x": 4.5, "y": 3.5},
         "object_ids": [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]},
        {"id": "room_2", "name": "lounge", "center": {"x": 7.0, "y": 5.0},
         "object_ids": [20, 21, 22, 23]},
        {"id": "room_3", "name": "kitchen", "center": {"x": 10.0, "y": 6.0},
         "object_ids": [24, 27]},
        {"id": "room_4", "name": "storage", "center": {"x": 8.5, "y": 3.5},
         "object_ids": [25, 26]},
    ]

    groups = [
        {"id": "group_0", "name": "safety_equipment", "room": "corridor",
         "object_ids": [1, 2, 30, 31]},
        {"id": "group_1", "name": "office_workstation", "room": "office",
         "object_ids": [10, 11, 12, 13, 14]},
        {"id": "group_2", "name": "lounge_furniture", "room": "lounge",
         "object_ids": [20, 21, 22]},
    ]

    return {
        "objects": objects,
        "relations": relations,
        "rooms": rooms,
        "groups": groups,
        "summary": "Office-corridor environment: corridor (13 objects), office (10), lounge (4), kitchen (2), storage (2)",
    }

def load_instruction_set() -> dict:
    """加载指令集 JSON。"""
    p = Path(__file__).resolve().parent / "fixtures" / "offline_instruction_set.json"
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)

class FastPathResult:
    instruction_id: str
    instruction_en: str
    instruction_zh: str
    level: str
    resolved: bool = False
    target_label_match: bool = False
    position_error: float = float("inf")
    success: bool = False
    confidence: float = 0.0
    latency_ms: float = 0.0
    reasoning: str = ""
