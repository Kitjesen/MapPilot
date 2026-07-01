"""PlannerState 枚举 — 独立模块，避免 mixin 与 planner_node 之间的循环导入。"""

from enum import Enum


class PlannerState(Enum):
    """规划器状态。"""
    IDLE = "idle"
    DECOMPOSING = "decomposing"
    RESOLVING = "resolving"
    NAVIGATING = "navigating"
    LOOKING_AROUND = "looking_around"
    APPROACHING = "approaching"
    VERIFYING = "verifying"
    EXPLORING = "exploring"
    BACKTRACKING = "backtracking"
    REPLANNING = "replanning"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"  # F1: 任务取消状态
