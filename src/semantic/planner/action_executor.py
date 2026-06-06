from __future__ import annotations

"""
动作执行器 — 将子目标动作映射为具体的机器人行为。

参考论文:
  - LOVON (2024): 定义 6 种动作原语
      APPROACH_TARGET, LOOK_AROUND, BACKTRACK, EXPLORE_FRONTIER,
      NAVIGATE_TO_POINT, VERIFY_TARGET
  - SayCan (Google, 2022): 动作的可行性评估
  - Inner Monologue (2022): 执行反馈 → LLM 重规划

核心设计:
  每个 SubGoalAction 映射到一个具体的执行流:
    NAVIGATE  → 发布 PoseStamped, 等待到达
    FIND      → 查询场景图, 匹配目标
    APPROACH  → 发布接近航点 (0.5m), 减速
    VERIFY    → 近距离拍照, CLIP/VLM 确认身份
    LOOK_AROUND → 发布旋转指令 (yaw 增量 360°)
    EXPLORE   → 选择 frontier 或最少访问方向
    BACKTRACK → 从拓扑记忆获取上一个位置, 导航回去
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

from core.msgs.numpy_compat import np

from core.runtime_interface import map_frame_id
from core.utils.sanitize import safe_json_loads

logger = logging.getLogger(__name__)
ACTION_COMMAND_MAP_FRAME_ID = map_frame_id()


class ActionStatus(Enum):
    """动作执行状态。"""
    IDLE = "idle"
    EXECUTING = "executing"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    TIMEOUT = "timeout"


@dataclass
class ActionCommand:
    """
    发给底层导航栈的命令。

    统一接口: 无论动作类型, 最终都变成以下几种命令之一。
    """
    command_type: str               # "goal" | "velocity" | "cancel"

    # goal 类型
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_yaw: float = 0.0        # rad, 目标朝向

    # velocity 类型 (用于 LOOK_AROUND 的旋转)
    linear_x: float = 0.0
    angular_z: float = 0.0

    # 参数
    timeout_sec: float = 30.0
    approach_speed: float = 0.3     # m/s (APPROACH 时减速)
    frame_id: str = ACTION_COMMAND_MAP_FRAME_ID


class ActionExecutor:
    """
    动作执行器: 将高层子目标 → 底层导航命令。

    这是 LOVON 论文中 "Action Primitive" 层的实现。

    使用流程:
      1. planner_node 调用 execute_subgoal(subgoal, context)
      2. executor 根据 action 类型生成 ActionCommand
      3. planner_node 将 ActionCommand 发布到 ROS2 topics
      4. executor 通过 update_feedback() 接收执行反馈
      5. 返回动作完成/失败状态
    """

    def __init__(
        self,
        approach_distance: float = 0.5,     # APPROACH 距离 (m)
        verify_distance: float = 0.8,       # VERIFY 距离 (m)
        look_around_speed: float = 0.5,     # rad/s 旋转速度
        look_around_duration: float = 12.0, # 秒 (≈ 360° at 0.5 rad/s)
        nav_timeout: float = 60.0,
    ) -> None:
        self.approach_distance = approach_distance
        self.verify_distance = verify_distance
        self.look_around_speed = look_around_speed
        self.look_around_duration = look_around_duration
        self.nav_timeout = nav_timeout

        self._status = ActionStatus.IDLE
        self._start_time = 0.0
        self._lock = threading.Lock()  # 保护 _status/_start_time 的跨线程访问

    def _set_executing(self) -> None:
        """线程安全地设置执行状态。"""
        with self._lock:
            self._status = ActionStatus.EXECUTING
            self._start_time = time.time()

    @property
    def status(self) -> ActionStatus:
        return self._status

    def generate_navigate_command(
        self,
        target_position: dict[str, float],
        robot_position: dict[str, float] | None = None,
    ) -> ActionCommand:
        """
        NAVIGATE: 导航到目标位置。

        Args:
            target_position: {"x": ..., "y": ..., "z": ...}
            robot_position: 当前位置 (用于计算朝向)

        Returns:
            ActionCommand
        """
        # 计算朝向目标方向
        yaw = 0.0
        if robot_position:
            dx = target_position["x"] - robot_position["x"]
            dy = target_position["y"] - robot_position["y"]
            yaw = math.atan2(dy, dx)

        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=target_position["x"],
            target_y=target_position["y"],
            target_z=target_position.get("z", 0.0),
            target_yaw=yaw,
            timeout_sec=self.nav_timeout,
        )

    def generate_approach_command(
        self,
        target_position: dict[str, float],
        robot_position: dict[str, float],
    ) -> ActionCommand:
        """
        APPROACH: 接近目标 (最后一段路, 减速)。

        LOVON 论文的关键动作: 最后 0.5m 减速接近,
        同时保持目标在视野中心, 为 VERIFY 做准备。

        Args:
            target_position: 目标 3D 位置
            robot_position: 当前位置

        Returns:
            ActionCommand (带有减速参数)
        """
        dx = target_position["x"] - robot_position["x"]
        dy = target_position["y"] - robot_position["y"]
        dist = math.sqrt(dx * dx + dy * dy)
        yaw = math.atan2(dy, dx)

        # 接近到 approach_distance 处 (不是到目标正上方)
        if dist > self.approach_distance:
            scale = (dist - self.approach_distance) / dist
            goal_x = robot_position["x"] + dx * scale
            goal_y = robot_position["y"] + dy * scale
        else:
            # 已经足够近, 不需要移动, 只调整朝向
            goal_x = robot_position["x"]
            goal_y = robot_position["y"]

        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=goal_x,
            target_y=goal_y,
            target_z=target_position.get("z", 0.0),
            target_yaw=yaw,
            approach_speed=0.15,  # 减速
            timeout_sec=20.0,
        )

    def generate_look_around_command(self) -> ActionCommand:
        """
        LOOK_AROUND: 原地 360° 旋转扫描。

        LOVON 的核心动作之一: 到达新区域后先扫描,
        让感知模块捕获周围所有物体。

        Returns:
            ActionCommand (velocity 类型, angular_z)
        """
        self._set_executing()

        return ActionCommand(
            command_type="velocity",
            angular_z=self.look_around_speed,
            timeout_sec=self.look_around_duration,
        )

    def generate_verify_command(
        self,
        target_position: dict[str, float],
        robot_position: dict[str, float],
    ) -> ActionCommand:
        """
        VERIFY: 面向目标, 准备视觉验证。

        验证流程 (LOVON):
          1. 面向目标 (调整 yaw)
          2. 感知模块拍摄高分辨率图
          3. CLIP / VLM 确认 "这是我要找的东西吗?"
          4. 确认 → 子目标完成; 否认 → 继续搜索

        这里只生成"面向目标"的命令, 实际验证由 planner_node 调用 VLM。
        """
        dx = target_position["x"] - robot_position["x"]
        dy = target_position["y"] - robot_position["y"]
        yaw = math.atan2(dy, dx)

        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=robot_position["x"],  # 不移动
            target_y=robot_position["y"],
            target_yaw=yaw,                # 只转向
            timeout_sec=10.0,
        )

    def generate_backtrack_command(
        self,
        backtrack_position: np.ndarray,
    ) -> ActionCommand:
        """
        BACKTRACK: 回退到之前的位置。

        LOVON: 当目标丢失时, 回到上次看到目标的位置重新搜索。
        """
        self._set_executing()

        return ActionCommand(
            command_type="goal",
            target_x=float(backtrack_position[0]),
            target_y=float(backtrack_position[1]),
            target_z=float(backtrack_position[2]) if len(backtrack_position) > 2 else 0.0,
            timeout_sec=self.nav_timeout,
        )

    def check_timeout(self) -> bool:
        """检查当前动作是否超时。

        NOTE: Called by planner_node._monitor_callback() for per-goal timeout detection.
        线程安全: _lock 保护 _status/_start_time 的原子读取。
        """
        with self._lock:
            if self._status != ActionStatus.EXECUTING:
                return False
            return (time.time() - self._start_time) > self.nav_timeout

    def mark_succeeded(self) -> None:
        with self._lock:
            self._status = ActionStatus.SUCCEEDED

    def mark_failed(self) -> None:
        with self._lock:
            self._status = ActionStatus.FAILED

    def reset(self) -> None:
        with self._lock:
            self._status = ActionStatus.IDLE
            self._start_time = 0.0

    # ── LERa 三步失败恢复: Look → Explain → Replan ──

    def lera_recover(
        self,
        failed_action: str,
        current_labels: list[str],
        original_goal: str,
        failure_count: int = 1,
        llm_client: Any | None = None,
        event_loop: Any | None = None,
    ) -> str:
        """
        LERa 三步失败恢复：Look → Explain → Replan。

        Look:    收集当前可见对象标签作为场景描述
        Explain: LLM 分析失败原因
        Replan:  LLM 输出恢复策略

        Args:
            failed_action: 失败的动作描述
            current_labels: 当前可见对象标签列表
            original_goal: 原始导航目标
            failure_count: 已连续失败次数
            llm_client: 可选的 LLM 客户端（需有 chat() 方法）
            event_loop: 已有的 asyncio 事件循环（避免阻塞 ROS2 回调线程）

        Returns:
            恢复策略: "retry_different_path" | "expand_search" |
                      "requery_goal" | "abort"
        """
        # Look：场景描述
        label_str = "、".join(current_labels[:8]) if current_labels else "无可见对象"
        scene_desc = f"当前附近可见对象：{label_str}"

        # Explain + Replan：一次 LLM 调用
        prompt = (
            "机器人导航动作失败，请分析原因并给出恢复方案。\n\n"
            f"失败动作：{failed_action}\n"
            f"{scene_desc}\n"
            f"导航目标：{original_goal}\n"
            f"已失败次数：{failure_count}\n\n"
            "请输出 JSON（只输出 JSON，不要其他文字）：\n"
            '{"reason": "失败原因一句话", '
            '"action": "retry_different_path|expand_search|requery_goal|abort", '
            '"params": {}}\n\n'
            "选择规则：\n"
            "- retry_different_path：路径被阻挡，但目标可能正确\n"
            "- expand_search：目标不在预期位置，需扩大搜索\n"
            "- requery_goal：目标描述可能有歧义\n"
            "- abort：连续失败>2次且无合理恢复方案"
        )

        valid_actions = ("retry_different_path", "expand_search", "requery_goal", "abort")

        if llm_client is not None:
            try:
                import asyncio
                import re

                messages = [{"role": "user", "content": prompt}]

                # P0-1 fix: 使用调用方提供的事件循环，避免在 ROS2 回调线程上
                # 创建阻塞的 event loop（会冻结导航 LLM 超时秒数）
                loop = event_loop
                if loop is not None and loop.is_running():
                    future = asyncio.run_coroutine_threadsafe(
                        llm_client.chat(messages, temperature=0.3), loop
                    )
                    response = future.result(timeout=15.0)
                else:
                    # 无可用 loop 时的降级：创建临时 loop（仅用于无 ROS2 上下文场景）
                    tmp_loop = asyncio.new_event_loop()
                    try:
                        response = tmp_loop.run_until_complete(
                            llm_client.chat(messages, temperature=0.3)
                        )
                    finally:
                        tmp_loop.close()

                if not response:
                    logger.warning("[LERa] LLM returned empty response")
                else:
                    match = re.search(r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}', response, re.DOTALL)
                    if match:
                        data = safe_json_loads(match.group(), default={})
                        action = data.get('action', '')
                        if action in valid_actions:
                            logger.info(
                                "[LERa] LLM recovery: reason=%s action=%s",
                                data.get('reason', '?'), action,
                            )
                            return action
            except Exception as e:
                logger.warning("[LERa] LLM call failed, using fallback: %s", e)

        # Fallback：基于失败次数的简单规则
        if failure_count >= 3:
            return "abort"
        elif failure_count >= 2:
            return "expand_search"
        return "retry_different_path"
