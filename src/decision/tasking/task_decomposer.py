"""
任务分解器 — 将复杂自然语言指令拆解为子目标序列。

参考论文:
  - SayCan (Google, 2022):      LLM 分解 → 可行性评分 → 执行
  - Inner Monologue (2022):     Chain-of-thought task decomposition
  - SG-Nav (NeurIPS 2024):      层次场景图 + 子目标规划

核心思想:
  "去厨房拿红色杯子" →
    SubGoal(action=NAVIGATE, target="kitchen area")
    SubGoal(action=FIND, target="red cup")
    SubGoal(action=APPROACH, target="red cup")
    SubGoal(action=VERIFY, target="red cup")

  每个子目标有自己的成功/失败条件和回退策略。

规则分解 (快速路径, 无需 LLM) 已移至 task_rules.TaskRulesMixin。
使用时通过继承引入:
    class TaskDecomposer(TaskRulesMixin): ...
"""

import json
import logging
import re
from dataclasses import dataclass, field
from enum import Enum

logger = logging.getLogger(__name__)

# TaskRulesMixin is imported lazily at class definition time (below) to avoid
# circular imports — task_rules itself does a deferred import of the dataclasses
# defined in this module.
from .task_rules import TaskRulesMixin  # noqa: E402


class SubGoalAction(Enum):
    """子目标动作类型 (参考 LOVON + SayCan)。"""
    NAVIGATE = "navigate"           # 导航到指定区域/位置
    FIND = "find"                   # 在当前视野中搜索目标
    APPROACH = "approach"           # 接近已发现的目标
    VERIFY = "verify"              # 近距离验证目标身份
    LOOK_AROUND = "look_around"    # 原地旋转扫描 (LOVON)
    EXPLORE = "explore"            # 探索未知区域
    BACKTRACK = "backtrack"        # 回退到上一个位置 (LOVON)
    WAIT = "wait"                  # 等待条件满足
    FOLLOW = "follow"              # 持续跟随动态目标 (人/物体)
    STOP = "stop"                  # 停止当前任务
    PICK = "pick"                  # 抓取/取物 (Manipulation)
    PLACE = "place"                # 放置物体 (Manipulation)
    STATUS = "status"              # 查询系统/机器人状态
    PATROL = "patrol"              # 巡检/巡逻
    SAVE_MAP = "save_map"          # 保存地图
    SAVE_POI = "save_poi"          # 保存/标记兴趣点
    SET_SPEED = "set_speed"        # 调整速度
    SET_GEOFENCE = "set_geofence"  # 设置电子围栏
    RETURN_HOME = "return_home"    # 返回基地/充电桩
    PAUSE = "pause"                # 暂停当前任务
    RESUME = "resume"              # 恢复暂停的任务


class SubGoalStatus(Enum):
    """子目标状态。"""
    PENDING = "pending"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


@dataclass
class SubGoal:
    """单个子目标。"""
    step_id: int
    action: SubGoalAction
    target: str                     # 目标描述 ("kitchen", "red cup", etc.)
    parameters: dict = field(default_factory=dict)  # 附加参数
    status: SubGoalStatus = SubGoalStatus.PENDING
    result: dict | None = None   # 执行结果
    retry_count: int = 0
    max_retries: int = 2

    def to_dict(self) -> dict:
        return {
            "step_id": self.step_id,
            "action": self.action.value,
            "target": self.target,
            "status": self.status.value,
            "retry_count": self.retry_count,
        }


@dataclass
class TaskPlan:
    """完整的任务计划。"""
    instruction: str
    subgoals: list[SubGoal] = field(default_factory=list)
    current_step: int = 0

    @property
    def is_complete(self) -> bool:
        return bool(self.subgoals) and all(
            sg.status in (SubGoalStatus.COMPLETED, SubGoalStatus.SKIPPED)
            for sg in self.subgoals
        )

    @property
    def is_failed(self) -> bool:
        return any(
            sg.status == SubGoalStatus.FAILED and sg.retry_count >= sg.max_retries
            for sg in self.subgoals
        )

    @property
    def active_subgoal(self) -> SubGoal | None:
        for sg in self.subgoals:
            if sg.status in (SubGoalStatus.PENDING, SubGoalStatus.ACTIVE):
                return sg
        return None

    def advance(self):
        """标记当前子目标完成, 前进到下一个。"""
        active = self.active_subgoal
        if active:
            active.status = SubGoalStatus.COMPLETED
            self.current_step += 1

    def fail_current(self):
        """标记当前子目标失败。"""
        active = self.active_subgoal
        if active:
            active.retry_count += 1
            if active.retry_count >= active.max_retries:
                active.status = SubGoalStatus.FAILED
            else:
                active.status = SubGoalStatus.PENDING  # 允许重试

    def to_dict(self) -> dict:
        return {
            "instruction": self.instruction,
            "total_steps": len(self.subgoals),
            "current_step": self.current_step,
            "is_complete": self.is_complete,
            "subgoals": [sg.to_dict() for sg in self.subgoals],
        }


class TaskDecomposer(TaskRulesMixin):
    """
    任务分解器: LLM 将自然语言 → 子目标序列。

    两种模式:
      1. LLM 分解: 发送指令给 LLM, 返回结构化子目标
      2. 规则分解: 简单指令的快速路径, 不调 LLM (由 TaskRulesMixin 提供)

    v2.0 新增:
      - KG 安全门: 规则分解前/后检查 KG 安全约束 (ConceptBot / SafeMind)
      - 可供性验证: PICK/PLACE 前查 KG 判断物体是否可操作
      - 开放词汇: 未知物体通过 KG 映射到已知概念获取属性

    规则匹配数据与 decompose_with_rules() 由 TaskRulesMixin 提供。
    """

    _knowledge_graph = None  # type: Any | None

    @classmethod
    def set_knowledge_graph(cls, kg) -> None:
        """注入知识图谱 (全局设置, planner_node 启动时调用)。"""
        cls._knowledge_graph = kg

    def build_decomposition_prompt(
        self,
        instruction: str,
        scene_summary: str = "",
        language: str = "zh",
    ) -> list[dict[str, str]]:
        """
        构建 LLM 分解 prompt (SayCan / Inner Monologue 风格)。

        Args:
            instruction: 用户指令
            scene_summary: 场景摘要
            language: "zh" / "en"

        Returns:
            messages list for LLM
        """
        available_actions = ", ".join(a.value for a in SubGoalAction)

        if language == "zh":
            system = f"""你是一个机器人任务规划器。将用户的自然语言指令分解为一系列可执行的子目标。

可用动作类型: {available_actions}

规则:
1. 每个子目标必须是原子操作 (单一动作)
2. navigate: 导航到一个区域或位置
3. find: 在场景图中搜索匹配物体
4. approach: 接近已发现的目标 (最后 0.5m)
5. verify: 近距离确认目标身份
6. look_around: 原地 360° 扫描
7. explore: 去未探索区域搜索
8. backtrack: 回到上一位置
9. wait: 等待 (用于动态场景)

输出格式 (严格 JSON):
{{
  "subgoals": [
    {{"action": "navigate", "target": "...", "parameters": {{}}}},
    ...
  ]
}}"""
            user_content = f"## 指令\n{instruction}"
            if scene_summary:
                user_content += f"\n\n## 当前场景\n{scene_summary}"
        else:
            system = f"""You are a robot task planner. Decompose the instruction into subgoals.

Available actions: {available_actions}

Rules:
1. Each subgoal must be atomic (single action)
2. navigate: go to a region or position
3. find: search for matching object in scene graph
4. approach: move to within 0.5m of target
5. verify: close-range identity confirmation
6. look_around: 360° scan in place
7. explore: go to unexplored area
8. backtrack: return to previous position
9. wait: wait for condition

Output format (strict JSON):
{{
  "subgoals": [
    {{"action": "navigate", "target": "...", "parameters": {{}}}},
    ...
  ]
}}"""
            user_content = f"## Instruction\n{instruction}"
            if scene_summary:
                user_content += f"\n\n## Current Scene\n{scene_summary}"

        return [
            {"role": "system", "content": system},
            {"role": "user", "content": user_content},
        ]

    def parse_decomposition_response(
        self, instruction: str, response_text
    ) -> TaskPlan:
        """
        解析 LLM 返回的子目标列表。

        Args:
            instruction: 原始指令
            response_text: LLM 响应文本 (str 或 dict)

        Returns:
            TaskPlan
        """
        # BUG FIX: LLM client (e.g. Claude) 可能已经返回 dict
        if isinstance(response_text, dict):
            data = response_text
            subgoals = []
            for i, sg_data in enumerate(data.get("subgoals", [])):
                action_str = sg_data.get("action", "navigate")
                try:
                    action = SubGoalAction(action_str)
                except ValueError:
                    action = SubGoalAction.NAVIGATE
                subgoals.append(SubGoal(
                    step_id=i,
                    action=action,
                    target=sg_data.get("target", instruction),
                    parameters=sg_data.get("parameters", {}),
                ))
            if not subgoals:
                subgoals = [
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                    SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                ]
            return TaskPlan(instruction=instruction, subgoals=subgoals)

        # 提取 JSON
        match = re.search(r"```(?:json)?\s*([\s\S]*?)```", response_text)
        if match:
            raw = match.group(1).strip()
        else:
            start = response_text.find("{")
            end = response_text.rfind("}")
            if start == -1 or end == -1:
                logger.error("No JSON in decomposition response: %s", response_text[:200])
                # Fallback: 单步导航
                return TaskPlan(
                    instruction=instruction,
                    subgoals=[
                        SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                        SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                    ],
                )
            raw = response_text[start:end + 1]

        try:
            data = json.loads(raw)
        except json.JSONDecodeError as e:
            logger.error("JSON parse error in decomposition: %s", e)
            return TaskPlan(
                instruction=instruction,
                subgoals=[
                    SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                    SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
                ],
            )

        subgoals = []
        for i, sg_data in enumerate(data.get("subgoals", [])):
            action_str = sg_data.get("action", "navigate")
            try:
                action = SubGoalAction(action_str)
            except ValueError:
                action = SubGoalAction.NAVIGATE

            subgoals.append(SubGoal(
                step_id=i,
                action=action,
                target=sg_data.get("target", instruction),
                parameters=sg_data.get("parameters", {}),
            ))

        if not subgoals:
            subgoals = [
                SubGoal(step_id=0, action=SubGoalAction.NAVIGATE, target=instruction),
                SubGoal(step_id=1, action=SubGoalAction.VERIFY, target=instruction),
            ]

        return TaskPlan(instruction=instruction, subgoals=subgoals)
