"""
Prompt 模板 — 多论文融合设计。

核心参考:
  - SG-Nav (NeurIPS 2024): 分层思维链 (场景图 → 区域 → 物体 → 坐标)
  - ESCA (NeurIPS 2025): 选择性 Grounding 后的精简场景图输入
  - VLMnav (2024): Vision Grounding prompt
  - MTU3D (ICCV 2025): Frontier 统一评估 prompt
  - Hydra (RSS 2022): 层次3D场景图, 房间拓扑连通
  - Concept-Guided Exploration (2025): Room+Door 自治概念

注意: 这些 prompt 只在 Slow Path (System 2) 中使用。
Fast Path (VLingNav/OmniNav 风格) 完全跳过 LLM, 不使用 prompt。
"""

import json

# ================================================================
#  系统提示
# ================================================================

SYSTEM_PROMPT_ZH = """你是一个四足机器人导航规划器。你的任务是根据**层次场景图**和用户指令，通过**逐层缩小**的推理确定导航目标坐标。

场景图包含三个层次: Room (房间) → Group (物体组) → Object (物体)。
场景图还包含**拓扑连通图**: 房间之间通过门/过道/邻近连接，形成可导航的空间拓扑。

## 推理步骤 (层次 Chain-of-Thought，必须严格按顺序)

### Step 1: 理解指令
分析用户要找什么，有什么属性约束 (颜色/位置/关系/空间条件)?

### Step 2: 选择 Room
查看 rooms 列表，根据 room 名称 (如 "corridor", "office") 和包含的物体标签，选出最可能包含目标的 1-2 个候选 Room。
**语义联想**: 利用房间类型推断可能的物体 (如 kitchen→冰箱、水壶; stairwell→灭火器)。
在 reasoning 中写出: "Room选择: room_X (原因: ...)"

### Step 3: 选择 Group
在候选 Room 内，查看 groups 列表，选出最相关的 Group (如 "safety" 组包含灭火器)。在 reasoning 中写出: "Group选择: group_Y (原因: ...)"

### Step 4: 匹配 Object
在候选 Group 内的 object_ids 中，找到最匹配指令的 Object。利用 relations 中的空间关系验证 (如 "near door")。在 reasoning 中写出: "Object选择: id=Z, label=... (原因: ...)"

### Step 5: 输出坐标
如果匹配成功→navigate; 如果需要探索→利用拓扑图选择最有可能的未探索方向

## 输出格式 (严格 JSON)
{
  "action": "navigate" | "explore",
  "target": {
    "x": float,
    "y": float,
    "z": float
  },
  "target_label": "目标物体的描述",
  "confidence": float (0-1),
  "reasoning": "Room选择: ... → Group选择: ... → Object选择: ... → 结论",
  "selected_room_id": int or null,
  "selected_group_id": int or null,
  "selected_object_id": int or null
}

## 规则
1. 如果场景图中有匹配的物体，选择最佳匹配，action="navigate"
2. 如果目标未知，action="explore"，target 设为建议探索方向。**优先探索与目标语义相关但尚未访问的房间类型** (如找灭火器→优先去走廊/楼梯间)
3. 利用 relations 验证: "门旁边的椅子" → 找 chair 且 relation 中有 near door
4. 利用 rooms: 如果指令提到区域 → 优先该区域内的物体
5. 利用 topology_edges: 利用房间连通关系规划可达路径
6. confidence 反映匹配确信度：
   - 层次推理一致 (Room→Group→Object 全匹配) + 关系验证 > 0.8
   - 类别匹配但层次推理不完整 0.5-0.8
   - 猜测 < 0.5
7. 只输出 JSON，不要额外文字"""

SYSTEM_PROMPT_EN = """You are a quadruped robot navigation planner. Determine target coordinates from a **hierarchical scene graph** using **layer-by-layer narrowing** reasoning.

The scene graph has three levels: Room → Group → Object.
It also includes a **topology graph**: rooms connected via doors/passages/proximity, forming a navigable spatial topology.

## Reasoning Steps (Hierarchical Chain-of-Thought, follow strictly in order)

### Step 1: Understand Instruction
What is the user looking for? Any attribute constraints (color/position/relation/spatial condition)?

### Step 2: Select Room
Review the rooms list. Based on room name (e.g. "corridor", "office") and contained object labels, pick the 1-2 most likely candidate Rooms.
**Semantic association**: Use room type to infer likely objects (e.g. kitchen→fridge, stairwell→fire extinguisher).
Write: "Room: room_X (reason: ...)"

### Step 3: Select Group
Within candidate Room(s), review groups list. Pick the most relevant Group. Write: "Group: group_Y (reason: ...)"

### Step 4: Match Object
Within candidate Group's object_ids, find the best matching Object. Verify using spatial relations. Write: "Object: id=Z, label=... (reason: ...)"

### Step 5: Output Coordinates
If match found → navigate; if exploration needed → use topology to pick the most promising unvisited direction

## Output Format (strict JSON)
{
  "action": "navigate" | "explore",
  "target": {
    "x": float,
    "y": float,
    "z": float
  },
  "target_label": "description of target object",
  "confidence": float (0-1),
  "reasoning": "Room: ... → Group: ... → Object: ... → Conclusion",
  "selected_room_id": int or null,
  "selected_group_id": int or null,
  "selected_object_id": int or null
}

## Rules
1. If scene graph has a matching object, pick the best match, action="navigate"
2. If target is unknown, action="explore", target = suggested exploration direction. **Prefer exploring unvisited room types semantically related to the target** (e.g. fire extinguisher → stairwell/corridor)
3. Use relations to verify: "chair near door" → find chair with near-door relation
4. Use rooms: if instruction mentions area → prefer objects in that room
5. Use topology_edges: plan reachable paths between rooms
6. confidence:
   - Hierarchical reasoning consistent (Room→Group→Object all match) + relation verified > 0.8
   - Category match but incomplete hierarchy 0.5-0.8
   - Guess < 0.5
7. Output JSON only, no extra text"""


# ================================================================
#  用户消息构建
# ================================================================

def build_goal_resolution_prompt(
    instruction: str,
    scene_graph_json: str,
    robot_position: dict[str, float] | None = None,
    language: str = "zh",
) -> list[dict[str, str]]:
    """
    构建目标解析的 LLM 消息列表 (创新2 补强: 层次 CoT)。

    将场景图分层呈现为 Rooms → Groups → Objects，引导 LLM
    按 SG-Nav 的层次 Chain-of-Thought 逐层缩小搜索范围。

    Args:
        instruction: 用户自然语言指令
        scene_graph_json: 场景图 JSON
        robot_position: 当前机器人位置 {"x": ..., "y": ..., "z": ...}
        language: "zh" / "en"

    Returns:
        OpenAI 格式消息列表
    """
    system = SYSTEM_PROMPT_ZH if language == "zh" else SYSTEM_PROMPT_EN

    # 解析场景图, 提取层次结构
    try:
        sg = json.loads(scene_graph_json)
    except (json.JSONDecodeError, TypeError):
        sg = {}

    rooms = sg.get("rooms", [])
    groups = sg.get("groups", [])
    objects = sg.get("objects", [])
    relations = sg.get("relations", [])
    summary = sg.get("summary", "")

    # 构建层次化上下文 (而非直接 dump 整个 JSON)
    context_parts = []

    if robot_position:
        pos_str = (
            f"x={robot_position['x']:.2f}, "
            f"y={robot_position['y']:.2f}, "
            f"z={robot_position['z']:.2f}"
        )
        if language == "zh":
            context_parts.append(f"## 当前位置\n{pos_str}")
        else:
            context_parts.append(f"## Current Position\n{pos_str}")

    if summary:
        if language == "zh":
            context_parts.append(f"## 场景概要\n{summary}")
        else:
            context_parts.append(f"## Scene Summary\n{summary}")

    # Layer 1: Rooms
    if rooms:
        rooms_compact = json.dumps(rooms, ensure_ascii=False, indent=None)
        if language == "zh":
            context_parts.append(f"## 层次1: Rooms (房间)\n```json\n{rooms_compact}\n```")
        else:
            context_parts.append(f"## Layer 1: Rooms\n```json\n{rooms_compact}\n```")

    # Layer 2: Groups
    if groups:
        groups_compact = json.dumps(groups, ensure_ascii=False, indent=None)
        if language == "zh":
            context_parts.append(f"## 层次2: Groups (物体组)\n```json\n{groups_compact}\n```")
        else:
            context_parts.append(f"## Layer 2: Groups\n```json\n{groups_compact}\n```")

    # Layer 3: Objects (limit to avoid token explosion)
    if objects:
        objects_compact = json.dumps(objects[:30], ensure_ascii=False, indent=None)
        if language == "zh":
            context_parts.append(f"## 层次3: Objects (物体, 前{min(30, len(objects))}个)\n```json\n{objects_compact}\n```")
        else:
            context_parts.append(f"## Layer 3: Objects (top {min(30, len(objects))})\n```json\n{objects_compact}\n```")

    # Relations
    if relations:
        relations_compact = json.dumps(relations[:20], ensure_ascii=False, indent=None)
        if language == "zh":
            context_parts.append(f"## 空间关系\n```json\n{relations_compact}\n```")
        else:
            context_parts.append(f"## Spatial Relations\n```json\n{relations_compact}\n```")

    # Topology edges (创新4: 房间拓扑连通图)
    topology_edges = sg.get("topology_edges", [])
    if topology_edges:
        topo_compact = json.dumps(topology_edges[:15], ensure_ascii=False, indent=None)
        if language == "zh":
            context_parts.append(f"## 拓扑连通 (房间之间的可达关系)\n```json\n{topo_compact}\n```")
        else:
            context_parts.append(f"## Topology (room connectivity)\n```json\n{topo_compact}\n```")

    if language == "zh":
        context_parts.append(f"## 指令\n{instruction}")
        context_parts.append("请按 Room → Group → Object 的顺序逐层推理，利用拓扑连通关系辅助判断，输出 JSON。")
    else:
        context_parts.append(f"## Instruction\n{instruction}")
        context_parts.append("Reason layer by layer: Room → Group → Object. Use topology for reachability. Output JSON.")

    user_content = "\n\n".join(context_parts)

    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user_content},
    ]


def build_exploration_prompt(
    instruction: str,
    explored_directions: list[dict[str, float]],
    robot_position: dict[str, float],
    language: str = "zh",
    topology_context: str | None = None,
    semantic_priors: list[dict] | None = None,
) -> list[dict[str, str]]:
    """
    构建拓扑感知探索建议的 LLM 消息列表 (创新4 增强)。

    当目标物体不在场景图中时, 让 LLM 基于:
    1. 常识推理
    2. 房间拓扑连通图
    3. 语义先验 (哪种房间最可能包含目标)
    4. 已探索记忆 (负面记忆: 哪些方向/房间已经去过没找到)
    建议下一个探索方向。

    Args:
        instruction: 用户自然语言指令
        explored_directions: 已探索过的方向列表
        robot_position: 当前机器人位置
        language: "zh" / "en"
        topology_context: 拓扑图摘要字符串
        semantic_priors: 语义先验推荐列表

    Returns:
        OpenAI 格式消息列表
    """
    if language == "zh":
        system = """你是一个机器人探索顾问。用户要找一个不在已知场景中的物体。

你拥有以下信息来做出最佳探索决策:
1. **空间拓扑图**: 已知房间之间的连通关系 (通过门/过道), 以及房间的访问状态
2. **前沿节点**: 已知空间边界处的未探索方向, 可能通向新的房间
3. **语义先验**: 目标物体最可能出现在哪种房间类型
4. **探索记忆**: 已经探索过的方向和房间 (负面记忆, 应该避开)
5. **常识推理**: 物体通常在什么环境中

策略: 优先去**语义先验最高** + **尚未探索** + **拓扑可达** + **信息增益最大**的方向。

输出格式 (严格 JSON):
{
  "explore_direction": {
    "x": float,
    "y": float
  },
  "target_room_type": "推测的目标房间类型 (如 corridor, stairwell, kitchen)",
  "reasoning": "基于拓扑+语义先验的探索理由"
}"""
        user_parts = [
            f"## 要找的目标\n{instruction}",
            f"## 当前位置\nx={robot_position['x']:.2f}, y={robot_position['y']:.2f}",
        ]

        if semantic_priors:
            priors_str = json.dumps(semantic_priors[:5], ensure_ascii=False)
            user_parts.append(f"## 语义先验 (目标最可能在哪种房间)\n{priors_str}")

        if topology_context:
            user_parts.append(f"## 空间拓扑\n{topology_context}")

        explored_str = json.dumps(explored_directions, ensure_ascii=False)
        user_parts.append(f"## 已探索方向 (负面记忆)\n{explored_str}")
        user_parts.append("请结合拓扑和语义先验，建议下一个最优探索方向。")
        user_content = "\n\n".join(user_parts)
    else:
        system = """You are a robot exploration advisor. The user is looking for an object not in the known scene.

You have the following information for optimal exploration decisions:
1. **Spatial topology**: Room connectivity (doors, passages) with visit status
2. **Frontier nodes**: Unexplored directions at the boundary of known space
3. **Semantic priors**: Which room types most likely contain the target
4. **Exploration memory**: Previously explored directions and rooms (negative memory)
5. **Common sense**: Where objects are typically found

Strategy: Prefer directions with **highest semantic prior** + **unexplored** + **topologically reachable** + **max information gain**.

Output format (strict JSON):
{
  "explore_direction": {
    "x": float,
    "y": float
  },
  "target_room_type": "predicted room type (e.g. corridor, stairwell, kitchen)",
  "reasoning": "topology + semantic prior based reasoning"
}"""
        user_parts = [
            f"## Target\n{instruction}",
            f"## Current Position\nx={robot_position['x']:.2f}, y={robot_position['y']:.2f}",
        ]

        if semantic_priors:
            priors_str = json.dumps(semantic_priors[:5])
            user_parts.append(f"## Semantic Priors (predicted room types)\n{priors_str}")

        if topology_context:
            user_parts.append(f"## Spatial Topology\n{topology_context}")

        explored_str = json.dumps(explored_directions)
        user_parts.append(f"## Explored Directions (negative memory)\n{explored_str}")
        user_parts.append("Based on topology and semantic priors, suggest the optimal exploration direction.")
        user_content = "\n\n".join(user_parts)

    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user_content},
    ]


def build_vision_grounding_prompt(
    instruction: str,
    scene_graph_json: str,
    language: str = "zh",
) -> tuple:
    """
    构建 VLM Vision Grounding prompt (参考 VLMnav 2024)。

    当场景图匹配不确定时, 直接发一帧图给 GPT-4o Vision,
    让它判断目标是否在图中, 以及大致位置。

    Args:
        instruction: 用户指令
        scene_graph_json: 当前场景图 JSON
        language: "zh" 或 "en"

    Returns:
        (system_prompt, user_prompt) 元组
    """
    if language == "zh":
        system = """你是一个机器人视觉助手。用户正在寻找一个目标物体。
你会看到机器人当前的相机画面和已知的场景图。

请判断:
1. 目标物体是否在画面中可见？
2. 如果可见, 它大概在画面的什么位置？(left/center/right, near/mid/far)
3. 如果不可见, 画面中有什么线索可以推断目标可能在哪？

输出格式 (严格 JSON):
{
  "target_visible": true/false,
  "position_in_frame": "center-near" / "left-far" / null,
  "confidence": 0.0-1.0,
  "reasoning": "解释"
}"""

        user_text = (
            f"## 要找的目标\n{instruction}\n\n"
            f"## 已知场景\n{scene_graph_json}\n\n"
            f"请分析这张相机图片, 判断目标是否可见。"
        )
    else:
        system = """You are a robot vision assistant. The user is looking for a target object.
You will see the robot's current camera view and the known scene graph.

Determine:
1. Is the target object visible in the image?
2. If visible, where approximately? (left/center/right, near/mid/far)
3. If not visible, what clues suggest where it might be?

Output format (strict JSON):
{
  "target_visible": true/false,
  "position_in_frame": "center-near" / "left-far" / null,
  "confidence": 0.0-1.0,
  "reasoning": "explanation"
}"""

        user_text = (
            f"## Target\n{instruction}\n\n"
            f"## Known Scene\n{scene_graph_json}\n\n"
            f"Analyze this camera image and determine if the target is visible."
        )

    return system, user_text


def build_room_naming_prompt(
    object_labels: list[str],
    language: str = "zh",
) -> list[dict[str, str]]:
    """
    构建 Room LLM 命名 prompt (创新1 补强)。

    给定一个 Region 中包含的物体标签列表, 让 LLM 推断该区域
    的功能性名称 (如 '走廊', '办公室', 'kitchen' 等)。

    Args:
        object_labels: 区域内物体标签列表
        language: "zh" / "en"

    Returns:
        OpenAI 格式消息列表
    """
    labels_str = ", ".join(object_labels[:12])

    if language == "zh":
        system = (
            "你是一个室内场景理解助手。根据房间内包含的物体，"
            "推断这个区域最可能是什么类型的空间。"
            "只输出一个简短的名称 (2-4个字)，如: 走廊、办公室、厨房、卫生间、会议室、大厅、储物间、楼梯间。"
            "不要输出任何额外文字。"
        )
        user = f"房间内包含以下物体: {labels_str}\n这个区域是什么?"
    else:
        system = (
            "You are an indoor scene understanding assistant. "
            "Given objects found in a room, infer the most likely room type. "
            "Output ONLY a short name (1-3 words), e.g.: corridor, office, kitchen, bathroom, "
            "meeting room, lobby, storage room, stairwell. No extra text."
        )
        user = f"Objects in this area: {labels_str}\nWhat is this room?"

    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user},
    ]


def build_sgnav_subgraph_prompt(
    instruction: str,
    subgraphs: list[dict],
    language: str = "zh",
    explored_summaries: list[str] | None = None,
    frontier_descriptions: list[str] | None = None,
) -> list[dict[str, str]]:
    """
    SG-Nav 风格子图评分 prompt — H-CoT 4 步链式思考。

    参考 SG-Nav (NeurIPS 2024) 的分层思维链:
      Step 1: 预测距离 — 估算机器人到每个子图的距离/可达性
      Step 2: 生成澄清问题 — 针对指令歧义, 提出需要确认的问题
      Step 3: 基于子图回答 — 利用子图内容回答 Step 2 的问题
      Step 4: 综合打分 — 结合以上三步, 输出每个子图的最终相关概率

    输入多个 room/group 子图摘要, 让 LLM 按 4 步推理后输出评分。
    """
    if language == "zh":
        system = """你是一个机器人导航推理器。请根据指令与子图摘要，通过 4 步层次链式思考 (H-CoT) 对每个子图评分。

## 推理步骤 (必须严格按顺序)

### Step 1: 预测距离
对每个子图, 根据其 center 坐标与物体数量, 估算机器人到达该子图的相对难度 (近/中/远)。考虑子图之间的空间分布。

### Step 2: 生成澄清问题
分析指令, 提出 1-2 个有助于区分子图的关键问题。例如:
- "目标更可能在室内还是走廊?"
- "指令中的'旁边'是指哪个参照物?"

### Step 3: 基于子图回答
对每个子图, 利用其 object_labels 和 relation_count 回答 Step 2 的问题。

### Step 4: 综合打分
结合 Step 1 (距离) + Step 2-3 (语义匹配) 的结论, 为每个子图给出最终 score。

## 输出格式 (严格 JSON)
{
  "step1_distances": [
    {"subgraph_id": "room_0", "proximity": "near/mid/far"}
  ],
  "step2_questions": ["问题1", "问题2"],
  "step3_answers": [
    {"subgraph_id": "room_0", "answer": "..."}
  ],
  "subgraph_scores": [
    {"subgraph_id": "room_0", "score": 0.82, "reason": "综合 Step 1-3 的理由"}
  ],
  "global_reasoning": "简短总结"
}

## 规则
- score 范围 [0,1]
- 仅输出 JSON, 不要额外文字"""

        user = (
            f"## 指令\n{instruction}\n\n"
            f"## 子图摘要\n```json\n{json.dumps(subgraphs, ensure_ascii=False)}\n```\n\n"
            f"请按 4 步 H-CoT 推理后输出每个子图的相关概率。"
        )
    else:
        system = """You are a robot navigation reasoner. Given an instruction and hierarchical subgraph summaries,
evaluate each subgraph using 4-step Hierarchical Chain-of-Thought (H-CoT).

## Reasoning Steps (follow strictly in order)

### Step 1: Predict Distance
For each subgraph, estimate relative traversal difficulty (near/mid/far) based on center coordinates and object count. Consider spatial distribution across subgraphs.

### Step 2: Generate Clarifying Questions
Analyze the instruction and formulate 1-2 key questions that help distinguish subgraphs. For example:
- "Is the target more likely indoors or in a corridor?"
- "What reference object does 'next to' refer to?"

### Step 3: Answer from Subgraph
For each subgraph, use its object_labels and relation_count to answer the Step 2 questions.

### Step 4: Synthesize Scores
Combine Step 1 (distance) + Step 2-3 (semantic match) conclusions to produce a final score for each subgraph.

## Output Format (strict JSON)
{
  "step1_distances": [
    {"subgraph_id": "room_0", "proximity": "near/mid/far"}
  ],
  "step2_questions": ["question1", "question2"],
  "step3_answers": [
    {"subgraph_id": "room_0", "answer": "..."}
  ],
  "subgraph_scores": [
    {"subgraph_id": "room_0", "score": 0.82, "reason": "reasoning from Steps 1-3"}
  ],
  "global_reasoning": "brief summary"
}

## Rules
- score in [0,1]
- Output JSON only, no extra text"""

        user = (
            f"## Instruction\n{instruction}\n\n"
            f"## Subgraph Summaries\n```json\n{json.dumps(subgraphs)}\n```\n\n"
            "Reason through the 4-step H-CoT, then return relevance probability for each subgraph."
        )

    # 追加已探索区域摘要和可探索方向描述 (VLingMem + L3MVN/OmniNav)
    if explored_summaries:
        summary_text = "\n".join(explored_summaries[:5])  # 最多5条
        user += f"\n\n【已探索区域记录】\n{summary_text}"

    if frontier_descriptions:
        desc_text = "\n".join([f"- {d}" for d in frontier_descriptions[:4]])
        user += f"\n\n【当前可探索方向】\n{desc_text}"

    return [
        {"role": "system", "content": system},
        {"role": "user", "content": user},
    ]


# ================================================================
#  意图解析 (Intent Classification) — Fast-Slow 双通道
# ================================================================

def build_intent_classification_prompt(
    instruction: str,
    robot_state: str = "IDLE",
    available_routes: str = "",
    available_pois: str = "",
) -> str:
    """
    构建意图分类的 LLM prompt — 当规则解析 (Fast Path) 失败时的 Slow Path 回退。

    当 task_decomposer.decompose_with_rules() 返回 None (无法确定意图) 时,
    将用户指令发给 LLM 做结构化意图分类, 返回 JSON 格式的意图 + 槽位。

    Args:
        instruction: 用户自然语言指令
        robot_state: 当前机器人状态 (IDLE / NAVIGATING / PATROLLING 等)
        available_routes: 已保存的巡逻路线列表 (逗号分隔)
        available_pois: 已保存的兴趣点列表 (逗号分隔)

    Returns:
        构建好的 prompt 字符串
    """
    return f"""你是一个机器人语音助手的意图解析器。用户对机器人下达了指令，请分析用户意图。

## 可用意图类型

| 意图 | 说明 | 需要的参数 |
|------|------|-----------|
| NAVIGATE | 导航到某个地点或物体 | target: 目标描述 |
| PATROL | 巡逻/巡检 | route: 路线名称 (可选) |
| SAVE_MAP | 保存当前地图 | 无 |
| SAVE_POI | 保存/标记当前位置为兴趣点 | name: 位置名称 |
| SET_SPEED | 调整速度 | value: 速度值 (m/s, 0.1~3.0) |
| RETURN_HOME | 返回出发点/充电桩 | 无 |
| STOP | 停止/取消当前任务 | 无 |
| PAUSE | 暂停任务 | 无 |
| RESUME | 恢复暂停的任务 | 无 |
| EXPLORE | 自由探索环境 | 无 |
| FIND | 寻找特定目标 | target: 目标描述 |
| FOLLOW | 跟随目标 | target: 跟随对象描述 |
| QUERY | 查询状态/信息 | query: 查询内容 |

## 当前机器人状态
{robot_state}

## 已保存的巡逻路线
{available_routes or '无'}

## 已保存的兴趣点
{available_pois or '无'}

## 用户指令
「{instruction}」

## 输出要求

返回 JSON，格式：
```json
{{
  "intents": [
    {{
      "action": "意图类型",
      "target": "目标 (如适用)",
      "params": {{}},
      "confidence": 0.95
    }}
  ],
  "response": "对用户的回复 (简短中文)",
  "is_compound": false
}}
```

如果是复合指令 (如 "巡逻完了回充电桩")，intents 数组包含多个按执行顺序排列的意图。

规则：
1. 只返回 JSON，不加其他文字
2. confidence 范围 0-1
3. 如果指令不明确，confidence 设低并在 response 中要求用户澄清
4. "去那边看看" 在没有明确目标时应该是 EXPLORE，不是 NAVIGATE
5. 涉及安全的操作 (STOP) 优先级最高
"""


def build_compound_decomposition_prompt(
    instruction: str,
    robot_state: str = "IDLE",
) -> str:
    """
    构建复合指令分解的 LLM prompt — 将复合语句拆解为按顺序执行的子任务。

    当 task_decomposer 检测到复杂度标记 (然后/接着/先..再..) 但无法通过
    规则单独完成分解时, 调用此 prompt 让 LLM 做结构化拆解。

    Args:
        instruction: 用户自然语言指令 (含复合动作)
        robot_state: 当前机器人状态 (IDLE / NAVIGATING / PATROLLING 等)

    Returns:
        构建好的 prompt 字符串
    """
    return f"""你是机器人任务分解器。将复合指令拆解为按顺序执行的子任务。

## 用户指令
「{instruction}」

## 当前状态
{robot_state}

## 可用动作
NAVIGATE(target), PATROL(route), SAVE_MAP, SAVE_POI(name), SET_SPEED(value),
RETURN_HOME, STOP, PAUSE, RESUME, EXPLORE, FIND(target), FOLLOW(target)

## 输出 JSON
```json
{{
  "steps": [
    {{"action": "动作", "target": "目标", "params": {{}}}},
    ...
  ],
  "response": "确认回复"
}}
```

规则：
1. 只返回 JSON
2. 步骤按执行顺序
3. "巡逻完了回充电桩" → [{{"action":"PATROL"}}, {{"action":"RETURN_HOME"}}]
4. "先保存地图再去大门" → [{{"action":"SAVE_MAP"}}, {{"action":"NAVIGATE","target":"大门"}}]
"""
