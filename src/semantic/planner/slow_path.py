"""
slow_path.py — Slow Path (System 2) LLM 推理 Mixin。

从 goal_resolver.py 提取，GoalResolver 通过多继承使用:
    class GoalResolver(FastPathMixin, SlowPathMixin): ...

包含:
  - resolve(): 完整解析入口 (AdaCoT 动态路由 + Fast/Slow 双进程)
  - generate_exploration_waypoint(): 拓扑感知探索航点生成
  - _try_tsg_exploration(): TSG 信息增益探索
  - _predict_adjacent_room_type(): 相邻房间类型预测
  - vision_grounding(): GPT-4o Vision 视觉确认
  - _selective_grounding(): ESCA 选择性 Grounding
  - reset_exploration(): 重置探索状态
  - set_room_object_kg() / update_visited_room() / topology_graph: 状态管理
  - _get_current_room_id(): 机器人当前房间
  - _call_with_fallback(): LLM 主/备切换
  - _parse_llm_response(): LLM JSON 响应解析
  - _extract_json() / _fix_truncated_json(): JSON 提取工具
"""
from __future__ import annotations

import json
import logging
from typing import TYPE_CHECKING, Any

from core.runtime_interface import map_frame_id

if TYPE_CHECKING:
    from .goal_resolver import GoalResult

from core.msgs.numpy_compat import np

logger = logging.getLogger(__name__)
SLOW_PATH_MAP_FRAME_ID = map_frame_id()


class SlowPathMixin:
    """
    Slow Path (System 2) 方法集合。

    依赖 self 上存在:
      - self._primary: LLMClientBase
      - self._fallback: LLMClientBase | None
      - self._adacot: AdaCoTRouter
      - self._explored_directions: list[Dict]
      - self._explore_step_count: int
      - self._visited_room_ids: set
      - self._tsg: TopologySemGraph | None
      - self._semantic_prior_engine: SemanticPriorEngine
      - self._room_object_kg: Any | None
      - self.fast_resolve() (来自 FastPathMixin)
      - self._extract_keywords() (来自 FastPathMixin)
      - self._selective_grounding() (本 Mixin)
    """

    # ================================================================
    #  Slow Path — System 2 (LLM 推理)
    # ================================================================

    async def resolve(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: dict[str, float] | None = None,
        language: str = "zh",
        explore_if_unknown: bool = True,
        clip_encoder: Any | None = None,
    ) -> GoalResult:
        """
        完整解析 (AdaCoT 动态路由 + Fast/Slow 双进程)。

        流程:
          0. AdaCoT 预判: 指令复杂度 + 场景图状态 → FAST / SLOW / AUTO
          1. FAST → 仅 Fast Path, 成功直返; 失败 fallback Slow
          2. SLOW → 跳过 Fast, 直走 Slow Path
          3. AUTO → 传统 Fast → Slow 流程

        Args:
            instruction: 用户自然语言指令
            scene_graph_json: 场景图 JSON
            robot_position: 当前机器人位置
            language: "zh" / "en"
            explore_if_unknown: 目标未知时是否自动探索
            clip_encoder: CLIP 编码器 (可选)

        Returns:
            GoalResult
        """
        from core.utils.sanitize import safe_json_loads

        from .adacot import AdaCoTDecision
        from .goal_resolver import GoalResult
        from .prompt_templates import build_goal_resolution_prompt

        # ── 层 0: Tag 记忆 (精确匹配，最快，无需场景图) ──
        tag_result = self._resolve_by_tag(instruction)
        if tag_result is not None:
            return tag_result

        # ── Step 0: AdaCoT 路径预判 (VLingNav 2026) ──
        sg_dict = safe_json_loads(scene_graph_json, default=None) if scene_graph_json else None

        if not sg_dict or (not sg_dict.get("objects") and not sg_dict.get("rooms")):
            logger.info(
                "Scene graph empty (objects=%d, rooms=%d), fast path will skip to exploration",
                len(sg_dict.get("objects", [])) if sg_dict else 0,
                len(sg_dict.get("rooms", [])) if sg_dict else 0,
            )

        adacot_decision = self._adacot.decide(
            instruction, scene_graph=sg_dict,
        )

        # ── FAST 路径: 仅尝试 Fast Path ──
        if adacot_decision != AdaCoTDecision.SLOW:
            fast_result = self.fast_resolve(
                instruction, scene_graph_json, robot_position, clip_encoder
            )
            if fast_result is not None:
                # AdaNav: 高熵 + 低置信度 → 强制 Slow Path 验证
                if (fast_result.score_entropy > 1.5
                        and fast_result.confidence < 0.85):
                    logger.info(
                        "[AdaNav] high entropy=%.2f, confidence=%.2f → "
                        "deferring to Slow Path for verification",
                        fast_result.score_entropy, fast_result.confidence,
                    )
                else:
                    return fast_result

            # AdaCoT 推荐 FAST 但匹配失败 → 仍需 fallback 到 Slow
            if adacot_decision == AdaCoTDecision.FAST:
                logger.info(
                    "AdaCoT recommended FAST but no match, falling through to Slow"
                )

        # ── SLOW 路径: 选择性 Grounding + LLM 推理 ──
        # (AdaCoT.SLOW 直接到这里; AdaCoT.AUTO / FAST-miss 也到这里)
        filtered_sg = self._selective_grounding(
            instruction, scene_graph_json, clip_encoder=clip_encoder
        )

        messages = build_goal_resolution_prompt(
            instruction, filtered_sg, robot_position, language
        )

        response_text = await self._call_with_fallback(messages)
        if response_text is None:
            return GoalResult(
                action="error",
                error="All LLM backends failed",
                is_valid=False,
            )

        result = self._parse_llm_response(
            response_text,
            safe_json_loads(filtered_sg, default={}) if isinstance(filtered_sg, str) else filtered_sg,
        )
        result.path = "slow"

        # 如果需要探索
        if (
            explore_if_unknown
            and result.action == "explore"
            and robot_position is not None
        ):
            logger.info(
                "Target unknown, generating exploration waypoint. "
                "Reason: %s", result.reasoning,
            )
            self._explored_directions.append(
                {"x": result.target_x, "y": result.target_y}
            )

        # OmniNav: 从 regions 查找目标所在房间, 生成层次子目标提示
        try:
            if sg_dict:
                regions = sg_dict.get('regions', [])
                target_id = result.candidate_id
                for region in regions:
                    if target_id in region.get('object_ids', []):
                        result.hint_room = region.get('name', '')
                        center = region.get('center', None)
                        if center:
                            result.hint_room_center = list(center)
                        break
        except (TypeError, KeyError, ValueError) as e:
            logger.debug("OmniNav region lookup failed (non-critical): %s", e)

        return result

    async def generate_exploration_waypoint(
        self,
        instruction: str,
        robot_position: dict[str, float],
        step_distance: float = 2.0,
        language: str = "zh",
        scene_graph_json: str = "",
    ) -> GoalResult:
        """
        生成拓扑感知探索航点 (创新5 升级: TSG 信息增益探索)。

        双层策略:
          Layer 1 (TSG): 如果拓扑语义图可用, 用 Algorithm 2 (Information Gain)
                         选择最优探索目标, 无需 LLM 调用 (~1ms)
          Layer 2 (LLM): 如果 TSG 不可用/评分过低, fallback 到 LLM 探索建议

        参考:
          - TopoNav (2025): 拓扑图作为空间记忆
          - L3MVN (IROS 2023): LLM-guided frontier scoring
          - SG-Nav: 子图评分插值到 frontier
        """
        from core.utils.sanitize import safe_json_loads

        from .goal_resolver import GoalResult
        from .prompt_templates import build_exploration_prompt

        # ── Layer 1: TSG 信息增益探索 (创新5) ──
        tsg_result = self._try_tsg_exploration(
            instruction, robot_position, scene_graph_json, step_distance,
        )
        if tsg_result is not None:
            return tsg_result

        # ── Layer 2: LLM 探索建议 (创新4 原方案) ──
        topology_context = None
        semantic_priors = None

        if scene_graph_json:
            sg = safe_json_loads(scene_graph_json, default={})
            if sg:
                try:
                    rooms = sg.get("rooms", [])
                    topology_edges = sg.get("topology_edges", [])

                    if rooms:
                        priors = self._semantic_prior_engine.get_unexplored_priors(
                            target_instruction=instruction,
                            rooms=rooms,
                            topology_edges=topology_edges,
                            visited_room_ids=self._visited_room_ids,
                            current_room_id=self._get_current_room_id(
                                robot_position, rooms
                            ),
                        )
                        if priors:
                            semantic_priors = priors[:5]

                    if topology_edges:
                        topo_parts = []
                        room_names = {
                            r.get("room_id", -1): r.get("name", "?")
                            for r in rooms
                        }
                        for te in topology_edges[:8]:
                            fr = te.get("from_room", -1)
                            to = te.get("to_room", -1)
                            fn = room_names.get(fr, f"room_{fr}")
                            tn = room_names.get(to, f"room_{to}")
                            visited_f = "v" if fr in self._visited_room_ids else "?"
                            visited_t = "v" if to in self._visited_room_ids else "?"
                            topo_parts.append(
                                f"{fn}[{visited_f}] <-> {tn}[{visited_t}] ({te.get('type', '?')})"
                            )
                        topology_context = "\n".join(topo_parts)

                    # 补充 TSG 拓扑摘要到 LLM 上下文
                    if self._tsg:
                        tsg_context = self._tsg.to_prompt_context(language)
                        if tsg_context:
                            topology_context = (
                                (topology_context or "") + "\n" + tsg_context
                            ).strip()
                except (TypeError, KeyError):
                    pass

        messages = build_exploration_prompt(
            instruction, self._explored_directions, robot_position, language,
            topology_context=topology_context,
            semantic_priors=semantic_priors,
        )

        response_text = await self._call_with_fallback(messages)
        if response_text is None:
            angle = np.random.uniform(0, 2 * np.pi)
            return GoalResult(
                action="explore",
                target_x=robot_position["x"] + step_distance * np.cos(angle),
                target_y=robot_position["y"] + step_distance * np.sin(angle),
                target_z=robot_position.get("z", 0.0),
                reasoning="Random exploration (LLM unavailable)",
                confidence=0.1,
                is_valid=True,
            )

        try:
            data = self._extract_json(response_text)
            direction = data.get("explore_direction", {})
            dx = float(direction.get("x", 0)) - robot_position["x"]
            dy = float(direction.get("y", 0)) - robot_position["y"]
            norm = np.sqrt(dx**2 + dy**2)
            if norm > 0:
                dx = dx / norm * step_distance
                dy = dy / norm * step_distance
            else:
                # LLM returned robot's own position — pick random direction
                import random
                angle = random.uniform(0, 2 * np.pi)
                dx = np.cos(angle) * step_distance
                dy = np.sin(angle) * step_distance
                logger.warning(
                    "LLM exploration returned robot's own position, using random direction"
                )

            self._explore_step_count += 1
            self._explored_directions.append({
                "x": robot_position["x"] + dx,
                "y": robot_position["y"] + dy,
            })

            return GoalResult(
                action="explore",
                target_x=robot_position["x"] + dx,
                target_y=robot_position["y"] + dy,
                target_z=robot_position.get("z", 0.0),
                reasoning=data.get("reasoning", "LLM exploration suggestion"),
                confidence=0.3,
                is_valid=True,
            )
        except Exception as e:
            logger.warning("Failed to parse exploration response: %s", e)
            return GoalResult(
                action="explore",
                error=str(e),
                is_valid=False,
            )

    def _try_tsg_exploration(
        self,
        instruction: str,
        robot_position: dict[str, float],
        scene_graph_json: str,
        step_distance: float,
    ) -> GoalResult | None:
        """
        尝试使用拓扑语义图 (TSG) 选择探索目标。

        如果 TSG 可用且返回高置信度探索目标, 直接返回 GoalResult;
        否则返回 None 让调用者 fallback 到 LLM。
        """
        from core.utils.sanitize import safe_json_loads

        from .goal_resolver import GoalResult

        if self._tsg is None or not scene_graph_json:
            return None

        sg = safe_json_loads(scene_graph_json, default=None)
        if sg is None:
            return None

        # 同步 TSG
        self._tsg.update_from_scene_graph(sg)

        # 注入前沿节点 (from scene graph)
        frontier_nodes = sg.get("frontier_nodes", [])
        if frontier_nodes:
            frontier_points = []
            frontier_sizes = []
            for fn in frontier_nodes:
                pos = fn.get("position", {})
                frontier_points.append(np.array([pos.get("x", 0), pos.get("y", 0)]))
                frontier_sizes.append(fn.get("frontier_size", 2.0))
            self._tsg.update_frontiers_from_costmap(frontier_points, frontier_sizes)

            # 预测前沿房间类型 (利用语义先验)
            for fnode in self._tsg.frontiers:
                nearest_room_id = None
                for edge in self._tsg._adjacency.get(fnode.node_id, []):
                    other_id = (
                        edge.to_id if edge.from_id == fnode.node_id else edge.from_id
                    )
                    other = self._tsg.get_node(other_id)
                    if other and other.node_type == "room":
                        nearest_room_id = other.node_id
                        break
                if nearest_room_id is not None:
                    room_node = self._tsg.get_node(nearest_room_id)
                    if room_node:
                        fnode.predicted_room_type = self._predict_adjacent_room_type(
                            room_node.room_type
                        )

        # 记录机器人位置 (触发房间切换检测)
        room_id = self._tsg.record_robot_position(
            robot_position["x"], robot_position["y"]
        )
        if room_id is not None:
            self._visited_room_ids.add(room_id)

        # 运行 Algorithm 2: Information Gain Exploration
        targets = self._tsg.get_best_exploration_target(
            instruction, self._semantic_prior_engine, top_k=3,
        )

        if not targets:
            return None

        best = targets[0]

        # 只有评分足够高才使用 TSG 结果 (避免低质量探索)
        if best.score < 0.05:
            return None

        dx = best.position[0] - robot_position["x"]
        dy = best.position[1] - robot_position["y"]
        norm = np.sqrt(dx**2 + dy**2)
        if norm > step_distance:
            dx = dx / norm * step_distance
            dy = dy / norm * step_distance

        self._explore_step_count += 1
        self._explored_directions.append({
            "x": robot_position["x"] + dx,
            "y": robot_position["y"] + dy,
        })

        logger.info(
            "TSG exploration: %s (score=%.3f, IG=%.3f, hops=%d) — %s",
            best.node_name, best.score, best.information_gain,
            best.hops, best.reasoning,
        )

        # 从场景图获取坐标系
        frame_id = sg.get("frame_id", SLOW_PATH_MAP_FRAME_ID)

        return GoalResult(
            action="explore",
            target_x=robot_position["x"] + dx,
            target_y=robot_position["y"] + dy,
            target_z=robot_position.get("z", 0.0),
            target_label=best.node_name,
            reasoning=f"[TSG-IG] {best.reasoning}",
            confidence=min(0.5, best.score),
            is_valid=True,
            path="fast",
            frame_id=frame_id,
        )

    def _predict_adjacent_room_type(self, current_room_type: str) -> str:
        """
        基于当前房间类型预测相邻房间类型。

        P1 升级: 优先使用 RoomObjectKG 中学习到的邻接关系,
        回退到 hand-coded 空间常识。

        返回最高概率的相邻类型。走廊是 hub 节点, 连接多种房间;
        功能房间通常与走廊相邻, 也可能与相近功能的房间相邻。
        """
        # P1: 尝试从 KG 邻接数据预测
        kg = getattr(self, '_room_object_kg', None)
        if kg is not None:
            adj_graph = kg.get_adjacency_graph()
            if adj_graph:
                # 找当前房间类型的所有邻接, 选 count 最高的
                best_neighbor = None
                best_count = 0
                for edge in adj_graph:
                    ft, tt = edge["from"], edge["to"]
                    count = edge["count"]
                    if ft == current_room_type and count > best_count:
                        best_neighbor = tt
                        best_count = count
                    elif tt == current_room_type and count > best_count:
                        best_neighbor = ft
                        best_count = count
                if best_neighbor is not None:
                    return best_neighbor

        # Fallback: hand-coded 空间常识
        adjacency_priors = {
            "corridor": "office",       # 走廊两侧最常见是办公室
            "office": "corridor",       # 办公室出门是走廊
            "kitchen": "corridor",
            "meeting_room": "corridor",
            "bathroom": "corridor",
            "stairwell": "corridor",
            "lobby": "corridor",        # 大厅连走廊
            "storage": "corridor",
            "lab": "corridor",
            "classroom": "corridor",
        }
        return adjacency_priors.get(current_room_type, "corridor")

    async def vision_grounding(
        self,
        instruction: str,
        scene_graph_json: str,
        image_base64: str,
        language: str = "zh",
    ) -> dict:
        """
        视觉 grounding — 发送相机帧给 GPT-4o Vision。

        参考 VLMnav (2024): 当场景图匹配置信度不够时,
        直接让 VLM 看图判断目标是否可见。

        使用场景:
          - resolve() 返回低置信度时
          - 探索阶段到达新视角时
          - 用户指令包含视觉属性 ("红色的", "坏掉的") 时

        Args:
            instruction: 用户指令
            scene_graph_json: 场景图 JSON
            image_base64: JPEG base64 编码
            language: "zh" / "en"

        Returns:
            dict: {target_visible, position_in_frame, confidence, reasoning}
        """
        from .llm_client import OpenAIClient
        from .prompt_templates import build_vision_grounding_prompt

        # Vision 只能用 OpenAI (GPT-4o) 后端
        client = None
        if isinstance(self._primary, OpenAIClient):
            client = self._primary
        elif self._fallback and isinstance(self._fallback, OpenAIClient):
            client = self._fallback

        if client is None or not hasattr(client, "chat_with_image"):
            logger.warning("No OpenAI client available for vision grounding")
            return {"target_visible": False, "confidence": 0.0, "reasoning": "No vision backend"}

        system, user_text = build_vision_grounding_prompt(
            instruction, scene_graph_json, language
        )

        try:
            response = await client.chat_with_image(
                text_prompt=user_text,
                image_base64=image_base64,
                system_prompt=system,
            )
            return self._extract_json(response)
        except Exception as e:
            logger.error("Vision grounding failed: %s", e)
            return {"target_visible": False, "confidence": 0.0, "reasoning": str(e)}

    # ================================================================
    #  选择性 Grounding (ESCA, NeurIPS 2025)
    # ================================================================

    def _selective_grounding(
        self,
        instruction: str,
        scene_graph_json: str,
        max_objects: int = 15,
        max_relations: int = 20,
        clip_encoder: Any | None = None,
    ) -> str:
        """
        选择性 Grounding: 只给 LLM 与指令相关的场景子图。

        D1 升级: CLIP 语义排序替换纯关键词匹配。

        参考:
          - ESCA / SGCLIP (NeurIPS 2025):
            "selective grounding — identifying only contextually relevant
             objects and relationships"
          - MSGNav (2025): "key subgraph selection enables efficient reasoning"

        策略:
          1. CLIP 语义排序 (如可用): 计算所有物体标签与指令的语义相似度
          2. 关键词匹配 (兜底): 标签含指令关键词的物体 → 必选
          3. 关系链扩展: 必选物体的 1-hop 邻居 → 加入 (SG-Nav)
          4. 区域内物体: 指令提到的区域 → 加入
          5. 限制总数避免 token 爆炸

        Args:
            instruction: 用户指令
            scene_graph_json: 完整场景图 JSON
            max_objects: 最多保留物体数
            max_relations: 最多保留关系数
            clip_encoder: CLIP 编码器 (可选, D1 新增)

        Returns:
            过滤后的场景图 JSON
        """

        from core.utils.sanitize import safe_json_loads

        sg = safe_json_loads(scene_graph_json, default=None)
        if sg is None:
            return scene_graph_json

        objects = sg.get("objects", [])
        relations = sg.get("relations", [])
        regions = sg.get("regions", [])

        if len(objects) <= max_objects:
            return scene_graph_json

        keywords = self._extract_keywords(instruction)
        inst_lower = instruction.lower()

        # ── D1: CLIP 语义排序 (替代纯关键词) ──
        clip_relevance: dict[int, float] = {}
        if clip_encoder is not None:
            try:
                labels = [obj.get("label", "") for obj in objects]
                if labels:
                    # 批量计算指令与所有物体标签的语义相似度
                    text_features = []
                    for lbl in labels:
                        text_features.append(lbl)
                    sims = clip_encoder.text_text_similarity(instruction, text_features)
                    if sims is None or len(sims) == 0:
                        # text_text 不可用, 尝试 text_image
                        for idx, obj in enumerate(objects):
                            feat = obj.get("clip_feature")
                            if feat is not None:
                                f = np.array(feat)
                                if f.size > 0:
                                    s = clip_encoder.text_image_similarity(instruction, [f])
                                    if s:
                                        clip_relevance[obj.get("id", idx)] = s[0]
                    else:
                        for idx, (obj, sim) in enumerate(zip(objects, sims)):
                            clip_relevance[obj.get("id", idx)] = float(sim)
            except Exception as e:
                logger.debug("CLIP selective grounding failed (falling back to keywords): %s", e)

        # ── 第 1 轮: CLIP + 关键词联合筛选 ──
        relevant_ids = set()
        relevance_scores: dict[int, float] = {}

        for obj in objects:
            oid = obj.get("id")
            label = obj.get("label", "").lower()

            # CLIP 相似度
            clip_sim = clip_relevance.get(oid, 0.0)

            # 关键词匹配分
            kw_score = 0.0
            if label in inst_lower or inst_lower in label:
                kw_score = 1.0
            else:
                for kw in keywords:
                    if kw in label or label in kw:
                        kw_score = max(kw_score, 0.8)

            # 综合: 有 CLIP 时 CLIP 权重 0.6 + 关键词 0.4; 无 CLIP 时纯关键词
            if clip_relevance:
                combined = 0.6 * clip_sim + 0.4 * kw_score
            else:
                combined = kw_score

            relevance_scores[oid] = combined
            if combined > 0.3:  # 相关性阈值
                relevant_ids.add(oid)

        # ── 第 2 轮: 关系链 1-hop 扩展 (SG-Nav 层次推理) ──
        hop1_ids = set()
        for rel in relations:
            sid = rel.get("subject_id")
            oid = rel.get("object_id")
            if sid in relevant_ids:
                hop1_ids.add(oid)
            if oid in relevant_ids:
                hop1_ids.add(sid)
        relevant_ids |= hop1_ids

        # ── 第 3 轮: 区域内物体 (如果指令提到了区域) ──
        for region in regions:
            region_name = region.get("name", "").lower()
            if any(kw in region_name for kw in keywords):
                relevant_ids |= set(region.get("object_ids", []))

        # ── 第 4 轮: 如果仍然为空, 取 CLIP 排序 top + 最高分物体 ──
        if not relevant_ids:
            if relevance_scores:
                sorted_by_relevance = sorted(
                    objects,
                    key=lambda o: relevance_scores.get(o.get("id"), 0.0),
                    reverse=True,
                )
            else:
                sorted_by_relevance = sorted(
                    objects,
                    key=lambda o: (o.get("score", 0) * o.get("detection_count", 1)),
                    reverse=True,
                )
            relevant_ids = {o["id"] for o in sorted_by_relevance[:max_objects]}

        # ── 构建过滤后的场景图 ──
        filtered_objects = [
            o for o in objects if o["id"] in relevant_ids
        ][:max_objects]
        filtered_obj_ids = {o["id"] for o in filtered_objects}

        filtered_relations = [
            r for r in relations
            if r.get("subject_id") in filtered_obj_ids
            and r.get("object_id") in filtered_obj_ids
        ][:max_relations]

        filtered_regions = [
            r for r in regions
            if any(oid in filtered_obj_ids for oid in r.get("object_ids", []))
        ]

        # 重建摘要
        summary = sg.get("summary", "")
        if filtered_objects:
            labels = [o["label"] for o in filtered_objects[:10]]
            summary = (
                f"Filtered {len(filtered_objects)}/{len(objects)} relevant objects: "
                + ", ".join(labels)
            )

        result = {
            "timestamp": sg.get("timestamp", 0),
            "object_count": len(filtered_objects),
            "objects": filtered_objects,
            "relations": filtered_relations,
            "regions": filtered_regions,
            "summary": summary,
            "_filter_note": f"ESCA selective grounding: {len(objects)}->{len(filtered_objects)} objects",
        }

        logger.debug(
            "Selective grounding: %d->%d objects, %d->%d relations",
            len(objects), len(filtered_objects),
            len(relations), len(filtered_relations),
        )

        return json.dumps(result, ensure_ascii=False)

    def reset_exploration(self) -> None:
        """重置探索状态 (新任务时调用)。"""
        self._explored_directions.clear()
        self._explore_step_count = 0
        self._visited_room_ids.clear()
        if self._tsg is not None:
            try:
                from memory.spatial.topology_graph import TopologySemGraph as _TSG
                self._tsg = _TSG()
            except ImportError:
                self._tsg = None

    def set_room_object_kg(self, kg: Any | None) -> None:
        """注入房间-物体知识图谱 (P1: KG-backed room adjacency prediction)。"""
        self._room_object_kg = kg

    def set_topology_graph_snapshot(self, snapshot: dict | None) -> bool:
        """Replace the internal TSG with a serialized SemanticMapper snapshot."""
        if not isinstance(snapshot, dict):
            return False
        try:
            from memory.spatial.topology_graph import TopologySemGraph
            self._tsg = TopologySemGraph.from_dict(snapshot)
            return True
        except Exception as exc:
            logger.debug("Failed to load topology graph snapshot: %s", exc)
            return False

    def update_visited_room(self, room_id: int) -> None:
        """标记某房间已探索 (拓扑感知探索用)。"""
        if room_id >= 0:
            self._visited_room_ids.add(room_id)

    @property
    def topology_graph(self) -> Any | None:
        """获取拓扑语义图实例 (供外部模块访问)。"""
        return self._tsg

    @staticmethod
    def _get_current_room_id(
        robot_position: dict[str, float],
        rooms: list[dict],
    ) -> int:
        """根据机器人位置找到当前所在的房间 ID。"""
        if not rooms:
            return -1
        robot_xy = np.array([
            robot_position.get("x", 0.0),
            robot_position.get("y", 0.0),
        ])
        best_id = -1
        best_dist = float("inf")
        for room in rooms:
            center = room.get("center", {})
            rx = float(center.get("x", 0.0))
            ry = float(center.get("y", 0.0))
            dist = float(np.linalg.norm(robot_xy - np.array([rx, ry])))
            if dist < best_dist:
                best_dist = dist
                best_id = room.get("room_id", -1)
        return best_id

    # ================================================================
    #  内部方法
    # ================================================================

    async def _call_with_fallback(
        self, messages: list[dict[str, str]]
    ) -> str | None:
        """调用主 LLM, 失败或空响应则尝试备用。"""
        from .llm_client import LLMError

        # 主 LLM
        if self._primary.is_available():
            try:
                result = await self._primary.chat(messages)
                if result and result.strip():
                    return result
                logger.warning("Primary LLM returned empty response, trying fallback")
            except LLMError as e:
                logger.warning("Primary LLM failed: %s", e)

        # 备用 LLM
        if self._fallback and self._fallback.is_available():
            try:
                logger.info("Trying fallback LLM...")
                result = await self._fallback.chat(messages)
                if result and result.strip():
                    return result
                logger.error("Fallback LLM also returned empty response")
            except LLMError as e:
                logger.error("Fallback LLM also failed: %s", e)

        return None

    def _parse_llm_response(self, response_text, scene_graph: dict | None = None) -> GoalResult:
        """解析 LLM JSON 响应。"""
        from core.utils.sanitize import sanitize_position

        from .goal_resolver import GoalResult

        try:
            data = self._extract_json(response_text)

            action = data.get("action", "navigate")
            target = data.get("target", {})

            # 从场景图获取坐标系，默认为 map
            frame_id = SLOW_PATH_MAP_FRAME_ID
            if scene_graph is not None:
                frame_id = scene_graph.get("frame_id", SLOW_PATH_MAP_FRAME_ID)

            # LLM 可能返回 NaN/Inf 坐标，用 sanitize_position 防御
            pos = sanitize_position([
                target.get("x", 0),
                target.get("y", 0),
                target.get("z", 0),
            ])

            return GoalResult(
                action=action,
                target_x=pos[0],
                target_y=pos[1],
                target_z=pos[2],
                target_label=data.get("target_label", ""),
                confidence=float(data.get("confidence", 0)),
                reasoning=data.get("reasoning", ""),
                is_valid=True,
                frame_id=frame_id,
            )
        except Exception as e:
            logger.error("Failed to parse LLM response: %s\nRaw: %s", e, response_text)
            return GoalResult(
                action="error",
                error=f"Parse error: {e}",
                is_valid=False,
            )

    @staticmethod
    def _extract_json(text) -> dict:
        """从 LLM 输出中提取 JSON (处理 markdown 代码块 + 截断修复)。"""
        import re
        # BUG FIX: LLM client 可能已经返回 dict，不需要再 json.loads
        if isinstance(text, dict):
            return text

        candidates = []

        # 尝试找 JSON 代码块
        match = re.search(r"```(?:json)?\s*([\s\S]*?)```", text)
        if match:
            candidates.append(match.group(1).strip())

        # 找第一个 { 和最后一个 }
        start = text.find("{")
        end = text.rfind("}")
        if start != -1 and end != -1:
            candidates.append(text[start:end + 1])

        for candidate in candidates:
            try:
                return json.loads(candidate)
            except json.JSONDecodeError:
                pass

        # 截断修复: LLM 输出可能在 reasoning 字段中被截断
        for candidate in candidates:
            try:
                fixed = SlowPathMixin._fix_truncated_json(candidate)
                if fixed:
                    return json.loads(fixed)
            except json.JSONDecodeError:
                pass

        raise ValueError(f"No JSON found in response: {text[:200]}")

    @staticmethod
    def _fix_truncated_json(text: str) -> str | None:
        """尝试修复被截断的 JSON (常见于 max_tokens 限制)。"""
        import re
        required_keys = {"action", "target", "confidence"}
        try:
            json.loads(text)
            return text
        except json.JSONDecodeError:
            pass

        # 检查是否包含必要的键
        if not all(f'"{k}"' in text for k in required_keys):
            return None

        # 尝试在不同位置截断并闭合 JSON
        # 策略: 找到最后一个完整的 key-value 对, 截断 reasoning 字段
        reasoning_match = re.search(r'"reasoning"\s*:\s*"', text)
        if reasoning_match:
            prefix = text[:reasoning_match.start()]
            # 检查 prefix 中是否有足够的字段
            if '"confidence"' in prefix:
                cleaned = prefix.rstrip().rstrip(",")
                if not cleaned.endswith("}"):
                    cleaned += "}"
                try:
                    json.loads(cleaned)  # 验证 JSON 有效
                    return cleaned       # 返回字符串，不是 dict
                except json.JSONDecodeError:
                    pass

        # 策略 2: 暴力闭合
        for trim_pos in range(len(text) - 1, max(len(text) - 200, 0), -1):
            ch = text[trim_pos]
            if ch in (',', '"', '}'):
                snippet = text[:trim_pos]
                # 闭合打开的字符串
                if snippet.count('"') % 2 == 1:
                    snippet += '"'
                # 闭合打开的对象
                open_braces = snippet.count('{') - snippet.count('}')
                snippet += "}" * max(open_braces, 0)
                try:
                    result = json.loads(snippet)
                    if isinstance(result, dict) and required_keys.issubset(result.keys()):
                        return snippet
                except json.JSONDecodeError:
                    continue

        return None
