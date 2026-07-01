"""
perception_publishers.py — 发布 Mixin

提供场景图、检测结果、语义地图 Marker 的所有发布方法。
由 SemanticPerceptionNode 通过多继承引入。
"""

import json

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class PerceptionPublishersMixin:
    """
    发布 Mixin — _publish_scene_graph / _publish_semantic_map_markers 等。

    需要宿主类提供以下属性 (均在 __init__ 中设置):
      self._tracker, self._prev_scene_snapshot,
      self._pub_scene_graph, self._pub_scene_diff, self._pub_semantic_markers,
      self._world_frame
    """

    # ================================================================
    #  场景图发布
    # ================================================================

    def _publish_scene_graph(self):
        """定时发布场景图 + DovSG diff + 语义地图 MarkerArray。"""
        if not self._tracker.objects:
            if self._pub_semantic_markers:
                self._publish_empty_semantic_markers()
            return

        msg = String()
        sg_json = self._tracker.get_scene_graph_json()

        # DovSG 动态场景图: 计算与上次快照的差异
        if self._prev_scene_snapshot:
            try:
                diff = self._tracker.compute_scene_diff(self._prev_scene_snapshot)
                if diff["total_events"] > 0:
                    sg_data = json.loads(sg_json)
                    sg_data["scene_diff"] = diff
                    sg_json = json.dumps(sg_data, ensure_ascii=False)
                    diff_msg = String()
                    diff_msg.data = json.dumps(diff, ensure_ascii=False)
                    self._pub_scene_diff.publish(diff_msg)
                    self.get_logger().info(
                        "Scene diff: %s", diff["summary"],
                    )
            except Exception as e:
                self.get_logger().debug(f"Scene diff failed: {e}")

        # 缓存快照用于下次 diff
        try:
            self._prev_scene_snapshot = json.loads(sg_json)
        except Exception as e:
            self.get_logger().debug(f"Scene snapshot cache failed: {e}")

        msg.data = sg_json
        self._pub_scene_graph.publish(msg)

        if self._pub_semantic_markers:
            self._publish_semantic_map_markers(msg.data)

    # ================================================================
    #  语义地图 Marker 发布
    # ================================================================

    def _publish_semantic_map_markers(self, scene_graph_json: str):
        """
        从场景图发布 MarkerArray，用于 RViz/地图上叠加语义物体。

        点云到语义地图渲染: 将 3D 检测物体以球体+文本标签形式渲染到地图上，
        可与 /cloud_map 点云叠加显示。
        """
        try:
            sg = json.loads(scene_graph_json)
            objects = sg.get("objects", [])
            if not isinstance(objects, list):
                return

            markers = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            frame = self._world_frame
            colors = [
                (1.0, 0.2, 0.2, 0.9),   # 红 - door
                (0.2, 0.6, 1.0, 0.9),   # 蓝 - chair
                (1.0, 0.5, 0.0, 0.9),   # 橙 - fire extinguisher
                (0.2, 0.8, 0.2, 0.9),   # 绿 - person/desk
                (0.6, 0.2, 0.8, 0.9),   # 紫 - stairs/elevator
                (1.0, 0.8, 0.2, 0.9),   # 黄 - sign
            ]
            color_by_label = {
                "door": 0, "chair": 1, "fire extinguisher": 2,
                "person": 3, "desk": 3, "stairs": 4, "elevator": 4, "sign": 5,
            }

            for i, obj in enumerate(objects[:80]):
                if not isinstance(obj, dict):
                    continue
                pos = obj.get("position", {})
                x = float(pos.get("x", 0))
                y = float(pos.get("y", 0))
                z = float(pos.get("z", 0))
                label = str(obj.get("label", "object"))
                score = float(obj.get("score", 0.5))
                first_word = label.lower().split()[0] if label else "object"
                idx = color_by_label.get(first_word, abs(hash(first_word)) % len(colors))
                r, g, b, a = colors[idx]

                sphere = Marker()
                sphere.header.frame_id = frame
                sphere.header.stamp = stamp
                sphere.ns = "semantic_objects"
                sphere.id = i * 2
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = x
                sphere.pose.position.y = y
                sphere.pose.position.z = z
                sphere.pose.orientation.w = 1.0
                size = max(0.15, min(0.5, 0.2 + score * 0.3))
                sphere.scale.x = sphere.scale.y = sphere.scale.z = size
                sphere.color.r = r
                sphere.color.g = g
                sphere.color.b = b
                sphere.color.a = a
                markers.markers.append(sphere)

                text = Marker()
                text.header.frame_id = frame
                text.header.stamp = stamp
                text.ns = "semantic_labels"
                text.id = i * 2 + 1
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = z + size * 0.8
                text.pose.orientation.w = 1.0
                text.scale.z = 0.15
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                text.text = label[:24]
                markers.markers.append(text)

            self._pub_semantic_markers.publish(markers)
        except (json.JSONDecodeError, TypeError, KeyError) as e:
            self.get_logger().debug(f"Semantic map markers error: {e}")

    def _publish_empty_semantic_markers(self):
        """发布空 MarkerArray，用于清除之前的语义标记。"""
        delete = Marker()
        delete.header.frame_id = self._world_frame
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = "semantic_objects"
        delete.id = 0
        delete.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(delete)
        self._pub_semantic_markers.publish(arr)

    # ================================================================
    #  Room LLM 命名 (创新1 补强)
    # ================================================================

    @staticmethod
    def _make_room_llm_namer(api_key: str, model: str, language: str):
        """创建异步 Room LLM 命名回调 (轻量直接 HTTP 调用, 不依赖 planner 包)。"""

        async def _namer(labels: list[str]) -> str:
            labels_str = ", ".join(labels[:12])
            if language == "zh":
                system = (
                    "你是一个室内场景理解助手。根据房间内包含的物体，"
                    "推断这个区域最可能是什么类型的空间。"
                    "只输出一个简短的名称 (2-4个字)，如: 走廊、办公室、厨房、"
                    "卫生间、会议室、大厅、储物间、楼梯间。不要输出任何额外文字。"
                )
                user = f"房间内包含以下物体: {labels_str}\n这个区域是什么?"
            else:
                system = (
                    "You are an indoor scene understanding assistant. "
                    "Given objects found in a room, infer the most likely room type. "
                    "Output ONLY a short name (1-3 words). No extra text."
                )
                user = f"Objects in this area: {labels_str}\nWhat is this room?"

            try:
                import httpx
                async with httpx.AsyncClient(timeout=10.0) as client:
                    resp = await client.post(
                        "https://api.openai.com/v1/chat/completions",
                        headers={
                            "Authorization": f"Bearer {api_key}",
                            "Content-Type": "application/json",
                        },
                        json={
                            "model": model,
                            "messages": [
                                {"role": "system", "content": system},
                                {"role": "user", "content": user},
                            ],
                            "temperature": 0.1,
                            "max_tokens": 20,
                        },
                    )
                    data = resp.json()
                    return data["choices"][0]["message"]["content"].strip()
            except ImportError:
                # httpx 不可用, 用 aiohttp
                try:
                    import aiohttp
                    async with aiohttp.ClientSession() as session:
                        async with session.post(
                            "https://api.openai.com/v1/chat/completions",
                            headers={
                                "Authorization": f"Bearer {api_key}",
                                "Content-Type": "application/json",
                            },
                            json={
                                "model": model,
                                "messages": [
                                    {"role": "system", "content": system},
                                    {"role": "user", "content": user},
                                ],
                                "temperature": 0.1,
                                "max_tokens": 20,
                            },
                            timeout=aiohttp.ClientTimeout(total=10),
                        ) as resp:
                            data = await resp.json()
                            return data["choices"][0]["message"]["content"].strip()
                except ImportError:
                    return ""

        return _namer
