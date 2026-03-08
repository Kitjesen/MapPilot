#!/usr/bin/env python3
"""
factory_stub_test.py — 语义导航桩测试节点

功能:
  1. 以 1Hz 发布工厂场景图到 /nav/semantic/scene_graph
  2. 等待 3s 后发送自然语言指令到 /nav/semantic/instruction
  3. 监听 /nav/adapter_status 和 /nav/semantic/status
  4. 120s 内出现 goal_reached → PASS, 超时 → FAIL
  5. 打印 JSON 测试结果

用法:
  python3 factory_stub_test.py [--instruction "导航到目标区域"] [--timeout 120]
"""
import json
import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy  # noqa
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# ── 工厂场景图 (一楼, 机器人在 2,2,0.35 附近) ──────────────────────
FACTORY_SCENE_GRAPH = json.dumps({
    "objects": [
        {"id": "obj_01", "label": "传送带",   "position": [10.0,  3.0, 0.35], "score": 0.92, "detection_count": 5},
        {"id": "obj_02", "label": "工厂大门", "position": [ 5.0,  2.0, 0.35], "score": 0.92, "detection_count": 5},
        {"id": "obj_03", "label": "楼梯",     "position": [ 4.0,  8.0, 0.35], "score": 0.92, "detection_count": 5},
        {"id": "obj_04", "label": "目标区域", "position": [14.0,  3.0, 0.35], "score": 0.95, "detection_count": 5},
        {"id": "obj_05", "label": "控制室",   "position": [12.0,  5.0, 0.35], "score": 0.92, "detection_count": 5},
        {"id": "obj_06", "label": "仓储区",   "position": [18.0,  2.0, 0.35], "score": 0.92, "detection_count": 5},
        {"id": "obj_07", "label": "机械设备", "position": [ 8.0,  4.0, 0.35], "score": 0.92, "detection_count": 5},
    ],
    "relations": [],
    "regions": [
        {
            "name": "一楼生产区",
            "object_ids": ["obj_01", "obj_02", "obj_03", "obj_04", "obj_05", "obj_06", "obj_07"]
        }
    ],
})


class SemanticStubNode(Node):
    def __init__(self, instruction: str, timeout_sec: float):
        super().__init__("semantic_stub_test")

        self._instruction = instruction
        self._timeout = timeout_sec
        self._start_time = time.time()
        self._instruction_sent = False
        self._goal_reached = False
        self._resolved_goal = None
        self._semantic_status = []
        self._final_event = None

        self._pub_scene  = self.create_publisher(String, "/nav/semantic/scene_graph", 10)
        self._pub_instr  = self.create_publisher(String, "/nav/semantic/instruction", 10)

        self.create_subscription(String, "/nav/semantic/status",   self._status_cb, 10)
        self.create_subscription(String, "/nav/adapter_status",    self._adapter_cb, 10)
        self.create_subscription(Odometry, "/nav/odometry",        self._odom_cb, 5)

        # 1Hz 场景图 + 指令调度
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"[stub] Ready. Will send: \"{instruction}\" in 3s")

    # ── Callbacks ────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            state = data.get("state", "")
            if state:
                self.get_logger().info(f"[semantic_status] {state}")
            self._semantic_status.append(data)
        except Exception:
            pass

    def _adapter_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            event = data.get("event", "")
            self.get_logger().info(f"[adapter_status] {event}")
            self._final_event = event
            if event == "goal_reached":
                self._goal_reached = True
        except Exception:
            pass

    def _odom_cb(self, msg: Odometry):
        pass  # 仅用于确认 odom 在线

    # ── Main tick ────────────────────────────────────────────────────

    def _tick(self):
        elapsed = time.time() - self._start_time

        # 持续发布场景图
        sg_msg = String()
        sg_msg.data = FACTORY_SCENE_GRAPH
        self._pub_scene.publish(sg_msg)

        # 5s 后发送指令 (确保 planner_node 收到多帧场景图)
        if not self._instruction_sent and elapsed >= 5.0:
            instr_payload = json.dumps({
                "instruction": self._instruction,
                "language": "zh",
                "explore_if_unknown": False,
            })
            instr_msg = String()
            instr_msg.data = instr_payload
            self._pub_instr.publish(instr_msg)
            self._instruction_sent = True
            self.get_logger().info(f"[stub] Instruction sent: {self._instruction}")

        # 每 10s 打印一次状态
        if int(elapsed) % 10 == 0 and elapsed > 0:
            self.get_logger().info(
                f"[stub] t={elapsed:.0f}s | goal_reached={self._goal_reached}"
            )

    # ── Result ───────────────────────────────────────────────────────

    def has_result(self) -> bool:
        elapsed = time.time() - self._start_time
        return self._goal_reached or elapsed >= self._timeout

    def get_result(self) -> dict:
        elapsed = time.time() - self._start_time
        passed = self._goal_reached
        return {
            "test": "semantic_nav_factory",
            "instruction": self._instruction,
            "checks": [
                {"name": "instruction_sent",       "pass": self._instruction_sent},
                {"name": "goal_reached_in_time",   "pass": passed},
            ],
            "duration_sec": round(elapsed, 1),
            "final_event": self._final_event,
            "pass": passed,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--instruction", default="导航到目标区域")
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = SemanticStubNode(args.instruction, args.timeout)

    try:
        while rclpy.ok() and not node.has_result():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass

    result = node.get_result()
    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 50)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("=" * 50)
    sys.exit(0 if result["pass"] else 1)


if __name__ == "__main__":
    main()
