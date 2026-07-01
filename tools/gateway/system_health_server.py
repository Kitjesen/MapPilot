#!/usr/bin/env python3
"""
系统健康状态 HTTP 服务器
========================
订阅 nav_rings 三环 + 其他关键话题，通过 HTTP 对外提供红/黄/绿灯状态。

端口: 8067 (避开 dashboard 8066)

HTTP 接口:
  GET /              → HTML 仪表盘 (自动刷新)
  GET /api/health    → JSON 完整状态
  GET /api/light     → JSON { "light": "green|yellow|red", "level": "OK|DEGRADED|WARN|DANGER|ESTOP" }

用法:
  # 在 S100P 上运行
  python3 /opt/nav/tools/gateway/system_health_server.py

  # 从外部访问
  curl http://192.168.66.190:8067/api/health
"""

import json
import subprocess
import string
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

PORT = 8067

# ── 全局状态 (线程安全写, HTTP 线程读) ──
_state = {
    "light": "red",           # green / yellow / red
    "level": "UNKNOWN",       # OK / DEGRADED / WARN / DANGER / ESTOP / UNKNOWN
    "uptime_sec": 0,
    "safety": None,           # Ring 1 raw
    "eval": None,             # Ring 2 raw
    "dialogue": None,         # Ring 3 raw
    "scene_graph": None,      # 场景图摘要
    "services": {},           # systemd 服务状态
    "topics": {},             # 话题活跃度
    "timestamp": 0,
}
_lock = threading.Lock()


def _light_from_level(level: str) -> str:
    if level in ("OK",):
        return "green"
    elif level in ("DEGRADED", "WARN"):
        return "yellow"
    elif level in ("DANGER", "ESTOP"):
        return "red"
    return "red"


class HealthCollector(Node):
    """ROS2 节点: 订阅关键话题，更新全局状态。"""

    def __init__(self):
        super().__init__("system_health")

        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # Ring 1: SafetyMonitor
        self.create_subscription(String, "/nav/safety_state", self._on_safety, 10)
        # Ring 2: Evaluator
        self.create_subscription(String, "/nav/execution_eval", self._on_eval, 10)
        # Ring 3: DialogueManager
        self.create_subscription(String, "/nav/dialogue_state", self._on_dialogue, 10)
        # 场景图
        self.create_subscription(String, "/nav/semantic/scene_graph", self._on_sg, 10)
        # 里程计 (心跳)
        self.create_subscription(Odometry, "/nav/odometry", self._on_odom, be)
        # 规划器状态
        self.create_subscription(String, "/nav/planner_status", self._on_planner, 10)
        # 语义状态
        self.create_subscription(String, "/nav/semantic/status", self._on_semantic, 10)

        self._topic_last_seen = {}
        self._start_time = time.monotonic()

        # 每 5 秒检查一次 systemd 服务
        self.create_timer(5.0, self._check_services)
        # 每秒更新一次全局状态
        self.create_timer(1.0, self._update_global)

        self.get_logger().info(f"HealthCollector started, HTTP on :{PORT}")

    def _touch(self, name: str):
        self._topic_last_seen[name] = time.monotonic()

    def _on_safety(self, msg: String):
        self._touch("safety_state")
        try:
            data = json.loads(msg.data)
            with _lock:
                _state["safety"] = data
                level = data.get("level", "UNKNOWN")
                _state["level"] = level
                _state["light"] = _light_from_level(level)
        except json.JSONDecodeError:
            pass

    def _on_eval(self, msg: String):
        self._touch("execution_eval")
        try:
            with _lock:
                _state["eval"] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_dialogue(self, msg: String):
        self._touch("dialogue_state")
        try:
            with _lock:
                _state["dialogue"] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_sg(self, msg: String):
        self._touch("scene_graph")
        try:
            sg = json.loads(msg.data)
            objects = sg.get("objects", [])
            labels = [o.get("label", "?") for o in objects[:20]]
            with _lock:
                _state["scene_graph"] = {
                    "object_count": len(objects),
                    "labels": labels,
                }
        except json.JSONDecodeError:
            pass

    def _on_odom(self, msg: Odometry):
        self._touch("odometry")

    def _on_planner(self, msg: String):
        self._touch("planner_status")

    def _on_semantic(self, msg: String):
        self._touch("semantic_status")

    def _check_services(self):
        services = [
            "nav-lidar", "nav-slam", "nav-planning", "nav-autonomy",
            "nav-grpc", "nav-perception", "nav-semantic",
        ]
        result = {}
        for svc in services:
            try:
                out = subprocess.run(
                    ["systemctl", "is-active", svc],
                    capture_output=True, text=True, timeout=2)
                status = out.stdout.strip()
                result[svc] = status
            except Exception:
                result[svc] = "unknown"
        with _lock:
            _state["services"] = result

    def _update_global(self):
        now = time.monotonic()
        topics = {}
        for name, last in self._topic_last_seen.items():
            age = now - last
            topics[name] = {
                "alive": age < 10.0,
                "age_sec": round(age, 1),
            }

        with _lock:
            _state["uptime_sec"] = round(now - self._start_time)
            _state["topics"] = topics
            _state["timestamp"] = time.time()

            # 如果没有 safety_state 话题，根据服务状态推断
            if _state["safety"] is None:
                active_count = sum(
                    1 for v in _state["services"].values() if v == "active")
                total = max(len(_state["services"]), 1)
                if active_count == total:
                    _state["level"] = "OK"
                    _state["light"] = "green"
                elif active_count >= total * 0.5:
                    _state["level"] = "DEGRADED"
                    _state["light"] = "yellow"
                else:
                    _state["level"] = "WARN"
                    _state["light"] = "red"


# ── HTML 仪表盘 ──

_HTML_TEMPLATE = string.Template("""<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta http-equiv="refresh" content="3">
<title>LingTu System Health</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: -apple-system, sans-serif; background: #1a1a2e; color: #e0e0e0; padding: 20px; }
  h1 { text-align: center; margin-bottom: 20px; }
  .light-container { text-align: center; margin: 20px 0; }
  .light { display: inline-block; width: 80px; height: 80px; border-radius: 50%;
           box-shadow: 0 0 30px currentColor; margin: 0 auto; }
  .light.green { background: #00ff88; color: #00ff88; }
  .light.yellow { background: #ffdd00; color: #ffdd00; }
  .light.red { background: #ff4444; color: #ff4444; }
  .level-text { font-size: 24px; margin-top: 10px; font-weight: bold; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(350px, 1fr)); gap: 16px; margin-top: 20px; }
  .card { background: #16213e; border-radius: 8px; padding: 16px; }
  .card h2 { font-size: 14px; color: #888; text-transform: uppercase; margin-bottom: 12px; }
  .svc { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px solid #1a1a3e; }
  .svc-name { font-family: monospace; }
  .active { color: #00ff88; }
  .inactive, .failed { color: #ff4444; }
  .unknown { color: #888; }
  .topic { display: flex; justify-content: space-between; padding: 4px 0; border-bottom: 1px solid #1a1a3e; }
  .alive { color: #00ff88; }
  .stale { color: #ff4444; }
  .dialogue-text { font-size: 18px; line-height: 1.6; padding: 8px 0; }
  .issues { color: #ffdd00; }
  .meta { text-align: center; color: #555; margin-top: 20px; font-size: 12px; }
</style>
</head><body>
<h1>LingTu System Health</h1>

<div class="light-container">
  <div class="light $light"></div>
  <div class="level-text">$level</div>
</div>

<div class="grid">

<div class="card">
  <h2>Systemd Services</h2>
  $services_html
</div>

<div class="card">
  <h2>ROS2 Topics</h2>
  $topics_html
</div>

<div class="card">
  <h2>Safety (Ring 1)</h2>
  $safety_html
</div>

<div class="card">
  <h2>Execution (Ring 2)</h2>
  $eval_html
</div>

<div class="card">
  <h2>Dialogue (Ring 3)</h2>
  $dialogue_html
</div>

<div class="card">
  <h2>Scene Graph</h2>
  $sg_html
</div>

</div>

<div class="meta">Uptime: ${uptime}s | Auto-refresh 3s</div>
</body></html>
""")


def _render_html() -> str:
    with _lock:
        s = dict(_state)

    # Services
    svc_lines = []
    for name, status in sorted(s.get("services", {}).items()):
        cls = status if status in ("active", "inactive", "failed") else "unknown"
        svc_lines.append(
            f'<div class="svc"><span class="svc-name">{name}</span>'
            f'<span class="{cls}">{status}</span></div>')
    services_html = "\n".join(svc_lines) or '<div class="svc">No services checked yet</div>'

    # Topics
    topic_lines = []
    for name, info in sorted(s.get("topics", {}).items()):
        cls = "alive" if info.get("alive") else "stale"
        age = info.get("age_sec", "?")
        topic_lines.append(
            f'<div class="topic"><span>{name}</span>'
            f'<span class="{cls}">{age}s ago</span></div>')
    topics_html = "\n".join(topic_lines) or '<div class="topic">No topics seen yet</div>'

    # Safety
    safety = s.get("safety")
    if safety:
        issues = safety.get("issues") or []
        issue_text = "; ".join(issues) if issues else "No issues"
        links = safety.get("links", {})
        link_lines = []
        for lname, linfo in links.items():
            alive = linfo.get("alive", False)
            cls = "alive" if alive else "stale"
            stale = linfo.get("stale_sec")
            detail = f"{stale}s" if stale is not None else "waiting"
            link_lines.append(
                f'<div class="svc"><span>{lname}</span>'
                f'<span class="{cls}">{detail}</span></div>')
        safety_html = "\n".join(link_lines)
        if issues:
            safety_html += f'\n<div class="issues" style="margin-top:8px">{issue_text}</div>'
    else:
        safety_html = '<div>SafetyMonitor not running</div>'

    # Eval
    ev = s.get("eval")
    if ev:
        assessment = ev.get("assessment", "?")
        cte = ev.get("cross_track_error", 0)
        dist = ev.get("distance_to_goal", "?")
        eval_html = (
            f'<div>Assessment: <b>{assessment}</b></div>'
            f'<div>Cross-track error: {cte}m</div>'
            f'<div>Distance to goal: {dist}m</div>'
        )
    else:
        eval_html = '<div>Evaluator not running</div>'

    # Dialogue
    dlg = s.get("dialogue")
    if dlg:
        doing = dlg.get("doing", "?")
        understood = dlg.get("understood") or ""
        safety_text = dlg.get("safety_text", "")
        issue = dlg.get("issue") or ""
        dialogue_html = f'<div class="dialogue-text">{doing}</div>'
        if understood:
            dialogue_html += f'<div>Understood: {understood}</div>'
        if safety_text:
            dialogue_html += f'<div>Safety: {safety_text}</div>'
        if issue:
            dialogue_html += f'<div class="issues">Issue: {issue}</div>'
    else:
        dialogue_html = '<div>DialogueManager not running</div>'

    # Scene graph
    sg = s.get("scene_graph")
    if sg:
        count = sg.get("object_count", 0)
        labels = sg.get("labels", [])
        label_text = ", ".join(labels[:10])
        sg_html = f'<div>Objects: <b>{count}</b></div><div>{label_text}</div>'
    else:
        sg_html = '<div>No scene graph data</div>'

    return _HTML_TEMPLATE.substitute(
        light=s.get("light", "red"),
        level=s.get("level", "UNKNOWN"),
        services_html=services_html,
        topics_html=topics_html,
        safety_html=safety_html,
        eval_html=eval_html,
        dialogue_html=dialogue_html,
        sg_html=sg_html,
        uptime=s.get("uptime_sec", 0),
    )


class HealthHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/api/health":
            with _lock:
                data = dict(_state)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

        elif self.path == "/api/light":
            with _lock:
                data = {"light": _state["light"], "level": _state["level"]}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())

        elif self.path == "/" or self.path == "/index.html":
            html = _render_html()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(html.encode())

        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass  # 静默 HTTP 日志


def _safe_spin(node):
    """rclpy.spin wrapper — 捕获关闭时的异常，避免 traceback 噪音。"""
    try:
        rclpy.spin(node)
    except Exception:
        pass  # 正常关闭时 rclpy 会抛异常，静默处理


def main():
    # 启动 ROS2
    rclpy.init()
    node = HealthCollector()

    # ROS2 spin 在后台线程
    spin_thread = threading.Thread(target=_safe_spin, args=(node,), daemon=True)
    spin_thread.start()

    # HTTP 服务器在主线程 (SO_REUSEADDR 防止 Address already in use)
    class ReusableHTTPServer(HTTPServer):
        allow_reuse_address = True
    server = ReusableHTTPServer(("0.0.0.0", PORT), HealthHandler)
    print(f"System Health Server on http://0.0.0.0:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
