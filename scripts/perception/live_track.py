#!/usr/bin/env python3
"""Live tracking web viewer — YOLO + FusionMOT + LLM person selection.

Usage (on S100P):
    source /opt/ros/humble/setup.bash
    export MOONSHOT_API_KEY="sk-..."
    python3 scripts/perception/live_track.py

Open http://192.168.66.190:8080 in browser.

Features:
  - BPU YOLO detection (11ms, 30fps)
  - FusionMOT tracking with stable track IDs
  - OSNet BPU Re-ID (3ms/person)
  - LLM person selection via text input
  - Locked target highlighted in green
"""
import sys
import os
import time
import json
import threading

import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, "/home/sunrise/data/inovxio/lingtu/src/perception")

latest_jpg = None
lock = threading.Lock()
follow_cmd = None  # set by web UI: {"action": "follow", "text": "..."}
follow_cmd_lock = threading.Lock()
status_info = {"tracking": [], "followed": None, "state": "idle"}

# Color palette for track IDs
COLORS = [
    (0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0),
    (255, 0, 255), (0, 255, 255), (128, 255, 0), (255, 128, 0),
]

HTML_PAGE = """<!DOCTYPE html>
<html><head><title>LingTu Live Tracking</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { background: #111; color: #eee; font-family: 'Courier New', monospace; }
.container { display: flex; height: 100vh; }
.video { flex: 1; display: flex; justify-content: center; align-items: center; }
.video img { max-width: 100%; max-height: 100%; border: 2px solid #0ff; }
.panel { width: 320px; background: #1a1a2e; padding: 16px; overflow-y: auto; }
h2 { color: #0ff; margin-bottom: 12px; font-size: 16px; }
.input-group { margin-bottom: 16px; }
.input-group input { width: 100%; padding: 8px; background: #222; color: #fff;
    border: 1px solid #0ff; border-radius: 4px; font-size: 14px; }
.input-group button { width: 100%; margin-top: 8px; padding: 8px;
    background: #0ff; color: #000; border: none; border-radius: 4px;
    cursor: pointer; font-weight: bold; }
.input-group button:hover { background: #0aa; }
.btn-stop { background: #f44 !important; color: #fff !important; }
.btn-stop:hover { background: #c33 !important; }
.status { margin-top: 12px; padding: 8px; background: #222; border-radius: 4px;
    font-size: 12px; line-height: 1.6; }
.track { padding: 4px 8px; margin: 2px 0; border-radius: 3px; font-size: 12px; }
.track.followed { background: #064; border: 1px solid #0f0; }
.track.normal { background: #333; }
</style></head>
<body>
<div class="container">
    <div class="video"><img id="stream" src="/stream"/></div>
    <div class="panel">
        <h2>LingTu Tracking</h2>
        <div class="input-group">
            <input id="desc" placeholder="e.g. the person in red shirt" />
            <button onclick="follow()">Follow Person</button>
            <button class="btn-stop" onclick="stop()">Stop Following</button>
        </div>
        <h2>Tracks</h2>
        <div id="tracks"></div>
        <div class="status" id="status">Connecting...</div>
    </div>
</div>
<script>
function follow() {
    var t = document.getElementById('desc').value;
    if (!t) return;
    fetch('/api/follow', {method:'POST',headers:{'Content-Type':'application/json'},
        body: JSON.stringify({text: t})});
}
function stop() {
    fetch('/api/stop', {method:'POST'});
}
setInterval(function(){
    fetch('/api/status').then(r=>r.json()).then(d=>{
        var el = document.getElementById('tracks');
        var html = '';
        (d.tracking||[]).forEach(function(t){
            var cls = t.id == d.followed ? 'followed' : 'normal';
            html += '<div class="track '+cls+'">ID:'+t.id+' conf:'+t.conf+'</div>';
        });
        el.innerHTML = html;
        document.getElementById('status').innerText =
            'State: '+(d.state||'idle')+'  Followed: '+(d.followed||'none');
    }).catch(function(){});
}, 500);
</script>
</body></html>"""


def camera_loop():
    global latest_jpg, follow_cmd, status_info
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage
    from perception.detection.bpu_detector import BPUDetector

    # Load YOLO
    det = BPUDetector(
        confidence=0.3,
        model_path="/home/sunrise/data/models/yolo11s_detect_nashe_640x640_nv12.hbm",
    )
    det.load_model()
    print(f"[DET] YOLO loaded: {det._model_name_short}")

    # Load FusionMOT + OSNet
    tracker = None
    reid_ext = None
    selector = None
    try:
        from qp_perception.tracking.fusion import FusionMOT, FusionMOTConfig
        from qp_perception.reid.bpu_extractor import BPUReIDConfig, BPUReIDExtractor
        from qp_perception.selection.person_following import (
            FollowingConfig, PersonFollowingSelector,
        )

        reid_ext = BPUReIDExtractor(BPUReIDConfig())
        tracker = FusionMOT(
            config=FusionMOTConfig(),
            feature_dim=reid_ext.feature_dim,
        )
        selector = PersonFollowingSelector(FollowingConfig(auto_lock=False))
        print(f"[MOT] FusionMOT + OSNet BPU + Selector loaded")
    except Exception as e:
        print(f"[MOT] Tracking unavailable: {e}")

    # Simple LLM selection via CLIP text-image matching
    clip_encoder = None
    try:
        from open_clip import create_model_and_transforms, get_tokenizer
        import torch
        model, _, preprocess = create_model_and_transforms(
            "ViT-B-32", pretrained="laion2b_s34b_b79k", device="cpu"
        )
        tokenizer = get_tokenizer("ViT-B-32")
        model.eval()
        clip_encoder = (model, preprocess, tokenizer)
        print("[CLIP] Loaded for text-based person selection")
    except Exception as e:
        print(f"[CLIP] Not available: {e}")

    rclpy.init()
    node = Node("live_track")
    fps_t = time.monotonic()
    fps_c = 0

    def select_person_by_text(text, frame, person_bboxes, track_ids):
        """Use CLIP to match text description to person crops."""
        if clip_encoder is None or not person_bboxes:
            # Fallback: pick first person
            return track_ids[0] if track_ids else None

        model, preprocess, tokenizer = clip_encoder
        import torch

        # Encode text
        tokens = tokenizer([text])
        with torch.no_grad():
            text_feat = model.encode_text(tokens)
            text_feat = text_feat / text_feat.norm(dim=-1, keepdim=True)

        # Encode person crops
        h, w = frame.shape[:2]
        best_sim = -1
        best_tid = None
        for bbox, tid in zip(person_bboxes, track_ids):
            x1, y1, x2, y2 = [int(v) for v in bbox]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 - x1 < 10 or y2 - y1 < 10:
                continue
            crop = frame[y1:y2, x1:x2]
            crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
            from PIL import Image
            pil_img = Image.fromarray(crop_rgb)
            img_tensor = preprocess(pil_img).unsqueeze(0)
            with torch.no_grad():
                img_feat = model.encode_image(img_tensor)
                img_feat = img_feat / img_feat.norm(dim=-1, keepdim=True)
            sim = float((text_feat @ img_feat.T).squeeze())
            if sim > best_sim:
                best_sim = sim
                best_tid = tid

        if best_tid is not None:
            print(f"[CLIP] Selected track {best_tid} for '{text}' (sim={best_sim:.3f})")
        return best_tid

    def _on_frame(msg):
        global latest_jpg, follow_cmd, status_info
        nonlocal fps_t, fps_c

        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        frame = img.copy()

        # Detect
        t0 = time.monotonic()
        results = det.detect(img, "")
        t_det = (time.monotonic() - t0) * 1000
        persons = [r for r in results if r.label == "person"]

        # Track
        tracked_ids = []
        followed_id = None
        state = "idle"

        if tracker is not None and persons:
            bboxes = np.array(
                [[r.bbox[0], r.bbox[1], r.bbox[2] - r.bbox[0], r.bbox[3] - r.bbox[1]]
                 for r in persons],
                dtype=np.float32,
            )
            confs = np.array([r.score for r in persons], dtype=np.float32)
            ts = time.time()

            tracks = tracker.update_selective(
                bboxes, confs, ts,
                reid_extractor=reid_ext,
                frame=frame,
            )

            # Check for follow command
            with follow_cmd_lock:
                cmd = follow_cmd
                follow_cmd = None

            if cmd is not None:
                if cmd.get("action") == "follow" and tracks:
                    text = cmd.get("text", "")
                    person_bboxes = [r.bbox for r in persons]
                    tid_list = [int(t[0]) for t in tracks]
                    selected = select_person_by_text(text, frame, person_bboxes, tid_list)
                    if selected is not None and selector is not None:
                        selector.lock_track(selected, description=text)
                        print(f"[FOLLOW] Locked track {selected}: '{text}'")
                elif cmd.get("action") == "stop" and selector is not None:
                    selector.unlock()
                    print("[FOLLOW] Unlocked")

            # Run selector
            if selector is not None:
                from qp_perception.types import BoundingBox, Track as QPTrack
                qp_tracks = []
                for tid, bbox_arr, conf in tracks:
                    x, y, w_b, h_b = bbox_arr
                    qp_tracks.append(QPTrack(
                        track_id=int(tid),
                        bbox=BoundingBox(x=float(x), y=float(y),
                                         w=float(w_b), h=float(h_b)),
                        confidence=float(conf),
                        class_id="person",
                        first_seen_ts=ts, last_seen_ts=ts,
                    ))
                obs = selector.select(qp_tracks, ts)
                if obs is not None:
                    followed_id = obs.track_id
                state = selector.state

            # Draw tracks
            for tid, bbox_arr, conf in tracks:
                x, y, w_b, h_b = bbox_arr
                x1, y1 = int(x), int(y)
                x2, y2 = int(x + w_b), int(y + h_b)
                is_followed = (tid == followed_id)
                color = (0, 255, 0) if is_followed else COLORS[int(tid) % len(COLORS)]
                thickness = 3 if is_followed else 2

                cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)
                label = f"ID:{int(tid)} {conf:.0%}"
                if is_followed:
                    label = f"[FOLLOW] {label}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 4, y1), color, -1)
                cv2.putText(img, label, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                tracked_ids.append({"id": int(tid), "conf": f"{conf:.0%}"})
        else:
            # No tracker: just draw detections
            for r in results:
                x1, y1, x2, y2 = [int(v) for v in r.bbox]
                color = (0, 255, 0) if r.label == "person" else (255, 180, 0)
                cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
                cv2.putText(img, f"{r.label} {r.score:.0%}", (x1, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Update status
        status_info = {
            "tracking": tracked_ids,
            "followed": followed_id,
            "state": state,
        }

        # HUD
        fps_c += 1
        if time.monotonic() - fps_t > 2:
            fps = fps_c / (time.monotonic() - fps_t)
            fps_t = time.monotonic()
            fps_c = 0
            print(f"[DET] {fps:.1f}fps {t_det:.0f}ms {len(persons)}p "
                  f"state={state} follow={followed_id}")

        hud = f"det:{t_det:.0f}ms tracks:{len(tracked_ids)} follow:{followed_id or '-'}"
        cv2.putText(img, hud, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        _, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock:
            latest_jpg = jpg.tobytes()

    node.create_subscription(RosImage, "/camera/color/image_raw", _on_frame, 1)
    print("[CAM] Subscribed")
    rclpy.spin(node)


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode())
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with lock:
                    jpg = latest_jpg
                if jpg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                        + jpg + b"\r\n"
                    )
                    time.sleep(0.05)
                except Exception:
                    break
        elif self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(status_info).encode())
        else:
            self.send_error(404)

    def do_POST(self):
        global follow_cmd
        if self.path == "/api/follow":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
            with follow_cmd_lock:
                follow_cmd = {"action": "follow", "text": body.get("text", "")}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        elif self.path == "/api/stop":
            with follow_cmd_lock:
                follow_cmd = {"action": "stop"}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(b'{"ok":true}')
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    time.sleep(4)
    port = 8080
    print(f"[WEB] http://0.0.0.0:{port}")
    HTTPServer(("0.0.0.0", port), Handler).serve_forever()
