#!/usr/bin/env python3
"""Live detection web viewer — MJPEG stream with BPU YOLO boxes.

Usage (on S100P):
    source /opt/ros/humble/setup.bash
    python3 scripts/perception/live_detect.py

Open http://192.168.66.190:8080 in browser.
"""
import sys
import time
import threading

import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler

sys.path.insert(0, "/home/sunrise/data/inovxio/lingtu/src/semantic/perception")

latest_jpg = None
lock = threading.Lock()

HTML = (
    b"<!DOCTYPE html><html><head><title>LingTu Detection</title>"
    b"<style>"
    b"body{margin:0;background:#111;display:flex;justify-content:center;"
    b"align-items:center;height:100vh;flex-direction:column}"
    b"h1{color:#0ff;font-family:monospace}"
    b"img{max-width:95vw;max-height:85vh;border:2px solid #0ff}"
    b"</style></head><body>"
    b"<h1>LingTu BPU Detection - Live</h1>"
    b'<img src="/stream"/>'
    b"</body></html>"
)


def camera_loop():
    global latest_jpg
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage
    from semantic.perception.bpu_detector import BPUDetector

    det = BPUDetector(
        confidence=0.3,
        model_path="/home/sunrise/data/models/yolo11s_detect_nashe_640x640_nv12.hbm",
    )
    det.load_model()
    print(f"[DET] YOLO loaded: {det._model_name_short}")

    rclpy.init()
    node = Node("live_detect")
    fps_t = time.monotonic()
    fps_c = 0

    def _on_frame(msg):
        global latest_jpg
        nonlocal fps_t, fps_c

        h, w = msg.height, msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        t0 = time.monotonic()
        results = det.detect(img, "")
        t_det = (time.monotonic() - t0) * 1000

        # Draw bounding boxes
        for r in results:
            x1, y1, x2, y2 = [int(v) for v in r.bbox]
            color = (0, 255, 0) if r.label == "person" else (255, 180, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label = f"{r.label} {r.score:.0%}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(img, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # FPS counter
        fps_c += 1
        if time.monotonic() - fps_t > 2:
            fps = fps_c / (time.monotonic() - fps_t)
            fps_t = time.monotonic()
            fps_c = 0
            print(f"[DET] {fps:.1f} FPS, {len(results)} objs, {t_det:.0f}ms")

        # HUD overlay
        cv2.putText(img, f"det:{t_det:.0f}ms objs:{len(results)}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        _, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock:
            latest_jpg = jpg.tobytes()

    node.create_subscription(RosImage, "/camera/color/image_raw", _on_frame, 1)
    print("[CAM] Subscribed to /camera/color/image_raw")
    rclpy.spin(node)


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML)
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
        else:
            self.send_error(404)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    time.sleep(3)
    port = 8080
    print(f"[WEB] http://0.0.0.0:{port}")
    HTTPServer(("0.0.0.0", port), MJPEGHandler).serve_forever()
