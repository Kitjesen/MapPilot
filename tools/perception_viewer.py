#!/usr/bin/env python3
"""实时感知可视化 — MJPEG 流 + BPU YOLO + 实例分割 + IoU追踪 + 卡尔曼平滑.

BPU 加速版:
- YOLO11s-seg 640×640 on D-Robotics Nash BPU (~15ms BPU + ~30ms CPU)
- 实例分割: proto mask × coefficients → 像素级物体轮廓
- 替代原 YOLO-World CPU 推理 (~500ms/帧)
- IoU 追踪 + CV 卡尔曼: 帧间平滑, 稳定 ID
- COCO 80 类 → 导航相关类过滤

用法: 在 S100P 上运行:
  source /opt/ros/humble/setup.bash
  python3 -u /opt/nav/tools/perception_viewer.py

浏览器打开: http://localhost:8070 (通过 SSH 隧道)
"""
import threading
import time
import json
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


# ═══════════════════════════════════════════════════════════════════
# COCO 80 类名
# ═══════════════════════════════════════════════════════════════════

COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

# 导航相关类 (只显示这些, 忽略动物/食物等)
NAV_CLASSES = {
    0,   # person
    13,  # bench
    24,  # backpack
    25,  # umbrella
    26,  # handbag
    28,  # suitcase
    39,  # bottle
    41,  # cup
    56,  # chair
    57,  # couch
    58,  # potted plant
    59,  # bed
    60,  # dining table
    61,  # toilet
    62,  # tv
    63,  # laptop
    66,  # keyboard
    67,  # cell phone
    72,  # refrigerator
    73,  # book
    74,  # clock
    75,  # vase
}


# ═══════════════════════════════════════════════════════════════════
# BPU YOLOv8n 推理器
# ═══════════════════════════════════════════════════════════════════

class BPUDetector:
    """YOLO on D-Robotics Nash BPU via HB_HBMRuntime.

    Supports detect and seg models (auto-detect output structure).
    Seg models output instance masks via proto × coefficients.
    Prefers seg > detect, higher quality > lower.
    """

    # Model preference: seg first (detect+seg in one), then detect-only
    MODEL_CANDIDATES = [
        "/home/sunrise/models/yolo11s_seg_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo12s_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo12n_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11s_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11n_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolov8n_detect_nashe_640x640_nv12.hbm",
    ]
    INPUT_SIZE = 640
    STRIDES = [8, 16, 32]

    def __init__(self, conf_thr=0.25, iou_thr=0.45, model_path=None):
        import os
        from hbm_runtime import HB_HBMRuntime

        if model_path:
            path = model_path
        else:
            path = None
            for p in self.MODEL_CANDIDATES:
                if os.path.exists(p):
                    path = p
                    break
            if path is None:
                raise FileNotFoundError("No BPU YOLO model found")

        base = os.path.basename(path)
        self._model_name_short = base.replace("_nashe_640x640_nv12.hbm", "").replace("_detect", "").replace("_seg", "-seg")
        self.has_seg = "_seg_" in base
        print(f"[BPU] Loading {self._model_name_short} from {path} (seg={self.has_seg})")

        self._rt = HB_HBMRuntime(path)
        self._mname = self._rt.model_names[0]
        self._conf_thr = conf_thr
        self._iou_thr = iou_thr

        # Run dummy inference to discover output structure
        y = np.zeros((1, self.INPUT_SIZE, self.INPUT_SIZE, 1), dtype=np.uint8)
        uv = np.zeros((1, self.INPUT_SIZE // 2, self.INPUT_SIZE // 2, 2), dtype=np.uint8)
        dummy_out = self._rt.run({"images_y": y, "images_uv": uv})[self._mname]

        # Classify outputs by last dimension
        cls_outs = {}   # grid_size → name  (ch=80)
        bbox_outs = {}  # grid_size → name  (ch=64)
        mask_outs = {}  # grid_size → name  (ch=32, mask coefficients)
        self._proto_name = None
        for name, arr in dummy_out.items():
            gs = arr.shape[1]
            ch = arr.shape[-1]
            if ch == 80:
                cls_outs[gs] = name
            elif ch == 64:
                bbox_outs[gs] = name
            elif ch == 32:
                if gs == self.INPUT_SIZE // 4:  # 160×160 = proto
                    self._proto_name = name
                else:
                    mask_outs[gs] = name

        self._output_map = []  # [(cls_name, bbox_name, mask_name_or_None)]
        for stride in self.STRIDES:
            gs = self.INPUT_SIZE // stride
            mc = mask_outs.get(gs)
            self._output_map.append((cls_outs[gs], bbox_outs[gs], mc))

        # Dequantization scales for int32 bbox (v8n)
        self._bbox_scales = {}
        try:
            oq = self._rt.output_quants[self._mname]
            for _, bbox_name, _ in self._output_map:
                qp = oq[bbox_name]
                if len(qp.scale) > 0:
                    self._bbox_scales[bbox_name] = qp.scale.astype(np.float32)
        except Exception:
            pass

        self._dfl_weights = np.arange(16, dtype=np.float32)

        seg_status = f", proto={self._proto_name}" if self._proto_name else ""
        print(f"[BPU] {self._model_name_short} ready, "
              f"outputs: {[(c,b,m) for c,b,m in self._output_map]}{seg_status}")

    def _preprocess(self, bgr):
        """Letterbox resize + BGR→NV12. Returns (y, uv, scale, pad_x, pad_y)."""
        h0, w0 = bgr.shape[:2]
        sz = self.INPUT_SIZE
        scale = min(sz / h0, sz / w0)
        nh, nw = int(h0 * scale), int(w0 * scale)
        pad_x, pad_y = (sz - nw) // 2, (sz - nh) // 2

        # Letterbox: resize + pad with gray (114)
        canvas = np.full((sz, sz, 3), 114, dtype=np.uint8)
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = resized

        # BGR → YUV I420 → NV12
        yuv = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420)
        y_plane = yuv[:sz, :].reshape(1, sz, sz, 1)
        u = yuv[sz:sz + sz // 4, :].reshape(sz // 2, sz // 2)
        v = yuv[sz + sz // 4:, :].reshape(sz // 2, sz // 2)
        uv_plane = np.stack([u, v], axis=-1).reshape(1, sz // 2, sz // 2, 2)

        return (np.ascontiguousarray(y_plane),
                np.ascontiguousarray(uv_plane),
                scale, pad_x, pad_y)

    def _dfl_decode(self, bbox_int32, scale_vec):
        """DFL decode: int32 → dequant → softmax(16 bins) → weighted sum → 4 LTRB."""
        # bbox_int32: (H, W, 64)
        H, W, _ = bbox_int32.shape
        # Dequantize per-channel
        bbox_f = bbox_int32.astype(np.float32) * scale_vec  # (H, W, 64)
        # Reshape to (H, W, 4, 16) — 4 sides × 16 bins
        bbox_f = bbox_f.reshape(H, W, 4, 16)
        # Softmax over bins (axis=-1)
        bbox_exp = np.exp(bbox_f - bbox_f.max(axis=-1, keepdims=True))
        bbox_sm = bbox_exp / bbox_exp.sum(axis=-1, keepdims=True)
        # Weighted sum: distance = sum(softmax × [0..15])
        dist = (bbox_sm * self._dfl_weights).sum(axis=-1)  # (H, W, 4)
        return dist  # LTRB distances in grid units

    def _postprocess(self, outputs, scale, pad_x, pad_y, orig_h, orig_w):
        """Decode all scales → NMS → detections + mask coefficients."""
        all_boxes = []
        all_scores = []
        all_classes = []
        all_mask_coeffs = []

        for i, stride in enumerate(self.STRIDES):
            cls_name, bbox_name, mask_name = self._output_map[i]
            cls_logits = outputs[cls_name][0]   # (H, W, 80) float32
            bbox_raw = outputs[bbox_name][0]    # (H, W, 64) float32 or int32
            H, W = cls_logits.shape[:2]

            # Sigmoid for class confidence
            cls_scores = 1.0 / (1.0 + np.exp(-cls_logits.astype(np.float32)))

            # Find cells with max score > threshold
            max_scores = cls_scores.max(axis=-1)  # (H, W)
            mask = max_scores > self._conf_thr
            if not mask.any():
                continue

            # Extract valid cells
            ys, xs = np.where(mask)
            valid_cls = cls_scores[ys, xs]        # (N, 80)
            valid_bbox = bbox_raw[ys, xs]          # (N, 64)
            valid_max = max_scores[ys, xs]         # (N,)
            valid_cids = valid_cls.argmax(axis=-1) # (N,)

            # Mask coefficients (seg models)
            if mask_name and mask_name in outputs:
                valid_mc = outputs[mask_name][0][ys, xs]  # (N, 32)
            else:
                valid_mc = None

            # DFL decode for valid cells only
            # Dequantize if int32 (v8n), passthrough if float32 (v11/v12)
            if bbox_name in self._bbox_scales:
                bbox_f = valid_bbox.astype(np.float32) * self._bbox_scales[bbox_name]
            else:
                bbox_f = valid_bbox.astype(np.float32)
            bbox_f = bbox_f.reshape(-1, 4, 16)
            bbox_exp = np.exp(bbox_f - bbox_f.max(axis=-1, keepdims=True))
            bbox_sm = bbox_exp / bbox_exp.sum(axis=-1, keepdims=True)
            dist = (bbox_sm * self._dfl_weights).sum(axis=-1)  # (N, 4) LTRB

            # Grid centers (in input pixels)
            cx = (xs.astype(np.float32) + 0.5) * stride
            cy = (ys.astype(np.float32) + 0.5) * stride

            # LTRB → xyxy (in input pixels)
            x1 = cx - dist[:, 0] * stride
            y1 = cy - dist[:, 1] * stride
            x2 = cx + dist[:, 2] * stride
            y2 = cy + dist[:, 3] * stride

            # Remove padding and rescale to original image
            x1 = (x1 - pad_x) / scale
            y1 = (y1 - pad_y) / scale
            x2 = (x2 - pad_x) / scale
            y2 = (y2 - pad_y) / scale

            # Clip to image bounds
            x1 = np.clip(x1, 0, orig_w)
            y1 = np.clip(y1, 0, orig_h)
            x2 = np.clip(x2, 0, orig_w)
            y2 = np.clip(y2, 0, orig_h)

            boxes = np.stack([x1, y1, x2, y2], axis=-1)  # (N, 4)
            all_boxes.append(boxes)
            all_scores.append(valid_max)
            all_classes.append(valid_cids)
            if valid_mc is not None:
                all_mask_coeffs.append(valid_mc)

        if not all_boxes:
            return [], None

        boxes = np.concatenate(all_boxes)
        scores = np.concatenate(all_scores)
        classes = np.concatenate(all_classes)
        mask_coeffs = np.concatenate(all_mask_coeffs) if all_mask_coeffs else None

        # NMS per class
        keep = self._nms(boxes, scores, classes)
        results = [(boxes[i], scores[i], classes[i]) for i in keep]
        kept_mc = mask_coeffs[keep] if mask_coeffs is not None else None
        return results, kept_mc

    def _nms(self, boxes, scores, classes):
        """Per-class NMS."""
        keep = []
        for cid in np.unique(classes):
            mask = classes == cid
            idx = np.where(mask)[0]
            if len(idx) == 0:
                continue
            b = boxes[idx]
            s = scores[idx]
            # Sort by score descending
            order = s.argsort()[::-1]
            picked = []
            while len(order) > 0:
                i = order[0]
                picked.append(idx[i])
                if len(order) == 1:
                    break
                rest = order[1:]
                ious = self._compute_iou(b[i], b[rest])
                order = rest[ious < self._iou_thr]
            keep.extend(picked)
        return keep

    @staticmethod
    def _compute_iou(box, boxes):
        """IoU of one box against many."""
        ix1 = np.maximum(box[0], boxes[:, 0])
        iy1 = np.maximum(box[1], boxes[:, 1])
        ix2 = np.minimum(box[2], boxes[:, 2])
        iy2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(ix2 - ix1, 0) * np.maximum(iy2 - iy1, 0)
        a1 = (box[2] - box[0]) * (box[3] - box[1])
        a2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        return inter / np.maximum(a1 + a2 - inter, 1e-6)

    def detect(self, bgr):
        """Run detection on BGR image. Returns list of {label, score, bbox, [mask]}.

        Seg models also return 'mask' (bool H×W crop) and 'mask_offset' (x, y).
        """
        h0, w0 = bgr.shape[:2]
        y, uv, scale, pad_x, pad_y = self._preprocess(bgr)

        inputs = {"images_y": y, "images_uv": uv}
        result = self._rt.run(inputs)
        outputs = result[self._mname]

        raw, kept_mc = self._postprocess(outputs, scale, pad_x, pad_y, h0, w0)

        # Generate instance masks (seg models only, batched matmul)
        masks_list = [None] * len(raw)
        if kept_mc is not None and self._proto_name and self._proto_name in outputs:
            proto = outputs[self._proto_name][0]  # (160, 160, 32)
            ph, pw = proto.shape[:2]
            # Batch matmul: (ph*pw, 32) @ (32, K) → (ph*pw, K), then sigmoid
            proto_flat = proto.reshape(-1, 32)  # (25600, 32)
            all_masks = proto_flat @ kept_mc.T   # (25600, K)
            np.clip(all_masks, -50, 50, out=all_masks)  # prevent overflow
            all_masks = 1.0 / (1.0 + np.exp(-all_masks))
            all_masks = all_masks.reshape(ph, pw, -1)  # (160, 160, K)
            for idx in range(len(raw)):
                box = raw[idx][0]
                bx1 = max(0, int((box[0] * scale + pad_x) / 4))
                by1 = max(0, int((box[1] * scale + pad_y) / 4))
                bx2 = min(pw, int((box[2] * scale + pad_x) / 4) + 1)
                by2 = min(ph, int((box[3] * scale + pad_y) / 4) + 1)
                if bx2 <= bx1 or by2 <= by1:
                    continue
                crop = all_masks[by1:by2, bx1:bx2, idx]
                ox1, oy1, ox2, oy2 = box.astype(int)
                cw, ch = max(1, ox2 - ox1), max(1, oy2 - oy1)
                mask_resized = cv2.resize(crop, (cw, ch), interpolation=cv2.INTER_LINEAR)
                masks_list[idx] = (mask_resized > 0.5, ox1, oy1)

        dets = []
        frame_area = h0 * w0
        for i, (box, score, cid) in enumerate(raw):
            cid = int(cid)
            if cid not in NAV_CLASSES:
                continue
            x1, y1, x2, y2 = box.astype(int).tolist()
            bw, bh = x2 - x1, y2 - y1
            if bw * bh > frame_area * 0.40:
                continue
            if bw < 20 or bh < 20:
                continue
            det = {
                "label": COCO_NAMES[cid],
                "score": float(score),
                "bbox": [x1, y1, x2, y2],
            }
            if masks_list[i] is not None:
                mask_bool, mx, my = masks_list[i]
                det["mask"] = mask_bool
                det["mask_offset"] = (mx, my)
            dets.append(det)
        return dets


# ═══════════════════════════════════════════════════════════════════
# 卡尔曼滤波 (移植自 RWS qp_perception/kalman.py — CV 模型)
# ═══════════════════════════════════════════════════════════════════

class CentroidKalman2D:
    """4-state constant-velocity Kalman filter: [cx, cy, vx, vy]."""

    def __init__(self, cx: float, cy: float):
        self._x = np.array([cx, cy, 0.0, 0.0], dtype=np.float64)
        meas_noise = 8.0
        self._P = np.diag([meas_noise**2, meas_noise**2, 200.0, 200.0])
        self._H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)
        self._R = np.diag([meas_noise**2, meas_noise**2])
        self._q_pos, self._q_vel = 3.0, 15.0

    def predict(self, dt: float):
        if dt <= 0:
            return
        F = np.array([
            [1, 0, dt, 0], [0, 1, 0, dt],
            [0, 0, 1, 0], [0, 0, 0, 1],
        ], dtype=np.float64)
        dt2, dt3 = dt * dt, dt**3
        qp, qv = self._q_pos**2, self._q_vel**2
        Q = np.array([
            [qp + qv * dt3 / 3, 0, qv * dt2 / 2, 0],
            [0, qp + qv * dt3 / 3, 0, qv * dt2 / 2],
            [qv * dt2 / 2, 0, qv * dt, 0],
            [0, qv * dt2 / 2, 0, qv * dt],
        ], dtype=np.float64)
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def update(self, cx: float, cy: float):
        z = np.array([cx, cy], dtype=np.float64)
        y = z - self._H @ self._x
        S = self._H @ self._P @ self._H.T + self._R
        K = self._P @ self._H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ self._H) @ self._P

    @property
    def position(self):
        return float(self._x[0]), float(self._x[1])

    @property
    def velocity(self):
        return float(self._x[2]), float(self._x[3])

    def predict_position(self, dt_ahead: float):
        return (self._x[0] + self._x[2] * dt_ahead,
                self._x[1] + self._x[3] * dt_ahead)


# ═══════════════════════════════════════════════════════════════════
# IoU 追踪器 (移植自 RWS qp_perception/tracking/iou.py)
# ═══════════════════════════════════════════════════════════════════

def _iou(a, b):
    """IoU between two [x1,y1,x2,y2] boxes."""
    ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
    inter = max(ix2 - ix1, 0) * max(iy2 - iy1, 0)
    aa = (a[2] - a[0]) * (a[3] - a[1])
    ab = (b[2] - b[0]) * (b[3] - b[1])
    union = aa + ab - inter
    return inter / union if union > 0 else 0.0


class TrackedObject:
    """单个被追踪的物体."""
    __slots__ = ('track_id', 'label', 'score', 'bbox', 'kf',
                 'color', 'misses', 'age', 'last_time', 'is_person',
                 'mask', 'mask_offset')

    def __init__(self, track_id, label, score, bbox, timestamp,
                 mask=None, mask_offset=None):
        self.track_id = track_id
        self.label = label
        self.score = score
        self.bbox = list(bbox)  # [x1, y1, x2, y2]
        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2
        self.kf = CentroidKalman2D(cx, cy)
        self.color = COLORS[track_id % len(COLORS)]
        self.misses = 0
        self.age = 1
        self.last_time = timestamp
        self.is_person = label.lower() in ('person', 'people', 'human', 'man', 'woman')
        self.mask = mask
        self.mask_offset = mask_offset


class IoUTracker:
    """IoU 匹配 + 卡尔曼平滑追踪器."""

    def __init__(self, iou_threshold=0.15, max_misses=3):
        self._iou_thr = iou_threshold
        self._max_misses = max_misses
        self._next_id = 1
        self._tracks: dict[int, TrackedObject] = {}

    def update(self, detections: list[dict], timestamp: float) -> list[TrackedObject]:
        """更新追踪, detections: [{label, score, bbox:[x1,y1,x2,y2]}]."""
        track_ids = list(self._tracks.keys())

        # 卡尔曼预测
        for t in self._tracks.values():
            dt = max(timestamp - t.last_time, 0.001)
            t.kf.predict(dt)

        # 匈牙利匹配
        matches = {}
        used_det = set()
        if track_ids and detections:
            cost = np.zeros((len(track_ids), len(detections)))
            for i, tid in enumerate(track_ids):
                t = self._tracks[tid]
                pcx, pcy = t.kf.position
                hw = (t.bbox[2] - t.bbox[0]) / 2
                hh = (t.bbox[3] - t.bbox[1]) / 2
                pred_bbox = [pcx - hw, pcy - hh, pcx + hw, pcy + hh]
                for j, det in enumerate(detections):
                    cost[i, j] = _iou(pred_bbox, det['bbox'])
            try:
                from scipy.optimize import linear_sum_assignment
                ri, ci = linear_sum_assignment(cost, maximize=True)
                for i, j in zip(ri, ci):
                    if cost[i, j] >= self._iou_thr:
                        matches[track_ids[i]] = j
                        used_det.add(j)
            except ImportError:
                for i, tid in enumerate(track_ids):
                    best_j, best_v = -1, self._iou_thr
                    for j in range(len(detections)):
                        if j in used_det:
                            continue
                        if cost[i, j] > best_v:
                            best_v = cost[i, j]
                            best_j = j
                    if best_j >= 0:
                        matches[tid] = best_j
                        used_det.add(best_j)

        for tid, di in matches.items():
            det = detections[di]
            t = self._tracks[tid]
            cx = (det['bbox'][0] + det['bbox'][2]) / 2
            cy = (det['bbox'][1] + det['bbox'][3]) / 2
            t.kf.update(cx, cy)
            t.bbox = list(det['bbox'])
            t.score = det['score']
            t.label = det['label']
            t.misses = 0
            t.age += 1
            t.last_time = timestamp
            t.is_person = det['label'].lower() in ('person', 'people', 'human', 'man', 'woman')
            t.mask = det.get('mask')
            t.mask_offset = det.get('mask_offset')

        for tid in track_ids:
            if tid not in matches:
                self._tracks[tid].misses += 1

        for j, det in enumerate(detections):
            if j in used_det:
                continue
            tid = self._next_id
            self._next_id += 1
            self._tracks[tid] = TrackedObject(tid, det['label'], det['score'],
                                              det['bbox'], timestamp,
                                              det.get('mask'), det.get('mask_offset'))

        stale = [tid for tid, t in self._tracks.items() if t.misses > self._max_misses]
        for tid in stale:
            del self._tracks[tid]

        return sorted(self._tracks.values(), key=lambda t: t.track_id)

    def get_interpolated(self, timestamp: float) -> list[dict]:
        """获取卡尔曼预测的当前位置 (用于帧间插值). 返回轻量 dict."""
        result = []
        for t in sorted(self._tracks.values(), key=lambda t: t.track_id):
            if t.misses > 3:
                continue
            dt = max(timestamp - t.last_time, 0.0)
            pcx, pcy = t.kf.predict_position(dt)
            hw = (t.bbox[2] - t.bbox[0]) / 2
            hh = (t.bbox[3] - t.bbox[1]) / 2
            vx, vy = t.kf.velocity
            entry = {
                "bbox": [int(pcx - hw), int(pcy - hh), int(pcx + hw), int(pcy + hh)],
                "label": t.label, "score": t.score, "track_id": t.track_id,
                "color": t.color, "is_person": t.is_person,
                "vx": vx, "vy": vy,
            }
            if t.mask is not None:
                entry["mask"] = t.mask
                entry["mask_offset"] = t.mask_offset
            result.append(entry)
        return result


# ═══════════════════════════════════════════════════════════════════
# 颜色 & 绘制
# ═══════════════════════════════════════════════════════════════════

COLORS = [
    (0, 255, 0), (255, 128, 0), (0, 200, 255), (255, 0, 128),
    (128, 255, 0), (255, 255, 0), (0, 128, 255), (200, 0, 255),
    (0, 255, 200), (255, 64, 64), (128, 0, 255), (0, 255, 128),
]
PERSON_COLOR = (0, 0, 255)


def draw_tracks_fast(bgr, tracks):
    """用 cv2 绘制追踪框 + ASCII 标签 (极快). tracks 是 dict 列表."""
    for t in tracks:
        x1, y1, x2, y2 = t["bbox"]
        is_person = t["is_person"]
        color = PERSON_COLOR if is_person else t["color"]
        thickness = 3 if is_person else 2
        tid = t["track_id"]
        label = t["label"]
        score = t["score"]

        # Mask overlay (seg models — semi-transparent fill)
        seg_mask = t.get("mask")
        if seg_mask is not None:
            mx, my = t.get("mask_offset", (x1, y1))
            mh, mw = seg_mask.shape[:2]
            # Clip to image bounds
            ix1 = max(0, mx)
            iy1 = max(0, my)
            ix2 = min(bgr.shape[1], mx + mw)
            iy2 = min(bgr.shape[0], my + mh)
            if ix2 > ix1 and iy2 > iy1:
                roi = bgr[iy1:iy2, ix1:ix2]
                m = seg_mask[iy1 - my:iy2 - my, ix1 - mx:ix2 - mx]
                overlay_c = np.array(color, dtype=np.uint8)
                roi[m] = (roi[m] * 0.55 + overlay_c * 0.45).astype(np.uint8)

        cv2.rectangle(bgr, (x1, y1), (x2, y2), color, thickness)

        if is_person:
            txt = f"PERSON #{tid} {score:.0%}"
        else:
            txt = f"#{tid} {label} {score:.0%}"

        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(bgr, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 0, 0), -1)
        cv2.putText(bgr, txt, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        if is_person:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.drawMarker(bgr, (cx, cy), PERSON_COLOR,
                           cv2.MARKER_CROSS, 20, 2, cv2.LINE_AA)
            vx, vy = t["vx"], t["vy"]
            speed = (vx**2 + vy**2) ** 0.5
            if speed > 5:
                ex = int(cx + vx * 0.3)
                ey = int(cy + vy * 0.3)
                cv2.arrowedLine(bgr, (cx, cy), (ex, ey),
                                (0, 200, 255), 2, tipLength=0.3)

    return bgr


# ═══════════════════════════════════════════════════════════════════
# 全局状态
# ═══════════════════════════════════════════════════════════════════

latest_jpeg = None
latest_bgr = None
latest_scene_graph = None
latest_planner_status = ""

frame_lock = threading.Lock()
bgr_lock = threading.Lock()
sg_lock = threading.Lock()

fps_counter = {"n": 0, "t": time.time(), "fps": 0.0}
det_info = {"count": 0, "time": 0.0, "n_objects": 0, "n_persons": 0}

tracker = IoUTracker(iou_threshold=0.15, max_misses=3)
tracker_lock = threading.Lock()
last_encode_time = 0.0
ENCODE_INTERVAL = 0.08  # ~12fps 给浏览器 (SSH 隧道带宽有限)
STREAM_WIDTH = 640       # 浏览器端缩放到 640px 宽 (原始 1280→640, 数据量减 75%)

_prev_dets = []  # 2 帧确认
_model_label = "YOLO"  # 当前 BPU 模型名


# ═══════════════════════════════════════════════════════════════════
# BPU 检测线程
# ═══════════════════════════════════════════════════════════════════

def yolo_detect_loop():
    """后台线程: BPU YOLO 检测 + IoU 追踪."""
    global _prev_dets, _model_label
    print("[BPU] Loading best available YOLO model...")
    try:
        detector = BPUDetector(conf_thr=0.25, iou_thr=0.45)
        _model_label = detector._model_name_short
        det_info["has_seg"] = detector.has_seg
        print(f"[BPU] Ready! {_model_label} (seg={detector.has_seg})")
    except Exception as e:
        print(f"[BPU] Load failed: {e}")
        print("[BPU] Falling back to YOLO-World CPU...")
        _fallback_yolo_detect_loop()
        return

    while True:
        with bgr_lock:
            frame = latest_bgr.copy() if latest_bgr is not None else None
        if frame is None:
            time.sleep(0.05)
            continue

        t0 = time.time()
        try:
            dets = detector.detect(frame)
            elapsed = time.time() - t0

            # 2 帧确认
            if _prev_dets:
                confirmed = []
                for det in dets:
                    for prev in _prev_dets:
                        if det["label"] != prev["label"]:
                            continue
                        if _iou(det["bbox"], prev["bbox"]) > 0.2:
                            confirmed.append(det)
                            break
                dets = confirmed if confirmed else dets[:3]
            _prev_dets = dets

            with tracker_lock:
                tracks = tracker.update(dets, time.time())

            n_persons = sum(1 for t in tracks if t.is_person)
            det_info["count"] += 1
            det_info["time"] = elapsed
            det_info["n_objects"] = len(tracks)
            det_info["n_persons"] = n_persons

            if det_info["count"] % 30 == 1:
                ids = [t.track_id for t in tracks]
                print(f"[BPU] {len(dets)} det -> {len(tracks)} tracks "
                      f"({n_persons} person) in {elapsed*1000:.1f}ms | IDs={ids}")

        except Exception as e:
            print(f"[BPU] Error: {e}")
            import traceback; traceback.print_exc()
            time.sleep(0.5)

        # BPU 够快, 但不用跑满 — 限制在 ~30 FPS 给其他线程 CPU 时间
        remaining = 0.033 - (time.time() - t0)
        if remaining > 0:
            time.sleep(remaining)


def _fallback_yolo_detect_loop():
    """CPU fallback: 原 YOLO-World 逻辑."""
    global _prev_dets
    print("[YOLO] Fallback: Loading YOLO-World on CPU...")
    try:
        from ultralytics import YOLO
        import os
        model = None
        for p in ["/home/sunrise/yolov8s-worldv2.pt", "/home/sunrise/yolov8l-world.pt"]:
            if os.path.exists(p):
                model = YOLO(p)
                nav_classes = [
                    "person", "door", "chair", "fire extinguisher",
                    "stairs", "elevator", "sign", "box", "plant",
                    "monitor", "shelf", "bottle", "table",
                    "forklift", "conveyor belt", "control panel",
                ]
                model.set_classes(nav_classes)
                print(f"[YOLO] Loaded: {p}")
                break
        if model is None:
            print("[YOLO] No model found!")
            return
    except Exception as e:
        print(f"[YOLO] Load failed: {e}")
        return

    while True:
        with bgr_lock:
            frame = latest_bgr.copy() if latest_bgr is not None else None
        if frame is None:
            time.sleep(0.2)
            continue

        t0 = time.time()
        try:
            fh, fw = frame.shape[:2]
            frame_area = fh * fw
            results = model.predict(frame, imgsz=320, conf=0.20, iou=0.45, verbose=False)
            dets = []
            if results and len(results) > 0:
                r = results[0]
                if r.boxes is not None:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        xyxy = box.xyxy[0].cpu().numpy().astype(int).tolist()
                        bw = xyxy[2] - xyxy[0]
                        bh = xyxy[3] - xyxy[1]
                        if bw * bh > frame_area * 0.40 or bw < 20 or bh < 20:
                            continue
                        label = nav_classes[cls_id] if cls_id < len(nav_classes) else f"cls_{cls_id}"
                        dets.append({"label": label, "score": conf, "bbox": xyxy})

            elapsed = time.time() - t0

            if _prev_dets:
                confirmed = []
                for det in dets:
                    for prev in _prev_dets:
                        if det["label"] != prev["label"]:
                            continue
                        if _iou(det["bbox"], prev["bbox"]) > 0.2:
                            confirmed.append(det)
                            break
                dets = confirmed if confirmed else dets[:3]
            _prev_dets = dets

            with tracker_lock:
                tracks = tracker.update(dets, time.time())

            n_persons = sum(1 for t in tracks if t.is_person)
            det_info["count"] += 1
            det_info["time"] = elapsed
            det_info["n_objects"] = len(tracks)
            det_info["n_persons"] = n_persons

        except Exception as e:
            print(f"[YOLO] Error: {e}")
            time.sleep(1)


# ═══════════════════════════════════════════════════════════════════
# ROS2 节点
# ═══════════════════════════════════════════════════════════════════

class PerceptionViewer(Node):
    def __init__(self):
        super().__init__("perception_viewer")
        self.create_subscription(Image, "/camera/color/image_raw", self._color_cb, 1)
        self.create_subscription(String, "/nav/semantic/scene_graph", self._sg_cb, 1)
        self.create_subscription(String, "/nav/semantic/status", self._status_cb, 1)
        self.get_logger().info("PerceptionViewer started on :8070")

    def _color_cb(self, msg):
        global latest_jpeg, latest_bgr, last_encode_time

        now = time.time()
        if now - last_encode_time < ENCODE_INTERVAL:
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                with bgr_lock:
                    latest_bgr = bgr
            except Exception:
                pass
            return

        last_encode_time = now

        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

            with bgr_lock:
                latest_bgr = bgr.copy()

            fps_counter["n"] += 1
            dt = now - fps_counter["t"]
            if dt >= 1.0:
                fps_counter["fps"] = fps_counter["n"] / dt
                fps_counter["n"] = 0
                fps_counter["t"] = now

            with tracker_lock:
                tracks = tracker.get_interpolated(now)

            if tracks:
                draw_tracks_fast(bgr, tracks)

            # HUD
            h, w = bgr.shape[:2]
            overlay = bgr.copy()
            cv2.rectangle(overlay, (0, 0), (w, 40), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, bgr, 0.4, 0, bgr)

            n_trk = det_info["n_objects"]
            n_per = det_info["n_persons"]
            inf_t = det_info["time"]
            hud = (f"Camera {w}x{h} | {fps_counter['fps']:.0f} FPS | "
                   f"{_model_label} BPU: {n_trk} tracked ({n_per} person) | "
                   f"{inf_t*1000:.0f}ms/frame")
            cv2.putText(bgr, hud, (10, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

            with sg_lock:
                sg = latest_scene_graph
            sg_n = len(sg.get("objects", [])) if sg else 0
            seg_tag = "+Seg" if det_info.get("has_seg") else ""
            cv2.putText(bgr, f"Scene Graph: {sg_n} obj | {_model_label}{seg_tag} BPU + IoU Tracker + Kalman",
                        (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

            # 缩小给浏览器 (720p→480p, 数据量减 ~75%)
            h, w = bgr.shape[:2]
            if w > STREAM_WIDTH:
                ratio = STREAM_WIDTH / w
                small = cv2.resize(bgr, (STREAM_WIDTH, int(h * ratio)),
                                   interpolation=cv2.INTER_AREA)
            else:
                small = bgr
            _, jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
            with frame_lock:
                latest_jpeg = jpeg.tobytes()

        except Exception as e:
            self.get_logger().warn(f"Frame error: {e}")

    def _sg_cb(self, msg):
        global latest_scene_graph
        try:
            with sg_lock:
                latest_scene_graph = json.loads(msg.data)
        except Exception:
            pass

    def _status_cb(self, msg):
        global latest_planner_status
        latest_planner_status = msg.data


# ═══════════════════════════════════════════════════════════════════
# HTML 页面
# ═══════════════════════════════════════════════════════════════════

HTML_PAGE = b"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>LingTu Perception Viewer</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body { background:#1a1a2e; color:#eee; font-family:'Segoe UI',monospace; }
  .header { background:#16213e; padding:10px 20px; display:flex; align-items:center; gap:12px; }
  .header h1 { font-size:18px; color:#0ff; }
  .badge { padding:3px 8px; border-radius:10px; font-size:11px; }
  .badge.live { background:#0a3; }
  .badge.track { background:#c50; }
  .badge.bpu { background:#07a; }
  .main { display:flex; height:calc(100vh - 44px); }
  .video { flex:2.5; padding:6px; display:flex; align-items:center; justify-content:center; background:#111; }
  .video img { max-width:100%; max-height:100%; border:2px solid #333; border-radius:6px; }
  .panel { flex:1; padding:10px; overflow-y:auto; border-left:1px solid #333; min-width:280px; }
  .card { background:#16213e; border-radius:6px; padding:10px; margin-bottom:10px; }
  .card h3 { color:#0ff; font-size:13px; margin-bottom:6px; border-bottom:1px solid #333; padding-bottom:4px; }
  .row { display:flex; justify-content:space-between; padding:3px 0; font-size:12px; border-bottom:1px solid #1a1a2e; }
  .lbl { color:#ff0; } .scr { color:#0f0; text-align:right; }
  .det-lbl { color:#f80; } .det-id { color:#888; font-size:10px; }
  .person-row { background:#300; border-radius:3px; padding:2px 4px; }
  .dot { width:7px; height:7px; border-radius:50%; display:inline-block; margin-right:5px; }
  .dot.on { background:#0f0; } .dot.off { background:#f00; } .dot.wait { background:#fa0; }
  .sline { padding:3px 0; font-size:12px; }
  #planner { font-size:11px; white-space:pre-wrap; max-height:150px; overflow-y:auto; color:#aaa; }
  .ft { color:#555; font-size:10px; text-align:center; padding:4px; }
  .kf-info { color:#888; font-size:10px; }
</style>
</head>
<body>
<div class="header">
  <h1>LingTu Perception Viewer</h1>
  <span class="badge live">LIVE</span>
  <span class="badge bpu">BPU</span>
  <span class="badge track">TRACKING</span>
  <span style="font-size:11px;color:#888" id="clk"></span>
</div>
<div class="main">
  <div class="video">
    <img id="stream" src="/video" alt="Camera Stream">
  </div>
  <div class="panel">
    <div class="card">
      <h3>System Status</h3>
      <div class="sline"><span class="dot on"></span>Camera Streaming</div>
      <div class="sline"><span class="dot" id="d-yolo"></span>YOLO BPU+Seg <span id="yolo-inf" class="kf-info"></span></div>
      <div class="sline"><span class="dot" id="d-track"></span>IoU Tracker + Kalman <span id="trk-inf" class="kf-info"></span></div>
      <div class="sline"><span class="dot on"></span>Scene Graph 1Hz</div>
    </div>
    <div class="card">
      <h3>Tracked Objects</h3>
      <div id="trk-list" style="color:#888">Loading BPU model...</div>
    </div>
    <div class="card">
      <h3>Scene Graph (KG)</h3>
      <div id="sg-list">Loading...</div>
    </div>
    <div class="card">
      <h3>Planner</h3>
      <div id="planner">Waiting...</div>
    </div>
  </div>
</div>
<div class="ft">LingTu v1.8.0 | YOLO11s-seg Nash BPU (detect+seg ~45ms) + IoU Tracker + Kalman Filter | Orbbec Gemini 335</div>
<script>
function poll(){
  fetch('/api/state').then(r=>r.json()).then(d=>{
    const trks=d.tracks||[];
    const dy=document.getElementById('d-yolo');
    const dt2=document.getElementById('d-track');
    const yi=document.getElementById('yolo-inf');
    const ti=document.getElementById('trk-inf');
    const nP=trks.filter(t=>t.is_person).length;
    if(trks.length>0){
      dy.className='dot on'; dt2.className='dot on';
      yi.textContent='('+d.det_time.toFixed(1)+'ms/frame)';
      ti.textContent='('+trks.length+' obj, '+nP+' person)';
      let h='';
      trks.forEach(t=>{
        const cls=t.is_person?'person-row':'';
        const icon=t.is_person?'\\u26A0\\uFE0F ':'';
        const spd=Math.sqrt(t.vx*t.vx+t.vy*t.vy).toFixed(0);
        h+='<div class="row '+cls+'">' +
          '<span class="det-lbl">'+icon+'#'+t.id+' '+t.label+'</span>' +
          '<span class="scr">'+Math.round(t.score*100)+'%</span>' +
          '<span class="kf-info">v='+spd+'px/s</span></div>';
      });
      document.getElementById('trk-list').innerHTML=h;
    } else {
      dy.className='dot wait'; dt2.className='dot wait';
      yi.textContent='(detecting...)';
    }
    const sg=d.scene_graph;
    if(sg&&sg.objects){
      let h='';
      sg.objects.forEach(o=>{
        h+='<div class="row"><span class="lbl">'+o.label+'</span>' +
          '<span class="scr">'+(o.score||0).toFixed(2)+'</span></div>';
      });
      h+='<div style="margin-top:4px;color:#555;font-size:10px">'+
        sg.objects.length+' objects</div>';
      document.getElementById('sg-list').innerHTML=h;
    }
    if(d.planner_status) document.getElementById('planner').textContent=d.planner_status;
    document.getElementById('clk').textContent=new Date().toLocaleTimeString();
  }).catch(()=>{});
}
setInterval(poll,1500);
poll();
</script>
</body>
</html>"""


# ═══════════════════════════════════════════════════════════════════
# HTTP Server
# ═══════════════════════════════════════════════════════════════════

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(HTML_PAGE)

        elif self.path == "/video":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            while True:
                with frame_lock:
                    jpeg = latest_jpeg
                if jpeg:
                    try:
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        break
                time.sleep(0.08)  # ~12fps (SSH 隧道带宽有限)

        elif self.path == "/api/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with sg_lock:
                sg = latest_scene_graph or {}
            with tracker_lock:
                trks = []
                for t in sorted(tracker._tracks.values(), key=lambda x: x.track_id):
                    vx, vy = t.kf.velocity
                    trks.append({
                        "id": t.track_id, "label": t.label, "score": t.score,
                        "bbox": t.bbox, "is_person": t.is_person,
                        "vx": round(vx, 1), "vy": round(vy, 1),
                    })
            resp = json.dumps({
                "scene_graph": sg, "tracks": trks,
                "det_time": det_info["time"] * 1000,  # ms
                "planner_status": latest_planner_status,
            }, ensure_ascii=False)
            self.wfile.write(resp.encode())

        elif self.path == "/api/scene_graph":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with sg_lock:
                sg = latest_scene_graph or {}
            self.wfile.write(json.dumps({"scene_graph": sg}, ensure_ascii=False).encode())
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


def ros_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = PerceptionViewer()

    t_ros = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    t_ros.start()

    t_yolo = threading.Thread(target=yolo_detect_loop, daemon=True)
    t_yolo.start()

    class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True
        allow_reuse_address = True

    server = ThreadedHTTPServer(("0.0.0.0", 8070), Handler)
    print("Perception Viewer: http://0.0.0.0:8070")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
