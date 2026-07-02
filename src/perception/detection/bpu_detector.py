"""
BPU YOLO detector adapter for D-Robotics Nash BPU.

This module loads HBM models through HB_HBMRuntime and exposes a DetectorBase
implementation for COCO-class detection and optional segmentation masks.

Supported model families:
  - YOLO v8n/11n/11s/12n/12s detection models
  - YOLO 11s segmentation models
  - YOLOE end-to-end segmentation format
  - COCO-80 prompt filtering through text_prompt
"""

import logging
import os

import numpy as np

from .detector_base import Detection2D, DetectorBase


def _require_cv2():
    """Lazy cv2 import; raises ImportError with a clear message at runtime.

    Kept out of module top-level so the module can be imported (and its
    classes patched) in environments without opencv-python installed,
    e.g. unit test CI that mocks BPUDetector entirely.
    """
    import cv2
    return cv2

logger = logging.getLogger(__name__)

# COCO 80 class names used by Ultralytics-compatible YOLO models.
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

# COCO class-name to class-id lookup.
_COCO_NAME_TO_ID = {name: i for i, name in enumerate(COCO_NAMES)}

# Small prompt synonym groups mapped to COCO labels.
_SYNONYM_GROUPS = [
    {"person", "people", "human", "pedestrian"},
    {"desk", "table", "dining table"},
    {"sofa", "couch"},
    {"monitor", "screen", "tv", "television"},
    {"plant", "potted plant", "flower"},
    {"phone", "cell phone", "mobile"},
    {"fridge", "refrigerator"},
    {"motorbike", "motorcycle"},
    {"aeroplane", "airplane"},
    {"bin", "trash can", "recycling bin"},
    {"bus", "truck"},  # Useful for ambiguous vehicle prompts.
]
# Expanded synonym lookup.
_SYNONYM_EXPAND: dict = {}
for _group in _SYNONYM_GROUPS:
    for _word in _group:
        _SYNONYM_EXPAND[_word] = _group

# Direct aliases to canonical COCO labels.
_SYNONYMS = {
    "desk": "dining table",
    "table": "dining table",
    "sofa": "couch",
    "monitor": "tv",
    "screen": "tv",
    "plant": "potted plant",
    "flower": "potted plant",
    "phone": "cell phone",
    "mobile": "cell phone",
    "fridge": "refrigerator",
    "bin": "trash can",  # Not a COCO class; mapped to trash can wording.
    "motorbike": "motorcycle",
    "aeroplane": "airplane",
    "couch": "couch",
    "television": "tv",
    "laptop computer": "laptop",
}


class BPUDetector(DetectorBase):
    """D-Robotics Nash BPU YOLO detector adapter.
    Supports COCO-class detection, optional segmentation masks, and prompt to
    class-id filtering for the perception pipeline.
    """

    MODEL_CANDIDATES = [
        "/home/sunrise/models/yoloe26s_seg_nav125_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11s_seg_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo12s_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo12n_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11s_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolo11n_detect_nashe_640x640_nv12.hbm",
        "/home/sunrise/models/yolov8n_detect_nashe_640x640_nv12.hbm",
    ]
    INPUT_SIZE = 640
    STRIDES = [8, 16, 32]

    def __init__(
        self,
        confidence: float = 0.25,
        iou_threshold: float = 0.45,
        model_path: str | None = None,
        max_detections: int = 64,
        min_box_size_px: int = 12,
    ):
        self._conf_thr = confidence
        self._iou_thr = iou_threshold
        self._model_path = model_path
        self._max_detections = max(int(max_detections), 0)
        self._min_box_size_px = max(int(min_box_size_px), 1)
        self._rt = None
        self._mname = None
        self._output_map = []
        self._proto_name = None
        self._bbox_scales = {}
        self._dfl_weights = np.arange(16, dtype=np.float32)
        self.has_seg = False
        self._model_name_short = ""
        self._prompt_cache_key = ""
        self._allowed_cids: set = set()

    def load_model(self) -> None:
        """Load the BPU .hbm model."""
        import glob as _glob

        from hbm_runtime import HB_HBMRuntime

        path = self._model_path
        if not path:
            for pattern in self.MODEL_CANDIDATES:
                if "*" in pattern:
                    matches = sorted(_glob.glob(pattern))
                    if matches:
                        path = matches[-1]  # Use the last sorted candidate match.
                        break
                elif os.path.exists(pattern):
                    path = pattern
                    break
            if path is None:
                raise FileNotFoundError(
                    f"No BPU YOLO model found. Searched: {self.MODEL_CANDIDATES}"
                )

        self._custom_vocab = None
        vocab_pattern = path.replace(".hbm", "").replace("_nashe_640x640_nv12", "") + "*_vocab.json"
        vocab_matches = _glob.glob(vocab_pattern)
        if not vocab_matches:
            model_dir = os.path.dirname(path)
            vocab_matches = _glob.glob(os.path.join(model_dir, "*_vocab.json"))
        if vocab_matches:
            import json
            try:
                with open(vocab_matches[0], encoding="utf-8") as f:
                    vdata = json.load(f)
                names = vdata.get("names", {})
                self._custom_vocab = {int(k): v for k, v in names.items()}
                print(f"[BPU] Custom vocabulary loaded: {len(self._custom_vocab)} classes from {vocab_matches[0]}")
            except Exception as e:
                print(f"[BPU] Failed to load vocab: {e}, using COCO-80")

        base = os.path.basename(path)
        self._model_name_short = (
            base.replace("_nashe_640x640_nv12.hbm", "")
            .replace("_detect", "")
            .replace("_seg", "-seg")
        )
        self.has_seg = "_seg_" in base

        self._rt = HB_HBMRuntime(path)
        self._mname = self._rt.model_names[0]

        y = np.zeros((1, self.INPUT_SIZE, self.INPUT_SIZE, 1), dtype=np.uint8)
        uv = np.zeros(
            (1, self.INPUT_SIZE // 2, self.INPUT_SIZE // 2, 2), dtype=np.uint8
        )
        dummy_out = self._rt.run({"images_y": y, "images_uv": uv})[self._mname]

        _candidate_cls_ch = set()
        for _name, arr in dummy_out.items():
            ch = arr.shape[-1]
            if ch not in (64, 32, 1) and ch != self.INPUT_SIZE // 4:
                _candidate_cls_ch.add(ch)
        if self._custom_vocab and len(self._custom_vocab) in _candidate_cls_ch:
            self._num_classes = len(self._custom_vocab)
        elif 80 in _candidate_cls_ch:
            self._num_classes = 80
            if self._custom_vocab:
                print(f"[BPU] WARNING: vocab has {len(self._custom_vocab)} classes but model has 80; using COCO-80")
                self._custom_vocab = None
        elif _candidate_cls_ch:
            self._num_classes = max(_candidate_cls_ch)
        else:
            self._num_classes = 80

        self._is_yoloe = False
        self._yoloe_det_name = None
        self._yoloe_proto_name = None

        for name, arr in dummy_out.items():
            if arr.ndim == 3 and arr.shape[1] <= 300 and arr.shape[2] > 4:
                self._is_yoloe = True
                self._yoloe_det_name = name
                # 38 = 4(bbox) + 1(score) + 1(class) + 32(mask_coeffs)
                self._num_classes = len(self._custom_vocab) if self._custom_vocab else 80
                print(f"[BPU] YOLOE end-to-end format: {name} shape={arr.shape}")

        if self._is_yoloe:
            for name, arr in dummy_out.items():
                if name != self._yoloe_det_name and arr.ndim == 4:
                    self._yoloe_proto_name = name
                    print(f"[BPU] YOLOE proto: {name} shape={arr.shape}")
            self._output_map = []
            self._proto_name = self._yoloe_proto_name
        else:
            cls_outs = {}
            bbox_outs = {}
            mask_outs = {}
            self._proto_name = None
            for name, arr in dummy_out.items():
                gs = arr.shape[1]
                ch = arr.shape[-1]
                if ch == self._num_classes:
                    cls_outs[gs] = name
                elif ch == 64:
                    bbox_outs[gs] = name
                elif ch == 32:
                    if gs == self.INPUT_SIZE // 4:
                        self._proto_name = name
                    else:
                        mask_outs[gs] = name

            self._output_map = []
            for stride in self.STRIDES:
                gs = self.INPUT_SIZE // stride
                mc = mask_outs.get(gs)
                self._output_map.append((cls_outs[gs], bbox_outs[gs], mc))

        self._bbox_scales = {}
        try:
            oq = self._rt.output_quants[self._mname]
            for _, bbox_name, _ in self._output_map:
                qp = oq[bbox_name]
                if len(qp.scale) > 0:
                    self._bbox_scales[bbox_name] = qp.scale.astype(np.float32)
        except Exception:
            pass

        seg_info = f", proto={self._proto_name}" if self._proto_name else ""
        print(
            f"[BPU] {self._model_name_short} loaded (seg={self.has_seg}{seg_info})"
        )

    def detect(self, rgb: np.ndarray, text_prompt: str) -> list[Detection2D]:
        """Run BPU inference and return filtered detections.

        Args:
            rgb: HxWx3 uint8 BGR image from perception_node.
            text_prompt: dot-separated labels such as "door . chair . person".

        Returns:
            Detection2D results matching the prompt and model classes.
        """
        if self._rt is None:
            return []

        allowed = self._parse_prompt(text_prompt)

        bgr = rgb
        h0, w0 = bgr.shape[:2]

        y_plane, uv_plane, scale, pad_x, pad_y = self._preprocess(bgr)

        outputs = self._rt.run({"images_y": y_plane, "images_uv": uv_plane})[
            self._mname
        ]

        if self._is_yoloe:
            return self._detect_yoloe(outputs, allowed, scale, pad_x, pad_y, h0, w0)

        raw, kept_mc = self._postprocess(outputs, scale, pad_x, pad_y, h0, w0)
        if not raw:
            return []

        masks_list = self._generate_masks(raw, kept_mc, outputs, scale, pad_x, pad_y)

        results: list[Detection2D] = []
        h0 * w0
        for i, (box, score, cid) in enumerate(raw):
            cid = int(cid)
            if allowed and cid not in allowed:
                continue
            x1, y1, x2, y2 = box.astype(int).tolist()
            bw, bh = x2 - x1, y2 - y1
            # Keep smaller far-field targets instead of hard-coding a 20px cutoff.
            if not self._is_box_large_enough(bw, bh):
                continue

            mask = None
            if masks_list[i] is not None:
                mask_crop, mx, my = masks_list[i]
                full_mask = np.zeros((h0, w0), dtype=bool)
                mh, mw = mask_crop.shape[:2]
                ix1 = max(0, mx)
                iy1 = max(0, my)
                ix2 = min(w0, mx + mw)
                iy2 = min(h0, my + mh)
                if ix2 > ix1 and iy2 > iy1:
                    full_mask[iy1:iy2, ix1:ix2] = mask_crop[
                        iy1 - my : iy2 - my, ix1 - mx : ix2 - mx
                    ]
                mask = full_mask

            if self._custom_vocab and cid in self._custom_vocab:
                label = self._custom_vocab[cid]
            elif cid < len(COCO_NAMES):
                label = COCO_NAMES[cid]
            else:
                label = f"class_{cid}"

            results.append(
                Detection2D(
                    bbox=np.array([x1, y1, x2, y2], dtype=np.float32),
                    score=float(score),
                    label=label,
                    class_id=cid,
                    mask=mask,
                )
            )

        return self._limit_detection_results(results)

    def _is_box_large_enough(self, width: float, height: float) -> bool:
        """Keep smaller far-field targets instead of hard-coding a 20px cutoff."""
        return width >= self._min_box_size_px and height >= self._min_box_size_px

    def _limit_detection_results(self, results: list[Detection2D]) -> list[Detection2D]:
        """Sort globally by confidence before truncation to avoid class-order bias."""
        if not results:
            return []

        ranked = sorted(results, key=lambda det: float(det.score), reverse=True)
        if self._max_detections > 0:
            return ranked[: self._max_detections]
        return ranked

    def _sort_and_limit_indices(self, scores: np.ndarray, keep: list[int]) -> list[int]:
        if not keep:
            return []

        ranked = sorted(keep, key=lambda idx: float(scores[idx]), reverse=True)
        if self._max_detections > 0:
            return ranked[: self._max_detections]
        return ranked

    def shutdown(self) -> None:
        """Release BPU runtime resources."""
        self._rt = None

    # ================================================================
    # ================================================================

    def _detect_yoloe(self, outputs, allowed, scale, pad_x, pad_y, h0, w0):
        """Decode YOLOE end-to-end output [1, 300, 38] into detections."""
        cv2 = _require_cv2()
        det_out = outputs[self._yoloe_det_name][0]  # (300, 38)
        # 38 = 4(bbox xyxy) + 1(score) + 1(class_id) + 32(mask_coeffs)
        results: list[Detection2D] = []

        for i in range(det_out.shape[0]):
            row = det_out[i]
            score = float(row[4])
            if score < self._conf_thr:
                continue

            cid = int(row[5])
            x1 = (float(row[0]) - pad_x) / scale
            y1 = (float(row[1]) - pad_y) / scale
            x2 = (float(row[2]) - pad_x) / scale
            y2 = (float(row[3]) - pad_y) / scale

            x1 = max(0, min(x1, w0))
            y1 = max(0, min(y1, h0))
            x2 = max(0, min(x2, w0))
            y2 = max(0, min(y2, h0))

            bw, bh = x2 - x1, y2 - y1
            if not self._is_box_large_enough(bw, bh):
                continue

            if allowed and cid not in allowed:
                continue
            # Prefer custom vocab labels, then COCO labels.
            if self._custom_vocab and cid in self._custom_vocab:
                label = self._custom_vocab[cid]
            elif cid < len(COCO_NAMES):
                label = COCO_NAMES[cid]
            else:
                label = f"class_{cid}"

            mask = None
            if self._yoloe_proto_name and self._yoloe_proto_name in outputs:
                mask_coeffs = row[6:38]  # 32 coefficients
                proto = outputs[self._yoloe_proto_name][0]  # (32, 160, 160)
                if proto.ndim == 3:
                    ph, pw = proto.shape[1], proto.shape[2]
                    proto_flat = proto.reshape(32, -1)  # (32, 25600)
                    mask_raw = mask_coeffs @ proto_flat  # (25600,)
                    mask_raw = mask_raw.reshape(ph, pw)
                    np.clip(mask_raw, -50, 50, out=mask_raw)
                    mask_prob = 1.0 / (1.0 + np.exp(-mask_raw))
                    bx1 = max(0, int((x1 * scale + pad_x) / 4))
                    by1 = max(0, int((y1 * scale + pad_y) / 4))
                    bx2 = min(pw, int((x2 * scale + pad_x) / 4) + 1)
                    by2 = min(ph, int((y2 * scale + pad_y) / 4) + 1)
                    if bx2 > bx1 and by2 > by1:
                        crop = mask_prob[by1:by2, bx1:bx2]
                        cw, ch = max(1, int(x2 - x1)), max(1, int(y2 - y1))
                        mask_resized = cv2.resize(crop, (cw, ch), interpolation=cv2.INTER_LINEAR)
                        full_mask = np.zeros((h0, w0), dtype=bool)
                        ix1, iy1 = max(0, int(x1)), max(0, int(y1))
                        ix2 = min(w0, ix1 + cw)
                        iy2 = min(h0, iy1 + ch)
                        mw = ix2 - ix1
                        mh = iy2 - iy1
                        if mw > 0 and mh > 0:
                            full_mask[iy1:iy2, ix1:ix2] = mask_resized[:mh, :mw] > 0.5
                            mask = full_mask

            results.append(Detection2D(
                bbox=np.array([x1, y1, x2, y2], dtype=np.float32),
                score=score,
                label=label,
                class_id=cid,
                mask=mask,
            ))

        return self._limit_detection_results(results)
    def _parse_prompt(self, text_prompt: str) -> set:
        """Parse a dot-separated prompt into allowed class ids.

        Matching order:
        1. direct class-name match;
        2. synonym lookup for COCO labels;
        3. substring fallback.
        """
        if text_prompt == self._prompt_cache_key:
            return self._allowed_cids

        self._prompt_cache_key = text_prompt
        labels = [label.strip().lower() for label in text_prompt.split(".") if label.strip()]
        expanded = set(labels)
        for label in labels:
            if label in _SYNONYM_EXPAND:
                expanded.update(_SYNONYM_EXPAND[label])
        labels = list(expanded)
        if not labels:
            self._allowed_cids = set()
            return self._allowed_cids

        if self._custom_vocab:
            name_to_id = {v.lower(): k for k, v in self._custom_vocab.items()}
        else:
            name_to_id = _COCO_NAME_TO_ID

        matched = set()
        for label in labels:
            if label in name_to_id:
                matched.add(name_to_id[label])
                continue
            if not self._custom_vocab and label in _SYNONYMS:
                syn = _SYNONYMS[label]
                if syn in name_to_id:
                    matched.add(name_to_id[syn])
                    continue
            for name, cid in name_to_id.items():
                if label in name or name in label:
                    matched.add(cid)

        self._allowed_cids = matched
        return self._allowed_cids

    # ================================================================
    # ================================================================

    def _preprocess(self, bgr):
        cv2 = _require_cv2()
        h0, w0 = bgr.shape[:2]
        sz = self.INPUT_SIZE
        scale = min(sz / h0, sz / w0)
        nh, nw = int(h0 * scale), int(w0 * scale)
        pad_x, pad_y = (sz - nw) // 2, (sz - nh) // 2

        canvas = np.full((sz, sz, 3), 114, dtype=np.uint8)
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas[pad_y : pad_y + nh, pad_x : pad_x + nw] = resized

        yuv = cv2.cvtColor(canvas, cv2.COLOR_BGR2YUV_I420)
        y_plane = yuv[:sz, :].reshape(1, sz, sz, 1)
        u = yuv[sz : sz + sz // 4, :].reshape(sz // 2, sz // 2)
        v = yuv[sz + sz // 4 :, :].reshape(sz // 2, sz // 2)
        uv_plane = np.stack([u, v], axis=-1).reshape(1, sz // 2, sz // 2, 2)

        return (
            np.ascontiguousarray(y_plane),
            np.ascontiguousarray(uv_plane),
            scale,
            pad_x,
            pad_y,
        )

    # ================================================================
    # ================================================================

    def _postprocess(self, outputs, scale, pad_x, pad_y, orig_h, orig_w):
        all_boxes = []
        all_scores = []
        all_classes = []
        all_mask_coeffs = []

        for i, stride in enumerate(self.STRIDES):
            cls_name, bbox_name, mask_name = self._output_map[i]
            cls_logits = outputs[cls_name][0]
            bbox_raw = outputs[bbox_name][0]
            _H, _W = cls_logits.shape[:2]

            cls_scores = 1.0 / (1.0 + np.exp(-cls_logits.astype(np.float32)))
            max_scores = cls_scores.max(axis=-1)
            mask = max_scores > self._conf_thr
            if not mask.any():
                continue

            ys, xs = np.where(mask)
            valid_cls = cls_scores[ys, xs]
            valid_bbox = bbox_raw[ys, xs]
            valid_max = max_scores[ys, xs]
            valid_cids = valid_cls.argmax(axis=-1)

            valid_mc = None
            if mask_name and mask_name in outputs:
                valid_mc = outputs[mask_name][0][ys, xs]

            if bbox_name in self._bbox_scales:
                bbox_f = valid_bbox.astype(np.float32) * self._bbox_scales[bbox_name]
            else:
                bbox_f = valid_bbox.astype(np.float32)
            bbox_f = bbox_f.reshape(-1, 4, 16)
            bbox_exp = np.exp(bbox_f - bbox_f.max(axis=-1, keepdims=True))
            bbox_sm = bbox_exp / bbox_exp.sum(axis=-1, keepdims=True)
            dist = (bbox_sm * self._dfl_weights).sum(axis=-1)

            cx = (xs.astype(np.float32) + 0.5) * stride
            cy = (ys.astype(np.float32) + 0.5) * stride

            x1 = (cx - dist[:, 0] * stride - pad_x) / scale
            y1 = (cy - dist[:, 1] * stride - pad_y) / scale
            x2 = (cx + dist[:, 2] * stride - pad_x) / scale
            y2 = (cy + dist[:, 3] * stride - pad_y) / scale

            x1 = np.clip(x1, 0, orig_w)
            y1 = np.clip(y1, 0, orig_h)
            x2 = np.clip(x2, 0, orig_w)
            y2 = np.clip(y2, 0, orig_h)

            boxes = np.stack([x1, y1, x2, y2], axis=-1)
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

        keep = self._sort_and_limit_indices(scores, self._nms(boxes, scores, classes))
        results = [(boxes[i], scores[i], classes[i]) for i in keep]
        kept_mc = mask_coeffs[keep] if mask_coeffs is not None else None
        return results, kept_mc

    def _nms(self, boxes, scores, classes):
        keep = []
        for cid in np.unique(classes):
            m = classes == cid
            idx = np.where(m)[0]
            if len(idx) == 0:
                continue
            b = boxes[idx]
            s = scores[idx]
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
        ix1 = np.maximum(box[0], boxes[:, 0])
        iy1 = np.maximum(box[1], boxes[:, 1])
        ix2 = np.minimum(box[2], boxes[:, 2])
        iy2 = np.minimum(box[3], boxes[:, 3])
        inter = np.maximum(ix2 - ix1, 0) * np.maximum(iy2 - iy1, 0)
        a1 = (box[2] - box[0]) * (box[3] - box[1])
        a2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        return inter / np.maximum(a1 + a2 - inter, 1e-6)

    # ================================================================
    # ================================================================

    def _generate_masks(self, raw, kept_mc, outputs, scale, pad_x, pad_y):
        """Decode instance masks from proto tensors and mask coefficients.

        Returns ``None`` masks when coefficients or proto tensors are absent.
        Otherwise applies sigmoid, resizes to each bbox, and clips to the bbox.
        """
        masks_list = [None] * len(raw)
        if kept_mc is None:
            return masks_list

        # Resolve proto tensor name: primary, then override fallback.
        proto_key: str | None = None
        if self._proto_name and self._proto_name in outputs:
            proto_key = self._proto_name
        else:
            override = getattr(self, "_proto_tensor_name_override", None)
            if override and override in outputs:
                proto_key = override

        if proto_key is None:
            if not getattr(self, "_proto_missing_logged", False):
                logger.error(
                    "Proto tensor not found in model outputs (tried '%s' and "
                    "override '%s'). Masks will be None; instance segmentation "
                    "disabled.",
                    self._proto_name,
                    getattr(self, "_proto_tensor_name_override", None),
                )
                self._proto_missing_logged = True
            return masks_list
        # Accept both (ph, pw, 32) and (32, ph, pw) layouts; transpose as needed.
        proto = outputs[proto_key][0]
        # Accept both (ph, pw, 32) and (32, ph, pw) layouts; transpose as needed.
        if proto.ndim == 3 and proto.shape[0] == 32 and proto.shape[-1] != 32:
            proto = np.transpose(proto, (1, 2, 0))
        ph, pw = proto.shape[:2]
        if proto.shape[-1] != kept_mc.shape[-1]:
            logger.error(
                "BPUDetector._generate_masks: proto channels=%d != kept_mc channels=%d",
                proto.shape[-1], kept_mc.shape[-1],
            )
            return masks_list

        proto_flat = proto.reshape(-1, proto.shape[-1])
        all_masks = proto_flat @ kept_mc.T
        np.clip(all_masks, -50, 50, out=all_masks)
        all_masks = 1.0 / (1.0 + np.exp(-all_masks))
        all_masks = all_masks.reshape(ph, pw, -1)

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
            if crop.size == 0:
                mask_resized = np.zeros((ch, cw), dtype=np.float32)
            else:
                try:
                    import cv2 as _cv2
                    mask_resized = _cv2.resize(
                        crop.astype(np.float32), (cw, ch),
                        interpolation=_cv2.INTER_LINEAR,
                    )
                except ImportError:
                    # numpy nearest-neighbour fallback for dev without cv2.
                    ys = (np.arange(ch) * crop.shape[0] / ch).astype(int)
                    xs = (np.arange(cw) * crop.shape[1] / cw).astype(int)
                    mask_resized = crop[np.ix_(ys, xs)]
            masks_list[idx] = (mask_resized > 0.5, ox1, oy1)

        return masks_list
