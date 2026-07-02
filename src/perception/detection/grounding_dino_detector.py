"""
GroundingDINO 开放词汇检测器实现。

依赖: third_party/GroundingDINO (需要先 pip install -e)
"""

import logging
import os

import numpy as np

from .detector_base import Detection2D, DetectorBase

logger = logging.getLogger(__name__)


class GroundingDINODetector(DetectorBase):
    """
    GroundingDINO 检测器包装。

    使用 GroundingDINO 的 predict() API 进行文本提示的目标检测。
    """

    def __init__(
        self,
        config_path: str | None = None,
        weights_path: str | None = None,
        box_threshold: float = 0.35,
        text_threshold: float = 0.25,
        device: str = "cuda",
    ):
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.device = device
        self._model = None

        # 自动查找路径
        third_party = os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "..", "third_party"
        )
        gdino_dir = os.path.join(third_party, "GroundingDINO")

        if config_path and os.path.isfile(config_path):
            self._config_path = config_path
        else:
            self._config_path = os.path.join(
                gdino_dir,
                "groundingdino", "config", "GroundingDINO_SwinB_cfg.py",
            )

        if weights_path and os.path.isfile(weights_path):
            self._weights_path = weights_path
        else:
            self._weights_path = os.path.join(
                gdino_dir, "weights", "groundingdino_swinb_cogcoor.pth"
            )

    def load_model(self) -> None:
        """加载 GroundingDINO 模型。"""
        try:
            from groundingdino.util.inference import load_model, predict
            self._predict_fn = predict

            logger.info(
                "Loading GroundingDINO: config=%s, weights=%s, device=%s",
                self._config_path, self._weights_path, self.device,
            )
            self._model = load_model(
                self._config_path, self._weights_path, device=self.device
            )
            logger.info("GroundingDINO model loaded successfully")
        except ImportError:
            logger.error(
                "GroundingDINO not installed. Run: "
                "cd third_party/GroundingDINO && pip install -e ."
            )
            raise
        except Exception as e:
            logger.error("Failed to load GroundingDINO: %s", e)
            raise

    def detect(self, rgb: np.ndarray, text_prompt: str) -> list[Detection2D]:
        """
        使用 GroundingDINO 进行检测。

        Args:
            rgb: HxWx3 uint8 BGR 图像
            text_prompt: 文本提示, 多类别用 " . " 分隔

        Returns:
            Detection2D 列表
        """
        if self._model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        import groundingdino.datasets.transforms as T
        from PIL import Image

        # BGR → RGB → PIL
        rgb_image = rgb[:, :, ::-1] if rgb.shape[2] == 3 else rgb
        pil_image = Image.fromarray(rgb_image)

        # GroundingDINO 预处理
        transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        image_tensor, _ = transform(pil_image, None)

        # 推理
        boxes, logits, phrases = self._predict_fn(
            model=self._model,
            image=image_tensor,
            caption=text_prompt,
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.device,
        )

        h, w = rgb.shape[:2]
        detections: list[Detection2D] = []

        for box, score, label in zip(boxes, logits, phrases):
            # box: center_x, center_y, w, h (normalized)
            cx, cy, bw, bh = box.tolist()
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)

            detections.append(Detection2D(
                bbox=np.array([x1, y1, x2, y2]),
                score=float(score),
                label=label.strip(),
            ))

        return detections

    def shutdown(self) -> None:
        """释放模型资源。"""
        self._model = None
        import torch
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        logger.info("GroundingDINO detector shut down")
