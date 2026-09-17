import base64
import io
import json

import numpy as np
import pytest
from PIL import Image as PILImage

from decision.semantic_navigation.observation import ObjectTarget
from decision.semantic_navigation.verification import VerificationSample, parse_verdict, verification_messages
from decision.vision import vlm_scene


@pytest.mark.parametrize("fallback", [False, True])
def test_verification_jpeg_preserves_red_with_both_encoders(monkeypatch, fallback):
    if fallback:
        monkeypatch.setattr(vlm_scene, "_encode_image", lambda image: None)
    bgr = np.zeros((16, 16, 3), dtype=np.uint8)
    bgr[:, :, 2] = 255
    encoded = vlm_scene.encode_image_b64(bgr)
    decoded = PILImage.open(io.BytesIO(base64.b64decode(encoded))).convert("RGB")
    red, green, blue = decoded.getpixel((8, 8))
    assert red > 220 and green < 20 and blue < 20


def test_verification_contains_context_crop_and_original_constraints():
    sample = VerificationSample(100, (2, 0, 1), (1.5, 0, 0.3), (4, 3, 12, 10),
                                np.zeros((16, 20, 3), dtype=np.uint8))
    messages = verification_messages("find the red chair beside the desk", ObjectTarget("42", "chair", (2, 0, 1)), sample)
    content = messages[1]["content"]
    assert json.loads(content[0]["text"])["instruction"] == "find the red chair beside the desk"
    assert "ALL requested" in messages[0]["content"]
    images = [PILImage.open(io.BytesIO(base64.b64decode(item["image_url"]["url"].split(",", 1)[1])))
              for item in content[1:]]
    assert [image.size for image in images] == [(20, 16), (8, 7)]


@pytest.mark.parametrize("payload", [
    '{"target_id":"other","verdict":"match","reason":"yes"}',
    '{"target_id":"42","verdict":true,"reason":"yes"}',
    '{"target_id":"42","verdict":"match","reason":""}',
    'true',
    'go forward now',
])
def test_invalid_visual_verdict_is_not_success(payload):
    with pytest.raises((ValueError, TypeError)):
        parse_verdict(payload, "42")
