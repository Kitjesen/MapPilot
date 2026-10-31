from __future__ import annotations

from types import SimpleNamespace

import torch
import torch.nn.functional as F
from tools.inference.export_follow import FixedPersonContrastiveHead, fix_person_contrastive_head


class _ContrastiveHead(torch.nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.logit_scale = torch.nn.Parameter(torch.tensor(2.0))
        self.bias = torch.nn.Parameter(torch.tensor([-3.0]))

    def forward(self, image_features: torch.Tensor, text: torch.Tensor) -> torch.Tensor:
        image_features = F.normalize(image_features, dim=1, p=2)
        text = F.normalize(text, dim=-1, p=2)
        similarity = torch.einsum("bchw,bkc->bkhw", image_features, text)
        return similarity * self.logit_scale.exp() + self.bias


class _BNContrastiveHead(_ContrastiveHead):
    def __init__(self) -> None:
        super().__init__()
        self.norm = torch.nn.BatchNorm2d(8).eval()

    def forward(self, image_features: torch.Tensor, text: torch.Tensor) -> torch.Tensor:
        image_features = self.norm(image_features)
        text = F.normalize(text, dim=-1, p=2)
        similarity = torch.einsum("bchw,bkc->bkhw", image_features, text)
        return similarity * self.logit_scale.exp() + self.bias


def test_fixed_person_head_matches_contrastive_formula() -> None:
    source = _ContrastiveHead()
    text = torch.randn(1, 1, 8)
    image = torch.randn(2, 8, 5, 7)

    fixed = FixedPersonContrastiveHead(source, text[0, 0])

    assert torch.allclose(source(image, text.repeat(2, 1, 1)), fixed(image, text), atol=1e-6)


def test_fixed_person_head_matches_batchnorm_contrastive_formula() -> None:
    source = _BNContrastiveHead()
    text = torch.randn(1, 1, 8)
    image = torch.randn(2, 8, 5, 7)

    fixed = FixedPersonContrastiveHead(source, text[0, 0])

    assert torch.allclose(source(image, text.repeat(2, 1, 1)), fixed(image, text), atol=1e-6)


def test_fix_person_head_replaces_every_detection_scale() -> None:
    source_heads = torch.nn.ModuleList([_ContrastiveHead(), _ContrastiveHead(), _ContrastiveHead()])
    detection_head = SimpleNamespace(cv4=source_heads)
    world_model = SimpleNamespace(txt_feats=torch.randn(1, 1, 8), model=[detection_head])
    wrapper = SimpleNamespace(model=world_model)

    classification_head = fix_person_contrastive_head(wrapper)

    assert all(isinstance(head, FixedPersonContrastiveHead) for head in source_heads)
    assert classification_head == "fixed_person_l2_conv1x1"
