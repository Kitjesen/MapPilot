#!/usr/bin/env python3
"""Evaluate predicted object tracks against a read-only Habitat identity image."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable, Sequence
from itertools import pairwise
from typing import Any

import numpy as np


def _vote(
    mask: np.ndarray,
    identity_mask: np.ndarray,
    valid_ids: Iterable[int] | None,
) -> tuple[int | None, float, int, int]:
    mask = np.asarray(mask, dtype=bool)
    identity_mask = np.asarray(identity_mask)
    if mask.shape != identity_mask.shape:
        raise ValueError("object and identity masks must have the same shape")
    mask_pixels = int(mask.sum())
    if mask_pixels == 0:
        return None, 0.0, 0, 0
    pixels = identity_mask[mask].astype(np.int64, copy=False)
    if valid_ids is None:
        valid = (pixels > 0) & (pixels != np.iinfo(np.uint32).max)
    else:
        valid = np.isin(pixels, np.fromiter(valid_ids, dtype=np.int64))
    pixels = pixels[valid]
    if not len(pixels):
        return None, 0.0, 0, mask_pixels
    ids, counts = np.unique(pixels, return_counts=True)
    selected = int(np.argmax(counts))
    return int(ids[selected]), float(counts[selected] / mask_pixels), int(counts[selected]), mask_pixels


def oracle_id(
    mask: np.ndarray,
    identity_mask: np.ndarray,
    minimum_fraction: float = 0.6,
    *,
    valid_ids: Iterable[int] | None = None,
) -> int | None:
    """Return the majority Habitat object ID when it covers enough predicted pixels."""
    identity, fraction, _, _ = _vote(mask, identity_mask, valid_ids)
    return identity if identity is not None and fraction >= minimum_fraction else None


def _field(value: Any, name: str) -> Any:
    return value[name] if isinstance(value, dict) else getattr(value, name)


def evaluate_masks(
    observations: Sequence[Any],
    matches: Sequence[Any],
    identity_mask: np.ndarray,
    *,
    valid_ids: Iterable[int] | None = None,
    minimum_fraction: float = 0.6,
) -> list[dict[str, Any]]:
    """Attach oracle identities after tracking without changing tracker inputs or scores."""
    if len(observations) != len(matches):
        raise ValueError("projected observations and tracker matches must align")
    valid_ids = tuple(valid_ids) if valid_ids is not None else None
    records = []
    for observation, match in zip(observations, matches):
        identity, fraction, identity_pixels, mask_pixels = _vote(
            _field(observation, "mask"),
            identity_mask,
            valid_ids,
        )
        accepted = identity if identity is not None and fraction >= minimum_fraction else None
        records.append({
            "track_id": int(_field(match, "object_id")),
            "oracle_id": accepted,
            "oracle_majority_id": identity,
            "oracle_pixel_fraction": fraction,
            "oracle_pixels": identity_pixels,
            "mask_pixels": mask_pixels,
        })
    return records


def association_report(
    records: Sequence[dict[str, Any]],
    *,
    minimum_fraction: float = 0.6,
) -> dict[str, Any]:
    """Summarize identity switches, mixed tracks, oracle splits, and label coverage."""
    by_track: dict[int, list[int]] = defaultdict(list)
    by_oracle: dict[int, set[int]] = defaultdict(set)
    for record in records:
        identity = record.get("oracle_id")
        if identity is None:
            continue
        track_id = int(record["track_id"])
        identity = int(identity)
        by_track[track_id].append(identity)
        by_oracle[identity].add(track_id)
    switches = {
        str(track_id): sum(left != right for left, right in pairwise(ids))
        for track_id, ids in by_track.items()
    }
    mixed = {
        str(track_id): sorted(set(ids))
        for track_id, ids in by_track.items()
        if len(set(ids)) > 1
    }
    split = {
        str(identity): sorted(track_ids)
        for identity, track_ids in by_oracle.items()
        if len(track_ids) > 1
    }
    attributed = sum(record.get("oracle_id") is not None for record in records)
    return {
        "minimum_oracle_pixel_fraction": minimum_fraction,
        "observations": len(records),
        "attributed_observations": attributed,
        "ambiguous_observations": len(records) - attributed,
        "attribution_coverage": attributed / len(records) if records else 0.0,
        "identity_switches": switches,
        "identity_switch_count": sum(switches.values()),
        "mixed_track_oracle_ids": mixed,
        "split_oracle_track_ids": split,
    }
