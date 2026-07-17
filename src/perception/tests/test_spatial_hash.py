# ruff: noqa: S101  (assert statements are standard in pytest test files)
"""
test_spatial_hash.py

Equivalence tests for the 3D spatial-hash acceleration of
SceneGraphBuilder.compute_spatial_relations.

The spatial hash is a *pure performance optimisation*: for large scenes it
replaces the exact O(n^2) pairwise scan with a candidate-pair generation based
on a 3D grid.  These tests prove the optimisation is behaviour-preserving —
the hash path must produce the *identical* list of relations (same content and
same order) as the brute-force path on the same input.
"""

import random
import time

import numpy as np

from perception.tracking.scene_graph_builder import SceneGraphBuilder
from perception.tracking.tracked_objects import TrackedObject


def _make_obj(
    object_id: int,
    pos: tuple[float, float, float],
    extent: tuple[float, float, float] = (0.2, 0.2, 0.2),
    label: str = "obj",
) -> TrackedObject:
    """Construct a minimal TrackedObject for spatial-relation tests."""
    return TrackedObject(
        object_id=object_id,
        label=label,
        position=np.array(pos, dtype=np.float64),
        best_score=0.9,
        extent=np.array(extent, dtype=np.float64),
    )


def _build_scene(n: int, seed: int) -> dict[int, TrackedObject]:
    """Build a deterministic scene of ``n`` objects with fixed seed.

    Positions are spread across a moderately dense volume so that many pairs
    fall within the relation radius (exercising the hash neighbourhood), while
    the z-range is kept bounded so no relation can form beyond the hash's
    coverage of ``max_relation_dist``.
    """
    rng = random.Random(seed)
    objects: dict[int, TrackedObject] = {}
    for oid in range(n):
        # x/y spread over ~25 m, z bounded to [0, 3] m.
        x = rng.uniform(0.0, 25.0)
        y = rng.uniform(0.0, 25.0)
        z = rng.uniform(0.0, 3.0)
        ext = rng.uniform(0.1, 0.4)
        objects[oid] = _make_obj(oid, (x, y, z), extent=(ext, ext, ext))
    return objects


def _rel_key(rel):
    """Sortable, order-independent key for a SpatialRelation."""
    return (rel.subject_id, rel.relation, rel.object_id, round(rel.distance, 6))


def test_hash_matches_bruteforce_exact_order():
    """Hash path == brute-force path: identical content AND order."""
    objects = _build_scene(n=150, seed=1234)

    builder = SceneGraphBuilder(objects)
    brute = builder.compute_spatial_relations(strategy="bruteforce")
    hashed = builder.compute_spatial_relations(strategy="hash")

    # There must actually be relations, otherwise the test is vacuous.
    assert len(brute) > 0
    # Identical in content and order (list equality via dataclass __eq__).
    assert hashed == brute


def test_auto_path_uses_hash_and_matches_bruteforce():
    """The default auto strategy (>100 objects -> hash) matches brute force."""
    objects = _build_scene(n=180, seed=7)

    builder = SceneGraphBuilder(objects)
    auto = builder.compute_spatial_relations()  # auto -> hash (n>100)
    brute = builder.compute_spatial_relations(strategy="bruteforce")

    assert len(brute) > 0
    assert auto == brute


def test_equivalence_content_sorted():
    """Sorted-content equivalence across several seeds (robust cross-check)."""
    for seed in (0, 1, 42, 2026, 99999):
        objects = _build_scene(n=130, seed=seed)
        builder = SceneGraphBuilder(objects)
        brute = builder.compute_spatial_relations(strategy="bruteforce")
        hashed = builder.compute_spatial_relations(strategy="hash")

        assert sorted(brute, key=_rel_key) == sorted(hashed, key=_rel_key)
        # Order equivalence too.
        assert hashed == brute


def test_small_scene_unchanged_below_threshold():
    """<= threshold uses brute force; auto == forced brute force."""
    objects = _build_scene(n=80, seed=3)
    builder = SceneGraphBuilder(objects)

    auto = builder.compute_spatial_relations()  # n<=100 -> brute force
    brute = builder.compute_spatial_relations(strategy="bruteforce")
    hashed = builder.compute_spatial_relations(strategy="hash")

    assert auto == brute
    # Even forced hash stays equivalent on small scenes.
    assert hashed == brute


def test_hash_candidate_pairs_are_superset_of_close_pairs():
    """Every pair within max_relation_dist must appear as a hash candidate."""
    objects = _build_scene(n=140, seed=555)
    objs = list(objects.values())
    n = len(objs)

    max_relation_dist = 1.5 * 3  # RELATION_NEAR_THRESHOLD * 3
    builder = SceneGraphBuilder(objects)
    candidates = set(builder._hash_candidate_pairs(objs, max_relation_dist))

    positions = np.array([o.position[:3] for o in objs])
    missing = 0
    for i in range(n):
        for j in range(i + 1, n):
            d = float(np.linalg.norm(positions[i] - positions[j]))
            if d <= max_relation_dist and (i, j) not in candidates:
                missing += 1
    assert missing == 0

    # Candidate pairs must be sorted ascending (drives brute-force order).
    cand_list = builder._hash_candidate_pairs(objs, max_relation_dist)
    assert cand_list == sorted(cand_list)


def test_hash_is_not_slower_on_large_scene():
    """Rough (non-strict) check that hashing is competitive on big scenes."""
    objects = _build_scene(n=600, seed=2024)
    builder = SceneGraphBuilder(objects)

    t0 = time.perf_counter()
    brute = builder.compute_spatial_relations(strategy="bruteforce")
    t_brute = time.perf_counter() - t0

    t0 = time.perf_counter()
    hashed = builder.compute_spatial_relations(strategy="hash")
    t_hash = time.perf_counter() - t0

    assert hashed == brute
    # Non-strict: hash should not be dramatically slower than brute force.
    assert t_hash <= t_brute * 5.0 + 0.5
