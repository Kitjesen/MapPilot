"""Query API for InstanceTracker — open-vocabulary, spatial, affordance, embedding queries.

Extracted from ``instance_tracker.py`` (Phase 3.5b) to keep the tracker focused
on the update/match pipeline while queries are organised in a dedicated module.

All public methods mirror the original ``InstanceTracker`` signatures so the
tracker can delegate transparently.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import numpy as np

from .tracked_objects import (
    REGION_CLUSTER_RADIUS,
    RELATION_NEAR_THRESHOLD,
    TrackedObject,
)

if TYPE_CHECKING:
    from .scene_graph_builder import SceneGraphBuilder

logger = logging.getLogger(__name__)


class InstanceTrackerQueries:
    """Standalone query engine for tracked-object stores.

    The class is intentionally decoupled from ``InstanceTracker`` — it only
    requires the minimal set of collaborators (object dict, optional KG,
    scene-graph builder) passed at construction time.
    """

    def __init__(
        self,
        objects: dict[int, TrackedObject],
        *,
        scene_graph_builder: SceneGraphBuilder,
        knowledge_graph: Any = None,
    ) -> None:
        self._objects = objects
        self._scene_graph_builder = scene_graph_builder
        self._knowledge_graph = knowledge_graph

        # Embedding index cache — rebuilt lazily by *build_embedding_index*.
        self._embedding_index: np.ndarray | None = None
        self._embedding_ids: list[int] = []

    # ------------------------------------------------------------------
    #  Helpers to keep references in sync with the owning tracker
    # ------------------------------------------------------------------

    def set_knowledge_graph(self, kg: Any) -> None:
        """Replace the knowledge-graph reference."""
        self._knowledge_graph = kg

    # ------------------------------------------------------------------
    #  Open-vocabulary text query  (EmbodiedRAG + CLIP)
    # ------------------------------------------------------------------

    def query_by_text(
        self,
        query: str,
        top_k: int = 5,
        clip_encoder: Any = None,
    ) -> list[TrackedObject]:
        """Match tracked objects by free-form text.

        Strategy cascade (LOVON / ConceptGraphs inspired):
          1. CLIP text-image cosine similarity (when *clip_encoder* provided).
          2. KG alias lookup (synonym expansion).
          3. Plain substring match (ultimate fallback).
        """
        objects = list(self._objects.values())
        if not objects:
            return []

        # -- 1. CLIP semantic match (precise) --
        if clip_encoder is not None:
            features = [obj.features for obj in objects]
            has_features = any(f.size > 0 for f in features)

            if has_features:
                similarities = clip_encoder.text_image_similarity(query, features)
                scored = list(zip(objects, similarities))
                scored.sort(key=lambda x: x[1], reverse=True)
                return [obj for obj, sim in scored[:top_k] if sim > 0.2]

        # -- 2. KG alias match --
        if self._knowledge_graph is not None:
            concept = self._knowledge_graph.lookup(query)
            if concept is not None:
                all_names = [n.lower() for n in concept.names_en + concept.names_zh]
                kg_matches = [
                    obj
                    for obj in objects
                    if obj.label.lower() in all_names or any(n in obj.label.lower() for n in all_names)
                ]
                if kg_matches:
                    kg_matches.sort(key=lambda o: o.credibility, reverse=True)
                    return kg_matches[:top_k]

        # -- 3. String match (fallback) --
        query_lower = query.lower()
        matches: list[TrackedObject] = []
        for obj in objects:
            if query_lower in obj.label.lower() or obj.label.lower() in query_lower:
                matches.append(obj)
        matches.sort(key=lambda o: o.best_score, reverse=True)
        return matches[:top_k]

    # ------------------------------------------------------------------
    #  Spatial query  (EmbodiedRAG + SG-Nav spatial reasoning)
    # ------------------------------------------------------------------

    def query_spatial(
        self,
        target: str,
        spatial_hint: str = "",
        anchor: str = "",
        top_k: int = 5,
        clip_encoder: Any = None,
    ) -> list[TrackedObject]:
        """Spatially-aware query: combine text match with spatial relations.

        Supports patterns like *"fire extinguisher near the door"* by first
        resolving candidates via :meth:`query_by_text`, then re-ranking with
        a proximity score against the anchor object.
        """
        candidates = self.query_by_text(target, top_k=50, clip_encoder=clip_encoder)
        if not candidates:
            return []

        if not spatial_hint or not anchor:
            return candidates[:top_k]

        anchor_objs = self.query_by_text(anchor, top_k=10, clip_encoder=clip_encoder)
        if not anchor_objs:
            return candidates[:top_k]

        relations = self._scene_graph_builder.compute_spatial_relations()

        scored: list[tuple[TrackedObject, float]] = []
        for cand in candidates:
            spatial_score = 0.0
            for anch in anchor_objs:
                for rel in relations:
                    if rel.relation == spatial_hint:
                        if (rel.subject_id == cand.object_id and rel.object_id == anch.object_id) or (
                            rel.object_id == cand.object_id and rel.subject_id == anch.object_id
                        ):
                            spatial_score = max(spatial_score, 1.0 - min(rel.distance / 5.0, 0.8))

                dist = float(np.linalg.norm(cand.position - anch.position))
                if dist < RELATION_NEAR_THRESHOLD * 2:
                    proximity_score = 1.0 - dist / (RELATION_NEAR_THRESHOLD * 2)
                    spatial_score = max(spatial_score, proximity_score * 0.5)

            final_score = 0.6 * cand.credibility + 0.4 * spatial_score
            scored.append((cand, final_score))

        scored.sort(key=lambda x: x[1], reverse=True)
        return [obj for obj, _ in scored[:top_k]]

    # ------------------------------------------------------------------
    #  Affordance query  (OpenFunGraph)
    # ------------------------------------------------------------------

    def query_by_affordance(
        self,
        affordance: str,
        top_k: int = 10,
    ) -> list[TrackedObject]:
        """Query by affordance tag (e.g. ``graspable``, ``sittable``)."""
        matches = [obj for obj in self._objects.values() if affordance in obj.affordances]
        matches.sort(key=lambda o: o.credibility, reverse=True)
        return matches[:top_k]

    # ------------------------------------------------------------------
    #  Safety-level query
    # ------------------------------------------------------------------

    def query_by_safety(
        self,
        safety_level: str = "dangerous",
    ) -> list[TrackedObject]:
        """Return all objects with the given *safety_level*."""
        return [obj for obj in self._objects.values() if obj.safety_level == safety_level]

    # ------------------------------------------------------------------
    #  Floor-level query  (SPADE hierarchical planning)
    # ------------------------------------------------------------------

    def query_by_floor(
        self,
        floor_level: int,
        label: str | None = None,
        top_k: int = 20,
    ) -> list[TrackedObject]:
        """Filter objects by floor level, optionally narrowing by label."""
        matches = [obj for obj in self._objects.values() if obj.floor_level == floor_level]
        if label:
            label_lower = label.lower()
            matches = [obj for obj in matches if label_lower in obj.label.lower() or obj.label.lower() in label_lower]
        matches.sort(key=lambda o: o.credibility, reverse=True)
        return matches[:top_k]

    # ------------------------------------------------------------------
    #  Task-relevant sub-graph extraction  (EmbodiedRAG)
    # ------------------------------------------------------------------

    def extract_subgraph_for_task(
        self,
        target: str,
        max_nodes: int = 30,
        clip_encoder: Any = None,
    ) -> dict:
        """Extract a compact sub-graph around *target* for LLM prompting.

        Instead of sending the full scene graph, only the task-relevant
        neighbourhood is included — reducing LLM tokens ~10x (EmbodiedRAG
        2024 core contribution).
        """
        target_objs = self.query_by_text(target, top_k=5, clip_encoder=clip_encoder)

        relevant_ids: set[int] = set()
        for obj in target_objs:
            relevant_ids.add(obj.object_id)

        relations = self._scene_graph_builder.compute_spatial_relations()
        for rel in relations:
            if rel.subject_id in relevant_ids or rel.object_id in relevant_ids:
                relevant_ids.add(rel.subject_id)
                relevant_ids.add(rel.object_id)

        if len(relevant_ids) < max_nodes:
            for obj in target_objs:
                for other in self._objects.values():
                    if other.object_id in relevant_ids:
                        continue
                    dist = float(np.linalg.norm(obj.position[:2] - other.position[:2]))
                    if dist < REGION_CLUSTER_RADIUS:
                        relevant_ids.add(other.object_id)
                    if len(relevant_ids) >= max_nodes:
                        break

        sub_objects: list[dict] = []
        for oid in relevant_ids:
            obj = self._objects.get(oid)
            if obj is None:
                continue
            entry: dict[str, Any] = {
                "id": obj.object_id,
                "label": obj.label,
                "position": {
                    "x": round(float(obj.position[0]), 2),
                    "y": round(float(obj.position[1]), 2),
                    "z": round(float(obj.position[2]), 2),
                },
                "credibility": round(obj.credibility, 2),
                "floor": obj.floor_level,
            }
            if obj.kg_concept_id:
                entry["safety"] = obj.safety_level
                entry["affordances"] = obj.affordances
            sub_objects.append(entry)

        sub_relations = [
            {
                "subject_id": r.subject_id,
                "relation": r.relation,
                "object_id": r.object_id,
                "distance": r.distance,
            }
            for r in relations
            if r.subject_id in relevant_ids and r.object_id in relevant_ids
        ]

        kg_notes: list[str] = []
        if self._knowledge_graph is not None:
            for obj in target_objs:
                constraint = self._knowledge_graph.check_safety(obj.label, "approach")
                if constraint:
                    kg_notes.append(constraint.message_en)
                locations = self._knowledge_graph.get_typical_locations(obj.label)
                if locations:
                    kg_notes.append(f"{obj.label} typically found in: {', '.join(locations[:3])}")

        return {
            "target": target,
            "subgraph_nodes": len(sub_objects),
            "objects": sub_objects,
            "relations": sub_relations,
            "kg_notes": kg_notes,
        }

    # ------------------------------------------------------------------
    #  CLIP embedding index  (EmbodiedRAG accelerated retrieval)
    # ------------------------------------------------------------------

    def build_embedding_index(self) -> bool:
        """Build a normalised CLIP-feature matrix for fast batch cosine queries.

        Returns ``True`` when the index was built, ``False`` when no objects
        carry features.
        """
        objects_with_features = [obj for obj in self._objects.values() if obj.features.size > 0]
        if not objects_with_features:
            self._embedding_index = None
            self._embedding_ids = []
            return False

        features = np.stack([obj.features for obj in objects_with_features])
        norms = np.linalg.norm(features, axis=1, keepdims=True)
        norms = np.where(norms > 0, norms, 1.0)
        self._embedding_index = features / norms
        self._embedding_ids = [obj.object_id for obj in objects_with_features]
        return True

    def query_by_embedding(
        self,
        query_embedding: np.ndarray,
        top_k: int = 5,
        min_similarity: float = 0.2,
    ) -> list[tuple[TrackedObject, float]]:
        """Batch cosine-similarity query against the embedding index.

        10-50x faster than per-object similarity in :meth:`query_by_text`.
        """
        if self._embedding_index is None or len(self._embedding_ids) == 0:
            self.build_embedding_index()
        if self._embedding_index is None or len(self._embedding_ids) == 0:
            return []

        q = np.asarray(query_embedding, dtype=np.float64)
        q_norm = np.linalg.norm(q)
        if q_norm > 0:
            q = q / q_norm

        sims = self._embedding_index @ q
        top_indices = np.argsort(sims)[::-1][:top_k]

        results: list[tuple[TrackedObject, float]] = []
        for idx in top_indices:
            sim = float(sims[idx])
            if sim < min_similarity:
                break
            oid = self._embedding_ids[idx]
            obj = self._objects.get(oid)
            if obj is not None:
                results.append((obj, sim))

        return results

    # ------------------------------------------------------------------
    #  Open-vocabulary fused query
    # ------------------------------------------------------------------

    def get_open_vocabulary_matches(
        self,
        query: str,
        clip_encoder: Any = None,
        top_k: int = 5,
    ) -> list[dict]:
        """Full-pipeline open-vocabulary query fusing CLIP + KG + string signals.

        Returns scored match dicts with keys: ``object_id``, ``label``,
        ``position``, ``clip_similarity``, ``kg_match``, ``credibility``,
        ``score``.
        """
        results: list[dict] = []

        # Signal 1: CLIP embedding match
        clip_matches: list[tuple[TrackedObject, float]] = []
        if clip_encoder is not None:
            try:
                q_embedding = clip_encoder.encode_text(query)
                clip_matches = self.query_by_embedding(
                    q_embedding,
                    top_k=top_k * 2,
                    min_similarity=0.15,
                )
            except Exception as e:
                logger.warning("CLIP query failed: %s", e)

        for obj, sim in clip_matches:
            results.append(
                {
                    "object_id": obj.object_id,
                    "label": obj.label,
                    "position": obj.position.tolist(),
                    "clip_similarity": round(sim, 3),
                    "kg_match": bool(obj.kg_concept_id),
                    "credibility": round(obj.credibility, 3),
                    "score": round(0.5 * sim + 0.3 * obj.credibility + 0.2 * (1.0 if obj.kg_concept_id else 0.0), 3),
                }
            )

        # Signal 2: KG alias match
        if self._knowledge_graph is not None:
            concept = self._knowledge_graph.lookup(query)
            if concept is not None:
                all_names = set(n.lower() for n in concept.names_en + concept.names_zh)
                for obj in self._objects.values():
                    if obj.label.lower() in all_names or any(n in obj.label.lower() for n in all_names):
                        existing = next((r for r in results if r["object_id"] == obj.object_id), None)
                        if existing is None:
                            results.append(
                                {
                                    "object_id": obj.object_id,
                                    "label": obj.label,
                                    "position": obj.position.tolist(),
                                    "clip_similarity": 0.0,
                                    "kg_match": True,
                                    "credibility": round(obj.credibility, 3),
                                    "score": round(0.6 * obj.credibility + 0.4, 3),
                                }
                            )
                        elif existing is not None:
                            existing["kg_match"] = True
                            existing["score"] = min(1.0, existing["score"] + 0.2)

        # Signal 3: String fallback
        query_lower = query.lower()
        for obj in self._objects.values():
            if query_lower in obj.label.lower() or obj.label.lower() in query_lower:
                existing = next((r for r in results if r["object_id"] == obj.object_id), None)
                if existing is None:
                    results.append(
                        {
                            "object_id": obj.object_id,
                            "label": obj.label,
                            "position": obj.position.tolist(),
                            "clip_similarity": 0.0,
                            "kg_match": bool(obj.kg_concept_id),
                            "credibility": round(obj.credibility, 3),
                            "score": round(0.5 * obj.credibility + 0.3, 3),
                        }
                    )

        results.sort(key=lambda r: r["score"], reverse=True)
        return results[:top_k]
