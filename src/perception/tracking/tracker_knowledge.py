"""KG / Belief enrichment service for InstanceTracker.

Extracted from ``instance_tracker.py`` (Phase 3.5c) to keep the tracker
focused on its core matching / update loop while KG knowledge enrichment,
belief-model loading, and training live in a dedicated, testable module.

Public surface
--------------
- :class:`TrackerKnowledgeEnrichment` – composition-based service that
  operates on the tracker's mutable state (objects dict, knowledge-graph
  reference, belief-model reference).
"""

from __future__ import annotations

import logging
from typing import Any

from .tracked_objects import (
    SAFETY_PRIOR_ALPHA_SCALE,
    SAFETY_THRESHOLDS_INTERACTION,
    SAFETY_THRESHOLDS_NAVIGATION,
    TrackedObject,
)

logger = logging.getLogger(__name__)


class TrackerKnowledgeEnrichment:
    """Knowledge-graph enrichment and belief-model management.

    This service is owned by :class:`InstanceTracker` via composition and
    replaces the inline KG / belief methods that previously lived directly
    on the tracker.

    Parameters
    ----------
    get_objects:
        Callable returning the current ``dict[int, TrackedObject]`` (the
        tracker's internal ``_objects``).  A callable is used so the
        service always sees the latest state without holding a stale
        reference.
    get_knowledge_graph:
        Callable returning the current knowledge-graph instance (or None).
    set_knowledge_graph:
        Callable to mutate the tracker's ``_knowledge_graph`` attribute.
    get_belief_model:
        Callable returning the current belief model (or None).
    set_belief_model:
        Callable to mutate the tracker's ``_belief_model`` attribute.
    queries:
        The :class:`InstanceTrackerQueries` instance so that KG changes
        can be propagated to the query layer.
    """

    def __init__(
        self,
        get_objects: Any,
        get_knowledge_graph: Any,
        set_knowledge_graph: Any,
        get_belief_model: Any,
        set_belief_model: Any,
        queries: Any,
    ) -> None:
        self._get_objects = get_objects
        self._get_kg = get_knowledge_graph
        self._set_kg = set_knowledge_graph
        self._get_belief_model = get_belief_model
        self._set_belief_model = set_belief_model
        self._queries = queries

    # ------------------------------------------------------------------
    #  KG injection
    # ------------------------------------------------------------------

    def set_knowledge_graph(self, kg: Any) -> None:
        """Inject a knowledge graph at runtime.

        Propagates the KG to the query layer and enriches every existing
        tracked object with KG-derived properties.
        """
        self._set_kg(kg)
        self._queries.set_knowledge_graph(kg)
        objects = self._get_objects()
        logger.info(
            "KG injected into InstanceTracker (%d existing objects to enrich)",
            len(objects),
        )
        for obj in objects.values():
            self.enrich_from_kg(obj)

    def enrich_from_kg(self, obj: TrackedObject) -> None:
        """Enrich a single *TrackedObject* with KG properties.

        Uses ConceptBot OPE style property injection plus safety-aware
        differential thresholds.
        """
        kg = self._get_kg()
        if kg is None:
            return
        props = kg.enrich_object_properties(obj.label)
        if props.get("kg_matched"):
            obj.kg_concept_id = props.get("concept_id", "")
            obj.safety_level = props.get("safety_level", "safe")
            obj.affordances = props.get("affordances", [])
            obj.functional_properties = props

            # Safety-Aware Differential Thresholds
            obj.safety_nav_threshold = SAFETY_THRESHOLDS_NAVIGATION.get(obj.safety_level, 0.25)
            obj.safety_interact_threshold = SAFETY_THRESHOLDS_INTERACTION.get(obj.safety_level, 0.40)

            # Protective bias: dangerous objects get a higher initial alpha
            # so they are "believed" sooner and trigger avoidance earlier.
            safety_boost = SAFETY_PRIOR_ALPHA_SCALE.get(obj.safety_level, 1.0)
            if safety_boost > 1.0 and obj.detection_count <= 1:
                obj.belief_alpha += (safety_boost - 1.0) * 0.5

    # ------------------------------------------------------------------
    #  Belief model (KG-BELIEF GCN)
    # ------------------------------------------------------------------

    def load_belief_model(self, path: str) -> bool:
        """Load pre-trained KG-BELIEF GCN weights.

        Returns ``True`` on success.  After loading, the downward phase
        of belief propagation will use GCN inference instead of KG
        table look-up.
        """
        if self._get_kg() is None:
            logger.warning("Cannot load belief model without KG")
            return False
        try:
            from memory.knowledge.belief.network import BeliefPredictor

            predictor = BeliefPredictor.from_kg(self._get_kg(), weights_path=path)
            self._set_belief_model(predictor)
            logger.info("Belief GCN model loaded from %s", path)
            return True
        except Exception as e:
            logger.warning("Failed to load belief model: %s", e)
            return False

    def train_belief_model(
        self,
        num_scenes: int = 5000,
        epochs: int = 50,
        save_path: str | None = None,
    ) -> bool:
        """Train a KG-BELIEF GCN model from synthetic KG data.

        Returns ``True`` on success.  The trained predictor is stored on
        the tracker so that subsequent belief propagation uses it.
        """
        kg = self._get_kg()
        if kg is None:
            logger.warning("Cannot train belief model without KG")
            return False
        try:
            import torch

            from memory.knowledge.belief.network import (
                BeliefPredictor,
                KGBeliefGCN,
                KGSceneGraphDataset,
                SafetyWeightedBCELoss,
                build_affordance_vectors,
                build_cooccurrence_matrix,
                build_object_vocabulary,
                build_room_prior_vectors,
                build_safety_loss_weights,
                build_safety_vector,
            )
            from memory.knowledge.belief.network import (
                BeliefTrainer as BTrainer,
            )

            label2idx, idx2label = build_object_vocabulary(kg)
            C = len(label2idx)
            cooc = build_cooccurrence_matrix(kg, label2idx)
            safety_vec = build_safety_vector(kg, label2idx)
            aff_mat = build_affordance_vectors(kg, label2idx)
            priors = build_room_prior_vectors(kg, label2idx)
            loss_weights = build_safety_loss_weights(kg, label2idx)

            model = KGBeliefGCN(num_objects=C)
            loss_fn = SafetyWeightedBCELoss(torch.tensor(loss_weights))

            n_train = int(num_scenes * 0.8)
            train_ds = KGSceneGraphDataset(
                kg,
                label2idx,
                cooc,
                safety_vec,
                aff_mat,
                priors,
                num_scenes=n_train,
                seed=42,
            )
            val_ds = KGSceneGraphDataset(
                kg,
                label2idx,
                cooc,
                safety_vec,
                aff_mat,
                priors,
                num_scenes=num_scenes - n_train,
                seed=123,
            )

            trainer = BTrainer(model, loss_fn)
            result = trainer.train(train_ds, val_ds, epochs=epochs)

            predictor = BeliefPredictor(
                model,
                label2idx,
                idx2label,
                cooc,
                safety_vec,
                aff_mat,
                priors,
            )
            self._set_belief_model(predictor)

            if save_path:
                predictor.save_weights(save_path)

            logger.info(
                "Belief GCN trained: %d scenes, %d epochs, best_val=%.4f",
                num_scenes,
                epochs,
                result["best_val_loss"],
            )
            return True
        except Exception as e:
            logger.warning("Failed to train belief model: %s", e)
            return False
