"""KG-BELIEF: Neuro-Symbolic GCN for Belief Scene Graphs.

Reference papers:
  - Belief Scene Graphs (ICRA 2024, arXiv:2402.03840)
  - Commonsense BSG (2025, arXiv:2505.02405)
  - HOV-SG (RSS 2024, arXiv:2403.17846)

Key differences from BSG:
  - Input: 5 KG-augmented channels (292-dim) vs BSG's 1 histogram (45-dim)
  - Loss: Safety-weighted BCE (dangerous objects penalized 3x) vs uniform BCE
  - Training data: KG-synthesized scenes vs HM3D dataset
  - Co-occurrence: KG-structured prior vs learned-from-scratch
"""

from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    from torch.utils.data import Dataset, DataLoader

    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

# Alias for backward compatibility with tests that import _TORCH_AVAILABLE
_TORCH_AVAILABLE = HAS_TORCH

logger = logging.getLogger(__name__)

# ════════════════════════════════════════════════════════════
#  Constants
# ════════════════════════════════════════════════════════════

ROOM_TYPES: List[str] = [
    "office", "kitchen", "corridor", "meeting_room", "bathroom",
    "stairwell", "lobby", "storage", "server_room", "warehouse",
    "lab", "parking", "outdoor", "elevator_hall", "factory",
    "hospital", "entrance", "utility_room", "bedroom", "living_room",
    "break_room", "laundry",
]

SAFETY_ENCODING = {"safe": 0.0, "caution": 0.33, "dangerous": 0.67, "forbidden": 1.0}
SAFETY_LOSS_WEIGHT = {"safe": 1.0, "caution": 1.5, "dangerous": 3.0, "forbidden": 5.0}


# ════════════════════════════════════════════════════════════
#  Vocabulary & Feature Builders (KG → tensor)
# ════════════════════════════════════════════════════════════

def build_object_vocabulary(kg) -> Tuple[Dict[str, int], List[str]]:
    """Build a label→index mapping from KG room-expected objects.

    Returns (label2idx dict, idx2label list). Sorted for determinism.
    """
    all_labels: set = set()
    for rt in ROOM_TYPES:
        all_labels.update(kg.get_room_expected_objects(rt))
    idx2label = sorted(all_labels)
    label2idx = {lbl: i for i, lbl in enumerate(idx2label)}
    return label2idx, idx2label


def build_cooccurrence_matrix(kg, label2idx: Dict[str, int]) -> np.ndarray:
    """Compute C x C co-occurrence matrix from KG room-type mappings.

    M[i][j] = number of room types where both object i and j are expected,
    normalized by total room types. This encodes KG structural knowledge
    that BSG's GCN must learn from data.
    """
    C = len(label2idx)
    M = np.zeros((C, C), dtype=np.float32)
    for rt in ROOM_TYPES:
        expected = kg.get_room_expected_objects(rt)
        indices = [label2idx[lbl] for lbl in expected if lbl in label2idx]
        for i in indices:
            for j in indices:
                M[i, j] += 1.0
    M /= max(len(ROOM_TYPES), 1)
    return M


def build_safety_vector(kg, label2idx: Dict[str, int]) -> np.ndarray:
    """Build per-object safety encoding vector (R^C)."""
    C = len(label2idx)
    vec = np.zeros(C, dtype=np.float32)
    for lbl, idx in label2idx.items():
        props = kg.enrich_object_properties(lbl)
        level = props.get("safety_level", "safe")
        vec[idx] = SAFETY_ENCODING.get(level, 0.0)
    return vec


def build_safety_loss_weights(kg, label2idx: Dict[str, int]) -> np.ndarray:
    """Build per-object loss weight vector for safety-weighted BCE."""
    C = len(label2idx)
    weights = np.ones(C, dtype=np.float32)
    for lbl, idx in label2idx.items():
        props = kg.enrich_object_properties(lbl)
        level = props.get("safety_level", "safe")
        weights[idx] = SAFETY_LOSS_WEIGHT.get(level, 1.0)
    return weights


def build_affordance_vectors(kg, label2idx: Dict[str, int]) -> np.ndarray:
    """Build C x A affordance matrix (A = number of affordance types)."""
    affordance_types = [
        "graspable", "openable", "sittable", "inspectable", "pushable",
        "containable", "closable", "passable", "readable", "switchable",
        "climbable", "supportive",
    ]
    aff2idx = {a: i for i, a in enumerate(affordance_types)}
    A = len(affordance_types)
    C = len(label2idx)
    mat = np.zeros((C, A), dtype=np.float32)
    for lbl, idx in label2idx.items():
        props = kg.enrich_object_properties(lbl)
        for aff in props.get("affordances", []):
            if aff in aff2idx:
                mat[idx, aff2idx[aff]] = 1.0
    return mat


NUM_AFFORDANCES = 12
# Alias for backward compatibility
NUM_AFFORDANCE_TYPES = NUM_AFFORDANCES

# Alias: build_affordance_matrix → build_affordance_vectors
build_affordance_matrix = build_affordance_vectors


def build_dangerous_mask(kg, label2idx: Dict[str, int]) -> np.ndarray:
    """Build boolean mask: True where object safety_level >= 'dangerous'."""
    safety_vec = build_safety_vector(kg, label2idx)
    return safety_vec >= SAFETY_ENCODING["dangerous"]


def build_room_prior_vectors(kg, label2idx: Dict[str, int]) -> Dict[str, np.ndarray]:
    """Build per-room-type prior vector (R^C) for each room type."""
    C = len(label2idx)
    priors: Dict[str, np.ndarray] = {}
    for rt in ROOM_TYPES:
        vec = np.zeros(C, dtype=np.float32)
        for lbl in kg.get_room_expected_objects(rt):
            if lbl in label2idx:
                vec[label2idx[lbl]] = 1.0
        priors[rt] = vec
    return priors


# ════════════════════════════════════════════════════════════
#  GCN Layer (hand-written, no torch_geometric)
# ════════════════════════════════════════════════════════════

if HAS_TORCH:

    class GCNConv(nn.Module):
        """Graph Convolutional layer: X' = sigma(D^{-1/2} A_hat D^{-1/2} X W).

        A_hat = A + I (self-loops). Implements Kipf & Welling (ICLR 2017)
        without torch_geometric dependency.
        """

        def __init__(self, in_features: int, out_features: int, bias: bool = True):
            super().__init__()
            self.weight = nn.Parameter(torch.empty(in_features, out_features))
            self.bias = nn.Parameter(torch.zeros(out_features)) if bias else None
            nn.init.xavier_uniform_(self.weight)

        def forward(self, x: torch.Tensor, adj: torch.Tensor) -> torch.Tensor:
            """Forward pass.

            Args:
                x: Node features (N, in_features)
                adj: Adjacency matrix (N, N), will be symmetrically normalized
            """
            a_hat = adj + torch.eye(adj.size(0), device=adj.device)
            d_inv_sqrt = torch.diag(1.0 / torch.sqrt(a_hat.sum(dim=1).clamp(min=1e-8)))
            a_norm = d_inv_sqrt @ a_hat @ d_inv_sqrt

            out = a_norm @ x @ self.weight
            if self.bias is not None:
                out = out + self.bias
            return out

    # ════════════════════════════════════════════════════════════
    #  KG-BELIEF GCN Model
    # ════════════════════════════════════════════════════════════

    class KGBeliefGCN(nn.Module):
        """Neuro-Symbolic GCN for Belief Scene Graph prediction.

        Architecture (per plan):
          Layer 1: GCNConv(input_dim, 128) + BN + ReLU + Dropout
          Layer 2: GCNConv(128, 128) + BN + ReLU + Dropout
          Layer 3: GCNConv(128, 64) + BN + ReLU + Dropout
          Layer 4: Linear(64, num_objects) -> sigmoid

        Input features per room node (292-dim with C=70, A=12):
          - observed_histogram (R^C): binary
          - kg_room_prior (R^C): soft prior from KG
          - kg_cooccurrence (R^C): aggregated co-occurrence
          - safety_encoding (R^C): per-object safety level
          - affordance_agg (R^A): aggregated affordance features
        """

        def __init__(self, num_objects: int, num_affordances: int = NUM_AFFORDANCES,
                     hidden_dim: int = 128, dropout: float = 0.3):
            super().__init__()
            input_dim = num_objects * 4 + num_affordances  # h + k + cooc + safety + aff

            self.gcn1 = GCNConv(input_dim, hidden_dim)
            self.bn1 = nn.BatchNorm1d(hidden_dim)

            self.gcn2 = GCNConv(hidden_dim, hidden_dim)
            self.bn2 = nn.BatchNorm1d(hidden_dim)

            self.gcn3 = GCNConv(hidden_dim, hidden_dim // 2)
            self.bn3 = nn.BatchNorm1d(hidden_dim // 2)

            self.head = nn.Linear(hidden_dim // 2, num_objects)
            self.dropout = nn.Dropout(dropout)

            self.num_objects = num_objects
            self.num_affordances = num_affordances

        def forward(self, x: torch.Tensor, adj: torch.Tensor) -> torch.Tensor:
            """Forward pass.

            Args:
                x: Node features (N, input_dim)
                adj: Adjacency matrix (N, N)
            Returns:
                Predicted object probabilities (N, num_objects)
            """
            h = self.gcn1(x, adj)
            h = self.bn1(h)
            h = F.relu(h)
            h = self.dropout(h)

            h = self.gcn2(h, adj)
            h = self.bn2(h)
            h = F.relu(h)
            h = self.dropout(h)

            h = self.gcn3(h, adj)
            h = self.bn3(h)
            h = F.relu(h)
            h = self.dropout(h)

            out = torch.sigmoid(self.head(h))
            return out

    # ════════════════════════════════════════════════════════════
    #  Safety-Weighted BCE Loss
    # ════════════════════════════════════════════════════════════

    class SafetyWeightedBCELoss(nn.Module):
        """BCE loss with per-class safety weighting.

        L = Σ_c w_c * BCE(pred_c, target_c)
        where w_c = SAFETY_LOSS_WEIGHT[safety_level(c)].

        Dangerous objects are penalized 3x, forbidden 5x.
        This is a paper contribution: BSG uses uniform BCE.
        """

        def __init__(self, safety_weights: torch.Tensor):
            super().__init__()
            self.register_buffer("safety_weights", safety_weights)

        def forward(self, pred: torch.Tensor, target: torch.Tensor) -> torch.Tensor:
            bce = F.binary_cross_entropy(pred, target, reduction="none")
            weighted = bce * self.safety_weights.unsqueeze(0)
            return weighted.mean()

    # ════════════════════════════════════════════════════════════
    #  Synthetic Scene Graph Dataset (from KG)
    # ════════════════════════════════════════════════════════════

    class KGSceneGraphDataset(Dataset):
        """Synthetic training dataset generated from KG room-type mappings.

        Follows BSG (ICRA 2024) Section III-B methodology:
          1. Sample building with 3-8 rooms
          2. For each room, sample room_type
          3. Sample objects from KG expected list + noise
          4. Ground truth = full histogram, input = partial (30-70% removed)

        But uses our KG instead of HM3D dataset.
        """

        def __init__(self, kg, label2idx: Dict[str, int],
                     cooccurrence: np.ndarray, safety_vec: np.ndarray,
                     affordance_mat: np.ndarray, room_priors: Dict[str, np.ndarray],
                     num_scenes: int = 5000, removal_range: Tuple[float, float] = (0.3, 0.7),
                     noise_rate: float = 0.10, seed: int = 42):
            super().__init__()
            self._kg = kg
            self._label2idx = label2idx
            self._cooccurrence = cooccurrence
            self._safety_vec = safety_vec
            self._affordance_mat = affordance_mat
            self._room_priors = room_priors
            self._removal_range = removal_range
            self._noise_rate = noise_rate
            self._C = len(label2idx)
            self._A = affordance_mat.shape[1]

            rng = random.Random(seed)
            np_rng = np.random.RandomState(seed)
            self._scenes = self._generate_scenes(num_scenes, rng, np_rng)

        def _generate_scenes(self, num_scenes: int, rng: random.Random,
                             np_rng: np.random.RandomState) -> List[Dict]:
            all_labels = list(self._label2idx.keys())
            scenes = []
            for _ in range(num_scenes):
                num_rooms = rng.randint(3, 8)
                room_types = [rng.choice(ROOM_TYPES) for _ in range(num_rooms)]

                gt_histograms = []
                partial_histograms = []
                room_prior_features = []

                for rt in room_types:
                    expected = self._kg.get_room_expected_objects(rt)
                    gt = np.zeros(self._C, dtype=np.float32)
                    for lbl in expected:
                        if lbl in self._label2idx:
                            gt[self._label2idx[lbl]] = 1.0

                    # Noise: add random unexpected objects at noise_rate
                    for lbl in all_labels:
                        if lbl not in expected and np_rng.random() < self._noise_rate:
                            gt[self._label2idx[lbl]] = 1.0

                    # Partial: randomly remove 30-70% of objects
                    partial = gt.copy()
                    present = np.where(gt > 0.5)[0]
                    if len(present) > 1:
                        remove_frac = np_rng.uniform(*self._removal_range)
                        n_remove = max(1, int(len(present) * remove_frac))
                        to_remove = np_rng.choice(present, size=n_remove, replace=False)
                        partial[to_remove] = 0.0

                    gt_histograms.append(gt)
                    partial_histograms.append(partial)
                    room_prior_features.append(
                        self._room_priors.get(rt, np.zeros(self._C, dtype=np.float32)))

                # Build adjacency: chain connectivity + random extra edges
                adj = np.eye(num_rooms, dtype=np.float32)
                for i in range(num_rooms - 1):
                    adj[i, i + 1] = 1.0
                    adj[i + 1, i] = 1.0
                for _ in range(num_rooms // 2):
                    a, b = rng.sample(range(num_rooms), 2)
                    adj[a, b] = 1.0
                    adj[b, a] = 1.0

                scenes.append({
                    "gt": np.stack(gt_histograms),           # (N, C)
                    "partial": np.stack(partial_histograms), # (N, C)
                    "room_prior": np.stack(room_prior_features),  # (N, C)
                    "adj": adj,                               # (N, N)
                    "room_types": room_types,
                })
            return scenes

        def __len__(self) -> int:
            return len(self._scenes)

        def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
            scene = self._scenes[idx]
            partial = scene["partial"]   # (N, C)
            N = partial.shape[0]

            # Build KG-augmented input features for each room node
            cooc_feat = np.zeros((N, self._C), dtype=np.float32)
            for i in range(N):
                observed_idx = np.where(partial[i] > 0.5)[0]
                if len(observed_idx) > 0:
                    cooc_feat[i] = self._cooccurrence[observed_idx].mean(axis=0)

            safety_feat = np.tile(self._safety_vec, (N, 1))  # (N, C)

            aff_feat = np.zeros((N, self._A), dtype=np.float32)
            for i in range(N):
                observed_idx = np.where(partial[i] > 0.5)[0]
                if len(observed_idx) > 0:
                    aff_feat[i] = self._affordance_mat[observed_idx].sum(axis=0)

            x = np.concatenate([
                partial,                   # observed histogram (N, C)
                scene["room_prior"],       # KG room prior (N, C)
                cooc_feat,                 # co-occurrence (N, C)
                safety_feat,               # safety encoding (N, C)
                aff_feat,                  # affordance agg (N, A)
            ], axis=1)  # (N, 4C + A)

            return {
                "x": torch.tensor(x, dtype=torch.float32),
                "adj": torch.tensor(scene["adj"], dtype=torch.float32),
                "target": torch.tensor(scene["gt"], dtype=torch.float32),
            }

    # ════════════════════════════════════════════════════════════
    #  Trainer
    # ════════════════════════════════════════════════════════════

    class BeliefTrainer:
        """Training loop for KGBeliefGCN with safety-weighted loss."""

        def __init__(self, model: KGBeliefGCN, loss_fn: SafetyWeightedBCELoss,
                     lr: float = 1e-3, weight_decay: float = 1e-4):
            self.model = model
            self.loss_fn = loss_fn
            self.optimizer = torch.optim.Adam(
                model.parameters(), lr=lr, weight_decay=weight_decay)
            self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                self.optimizer, mode="min", patience=5, factor=0.5)
            self.train_losses: List[float] = []
            self.val_losses: List[float] = []

        def train_epoch(self, dataset: KGSceneGraphDataset) -> float:
            self.model.train()
            total_loss = 0.0
            count = 0
            for i in range(len(dataset)):
                batch = dataset[i]
                x, adj, target = batch["x"], batch["adj"], batch["target"]

                pred = self.model(x, adj)
                loss = self.loss_fn(pred, target)

                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.model.parameters(), 1.0)
                self.optimizer.step()

                total_loss += loss.item()
                count += 1
            avg = total_loss / max(count, 1)
            self.train_losses.append(avg)
            return avg

        @torch.no_grad()
        def validate(self, dataset: KGSceneGraphDataset,
                     start_idx: int = 0, end_idx: int = -1) -> float:
            self.model.eval()
            if end_idx < 0:
                end_idx = len(dataset)
            total_loss = 0.0
            count = 0
            for i in range(start_idx, end_idx):
                batch = dataset[i]
                x, adj, target = batch["x"], batch["adj"], batch["target"]
                pred = self.model(x, adj)
                loss = self.loss_fn(pred, target)
                total_loss += loss.item()
                count += 1
            avg = total_loss / max(count, 1)
            self.val_losses.append(avg)
            self.scheduler.step(avg)
            return avg

        def train(self, train_dataset: KGSceneGraphDataset,
                  val_dataset: KGSceneGraphDataset,
                  epochs: int = 50, log_interval: int = 10) -> Dict:
            best_val = float("inf")
            best_state = None
            for epoch in range(epochs):
                train_loss = self.train_epoch(train_dataset)
                val_loss = self.validate(val_dataset)
                if val_loss < best_val:
                    best_val = val_loss
                    best_state = {k: v.cpu().clone()
                                  for k, v in self.model.state_dict().items()}
                if (epoch + 1) % log_interval == 0:
                    logger.info("Epoch %d/%d  train=%.4f  val=%.4f  best=%.4f",
                                epoch + 1, epochs, train_loss, val_loss, best_val)
            if best_state is not None:
                self.model.load_state_dict(best_state)
            return {
                "best_val_loss": best_val,
                "train_losses": self.train_losses,
                "val_losses": self.val_losses,
                "epochs": epochs,
            }

    # ════════════════════════════════════════════════════════════
    #  Predictor (inference wrapper)
    # ════════════════════════════════════════════════════════════

    class BeliefPredictor:
        """Inference wrapper — integrates with InstanceTracker.

        Usage:
            predictor = BeliefPredictor.from_kg(kg)
            predictor.load_weights("model.pt")
            preds = predictor.predict(observed_labels, room_adj)
        """

        def __init__(self, model: KGBeliefGCN, label2idx: Dict[str, int],
                     idx2label: List[str], cooccurrence: np.ndarray,
                     safety_vec: np.ndarray, affordance_mat: np.ndarray,
                     room_priors: Dict[str, np.ndarray]):
            self.model = model
            self.label2idx = label2idx
            self.idx2label = idx2label
            self._cooccurrence = cooccurrence
            self._safety_vec = safety_vec
            self._affordance_mat = affordance_mat
            self._room_priors = room_priors
            self._C = len(label2idx)
            self._A = affordance_mat.shape[1]

        @classmethod
        def from_kg(cls, kg, weights_path: Optional[str] = None) -> "BeliefPredictor":
            label2idx, idx2label = build_object_vocabulary(kg)
            C = len(label2idx)
            cooc = build_cooccurrence_matrix(kg, label2idx)
            safety = build_safety_vector(kg, label2idx)
            aff = build_affordance_vectors(kg, label2idx)
            priors = build_room_prior_vectors(kg, label2idx)

            model = KGBeliefGCN(num_objects=C)
            if weights_path and Path(weights_path).exists():
                state = torch.load(weights_path, map_location="cpu", weights_only=True)
                model.load_state_dict(state)
                logger.info("Loaded belief model weights from %s", weights_path)
            model.eval()

            return cls(model, label2idx, idx2label, cooc, safety, aff, priors)

        def load_weights(self, path: str) -> None:
            state = torch.load(path, map_location="cpu", weights_only=True)
            self.model.load_state_dict(state)
            self.model.eval()

        def save_weights(self, path: str) -> None:
            torch.save(self.model.state_dict(), path)

        @torch.no_grad()
        def predict(self, rooms_labels: List[List[str]],
                    rooms_types: Optional[List[str]] = None,
                    adj: Optional[np.ndarray] = None) -> np.ndarray:
            """Predict complete object histograms for a set of rooms.

            Args:
                rooms_labels: list of observed labels per room
                rooms_types: optional inferred room types per room
                adj: optional adjacency matrix (N,N); defaults to chain

            Returns:
                (N, C) array of predicted object probabilities per room
            """
            N = len(rooms_labels)
            if N == 0:
                return np.zeros((0, self._C), dtype=np.float32)

            # Build feature matrix
            partial = np.zeros((N, self._C), dtype=np.float32)
            for i, labels in enumerate(rooms_labels):
                for lbl in labels:
                    lbl_lower = lbl.lower()
                    if lbl_lower in self.label2idx:
                        partial[i, self.label2idx[lbl_lower]] = 1.0

            room_prior = np.zeros((N, self._C), dtype=np.float32)
            if rooms_types:
                for i, rt in enumerate(rooms_types):
                    if rt in self._room_priors:
                        room_prior[i] = self._room_priors[rt]

            cooc_feat = np.zeros((N, self._C), dtype=np.float32)
            for i in range(N):
                obs_idx = np.where(partial[i] > 0.5)[0]
                if len(obs_idx) > 0:
                    cooc_feat[i] = self._cooccurrence[obs_idx].mean(axis=0)

            safety_feat = np.tile(self._safety_vec, (N, 1))

            aff_feat = np.zeros((N, self._A), dtype=np.float32)
            for i in range(N):
                obs_idx = np.where(partial[i] > 0.5)[0]
                if len(obs_idx) > 0:
                    aff_feat[i] = self._affordance_mat[obs_idx].sum(axis=0)

            x = np.concatenate([partial, room_prior, cooc_feat, safety_feat, aff_feat], axis=1)
            x_t = torch.tensor(x, dtype=torch.float32)

            if adj is None:
                adj_np = np.eye(N, dtype=np.float32)
                for i in range(N - 1):
                    adj_np[i, i + 1] = 1.0
                    adj_np[i + 1, i] = 1.0
            else:
                adj_np = adj.astype(np.float32)
            adj_t = torch.tensor(adj_np, dtype=torch.float32)

            self.model.eval()
            pred = self.model(x_t, adj_t)
            return pred.numpy()

        def predict_for_room(self, labels: List[str],
                             room_type: Optional[str] = None) -> Dict[str, float]:
            """Predict object probabilities for a single room.

            Returns dict of {object_label: probability} for objects not yet observed.
            """
            preds = self.predict([labels], [room_type] if room_type else None)
            result: Dict[str, float] = {}
            observed = {lbl.lower() for lbl in labels}
            for idx, prob in enumerate(preds[0]):
                lbl = self.idx2label[idx]
                if lbl not in observed and prob > 0.1:
                    result[lbl] = float(prob)
            return dict(sorted(result.items(), key=lambda x: -x[1]))

else:
    # Stubs when torch is not available
    class GCNConv:  # type: ignore[no-redef]
        pass

    class KGBeliefGCN:  # type: ignore[no-redef]
        pass

    class SafetyWeightedBCELoss:  # type: ignore[no-redef]
        pass

    class KGSceneGraphDataset:  # type: ignore[no-redef]
        pass

    class BeliefTrainer:  # type: ignore[no-redef]
        pass

    class BeliefPredictor:  # type: ignore[no-redef]
        @classmethod
        def from_kg(cls, kg, weights_path=None):
            return None
