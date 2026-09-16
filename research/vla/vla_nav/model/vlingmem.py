"""
Visual-assisted Linguistic Memory (VLingMem) Module.

Reference: VLingNav (arXiv 2601.08665) Section 3.3.3
  - Persistent cross-modal memory bank for long-horizon navigation
  - Each entry stores: (CoT summary embedding, visual key feature, position, timestamp)
  - Retrieval: attention-based query using current hidden state
  - Purpose: prevent repetitive exploration, recall past observations,
    infer movement trends in dynamic environments

Compared to the TopologicalMemory in the old modular pipeline, VLingMem:
  1. Stores *linguistic* summaries (from CoT reasoning) instead of raw labels
  2. Uses learned embeddings instead of string matching for retrieval
  3. Is tightly integrated with the VLM backbone's representation space
"""

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

logger = logging.getLogger(__name__)


@dataclass
class MemoryEntry:
    """A single entry in the VLingMem memory bank."""
    entry_id: int
    timestamp: float

    # Linguistic component
    cot_summary: str                # Compressed CoT text
    summary_embedding: torch.Tensor  # (hidden_dim,) or (summary_dim,)

    # Visual component
    visual_feature: torch.Tensor     # (visual_dim,) from vision encoder

    # Spatial component
    position: np.ndarray             # Robot (x, y, z) when this memory was formed
    heading: float                   # Robot yaw when this memory was formed

    # Metadata
    importance: float = 1.0          # For importance-based eviction
    access_count: int = 0            # How many times this entry was retrieved


class SummaryEncoder(nn.Module):
    """
    Compress VLM hidden states from CoT output into a fixed-size summary vector.

    Uses a single-layer GRU to process a sequence of hidden states
    (from the CoT text tokens) and output a summary vector.
    """

    def __init__(self, hidden_dim: int = 2048, summary_dim: int = 512):
        super().__init__()
        self.gru = nn.GRU(
            input_size=hidden_dim,
            hidden_size=summary_dim,
            num_layers=1,
            batch_first=True,
        )
        self.projection = nn.Linear(summary_dim, summary_dim)

    def forward(self, hidden_states: torch.Tensor) -> torch.Tensor:
        """
        Args:
            hidden_states: (1, seq_len, hidden_dim) — from CoT token hidden states

        Returns:
            summary: (1, summary_dim)
        """
        _, h_n = self.gru(hidden_states)  # h_n: (1, 1, summary_dim)
        summary = h_n.squeeze(0)          # (1, summary_dim)
        summary = self.projection(summary)
        summary = F.normalize(summary, dim=-1)
        return summary


class VisualKeyEncoder(nn.Module):
    """
    Project vision encoder features into the memory's visual key space.

    Takes the pooled visual features from the VLM's vision encoder
    and projects them into a compact representation for memory storage.
    """

    def __init__(self, visual_dim: int = 2048, key_dim: int = 256):
        super().__init__()
        self.projection = nn.Sequential(
            nn.Linear(visual_dim, key_dim),
            nn.GELU(),
            nn.Linear(key_dim, key_dim),
        )

    def forward(self, visual_features: torch.Tensor) -> torch.Tensor:
        """
        Args:
            visual_features: (1, visual_dim) — pooled from vision encoder

        Returns:
            visual_key: (1, key_dim)
        """
        key = self.projection(visual_features)
        return F.normalize(key, dim=-1)


class VLingMemModule(nn.Module):
    """
    Visual-assisted Linguistic Memory for long-horizon embodied navigation.

    Key operations:
      1. store()    — Add a new memory entry (after THINK step)
      2. retrieve() — Query memory bank using current hidden state
      3. format()   — Convert retrieved memories to text for VLM prompt
      4. evict()    — Remove oldest/least important entries when full

    The memory bank is a simple list; for Jetson deployment the typical size
    is ≤100 entries, so linear scan is faster than approximate-NN indices.
    """

    def __init__(
        self,
        hidden_dim: int = 2048,
        summary_dim: int = 512,
        visual_key_dim: int = 256,
        max_entries: int = 100,
        top_k: int = 5,
        eviction_strategy: str = "fifo",
    ):
        super().__init__()
        self.summary_encoder = SummaryEncoder(hidden_dim, summary_dim)
        self.visual_key_encoder = VisualKeyEncoder(hidden_dim, visual_key_dim)

        # Query projections for attention-based retrieval
        self.query_proj = nn.Linear(hidden_dim, summary_dim)

        self.max_entries = max_entries
        self.top_k = top_k
        self.eviction_strategy = eviction_strategy

        self._bank: List[MemoryEntry] = []
        self._next_id: int = 0

    @property
    def size(self) -> int:
        return len(self._bank)

    def clear(self):
        """Remove all memory entries."""
        self._bank.clear()
        self._next_id = 0

    # ---- store ---------------------------------------------------------------

    def store(
        self,
        cot_text: str,
        cot_hidden_states: Optional[torch.Tensor],
        visual_feature: torch.Tensor,
        position: np.ndarray,
        heading: float,
        timestamp: Optional[float] = None,
    ):
        """
        Store a new memory entry after a THINK step.

        Args:
            cot_text:          The generated CoT reasoning text
            cot_hidden_states: (1, seq_len, hidden_dim) from CoT tokens — or None
            visual_feature:    (1, hidden_dim) from vision encoder
            position:          Robot (x, y, z)
            heading:           Robot yaw (radians)
            timestamp:         Time of memory formation
        """
        ts = timestamp if timestamp is not None else time.monotonic()

        # Encode summary
        if cot_hidden_states is not None:
            with torch.no_grad():
                summary_emb = self.summary_encoder(cot_hidden_states).squeeze(0)
        else:
            # Fallback: zero vector (will be updated during training)
            summary_emb = torch.zeros(
                self.summary_encoder.projection.out_features,
                device=visual_feature.device,
            )

        # Encode visual key
        with torch.no_grad():
            vis_key = self.visual_key_encoder(visual_feature).squeeze(0)

        entry = MemoryEntry(
            entry_id=self._next_id,
            timestamp=ts,
            cot_summary=cot_text,
            summary_embedding=summary_emb.detach(),
            visual_feature=vis_key.detach(),
            position=position.copy() if isinstance(position, np.ndarray) else np.array(position),
            heading=heading,
        )
        self._next_id += 1
        self._bank.append(entry)

        # Evict if over capacity
        while len(self._bank) > self.max_entries:
            self._evict()

        logger.debug(
            "VLingMem stored entry #%d at (%.1f, %.1f) — bank size: %d",
            entry.entry_id,
            position[0],
            position[1],
            len(self._bank),
        )

    # ---- retrieve ------------------------------------------------------------

    def retrieve(
        self,
        query_hidden: torch.Tensor,
        top_k: Optional[int] = None,
    ) -> List[MemoryEntry]:
        """
        Retrieve the most relevant memory entries for the current state.

        Uses cosine similarity between the projected query and stored
        summary embeddings.

        Args:
            query_hidden: (1, hidden_dim) — current VLM hidden state
            top_k:        Override default top-k

        Returns:
            List of MemoryEntry, sorted by relevance (most relevant first)
        """
        if not self._bank:
            return []

        k = top_k if top_k is not None else self.top_k

        with torch.no_grad():
            q = self.query_proj(query_hidden)  # (1, summary_dim)
            q = F.normalize(q, dim=-1)

            similarities = []
            for entry in self._bank:
                emb = entry.summary_embedding.to(q.device)
                if emb.dim() == 1:
                    emb = emb.unsqueeze(0)
                sim = F.cosine_similarity(q, emb, dim=-1).item()
                similarities.append(sim)

        # Top-k indices
        indexed = sorted(
            enumerate(similarities),
            key=lambda x: x[1],
            reverse=True,
        )[:k]

        results = []
        for idx, sim in indexed:
            self._bank[idx].access_count += 1
            results.append(self._bank[idx])

        return results

    # ---- format for VLM prompt -----------------------------------------------

    def format_memory_text(
        self,
        entries: List[MemoryEntry],
        current_position: Optional[np.ndarray] = None,
    ) -> str:
        """
        Format retrieved memory entries as text for the VLM prompt.

        Returns something like:
          [Mem 1] At (2.1, 3.4), heading NE: "Saw a door ahead, turned right toward corridor"
          [Mem 2] At (5.0, 3.2), heading E:  "Reached kitchen, target not here, backtracking"
        """
        if not entries:
            return "No relevant memories."

        lines = []
        for i, entry in enumerate(entries, 1):
            pos = entry.position
            dist_str = ""
            if current_position is not None:
                dist = float(np.linalg.norm(pos[:2] - current_position[:2]))
                dist_str = f", {dist:.1f}m away"

            heading_str = _heading_to_cardinal(entry.heading)
            summary = entry.cot_summary[:120]
            lines.append(
                f"[Mem {i}] At ({pos[0]:.1f}, {pos[1]:.1f}), "
                f"facing {heading_str}{dist_str}: \"{summary}\""
            )

        return "\n".join(lines)

    # ---- eviction ------------------------------------------------------------

    def _evict(self):
        """Remove one entry according to the eviction strategy."""
        if not self._bank:
            return

        if self.eviction_strategy == "fifo":
            removed = self._bank.pop(0)
        elif self.eviction_strategy == "lru":
            # Least recently accessed
            lru_idx = min(range(len(self._bank)), key=lambda i: self._bank[i].access_count)
            removed = self._bank.pop(lru_idx)
        elif self.eviction_strategy == "importance":
            # Lowest importance score
            low_idx = min(range(len(self._bank)), key=lambda i: self._bank[i].importance)
            removed = self._bank.pop(low_idx)
        else:
            removed = self._bank.pop(0)

        logger.debug("VLingMem evicted entry #%d", removed.entry_id)

    # ---- serialization -------------------------------------------------------

    def to_dict(self) -> dict:
        """Serialize memory bank for persistence (JSON/SQLite)."""
        return {
            "entries": [
                {
                    "id": e.entry_id,
                    "timestamp": e.timestamp,
                    "cot_summary": e.cot_summary,
                    "position": e.position.tolist(),
                    "heading": e.heading,
                    "importance": e.importance,
                    "access_count": e.access_count,
                }
                for e in self._bank
            ],
            "next_id": self._next_id,
        }

    def get_statistics(self) -> dict:
        return {
            "bank_size": len(self._bank),
            "max_entries": self.max_entries,
            "total_stored": self._next_id,
            "avg_access_count": (
                sum(e.access_count for e in self._bank) / max(len(self._bank), 1)
            ),
        }


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
_CARDINAL_DIRS = [
    (0, "N"), (45, "NE"), (90, "E"), (135, "SE"),
    (180, "S"), (225, "SW"), (270, "W"), (315, "NW"), (360, "N"),
]


def _heading_to_cardinal(heading_rad: float) -> str:
    """Convert heading in radians to nearest cardinal direction string."""
    deg = (np.degrees(heading_rad) % 360 + 360) % 360
    closest = min(_CARDINAL_DIRS, key=lambda x: abs(x[0] - deg))
    return closest[1]
