"""SemanticMapperModule -drives RoomObjectKG + TopologySemGraph from live SceneGraph.

Subscribes to the scene_graph (SceneGraph) published by PerceptionService and
odometry from the driver.  On every scene-graph update:

  1. Converts SceneGraph.regions ->TopologySemGraph room nodes (stable int IDs)
  2. Calls RoomObjectKG.observe_room() for each region
  3. Records robot position for traversal memory
  4. Publishes topo_summary (LLM-ready text) and room_graph (serialised TSG dict)
  5. Auto-saves every `save_interval_s` seconds

On load it merges any previously-saved KG so the planner benefits immediately.

Ports:
  In:  scene_graph (SceneGraph), odometry (Odometry)
  Out: topo_summary (str)   -LLM-consumable room context
       room_graph (dict)    -TopologySemGraph.to_dict() snapshot
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any

from runtime.module import Module, skill
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import SceneGraph
from runtime.registry import register
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

_DEFAULT_SAVE_DIR = os.path.join(os.path.expanduser("~"), ".nova", "semantic")


@register("semantic", "mapper", description="Drives RoomObjectKG + TopologySemGraph from SceneGraph")
class SemanticMapperModule(Module, layer=3):
    _run_in_worker = True
    _worker_group = "semantic"
    """Real-time semantic map builder: SceneGraph ->KG + topological graph.

    Bridges the gap between the live perception stream and the two persistent
    semantic data structures:
      - RoomObjectKG:      room-type x object co-occurrence probabilities
      - TopologySemGraph:  room-level topological graph with traversal memory

    Both are saved to `save_dir` periodically and on stop().  GoalResolver
    loads the KG on startup via SemanticPriorEngine(kg_path=...) to replace
    hand-coded ROOM_OBJECT_PRIORS with learned statistics.
    """

    scene_graph: In[SceneGraph]
    odometry:    In[Odometry]

    topo_summary: Out[str]   # LLM-readable room context ->SemanticPlanner
    room_graph:   Out[dict]  # TSG dict snapshot ->Navigation (optional)

    def __init__(
        self,
        save_dir: str = "",
        save_interval_s: float = 30.0,
        min_observations_for_commit: int = 1,
        **kw,
    ):
        super().__init__(**kw)
        self._save_dir = save_dir or _DEFAULT_SAVE_DIR
        self._save_interval = save_interval_s

        # Minimum times a (room, object_label) pair must be observed before
        # it is committed to RoomObjectKG.  Default=1 preserves backward-compatible
        # behaviour (every observation commits).  Set to 3 or higher in production
        # deployments to filter single-frame misdetections.
        self._min_obs: int = min_observations_for_commit

        # Per-(room_name, object_label) raw observation counter accumulated on
        # every scene-graph update.  A pair is forwarded to KG only once its
        # counter reaches _min_obs.
        self._obs_counts: dict[tuple[str, str], int] = {}

        # Dirichlet-Multinomial posterior counts: per-room raw tallies used to
        # compute P(O|R) = (count(R,O) + alpha) / (sum_O' count(R,O') + alpha*K)
        # with alpha=1.0 (Laplace) and K = observed object types in room R.
        # Updated unconditionally (before and after the gate threshold).
        self._dm_counts: dict[str, dict[str, int]] = {}  # room ->{label ->count}
        self._dm_alpha: float = 1.0

        self._kg = None         # RoomObjectKG -lazy init in setup()
        self._tsg = None        # TopologySemGraph -lazy init in setup()
        self._robot_xy = (0.0, 0.0)

        # name ->stable int ID for TSG room nodes
        self._room_name_to_id: dict[str, int] = {}
        self._next_room_id: int = 0

        self._last_save_time: float = 0.0
        self._sg_count: int = 0

    # Lifecycle

    def setup(self) -> None:
        self._init_backends()
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)
        # Throttle scene_graph to 2 Hz -heavy DBSCAN already done upstream
        self.scene_graph.set_policy("throttle", interval=0.5)

    def _init_backends(self) -> None:
        try:
            from memory.knowledge.room_object_kg import RoomObjectKG
            self._kg = RoomObjectKG()
            kg_path = self._kg_path()
            if os.path.exists(kg_path):
                self._kg.load(kg_path)
                logger.info("RoomObjectKG loaded from %s", kg_path)
            else:
                logger.info("RoomObjectKG: no existing file at %s, starting fresh", kg_path)
        except ImportError:
            logger.warning("RoomObjectKG not available")

        try:
            from memory.spatial.topology_graph import TopologySemGraph
            self._tsg = TopologySemGraph()
            tsg_path = self._tsg_path()
            if os.path.exists(tsg_path):
                loaded = TopologySemGraph.load_from_file(tsg_path)
                if loaded is not None:
                    self._tsg = loaded
                    logger.info("TopologySemGraph loaded from %s", tsg_path)
        except ImportError:
            logger.warning("TopologySemGraph not available")

    def stop(self) -> None:
        self._save_now()
        super().stop()

    # Input handlers

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_xy = (odom.x, odom.y)
        if self._tsg is not None:
            self._tsg.record_robot_position(odom.x, odom.y)

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        if not sg.regions:
            return
        self._sg_count += 1

        self._update_kg(sg)
        self._update_tsg(sg)

        # Publish topo summary at low frequency (1 Hz max)
        if self._tsg is not None:
            summary = self._tsg.to_prompt_context(language="zh")
            if summary:
                self.topo_summary.publish(summary)
            self.room_graph.publish(self._tsg.to_dict())

        # Periodic auto-save
        now = time.time()
        if now - self._last_save_time >= self._save_interval:
            self._save_now()

    # Core update logic

    def _update_kg(self, sg: SceneGraph) -> None:
        """Update RoomObjectKG with Bayesian Dirichlet-Multinomial smoothing gate.

        Each (room, label) pair must be observed at least _min_obs times before
        being committed to the KG.  This prevents single misdetections (e.g.
        "person in kitchen") from polluting future scene understanding.

        DM counts are updated unconditionally so get_posterior() always reflects
        the current evidence, even for sub-threshold observations.
        When _min_obs == 1 (the default), every observation commits immediately,
        preserving backward-compatible behaviour.
        """
        if self._kg is None:
            return

        for region in sg.regions:
            room_name = (region.name or "").strip()
            if not room_name:
                continue
            labels, confs = self._extract_objects(sg, region.object_ids)
            if not labels:
                continue

            # Always update DM counts (used by get_posterior regardless of gate).
            room_dm = self._dm_counts.setdefault(room_name, {})
            for lbl in labels:
                room_dm[lbl] = room_dm.get(lbl, 0) + 1

            # Accumulate per-(room, label) observation counts and gate KG writes.
            committed_labels: list[str] = []
            committed_confs: list[float] = []
            for lbl, conf in zip(labels, confs):
                key = (room_name, lbl)
                prev = self._obs_counts.get(key, 0)
                self._obs_counts[key] = prev + 1
                if self._obs_counts[key] == self._min_obs and self._min_obs > 1:
                    # First time this pair crosses the threshold -log it.
                    logger.info(
                        "SemanticMapper: (%s, %s) reached %d observations -committing to KG",
                        room_name, lbl, self._min_obs,
                    )
                if self._obs_counts[key] >= self._min_obs:
                    committed_labels.append(lbl)
                    committed_confs.append(conf)

            if committed_labels:
                self._kg.observe_room(room_name, committed_labels, committed_confs)

    def get_posterior(self, room: str, label: str) -> float:
        """Return Dirichlet-Multinomial posterior P(label | room).

        P(O|R) = (count(R,O) + alpha) / (sum_O count(R,O) + alpha*K)
        where alpha=1.0 (Laplace smoothing) and K is the number of distinct
        object types observed in room R.  Returns 0.0 for unseen rooms.
        """
        room_dm = self._dm_counts.get(room, {})
        if not room_dm:
            return 0.0
        count_ro = room_dm.get(label, 0)
        k = len(room_dm)
        total = sum(room_dm.values())
        return (count_ro + self._dm_alpha) / (total + self._dm_alpha * k)

    def _update_tsg(self, sg: SceneGraph) -> None:
        if self._tsg is None:
            return
        rooms = []
        for region in sg.regions:
            room_name = (region.name or "").strip()
            if not room_name:
                continue
            rid = self._stable_room_id(room_name)
            labels, _ = self._extract_objects(sg, region.object_ids)
            center = {"x": 0.0, "y": 0.0}
            if region.center is not None:
                center = {"x": float(region.center.x), "y": float(region.center.y)}
            rooms.append({
                "room_id": rid,
                "name": room_name,
                "center": center,
                "semantic_labels": labels[:12],
            })
        if rooms:
            self._tsg.update_from_scene_graph({"rooms": rooms, "topology_edges": []})

    def _extract_objects(
        self,
        sg: SceneGraph,
        object_ids: list[str],
    ):
        labels: list[str] = []
        confs: list[float] = []
        for oid in object_ids:
            obj = sg.get_object_by_id(oid)
            if obj and obj.label:
                labels.append(obj.label.lower().strip())
                confs.append(float(getattr(obj, "confidence", 0.8)))
        return labels, confs

    def _stable_room_id(self, name: str) -> int:
        if name not in self._room_name_to_id:
            self._room_name_to_id[name] = self._next_room_id
            self._next_room_id += 1
        return self._room_name_to_id[name]

    # Persistence

    def _kg_path(self) -> str:
        return os.path.join(self._save_dir, "room_object_kg.json")

    def _tsg_path(self) -> str:
        return os.path.join(self._save_dir, "topology_graph.json")

    def _save_now(self) -> None:
        os.makedirs(self._save_dir, exist_ok=True)
        if self._kg is not None and not self._kg.is_empty:
            self._kg.save(self._kg_path())
        if self._tsg is not None and self._tsg.rooms:
            self._tsg.save_to_file(self._tsg_path())
        self._last_save_time = time.time()
        logger.debug("SemanticMapperModule: saved to %s", self._save_dir)

    def load_from_dir(self, directory: str) -> bool:
        """Hot-reload semantic maps from a directory.  Called by REPL smap load."""
        ok = True
        try:
            from memory.knowledge.room_object_kg import RoomObjectKG
            from memory.spatial.topology_graph import TopologySemGraph
        except ImportError:
            return False

        kg_path = os.path.join(directory, "room_object_kg.json")
        tsg_path = os.path.join(directory, "topology_graph.json")

        if os.path.exists(kg_path):
            new_kg = RoomObjectKG()
            if new_kg.load(kg_path):
                self._kg = new_kg
                logger.info("KG hot-reloaded from %s", kg_path)
            else:
                ok = False

        if os.path.exists(tsg_path):
            new_tsg = TopologySemGraph.load_from_file(tsg_path)
            if new_tsg is not None:
                self._tsg = new_tsg
                logger.info("TSG hot-reloaded from %s", tsg_path)
            else:
                ok = False

        return ok

    # @skill methods (MCP-exposed)

    @skill
    def get_room_summary(self) -> str:
        """Return a text summary of all known rooms and their objects."""
        if self._tsg is None or not self._tsg.rooms:
            return "No rooms mapped yet."
        lines = []
        for room in self._tsg.rooms:
            labels = room.semantic_labels[:8]
            visited = "visited" if room.visited else "unvisited"
            lines.append(
                f"{visited} {room.name} (id={room.node_id}): "
                + (", ".join(labels) if labels else "no objects")
            )
        return "\n".join(lines)

    @skill
    def query_room_for_object(self, label: str) -> dict:
        """Return which room types are most likely to contain the given object label."""
        if self._kg is None or self._kg.is_empty:
            return {"label": label, "rooms": [], "source": "no_data"}
        results = self._kg.get_object_rooms(label)
        return {
            "label": label,
            "rooms": [{"room": rt, "probability": prob} for rt, prob in results[:5]],
            "source": "learned_kg",
        }

    @skill
    def get_exploration_target(self, instruction: str = "") -> dict:
        """Return the best exploration target for a given instruction."""
        if self._tsg is None or not self._tsg.rooms:
            return {"target": None, "reason": "no topology built yet"}
        targets = self._tsg.get_best_exploration_target(instruction or "explore")
        if not targets:
            return {"target": None, "reason": "no candidates"}
        best = targets[0]
        return {
            "target": best.node_name,
            "position": best.position.tolist(),
            "score": round(best.score, 3),
            "reasoning": best.reasoning,
        }

    @skill
    def get_semantic_status(self) -> dict:
        """Return KG + TSG statistics."""
        kg_stats = self._kg.get_stats() if self._kg else {}
        return {
            "kg": kg_stats,
            "tsg": {
                "rooms": len(self._tsg.rooms) if self._tsg else 0,
                "frontiers": len(self._tsg.frontiers) if self._tsg else 0,
                "current_room": (
                    self._tsg.get_node(self._tsg.current_room_id).name
                    if self._tsg and self._tsg.current_room_id >= 0 else "unknown"
                ),
            },
            "save_dir": self._save_dir,
            "sg_updates": self._sg_count,
        }

    # Health

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["semantic_mapper"] = {
            "save_dir":    self._save_dir,
            "kg_rooms":    len(self._kg._rooms) if self._kg else 0,
            "tsg_nodes":   len(self._tsg._nodes) if self._tsg else 0,
            "sg_updates":  self._sg_count,
            "last_save":   self._last_save_time,
        }
        return info
