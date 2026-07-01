# 3 Method

## 3.1 System Overview

NaviMind consists of three tightly integrated modules (Figure 1):

- **Perception Module** (§3.2): Constructs and maintains an online hierarchical scene graph $\mathcal{G}$ from streaming RGB-D observations.
- **Planning Module** (§3.3–3.5): Resolves natural language instructions to target positions using Fast-Slow dual-path reasoning over $\mathcal{G}$, with continuous re-perception during execution.
- **Exploration Module** (§3.6): When the target is not in $\mathcal{G}$, selects the most promising frontier using vision-augmented multi-factor scoring with subgraph interpolation.

The system operates in a continuous loop: perceive → update scene graph → resolve goal → navigate/explore → re-perceive. All modules run onboard a Jetson Orin NX (16GB) with no cloud dependency except optional LLM API calls for the Slow Path.

**Problem Definition.** Given a natural language instruction $I$ (e.g., "find the fire extinguisher near the corridor door"), the robot must navigate to the target object in an unknown indoor environment. The robot receives streaming RGB-D images and LiDAR-inertial odometry, and can execute discrete navigation actions via a Nav2 [28] local planner. The task succeeds if the robot stops within distance $d_s$ of the ground truth target. No prior map, training data, or environment-specific fine-tuning is available.

---

## 3.2 Online Hierarchical Scene Graph Construction

### 3.2.1 Scene Graph Definition

We define the hierarchical scene graph as $\mathcal{G} = (\mathcal{V}, \mathcal{E})$, where nodes $\mathcal{V}$ are organized into four levels:

$$\mathcal{V} = \mathcal{V}^{\text{obj}} \cup \mathcal{V}^{\text{grp}} \cup \mathcal{V}^{\text{room}} \cup \mathcal{V}^{\text{floor}}$$

- **Object nodes** $v_i^{\text{obj}} = (\text{id}, \ell_i, \mathbf{p}_i, \mathbf{f}_i, s_i, n_i, \mathbf{e}_i)$: Each object has a semantic label $\ell_i$, 3D position $\mathbf{p}_i \in \mathbb{R}^3$, CLIP feature vector $\mathbf{f}_i \in \mathbb{R}^{512}$, best detection score $s_i$, detection count $n_i$, and spatial extent $\mathbf{e}_i \in \mathbb{R}^3$.

- **Group nodes** $v_j^{\text{grp}} = (\text{id}, \text{name}, \mathbf{c}_j, \mathcal{O}_j, \mathcal{L}_j)$: Groups aggregate semantically related objects within a room (e.g., "desk + chair + monitor" → "office_workstation"). $\mathcal{O}_j$ is the set of member object IDs, $\mathcal{L}_j$ is the set of semantic labels.

- **Room nodes** $v_k^{\text{room}} = (\text{id}, \text{name}, \mathbf{c}_k, \mathcal{O}_k, \mathcal{G}_k)$: Rooms represent semantically coherent spatial regions. $\text{name}$ is inferred from contained objects (e.g., "corridor", "office"). $\mathcal{G}_k$ is the set of member group IDs.

- **Floor node** $v^{\text{floor}}$: A single root node containing all rooms (we focus on single-floor environments).

Edges $\mathcal{E}$ include:
- **Hierarchical edges**: Object↔Group, Group↔Room, Room↔Floor (affiliation).
- **Spatial relation edges**: Between object pairs (near, on, left_of, right_of, in_front_of, behind).

### 3.2.2 Incremental Object Detection and Tracking

At each timestep $t$, we detect objects from the RGB image using YOLO-World [12] with open vocabulary and project detections to 3D using depth and camera-LiDAR calibration:

$$\mathbf{p}_{\text{3d}} = T_{\text{map←camera}} \cdot \pi^{-1}(\mathbf{u}, d)$$

where $\mathbf{u}$ is the pixel coordinate, $d$ is the depth value, $\pi^{-1}$ is the inverse camera projection, and $T_{\text{map←camera}}$ is obtained from TF2 transforms via SLAM.

**Blur Filtering.** Before detection, we compute the Laplacian variance of the image to reject motion-blurred frames caused by quadruped locomotion:

$$L_{\text{var}} = \text{Var}(\nabla^2 I_{\text{gray}})$$

Frames with $L_{\text{var}} < \tau_{\text{blur}}$ are discarded.

**Instance Association.** New detections are matched to existing tracked objects using spatial proximity (3D distance < association threshold) and CLIP feature similarity (cosine similarity > 0.8). Unmatched detections are registered as new objects.

### 3.2.3 Quality-Aware CLIP Feature Fusion

Unlike ConceptGraphs [7] which uses uniform EMA, we propose a **quality-aware** feature fusion strategy. When a tracked object $v_i$ receives a new detection with CLIP feature $\mathbf{f}_{\text{new}}$, we update its stored feature using:

$$\alpha = \min\left(0.5,\ \alpha_{\text{base}} \cdot \text{clamp}\left(\frac{s_i}{s_{\text{ref}}},\ 0.5,\ 1.5\right)\right)$$

$$\mathbf{f}_i \leftarrow \frac{(1 - \alpha)\mathbf{f}_i + \alpha\mathbf{f}_{\text{new}}}{\|(1 - \alpha)\mathbf{f}_i + \alpha\mathbf{f}_{\text{new}}\|}$$

where $\alpha_{\text{base}} = 0.3$, $s_{\text{ref}} = 0.8$, and $s_i$ is the object's best detection sruntime. The quality factor $s_i / s_{\text{ref}}$ increases the learning rate for high-confidence detections and decreases it for uncertain ones, producing more stable CLIP features over time.

### 3.2.4 Region Clustering and Room Inference

**DBSCAN Clustering.** Objects are clustered into spatial regions using DBSCAN [29] on their 2D positions (x, y):

$$\text{regions} = \text{DBSCAN}(\{\mathbf{p}_i^{xy}\}_{i \in \mathcal{V}^{\text{obj}}},\ \epsilon = 3.0\text{m},\ \text{min\_samples} = 1)$$

Each region's center is the mean position of its member objects.

**Rule-Based Room Naming.** We define 8 room type rules $\mathcal{R} = \{r_1, \ldots, r_8\}$, each with a keyword set $K_r$, minimum match count $m_r$, and priority $p_r$:

$$\text{room\_type} = \arg\max_{r \in \mathcal{R}} \left(|\mathcal{L}_k \cap K_r| \cdot 10 + p_r\right) \quad \text{s.t.} \quad |\mathcal{L}_k \cap K_r| \geq m_r$$

where $\mathcal{L}_k$ is the set of object labels in room $k$. Room types include: corridor (fire extinguisher, sign), office (desk, monitor, chair), kitchen (sink, refrigerator), meeting room (projector, whiteboard), bathroom, stairwell, lobby, and storage.

**Optional LLM Room Naming.** For richer semantic room names, we optionally invoke an LLM when a region becomes stable (≥3 objects, stable for ≥10 seconds). The LLM receives the object label list and returns a concise room name (e.g., "主走廊" / "main corridor"). Results are cached to avoid redundant API calls.

### 3.2.5 Group Construction

Objects within the same room are grouped by semantic category using predefined keyword families (safety, furniture, structure, electronics, utility). Groups aggregate related objects to reduce graph complexity and enable group-level reasoning.

---

## 3.3 Fast-Slow Dual-Path Goal Resolution

Given instruction $I$ and current scene graph $\mathcal{G}$, goal resolution identifies the target object $v^*$ and its 3D position $\mathbf{p}^*$.

### 3.3.1 Fast Path: Multi-Source Confidence Fusion

The Fast Path computes a fused confidence score for each object $v_i \in \mathcal{V}^{\text{obj}}$ without LLM calls:

$$s_{\text{fused}}(v_i, I) = w_{\ell} \cdot s_{\text{label}}(v_i, I) + w_c \cdot s_{\text{clip}}(v_i, I) + w_d \cdot s_{\text{det}}(v_i) + w_r \cdot s_{\text{spatial}}(v_i, I)$$

where:

- **Label matching** $s_{\text{label}}$: Keyword extraction from $I$ (supporting Chinese via jieba tokenization) with fuzzy string matching against object labels. Subject match yields 1.0, modifier match 0.3, keyword match 0.5.

- **CLIP similarity** $s_{\text{clip}}$: Cosine similarity between CLIP text encoding of $I$ and stored object feature $\mathbf{f}_i$:
$$s_{\text{clip}}(v_i, I) = \frac{\text{CLIP}_{\text{text}}(I) \cdot \mathbf{f}_i}{\|\text{CLIP}_{\text{text}}(I)\| \cdot \|\mathbf{f}_i\|}$$

- **Detector confidence** $s_{\text{det}}$: YOLO-World confidence, normalized by detection count.

- **Spatial hint** $s_{\text{spatial}}$: Score based on spatial relation keywords in $I$ (e.g., "near the door", "left of the desk").

Empirical weights are $w_{\ell} = w_c = 0.35$, $w_d = w_r = 0.15$, prioritizing semantic matching (70%) over detection confidence (30%).

**Routing Decision.** If $\max_i s_{\text{fused}}(v_i, I) \geq \tau_{\text{fast}} = 0.75$, the Fast Path returns the top-scored object directly. Otherwise, the system falls through to the Slow Path.

### 3.3.2 Slow Path: Hierarchical CoT LLM Reasoning

For complex queries that require spatial reasoning or disambiguation, the Slow Path invokes an LLM with a structured prompt derived from $\mathcal{G}$.

**Selective Grounding.** To reduce LLM token consumption, we filter the scene graph to retain only relevant context:

1. **Semantic ranking**: Compute combined relevance $r_i = 0.6 \cdot s_{\text{clip}}(v_i, I) + 0.4 \cdot s_{\text{kw}}(v_i, I)$ for each object.
2. **Relation expansion**: Add 1-hop neighbors of top-ranked objects.
3. **Capacity limit**: Retain top-$K$ objects ($K = 15$) and top-$M$ relations ($M = 20$).

This filtering reduces scene graph tokens by ~90% (e.g., 200 → 15 objects) while preserving the target and its spatial context.

**Hierarchical CoT Prompt.** The filtered scene graph is presented to the LLM in a structured, layered format:

```
[System] You are a robot navigation assistant. Reason step by step:
  Step 1: Understand the instruction
  Step 2: Select 1-2 candidate Rooms from the room list
  Step 3: Select candidate Groups within those Rooms
  Step 4: Match the target Object within candidate Groups, verify spatial relations
  Step 5: Output the target's 3D coordinates with confidence

[User]
  Instruction: {I}
  === Rooms ===
  {room_id: name, objects, center}
  === Groups ===
  {group_id: name, room_id, objects}
  === Objects (top 30) ===
  {id, label, position, room, group}
  === Relations (top 20) ===
  {subject → relation → object}
```

The LLM returns a structured JSON response including `selected_room_id`, `selected_group_id`, `selected_object_id`, target coordinates, confidence, and reasoning chain.

### 3.3.3 Overall Algorithm

```
Algorithm 1: Fast-Slow Goal Resolution
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Instruction I, Scene Graph G
Output: Target object v*, position p*, confidence c

1: // Fast Path
2: for each v_i in G.objects do
3:     s_i ← w_ℓ·s_label + w_c·s_clip + w_d·s_det + w_r·s_spatial
4: end for
5: v_fast ← argmax_i(s_i), c_fast ← max_i(s_i)
6: if c_fast ≥ τ_fast then
7:     return (v_fast, p_fast, c_fast)        ▷ Fast hit (~75% cases)
8: end if
9:
10: // Slow Path
11: G_filtered ← SelectiveGrounding(I, G, K=15)
12: prompt ← BuildHierarchicalCoTPrompt(I, G_filtered)
13: response ← LLM(prompt)
14: v_slow, p_slow, c_slow ← ParseResponse(response)
15: return (v_slow, p_slow, c_slow)
```

---

## 3.4 Belief-Aware Scene Graph and Risk-Sensitive Planning (BA-HSG)

> The full formalization of BA-HSG is presented in the companion document `03b_belief_graph.md`. Here we summarize the key ideas.

False positive detections are a major failure mode in object navigation [3]. Unlike prior work that treats scene graph nodes as deterministic entities (ConceptGraphs [7], HOV-SG [8]), we augment every node with a probabilistic **belief state** and use it for risk-sensitive goal selection and VoI-driven scheduling.

### 3.4.1 Node-Level Belief State

Each object $v_i$ maintains three belief components:

**Existence belief** via Beta distribution: $P(\text{exists}_i) \sim \text{Beta}(\alpha_i, \beta_i)$, updated with positive evidence (high-confidence detections with valid depth and geometric consistency) and negative evidence (expected-visible but undetected). Expected existence: $\bar{P}_i = \alpha_i / (\alpha_i + \beta_i)$.

**Position uncertainty** via isotropic Gaussian: $\mathbf{p}_i \sim \mathcal{N}(\boldsymbol{\mu}_i, \sigma_i^2 \mathbf{I})$, updated via Kalman-style fusion with depth-proportional observation noise $\sigma_{\text{obs}} = \sigma_0 + \sigma_d \cdot d$.

**Composite credibility** combining detection, view diversity, temporal freshness, and geometric consistency:

$$C_i = w_s \cdot \bar{P}_i + w_v \cdot A_i + w_t \cdot e^{-\Delta t_i / \tau_t} + w_g \cdot e^{-\epsilon_i^{\text{reproj}} / \kappa}$$

**Belief propagation (graph diffusion).** Credibility propagates through the hierarchy: room credibility is the weighted average of its objects'; well-established rooms boost newly detected objects within them; spatially related objects share partial credibility.

### 3.4.2 Risk-Sensitive Goal Selection

When multiple candidates exist, we maintain a multi-hypothesis posterior:

$$p_i \propto \exp\left(\gamma_1 \cdot s_{\text{fused}}(v_i, I) + \gamma_2 \cdot C_i + \gamma_3 \cdot P(\text{room}(v_i) | I)\right)$$

Updated via Bayesian verification upon approach: $p_i \leftarrow p_i \cdot L(o | v_i) / Z$.

Target selection optimizes expected cost: $i^* = \arg\min_i\ \mathbb{E}[d] - \beta p_i + \rho \cdot \text{InfoGain}(i)$, balancing exploitation (high-probability targets), cost (Nav2 distance), and exploration (disambiguation value).

### 3.4.3 VoI-Driven Reasoning and Re-Perception Scheduling

Instead of fixed-interval re-perception (every 2m), we formalize the scheduling as Value of Information optimization over three meta-actions: **continue**, **re-perceive**, **slow-reason**. Each action's utility is:

$$U(a) = \Delta \mathbb{E}[S(a)] - \lambda_t T(a) - \lambda_d d(a)$$

where $\Delta \mathbb{E}[S]$ is estimated from expected entropy reduction in the target posterior. Cooldown constraints prevent oscillation. On Jetson, VoI estimation uses pre-computed lookup tables (<0.1ms overhead).

### 3.4.4 Arrival and Continuous Re-Perception

At the **arrival layer**, when Nav2 goal completes, close-range detection verifies target presence and updates the Beta posterior. At the **continuous layer**, VoI-triggered re-perception updates beliefs during navigation, canceling goals to stale or false-positive targets when credibility drops below $\tau_{\text{reject}} = 0.25$.

---

## 3.5 Multi-Step Instruction Execution

For instructions involving sequential sub-goals (e.g., "first go to the door, then find the fire extinguisher nearby"), the system decomposes the instruction into an ordered sequence of sub-goals using template matching or LLM-based decomposition.

**Execution with Feedback.** Each sub-goal is executed sequentially. Upon completion:
- **Success**: Generate a scene summary (current position, visible objects, nearby rooms) and append to execution context. Advance to next sub-goal.
- **Failure**: Retry up to $N_{\text{retry}}$ times. If retries are exhausted, invoke LLM replanning with accumulated execution context (scene summaries, failure reasons) to generate a revised plan.

This closed-loop execution enables adaptation to unexpected situations (e.g., a door is closed, requiring an alternative route).

---

## 3.6 Vision-Augmented Frontier Exploration

When the target object is not in the current scene graph, the robot must explore to expand $\mathcal{G}$.

### 3.6.1 Frontier Extraction

Frontiers are extracted from the 2D occupancy grid as boundaries between explored free space and unknown space [14]. We filter frontiers by minimum size and accessibility (reachable via the costmap).

### 3.6.2 Multi-Factor Frontier Scoring

Each frontier $f_j$ is scored by five factors:

$$s_{\text{frontier}}(f_j) = \sum_{k=1}^{5} \frac{w_k}{\sum w} \cdot s_k(f_j)$$

where the factors are:

1. **Distance efficiency** ($w_1 = 0.2$):
$$s_{\text{dist}}(f_j) = 1 - \frac{d(f_j, \mathbf{p}_{\text{robot}})}{\max_j d(f_j, \mathbf{p}_{\text{robot}})}$$

2. **Spatial novelty** ($w_2 = 0.3$): Favors frontiers far from previously visited locations:
$$s_{\text{nov}}(f_j) = \min\left(1,\ \frac{\min_m d(f_j, \mathbf{p}_m^{\text{visited}})}{d_{\text{nov}}}\right), \quad d_{\text{nov}} = 5.0\text{m}$$

3. **Language grounding** ($w_3 = 0.2$): Keyword matching and co-occurrence bonus from nearby objects within 3m of the frontier.

4. **Scene graph context** ($w_4 = 0.3$): Subgraph-to-frontier interpolation using distance-weighted subgraph scores:
$$s_{\text{sg}}(f_j) = \frac{\sum_{g \in \mathcal{S}} \exp(-d(f_j, g) / \lambda) \cdot s_g}{\sum_{g \in \mathcal{S}} \exp(-d(f_j, g) / \lambda)}, \quad \lambda = 4.0\text{m}$$
where $\mathcal{S}$ is the set of scored subgraphs and $s_g$ is each subgraph's relevance sruntime.

5. **Vision score** ($w_5$, default 0.0, auto-enabled to 0.15):
$$s_{\text{vis}}(f_j) = \text{CLIP}_{\text{sim}}(I,\ \mathbf{f}_{\theta(f_j)}^{\text{img}})$$
where $\theta(f_j)$ is the angular direction of frontier $f_j$ from the robot, and $\mathbf{f}_{\theta}^{\text{img}}$ is the cached CLIP image feature from the nearest directional observation (8 angular bins of 45° each).

### 3.6.3 Subgraph Scoring

Subgraphs are extracted from $\mathcal{G}$ centered on each object node with its parent room/group nodes and connected neighbors. Each subgraph is scored by a weighted combination of heuristic and optional LLM assessment:

$$s_g = w_h \cdot s_g^{\text{heur}} + w_l \cdot s_g^{\text{llm}}$$

where $w_h = 0.45$, $w_l = 0.55$. The heuristic score combines:

$$s_g^{\text{heur}} = 0.55 \cdot s_{\text{kw}} + 0.20 \cdot s_{\text{rel}} + 0.15 \cdot s_{\text{prox}} + 0.10 \cdot s_{\text{rich}}$$

- $s_{\text{kw}} = |\text{keywords}(I) \cap \text{labels}(g)| / |\text{keywords}(I)|$: keyword overlap
- $s_{\text{rel}} = \min(1, |\text{relations}(g)| / 6)$: relation richness
- $s_{\text{prox}} = \exp(-d(g, \mathbf{p}_{\text{robot}}) / \lambda)$: proximity
- $s_{\text{rich}} = \min(1, |\text{objects}(g)| / 8)$: object count richness

**Room-Level Gating.** After scoring, we apply room-level gating: if a subgraph's parent room has a high relevance score, non-room subgraphs in different rooms are penalized, focusing exploration on semantically promising areas.

---

## 3.7 Topology-Aware Semantic Exploration (TSG)

When the target object is absent from the scene graph $\mathcal{G}$, the robot must decide **which direction to explore**. Prior work either relies on heuristic frontier scoring (SG-Nav [1]) or LLM-based common-sense reasoning (L3MVN [15]), but neither leverages the structural topology of the environment. We introduce a **Topological Semantic Graph (TSG)** that enables information-gain-driven exploration decisions.

### 3.7.1 Topological Semantic Graph Definition

The TSG $\mathcal{T} = (\mathcal{N}, \mathcal{E}_T)$ is a room-level abstraction of the scene graph where:

- **Room nodes** $n_k^{\text{room}}$: Each room from $\mathcal{G}$ with visit state $(v_k, c_k, t_k)$ — visited flag, visit count, last visit time.
- **Frontier nodes** $n_j^{\text{frontier}}$: Unexplored boundaries at the edge of known space, with direction $\mathbf{d}_j$ and predicted room type $\hat{r}_j$.
- **Topology edges** $e_{kl}$: Room connectivity through doors, passages, or proximity, with traversal memory $(n_{\text{trav}}, t_{\text{last}}, c_{\text{conf}})$.

Room connectivity is detected via three strategies (adapted from Hydra [6] and Concept-Guided Exploration [16]):
1. **Door-mediated**: A "door" object near two room centers → high-confidence edge
2. **Proximity**: Room centers within $2\epsilon_{\text{DBSCAN}}$ with boundary objects → medium-confidence edge
3. **Passage-mediated**: Corridor-type rooms connect to all nearby rooms

Frontier detection uses two complementary methods:
1. **Door-outward**: A door object at the boundary of a room with no corresponding room on the other side → a frontier pointing outward through that door
2. **Sparse-sector**: For rooms with $\geq 3$ objects, angular sectors with zero objects suggest unexplored directions

### 3.7.2 Information Gain Exploration (Algorithm 2)

For each node $n \in \mathcal{T}$, we compute an information gain score:

$$\text{IG}(n) = S_{\text{sem}}(n) \times N(n) \times U(n)$$

where:

- **Semantic prior** $S_{\text{sem}}(n)$: The probability that target $I$ exists in room type $r_n$, from a knowledge base $\mathcal{K}: \text{room\_type} \to \{(\text{object}, p)\}$ containing 10 room types and 100+ object-room associations. For frontier nodes, $S_{\text{sem}}$ uses the predicted room type $\hat{r}_j$.

- **Novelty** $N(n)$: Penalizes revisited rooms while allowing temporal recovery:
$$N(n) = \begin{cases} 1.0 & \text{if not visited} \\ \eta + (1-\eta) \cdot e^{-0.5 c_n} \cdot (1 - e^{-\Delta t / \tau_N}) & \text{if visited} \end{cases}$$
where $\eta = 0.1$ is the floor penalty, $c_n$ is visit count, and $\tau_N = 120$s.

- **Uncertainty reduction** $U(n)$: Frontier nodes and rooms with few known objects have higher uncertainty reduction potential.

The exploration target is selected by combining IG with topological reachability:

$$n^* = \arg\max_{n \in \mathcal{T}} \text{IG}(n) \times \frac{1}{1 + \lambda_R \cdot d_{\text{path}}(n_{\text{current}}, n)}$$

where $d_{\text{path}}$ is the Dijkstra shortest-path distance on $\mathcal{T}$ and $\lambda_R = 0.3$.

```
Algorithm 2: Topology-Aware Information Gain Exploration
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Input: Instruction I, TSG T, Knowledge base K, Robot position p
Output: Exploration target n*, waypoint p*

1:  Update T from scene graph G
2:  Update frontier nodes from scene boundary
3:  Record robot position → detect room transitions
4:  for each node n in T do
5:      S_sem ← K.score(n.room_type, I)
6:      N ← novelty(n.visited, n.visit_count, n.last_visited)
7:      U ← uncertainty(n)
8:      IG(n) ← S_sem × N × U
9:  end for
10: for each node n in T do
11:     d_path ← Dijkstra(current_room, n)
12:     score(n) ← IG(n) / (1 + λ_R · d_path)
13: end for
14: n* ← argmax score(n)
15: return n*, n*.center
```

### 3.7.3 Dual-Layer Exploration Strategy

The exploration module uses a two-layer strategy:

- **Layer 1 (TSG, ~1ms)**: If the TSG has sufficient room topology (≥2 rooms and edges), Algorithm 2 selects the exploration target without any LLM call. This is analogous to the Fast Path for goal resolution — a zero-cost, real-time decision.

- **Layer 2 (LLM, ~2s)**: If TSG coverage is insufficient or all IG scores are below threshold, the system falls back to LLM-based exploration advice. The TSG's topology summary (rooms with visit status, connections, frontiers) is injected into the LLM prompt, providing structured spatial context that improves the LLM's common-sense reasoning.

This dual-layer design ensures that exploration is both fast (TSG covers ~70% of decisions in a well-mapped environment) and robust (LLM handles early-stage or ambiguous cases).

### 3.7.4 Traversal Memory

The TSG records every room transition as a traversal event, building an increasingly accurate model of room connectivity. Traversal memory serves three purposes:
1. **Edge confidence**: Edges with actual traversals receive higher confidence, improving Dijkstra routing
2. **Negative memory**: Visited rooms that did not contain the target are penalized in future IG calculations
3. **LLM context**: The full traversal history is available as structured context for LLM-based exploration

---

## 3.8 Implementation Details

**Hardware.** Unitree Go2 quadruped robot equipped with:
- Compute: NVIDIA Jetson Orin NX 16GB
- RGB-D: Orbbec Femto (640×480 @ 30fps)
- LiDAR: Livox Mid-360
- SLAM: Fast-LIO2 with IMU-LiDAR fusion

**Software.** ROS2 Humble, Python 3.10, PyTorch, TensorRT.

**Models.**
- Detection: YOLO-World-L with TensorRT optimization (target >10 FPS)
- Encoding: CLIP ViT-B/32 (512-dim features, LRU-cached)
- LLM: GPT-4o / Claude (Slow Path only, via API)

**Parameters.** Key hyperparameters are summarized in Table 2.

**Table 2: Key hyperparameters.**

| Parameter | Value | Description |
|-----------|-------|-------------|
| $\epsilon_{\text{DBSCAN}}$ | 3.0 m | Region clustering radius |
| $\alpha_{\text{base}}$ | 0.3 | Base EMA rate for CLIP fusion |
| $\tau_{\text{fast}}$ | 0.75 | Fast Path confidence threshold |
| $K$ | 15 | Max objects for Slow Path |
| $\gamma$ | 0.9 | Credibility decay factor |
| $\tau_{\text{reject}}$ | 0.25 | Credibility rejection threshold |
| $\delta_{\text{fp}}$ | 0.2 | False positive penalty |
| $\Delta d_{\text{reperceive}}$ | 2.0 m | Continuous re-perception interval |
| $\lambda$ | 4.0 m | Subgraph interpolation decay |
| $d_{\text{nov}}$ | 5.0 m | Novelty normalization distance |
