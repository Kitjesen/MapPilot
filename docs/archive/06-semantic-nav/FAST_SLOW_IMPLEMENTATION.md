# Fast / Slow Goal Resolver

> "Where is the goal in 3D?" is answered by `GoalResolver`. It runs a System 1
> path (cheap, deterministic) and falls back to a System 2 path (LLM) only
> when System 1's confidence is low.
>
> Inspired by VLingNav (arXiv 2601.08665), OmniNav (ICLR 2026) and ESCA / SGCLIP
> (NeurIPS 2025) — the implementation is rule-driven, not a re-implementation
> of any single paper.

## File Layout

| File | Purpose |
|------|---------|
| `src/decision/semantic_planner/goal_resolver.py` | `GoalResolver` class, `GoalResult`, `TargetHypothesis`, `TargetBeliefManager`. Inherits from the two mixins below. |
| `src/decision/semantic_planner/fast_path.py` | `FastPathMixin.fast_resolve()` and helpers (CLIP attribute disambig, score entropy, distance decay). |
| `src/decision/semantic_planner/slow_path.py` | `SlowPathMixin.resolve()` (entry point), `_selective_grounding()`, LLM resolution. |
| `src/decision/semantic_planner/adacot.py` | `AdaCoTRouter` — entropy-based router that can force a Slow Path call when Fast Path looks ambiguous. |
| `src/decision/semantic_planner/goal_resolver_module.py` | Module wrapper; ports + thread integration. |
| `src/decision/semantic_planner/llm_client.py` | Backend factory (`openai`, `claude`, `qwen`, `moonshot`/`kimi`, `mock`). |

The class composition is

```python
class GoalResolver(FastPathMixin, SlowPathMixin):
    fast_path_threshold: float = 0.75
    confidence_threshold: float = 0.6
```

## Flow

```
instruction + scene_graph_json + (optional) clip_encoder
        ↓
SlowPathMixin.resolve()
        ├─ fast_resolve() ──────────► GoalResult(path="fast")  if best_score >= 0.75
        │                                       and AdaCoT does not force slow
        │
        └─ _selective_grounding()  → filtered scene graph (~15 objects max)
              ↓
            LLM call (selected via llm_client.create_llm_client)
              ↓
            GoalResult(path="slow")
```

## Fast Path Details

### 1. Tokenisation

`_extract_keywords()` uses jieba for Chinese with a regex fallback for
English / ASCII text. The stop-word list is shared with the task decomposer
(`task_rules.TaskRulesMixin`).

### 2. Multi-source Scoring

Weights live at the top of `goal_resolver.py`:

```python
WEIGHT_LABEL_MATCH    = 0.35   # text label match
WEIGHT_CLIP_SIM       = 0.35   # CLIP text-image similarity
WEIGHT_DETECTOR_SCORE = 0.15   # detector confidence × observation count
WEIGHT_SPATIAL_HINT   = 0.15   # spatial-relation hint hit
```

Per-candidate scoring (in `fast_path.py`):

| Source | Computation |
|--------|-------------|
| Label | 1.0 if `label in instruction`; 0.9 if `instruction in label`; 0.7 if any keyword overlap; 0 otherwise. |
| Detector | `min(score, 1.0) * min(detection_count / 3, 1.0)`. |
| CLIP | If a CLIP encoder and the object's `clip_feature` are present, call `clip_encoder.text_image_similarity(instruction, [feat])`. Otherwise fall back to `0.8 * label_score`. |
| Spatial | 1.0 if a relation chain links this object to another object whose label appears in the instruction (e.g. "the chair next to the door"); 0.3 if there is a `near` relation. |

The fused score is a weighted sum.

### 3. CLIP Attribute Disambiguation

When the top-N candidates share the same core noun ("red chair" vs "blue
chair"), `_clip_attribute_disambiguate` re-ranks them with the CLIP text
embedding so attributes carry weight even though the labels are identical.

### 4. Distance Decay

If `robot_position` is provided and a candidate within 10 % of `best_score`
is at half the distance, the closer candidate wins. This avoids walking past
a closer match because it scored marginally lower.

### 5. Threshold Decision

```python
if best_score < self._fast_path_threshold:
    logger.info("Fast path %.2f < %.2f — defer to Slow", best_score,
                self._fast_path_threshold)
    return None    # Slow Path takes over
```

Returned `GoalResult` carries `path="fast"` and `score_entropy` (used by the
AdaCoT router).

## Slow Path Details

`SlowPathMixin.resolve()` is the public async entry point:

1. Try Fast Path first.
2. If Fast Path returned a result *and* AdaCoT agrees (entropy < 1.5 or
   confidence ≥ 0.85), return it.
3. Otherwise call `_selective_grounding(instruction, scene_graph_json,
   max_objects=15, max_relations=20, clip_encoder=...)`.
4. Build the LLM prompt via `prompt_templates.build_goal_resolution_prompt`
   and dispatch through `LLMClientBase.chat`.
5. Parse the JSON reply, sanitize the position, return `GoalResult(path="slow")`.

### Selective Grounding

Implemented in `slow_path.py:_selective_grounding`:

1. Optional CLIP semantic ranking of all object labels against the
   instruction.
2. Keyword match seeds the relevant set.
3. 1-hop relation expansion adds neighbouring objects (SG-Nav style).
4. Region expansion adds same-region objects when the instruction mentions
   the region label.
5. Cap at `max_objects` / `max_relations` to keep the LLM prompt small.

### LLM Backends

| Backend alias | Class | Env var |
|---------------|-------|---------|
| `openai`, `gpt` | `OpenAIClient` | `OPENAI_API_KEY` |
| `claude`, `anthropic` | `ClaudeClient` | `ANTHROPIC_API_KEY` |
| `qwen`, `dashscope` | `QwenClient` | `DASHSCOPE_API_KEY` |
| `moonshot`, `kimi` | `MoonshotClient` (subclass of OpenAIClient) | `MOONSHOT_API_KEY` |
| `mock`, `offline`, `test` | `MockLLMClient` | — |

Selected via `create_llm_client(LLMConfig(backend=..., model=...))`.

## Module Wiring

`goal_resolver_module.py` wraps the resolver in a Module. Key knobs:

```python
GoalResolverModule(
    fast_path_threshold=0.75,
    entropy_threshold=1.5,        # AdaCoT escalation
    confidence_threshold=0.85,    # AdaCoT pass-through
    save_dir="...",               # tagged-locations + KG persistence
)
```

`maybe_reload_kg()` is called by `SemanticPlannerModule` every 120 s to pick
up updates to the room-object KG without restarting the process.

## Empirical Notes

- Per-call Fast Path timing on S100P sits in the sub-millisecond range for
  scene graphs up to ~50 objects; the dominant cost is the optional CLIP call.
- Slow Path latency tracks the LLM provider; Kimi over the Chinese mainland
  routes runs at ~1.5–2 s for the 15-object selective prompt.
- The "fast wins ~70 %" figure quoted by VLingNav is consistent with our
  preliminary logs but has not been independently benchmarked on LingTu data.

## Related Tests

| Test | Coverage |
|------|----------|
| `test_fast_resolve.py` | Fast Path candidate scoring + attribute disambig |
| `test_goal_resolver.py` | End-to-end Fast→Slow flow with mocked LLM |
| `test_slow_path_llm.py`, `test_slow_path_real_llm.py` | Selective grounding + real LLM contract |
| `test_fast_slow_benchmark.py`, `test_fast_slow_efficiency.py` | Timing harness |
| `test_uncertainty_scoring.py` | Score-entropy / AdaCoT routing |
