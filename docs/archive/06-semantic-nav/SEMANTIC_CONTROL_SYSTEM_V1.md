# Semantic Command Parser

> Reference for the rule-based command parser that lives in
> `src/decision/semantic_planner/task_decomposer.py` +
> `task_rules.py`. It maps natural-language utterances into a
> `TaskPlan` of atomic `SubGoal`s.
>
> The parser is **rule-first, LLM-fallback**: simple commands resolve in
> microseconds without an LLM call; only complex / multi-step / conditional
> utterances escalate to `LLMClientBase.chat`.

## 1. Pipeline Overview

```
Utterance
   │
   ├── Complexity Guard (token / punctuation heuristics) ──► LLM path
   │
   ├── Rule Engine (longest-prefix wins, English + Chinese)
   │     ├── 0  STOP    → [STOP]
   │     ├── 1  STATUS  → [STATUS]
   │     ├── 2  EXPLORE → [EXPLORE]
   │     ├── 3  PLACE   → [NAVIGATE → PLACE]
   │     ├── 4  PICK    → [FIND → APPROACH → PICK]
   │     ├── 5  INSPECT → [FIND → LOOK_AROUND → APPROACH → VERIFY]
   │     ├── 6  FOLLOW  → [FIND → FOLLOW]
   │     ├── 7  NAV     → [NAVIGATE → APPROACH → VERIFY]
   │     ├── 8  FIND    → [FIND → LOOK_AROUND → NAVIGATE → APPROACH → VERIFY]
   │     ├── 9  Conversational regex (zh)
   │     └── 10 Conversational regex (en)
   │
   └── No rule fires ──► LLM path
```

Rules:

1. **Rule first, LLM as fallback.** A successful rule match is < 5 ms.
2. **Longest prefix wins.** All `SIMPLE_*_PATTERNS_*` lists are sorted by
   length descending so "导航到" cannot be eaten by "到".
3. **Complexity guard escalates early.** Conditionals, multi-step, and
   periodic instructions go straight to the LLM.
4. **Action verbs are not stopwords.** "go", "find", "take", "bring" carry
   intent and must survive into the parser even though they are stripped
   during noun extraction.

## 2. SubGoalAction Enum

Defined in `task_decomposer.py` (real source — keep this list in sync):

```python
class SubGoalAction(Enum):
    NAVIGATE      = "navigate"
    FIND          = "find"
    APPROACH      = "approach"
    VERIFY        = "verify"
    LOOK_AROUND   = "look_around"
    EXPLORE       = "explore"
    BACKTRACK     = "backtrack"
    WAIT          = "wait"
    FOLLOW        = "follow"
    STOP          = "stop"
    PICK          = "pick"
    PLACE         = "place"
    STATUS        = "status"
    PATROL        = "patrol"
    SAVE_MAP      = "save_map"
    SAVE_POI      = "save_poi"
    SET_SPEED     = "set_speed"
    SET_GEOFENCE  = "set_geofence"
    RETURN_HOME   = "return_home"
    PAUSE         = "pause"
    RESUME        = "resume"
```

`SubGoalStatus` is the standard PENDING / ACTIVE / COMPLETED / FAILED /
SKIPPED set.

## 3. Pattern Tables (high level)

The actual lists are kept in `task_rules.TaskRulesMixin`. They are the
source of truth — the numbers below are indicative.

| Group | Variable | Approx count | Examples |
|-------|----------|--------------|----------|
| Navigate (zh) | `SIMPLE_NAV_PATTERNS_ZH` | ~40 | 去, 到, 走到, 导航到, 带我去, 立刻去 |
| Navigate (en) | `SIMPLE_NAV_PATTERNS_EN` | ~19 | go to, navigate to, head to, swing by, drop by |
| Find (zh) | `SIMPLE_FIND_PATTERNS_ZH` | ~42 | 找, 寻找, 帮我找, 锁定 |
| Find (en) | inline list | ~10 | find, search for, locate, where is |
| Inspect (zh) | `SIMPLE_INSPECT_PATTERNS_ZH` | ~21 | 检查, 巡检, 查验 |
| Inspect (en) | `SIMPLE_INSPECT_PATTERNS_EN` | 3 | inspect, examine, audit |
| Follow (zh) | `SIMPLE_FOLLOW_PATTERNS_ZH` | ~30 | 跟着, 跟随, 紧跟 |
| Follow (en) | `SIMPLE_FOLLOW_PATTERNS_EN` | ~10 | follow, track, stay with |
| Explore (zh) | `SIMPLE_EXPLORE_PATTERNS_ZH` | ~20 | 探索, 巡视, 勘察 |
| Explore (en) | inline list | ~5 | explore, look around, survey |
| Stop (zh) | `SIMPLE_STOP_PATTERNS_ZH` | ~26 | 停, 停下, 取消, 急停 |
| Stop (en) | inline list | ~9 | stop, halt, cancel, abort |
| Pick (zh) | `SIMPLE_PICK_PATTERNS_ZH` | ~37 | 拿, 取, 帮我拿, 抓取 |
| Pick (en) | `SIMPLE_PICK_PATTERNS_EN` | ~13 | pick up, grab, fetch, bring me |
| Place (zh) | `SIMPLE_PLACE_PATTERNS_ZH` | ~30 | 放, 放下, 摆到, 归位 |
| Place (en) | `SIMPLE_PLACE_PATTERNS_EN` | ~10 | put, place, drop, set down |
| Status (zh) | `SIMPLE_STATUS_PATTERNS_ZH` | ~31 | 电量, 状态, 当前位置 |
| Status (en) | `SIMPLE_STATUS_PATTERNS_EN` | ~22 | battery level, system status |
| Conv. find (zh) | `CONVERSATIONAL_FIND_RE_ZH` | ~22 | 灭火器在哪 / 哪里有椅子 / 带我去会议室 |
| Conv. find (en) | `CONVERSATIONAL_FIND_RE_EN` | ~19 | where is X / show me X / let's find X |

`check X` → INSPECT, but `check if X` / `check whether X` → LLM (conditional).

## 4. Complexity Guard

A short list of tokens trips the guard. Any hit pushes the utterance to the
LLM path even if a prefix would otherwise match.

| Type | Tokens (excerpt) |
|------|------------------|
| Conditional (zh) | 如果, 否则 |
| Sequence (zh) | 然后再, 接着, 之后再, 然后, 接着, 再去, 再找 |
| Conjunction (zh) | 并且, 而且, 同时, 以及 |
| Quantification (zh) | 每个, 每一个, 所有, 全部, 依次, 逐一 |
| Periodicity (zh) | 每隔, 定期, 循环, 反复 |
| Conditional (en) | if, else, otherwise |
| Sequence (en) | then, and then, after that, before, once done |
| Quantification (en) | every, each, all |
| Periodicity (en) | repeat, periodically, continuously |

Punctuation rule: ≥ 2 commas (Chinese `，` or English `,`) → multi-step → LLM.

## 5. Stopwords

`STOP_WORDS` is consumed by `ChineseTokenizer` only during **noun
extraction**. It does **not** delete intent verbs (`go`, `find`, `take`,
`see`) which are needed for intent detection earlier in the pipeline.

The list contains roughly:

- ~50 English tokens — articles, prepositions, pronouns, modals, fillers,
  quantifiers.
- ~100+ Chinese tokens — particles, modal markers, dialect fillers, verbs of
  inspection ("看一看", "查一下"), question words, polite framing ("帮我",
  "麻烦"), negations, intensifiers, etc.

Full table is in `chinese_tokenizer.py` and `task_rules.py`.

## 6. SubGoal Chains by Intent

| Intent | Resulting chain |
|--------|-----------------|
| STOP | `[STOP]` |
| STATUS | `[STATUS]` |
| EXPLORE | `[EXPLORE]` |
| PLACE | `[NAVIGATE → PLACE]` |
| PICK | `[FIND → APPROACH → PICK]` |
| INSPECT | `[FIND → LOOK_AROUND → APPROACH → VERIFY]` |
| FOLLOW | `[FIND → FOLLOW]` |
| NAVIGATE | `[NAVIGATE → APPROACH → VERIFY]` |
| FIND | `[FIND → LOOK_AROUND → NAVIGATE → APPROACH → VERIFY]` |

## 7. Dispatch in the Module Stack

| SubGoalAction | Where it lands |
|---------------|----------------|
| NAVIGATE | `SemanticPlannerModule.servo_target` → `GoalResolver.resolve()` → `NavigationModule.goal_pose` |
| FIND | `GoalResolver.fast_resolve()`; on miss, `VisualServoModule.servo_target = "find:<label>"` |
| APPROACH | `VisualServoModule.servo_target = "find:..."` (close-range PD via `BBoxNavigator`) |
| VERIFY | VLM bbox query (`vlm_bbox_query`) for confirmation |
| LOOK_AROUND | `NavigationModule.recovery_cmd_vel` rotates in place |
| EXPLORE | `NavigationModule` exploration mode (frontier + topology graph) |
| BACKTRACK | `NavigationModule` revisits previous waypoint |
| WAIT | timer in `SemanticPlannerModule` |
| FOLLOW | `VisualServoModule.servo_target = "follow:..."` (PersonTracker) |
| STOP | `SafetyRingModule.stop_cmd` + cancel mission |
| PICK / PLACE | currently no manipulator on Thunder; no-op + mission failure |
| STATUS | gateway `/api/v1/health` |
| Others (PATROL, SAVE_MAP, …) | dispatched by `nav_services/*_module.py` if registered |

## 8. Tests

Located in `src/decision/tests/`:

| Test | Focus |
|------|-------|
| `test_task_decomposer.py` | Rule engine + complexity guard |
| `test_intent_parsing.py` | Conversational regex + multilingual coverage |
| `test_episodic_memory.py`, `test_tagged_locations.py` | Memory side-effects |
| `test_action_executor.py` | Dispatch into module ports |
| `test_fast_resolve.py`, `test_goal_resolver.py` | Fast / Slow path |
| `test_slow_path_real_llm.py` | Live LLM contract (skipped without API keys) |

## 9. Source Files

| File | Role |
|------|------|
| `src/decision/semantic_planner/task_decomposer.py` | `SubGoal*` dataclasses + LLM prompt builder |
| `src/decision/semantic_planner/task_rules.py` | All pattern tables + rule engine |
| `src/decision/semantic_planner/chinese_tokenizer.py` | jieba wrapper + stop-word handling |
| `src/decision/semantic_planner/task_decomposer_module.py` | Module wrapper |
| `src/decision/semantic_planner/action_executor.py` | Dispatches resolved SubGoals to module ports |
| `src/decision/semantic_planner/agent_loop.py` | Multi-turn LLM tool-call loop (used when the rule engine + Fast Path can't resolve) |
| `src/decision/semantic_planner/llm_client.py` | OpenAI / Claude / Qwen / Moonshot / Mock backends |

## 10. Open Items

- PATROL multi-waypoint loop is enumerated but the executor relies on
  `PatrolManagerModule`, which only ships a basic implementation.
- Multi-target collection ("find all fire extinguishers") still goes through
  the LLM path; there is no native FIND-LOOP rule yet.
- "最近的 / nearest" comparators are detected but not used by the resolver to
  re-rank candidates beyond the simple distance decay in `fast_path.py`.
- Multi-turn disambiguation ("not that one, the other one") is handled only
  when the agent loop is engaged.
