# Semantic Planner Tests

Tests for semantic planning and LLM reasoning: SemanticPlannerModule, GoalResolver (5-level resolution chain), VisualServoModule (BBoxNavigator + PersonTracker), AgentLoop (multi-turn LLM tool calling), and ActionExecutor.

```bash
python -m pytest src/semantic/planner/tests/ -q
```

No special markers required.
