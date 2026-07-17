"""Decision-layer package.

Functional packages:

- modules: production runtime Module entrypoints.
- goals: fast/slow goal grounding and SG-Nav reasoning.
- llm: LLM clients and prompt builders.
- tasks: agent loop, task decomposition, execution, and planner state.
- frontiers: frontier scoring and exploration strategies.
- vision: visual servo helpers, VLM queries, person tracking, and Re-ID.
"""

from decision.backends import BackendManager
