# nav.skills

`nav.skills` is the L6 MCP/agent adapter for navigation. It does not plan,
track paths, own patrol state, or publish emergency-stop signals.

```text
MCP / Agent
  -> nav.skills.goal_command
  -> nav.goals.goal_command
  -> nav.mission or native C++ DDS endpoint
  <- nav.goals.goal_status
  <- nav.mission.mission_status
```

## Ownership

- `NavSkills`: MCP schemas, command submission, command ACKs, status reads.
- `GoalService`: validation, frame normalization, module/native endpoint routing.
- `Navigation`: mission FSM, planning, recovery, and patrol execution.
- `SafetyRing`: hardware emergency stop.
- `SemanticPlannerModule`: free-text semantic instructions.

The class is `NavSkills` and its runtime identity is `nav.skills`.
