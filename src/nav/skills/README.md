# nav.skills

`nav.skills` is the L6 MCP/agent adapter for navigation. It does not plan,
track paths, own patrol state, or publish emergency-stop signals.

```text
MCP / Agent
  -> nav.skills.goal_command
  -> nav.goals.goal_command
  -> nav.commands
  -> native C++ DDS endpoint
  <- nav.goals.goal_status
  <- native navigation status
```

## Ownership

- `NavSkills`: MCP schemas, command submission, command ACKs, status reads.
- `GoalService`: validation, frame normalization, task identity, and native command dispatch.
- Native `navd`: planning, route execution, following, safety, and final velocity.
- `SafetyRing`: hardware emergency stop.
- `SemanticPlannerModule`: free-text semantic instructions.

The class is `NavSkills` and its runtime identity is `nav.skills`.

Live status, progress, and activity reads share one snapshot. If no navigation
state arrives for two seconds (the Host bus default freshness window), reads
return `UNKNOWN` with `navigation_state_stale`. Completed request results remain
queryable; they are history, not live state. No polling timer is added.

`navigate_to` and `navigate_to_deg` preserve an explicit map-frame `z`. When
omitted, `z` uses the latest robot position through the existing odometry and
`map_odom_tf` ports. Odom-frame positions use the canonical map-from-odom
transform; map-frame positions are not transformed twice. These observations
must have arrived within two seconds. Without a usable position/transform,
the caller must supply `z`; map zero is not a valid height default. This is
coordinate completion, not a second localization or navigation gate.
