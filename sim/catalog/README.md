# Simulation catalog

The catalog is the first boundary of the generic simulation Runtime. It reads
exact-version local packages and emits deterministic plans; it does not launch
MuJoCo, Unreal, DDS, or a controller process.

Resolve the checked-in ThunderV4 + OmniCart contract session with:

```powershell
python -m sim.catalog sim/scenarios/catalog/thunder_omni_contract/session.yaml `
  --repo-root . `
  --output-dir runs/catalog-contract
```

The output contains:

- `session.lock.json`: exact package references, final values, content hashes,
  and the stable `session_digest`.
- `physics.plan.json`: one session-level MuJoCo composition plan with one
  namespace per robot instance.

The `physics.plan.json` file is the planned input to the C++
`PhysicsSceneComposer`. Runtime allocation (DDS domain, ports, shared-memory
names, PIDs, and logs) is intentionally not part of this command or digest.
