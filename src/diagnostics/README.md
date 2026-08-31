# Diagnostics

This package contains checks used before or during field acceptance. It is not
part of ProductControl, does not switch Products, and does not publish robot
commands. Most checks are read-only. The Go2 no-motion harness is the one
exception: it owns the LiDAR and SLAM child processes that it starts, while
leaving every motion process stopped.

```text
diagnostics/
  saved_map_display.py   text output for the saved-map gate
  field/
    doctor.py            current Gateway/native readiness snapshot
    dds_readiness.py     one-shot native DDS Topic liveness report
    field_check.py       one read-only Product field summary
    gateway_acceptance.py
                         evaluate the current Gateway/readiness snapshots
    evidence.py          validate observed native DDS and real-driver evidence
    gates.py             the two executable field-gate descriptions
    inspection.py        saved-location path preview; never executes motion
    _preflight.py        shared parsing and result helpers for preflights
    teleop*_preflight.py Product-specific operator preflight checks
    go2_mid360_no_motion.py
                         Go2 + MID-360 field check with motion disabled
```

The main gate wrappers live in `scripts/gates/`. Doctor, DDS readiness, and the
Go2 no-motion harness can also run directly with `python -m`. Static Runtime
Graph, architecture, and Topic declaration checks stay with their existing
development validators instead of being repeated here.
