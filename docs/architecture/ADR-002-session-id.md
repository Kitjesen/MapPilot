# ADR-002: Session identity and runtime allocation

Status: Accepted

`SessionSpec` provides a readable `session_id`. The Catalog Resolver writes the
resolved `session.yaml` and backend plans. Runtime consumers verify that these
files agree on `session_id`, package identity, schema, and referenced paths.

Per-run resources remain separate. DDS domains, ports, shared-memory names,
PIDs, log directories, and boot IDs belong to `RunAllocation`; changing one of
them does not create a different simulation session.
