# ADR-002: Deterministic session lock and runtime allocation

Status: Accepted

`SessionSpec` describes the requested simulation. The pure Catalog Resolver
produces a content-addressed `session.lock.json`, a `session_digest`, and
backend-specific plans. Package versions, file hashes, MuJoCo version, seed,
and final resolved values are included in the digest.

Per-run resources are deliberately excluded. DDS domains, ports, shared-memory
names, PIDs, log directories, and boot IDs belong to a later `RunAllocation`.
Changing a port must not make the simulation content a different session.
