# ADR-003: Separate package, session, and binding lifecycles

Status: Accepted

Package lifecycle is `DISCOVERED -> SCHEMA_VALIDATED -> INTEGRITY_VERIFIED ->
INDEXED -> AVAILABLE`, with quarantine on failure. Session lifecycle is
`DECLARED -> RESOLVING -> RESOLVED -> COMPILED -> STAGED -> PREPARING ->
READY -> RUNNING -> QUIESCING -> STOPPED`, with failure and rollback edges.

Robot bindings are independent dimensions (`physics`, `visual`, `sensors`,
`control`) rather than one mandatory linear state machine. Headless mode need
not bind visual assets; a passive robot need not bind a controller.
