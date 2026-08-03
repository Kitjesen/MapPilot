# ADR-005: Embedded-first, headless-compatible Runtime topology

Status: Accepted

The first physics implementation is a reusable C++ Runtime that can run
headless and can later be embedded behind the Unreal Runtime. UE consumes the
same VisualPlan/SensorPlan and never becomes a second source of MuJoCo truth.

The initial cross-system seam is the generated plan, followed by the existing
typed DDS and camera shared-memory contracts. No new generic bridge is assumed
to understand LingTu custom types, and the simulation backend remains internal
to the existing `env=sim` architecture.
