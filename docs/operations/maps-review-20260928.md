# Maps fixes before field validation — 2026-09-28

This change repairs query and save responsibilities without changing planner
collision, ground, unknown-space, or SCAN execution rules.

- List/active-map serialization uses the activation result obtained under its
  existing map lock. Nested health serialization no longer acquires that lock.
- Query summaries read metadata, file presence, and the OctoMap header, without
  loading every PCD/OctoMap/occupancy payload. Actual activation and explicit
  artifact validation retain full payload validation.
- Saved-ray construction reuses the loaded retained points and skips the
  sampled-preview occupancy/support preparation. Ray hit/miss semantics stay
  unchanged.
- Default SaveMap/navigation-package output is the 3D OctoMap. The 2D projection
  is built only on explicit request or as an ESDF/traversability dependency.
  A rebuild removes omitted stale derived artifacts transactionally.
- Snapshot handoff, failed-save rollback, and active-map mutation exclusion
  remain in place. Saved-map activation remains owned by ProductControl.

## Local verification

MSVC Release, native OctoMap enabled, assertions enabled:
`lingtu_maps_store_test`, `lingtu_maps_save_map_test`, `lingtu_maps_ray_test`,
`lingtu_maps_map_activation_test`, and `lingtu_maps_mapd_service_dispatch_test`
passed. Coverage includes valid list/health agreement, full validation rejecting
invalid payloads, optional 2D artifacts, rollback, and active-map exclusion.
`npm --prefix web run build` passed (TypeScript and production bundle).

Existing build warnings remain in mapd engine double-to-float conversion and
mapctl getenv usage; the frontend reports a large Three.js chunk. These are
not introduced by this patch.

At the initial field check, small PC 192.168.66.95 and NX 192.168.123.18 were
reachable; NX still had .82 installed and the queried services were inactive.
These local checks do not establish MuJoCo or field navigation success.
Deployment and field results must be recorded separately after completion.
