# Camera Transport Decision

Status: accepted (2026-07-13)
Audience: Gateway/Web/media maintainers
Replaced by: not replaced

## Decision

Use go2rtc WHEP as the low-latency browser camera path and Gateway
JPEG-over-WebSocket as the fallback. The Gateway bootstrap advertises WHEP
support; live sidecar health is queried separately through
`GET /api/v1/webrtc/go2rtc/status`.

This decision is browser transport only. The robot-side camera data plane is a
native camera service with POSIX SHM for image payloads and typed DDS/status
metadata such as `rt/camera/info`; readiness checks reject field claims when
the camera status does not report `data_plane=posix_shm`. Gateway may cache and
serve JPEG frames for `/ws/camera` and `/api/v1/camera/snapshot`, but that is
the UI fallback, not the primary field image transport.

The old in-process Python WebRTC module, signalling routes, bitrate control,
stats endpoint, and browser hook are removed as one vertical slice.

## Alternatives considered

- Keep the old routes returning 503: rejected because it preserves a dead API
  contract and falsely suggests a recoverable runtime feature.
- Return 410 during a deprecation period: rejected because no active consumer
  was found and the dashboard already has retained transports.
- Remove WHEP as well: rejected because it is the intended low-latency path.
- Hard-delete only the old in-process implementation: chosen because it leaves
  one WebRTC implementation plus one protocol-independent fallback.

## Consequences

If go2rtc is unavailable, the dashboard falls directly back to JPEG after one
probe or timeout; there is no intermediate low-latency implementation. This
reduces encoder ownership, signalling, schema, and peer-state duplication.

Rollback is a source-control revert of this decision's code changes. No
dormant compatibility shell is kept in the runtime.
