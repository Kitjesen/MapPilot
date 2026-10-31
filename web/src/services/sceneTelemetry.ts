import type { OdometryEvent, PathPoint } from '../types/index.ts'

type RecordValue = Record<string, unknown>

/** The coordinate era comes from the published session and SLAM reset identity. */
export function scenePoseEpoch(sessionId: unknown, localization: RecordValue | null | undefined): string | null {
  const session = typeof sessionId === 'string' && sessionId ? sessionId : null
  const runtime = typeof localization?.runtime_instance_id === 'string' && localization.runtime_instance_id
    ? localization.runtime_instance_id : null
  const jump = localization?.map_frame_jump_sequence
  const jumpSequence = typeof jump === 'number' && Number.isInteger(jump) && jump >= 0 ? jump : null
  return session !== null || runtime !== null || jumpSequence !== null
    ? JSON.stringify([session, runtime, jumpSequence]) : null
}

export function sceneTrailStorageKey(map: unknown, poseEpoch: string | null): string | null {
  return poseEpoch === null ? null : `lingtu.trail.${JSON.stringify([typeof map === 'string' ? map : null, poseEpoch])}`
}

export function freshSource(stamp: unknown, nowS: number, maxAgeS = 3): boolean {
  return typeof stamp === 'number' && Number.isFinite(stamp)
    && stamp > 0 && nowS - stamp <= maxAgeS && nowS - stamp >= -5
}

/** Simulation poses age with source progress, independently of physics running slower than wall time. */
export function freshScenePose(stamp: unknown, nowS: number, env: unknown,
  localization: RecordValue | null | undefined): boolean {
  if (env !== 'sim') return freshSource(stamp, nowS)
  const sourceStamp = localization?.ts
  const receivedStamp = localization?.diag_received_ts
  const raw = localization?.raw as RecordValue | null | undefined
  const sourceAge = raw?.source_data_age_s
  if (typeof sourceStamp !== 'number' || !Number.isFinite(sourceStamp) || sourceStamp <= 0
    || typeof receivedStamp !== 'number' || !Number.isFinite(receivedStamp) || receivedStamp <= 0
    || typeof sourceAge !== 'number' || !Number.isFinite(sourceAge) || sourceAge < 0) return false
  return freshSource(stamp, sourceStamp + sourceAge + Math.max(0, nowS - receivedStamp))
}

/** Compare robot telemetry in the Gateway clock, continuing to age after disconnect. */
export function estimateSceneTime(localNowS: number, snapshotStamp: unknown, receivedAtMs: number | null | undefined): number {
  if (typeof snapshotStamp !== 'number' || !Number.isFinite(snapshotStamp) || snapshotStamp <= 0
    || typeof receivedAtMs !== 'number' || !Number.isFinite(receivedAtMs) || receivedAtMs <= 0) return localNowS
  return snapshotStamp + Math.max(0, localNowS - receivedAtMs / 1000)
}

/** Native SLAM publishes map poses; the viewer never resolves runtime TF. */
export function projectScenePose(odom: OdometryEvent | null): OdometryEvent | null {
  return odom?.frame_id === 'map' && [odom.x, odom.y, odom.z ?? 0, odom.yaw].every(Number.isFinite) ? odom : null
}

/** A clicked map point needs a heading from the same valid map-frame pose. */
export function sceneRelocalizationSeed(x: number, y: number, mapPose: OdometryEvent | null): { x: number; y: number; z: number; yaw: number } | null {
  if (!mapPose || mapPose.frame_id !== 'map' || typeof mapPose.z !== 'number' || ![x, y, mapPose.z, mapPose.yaw].every(Number.isFinite)) return null
  return { x, y, z: mapPose.z, yaw: mapPose.yaw }
}

/** Automatic matching targets the running session, not a manual-panel selection. */
export function activeMapRelocalizationTarget(session: { active_map?: unknown } | null | undefined): string | null {
  return typeof session?.active_map === 'string' && session.active_map ? session.active_map : null
}

export function projectScenePath(points: PathPoint[], frameId: unknown): PathPoint[] {
  return frameId === 'map' ? points : []
}

export function currentNativeLocalPath(native: RecordValue | null | undefined,
  connected: boolean, nowS: number): PathPoint[] {
  if (!connected || !native || !freshSource(native.stamp_s, nowS)) return []
  const local = native.last_local as RecordValue | undefined
  const tracking = local?.tracking as RecordValue | undefined
  if (tracking?.active !== true || !Array.isArray(native.local_path)) return []
  const points = native.local_path
    .map(p => Array.isArray(p) ? { x: p[0], y: p[1], z: p[2] ?? 0 } : p)
    .filter((p): p is PathPoint => p != null && Number.isFinite(p.x) && Number.isFinite(p.y))
  return projectScenePath(points, native.planning_frame_id)
}

export function scanCanStandAlone(scan: { count: number; frameId: string | null; epoch: number | null },
  map: { count: number; frameId: string | null; epoch: number | null }): boolean {
  return map.count === 0 && scan.count > 0 && scan.frameId === 'map' && scan.epoch !== null
    && (map.epoch === null || (map.epoch === scan.epoch && (map.frameId === null || map.frameId === 'map')))
}
