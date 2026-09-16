import { Quaternion } from 'three'
import type { OdometryEvent, PathPoint } from '../types/index.ts'

type RecordValue = Record<string, unknown>
type MapTransform = { translation: number[]; rotation: number[] }

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

/** Compare robot telemetry in the Gateway clock, continuing to age after disconnect. */
export function estimateSceneTime(localNowS: number, snapshotStamp: unknown, receivedAtMs: number | null | undefined): number {
  if (typeof snapshotStamp !== 'number' || !Number.isFinite(snapshotStamp) || snapshotStamp <= 0
    || typeof receivedAtMs !== 'number' || !Number.isFinite(receivedAtMs) || receivedAtMs <= 0) return localNowS
  return snapshotStamp + Math.max(0, localNowS - receivedAtMs / 1000)
}

function mapTransform(value: unknown): MapTransform | null {
  if (!value || typeof value !== 'object') return null
  const tf = value as RecordValue
  if (tf.valid !== true || tf.frame_id !== 'map' || tf.child_frame_id !== 'odom') return null
  const translation = [tf.tx, tf.ty, tf.tz]
  const rotation = [tf.qx, tf.qy, tf.qz, tf.qw]
  if (![...translation, ...rotation].every(v => typeof v === 'number' && Number.isFinite(v))) return null
  const norm = Math.hypot(...rotation as number[])
  if (norm < 1e-9) return null
  return { translation: translation as number[], rotation: (rotation as number[]).map(v => v / norm) }
}

function rotate(point: number[], q: number[]): number[] {
  const [x, y, z] = point
  const [qx, qy, qz, qw] = q
  const tx = 2 * (qy * z - qz * y)
  const ty = 2 * (qz * x - qx * z)
  const tz = 2 * (qx * y - qy * x)
  return [x + qw * tx + qy * tz - qz * ty,
    y + qw * ty + qz * tx - qx * tz,
    z + qw * tz + qx * ty - qy * tx]
}

export function projectScenePose(odom: OdometryEvent | null, transform: unknown): OdometryEvent | null {
  if (!odom || ![odom.x, odom.y, odom.z ?? 0, odom.yaw].every(Number.isFinite)) return null
  if (odom.frame_id === 'map') return odom
  const tf = odom.frame_id === 'odom' ? mapTransform(transform) : null
  if (!tf) return null
  const position = rotate([odom.x, odom.y, odom.z ?? 0], tf.rotation)
    .map((v, axis) => v + tf.translation[axis])
  const orientation = odom.orientation
    ? new Quaternion().fromArray(tf.rotation).multiply(new Quaternion().fromArray(odom.orientation)).normalize().toArray()
    : undefined
  const forward = orientation ? rotate([1, 0, 0], orientation)
    : rotate([Math.cos(odom.yaw), Math.sin(odom.yaw), 0], tf.rotation)
  return { ...odom, orientation, frame_id: 'map', x: position[0], y: position[1], z: position[2],
    yaw: Math.atan2(forward[1], forward[0]) }
}

/** A clicked map point needs a heading from the same valid map-frame pose. */
export function sceneRelocalizationSeed(x: number, y: number, mapPose: OdometryEvent | null): { x: number; y: number; yaw: number } | null {
  if (!mapPose || mapPose.frame_id !== 'map' || ![x, y, mapPose.yaw].every(Number.isFinite)) return null
  return { x, y, yaw: mapPose.yaw }
}

/** Automatic matching targets the running session, not a manual-panel selection. */
export function activeMapRelocalizationTarget(session: { active_map?: unknown } | null | undefined): string | null {
  return typeof session?.active_map === 'string' && session.active_map ? session.active_map : null
}

export function projectScenePath(points: PathPoint[], frameId: unknown, transform: unknown): PathPoint[] {
  if (frameId === 'map') return points
  const tf = frameId === 'odom' ? mapTransform(transform) : null
  if (!tf) return []
  return points.filter(p => [p.x, p.y, p.z ?? 0].every(Number.isFinite)).map(point => {
    const p = rotate([point.x, point.y, point.z ?? 0], tf.rotation)
      .map((v, axis) => v + tf.translation[axis])
    return { ...point, x: p[0], y: p[1], z: p[2], frame_id: 'map' }
  })
}

export function currentNativeLocalPath(native: RecordValue | null | undefined,
  connected: boolean, nowS: number, transform: unknown): PathPoint[] {
  if (!connected || !native || !freshSource(native.stamp_s, nowS)) return []
  const local = native.last_local as RecordValue | undefined
  const tracking = local?.tracking as RecordValue | undefined
  if (tracking?.active !== true || !Array.isArray(native.local_path)) return []
  const points = native.local_path
    .map(p => Array.isArray(p) ? { x: p[0], y: p[1], z: p[2] ?? 0 } : p)
    .filter((p): p is PathPoint => p != null && Number.isFinite(p.x) && Number.isFinite(p.y))
  return projectScenePath(points, native.planning_frame_id, transform)
}

export function scanCanStandAlone(scan: { count: number; frameId: string | null; epoch: number | null },
  map: { count: number; frameId: string | null; epoch: number | null }): boolean {
  return map.count === 0 && scan.count > 0 && scan.frameId === 'map' && scan.epoch !== null
    && (map.epoch === null || (map.epoch === scan.epoch && (map.frameId === null || map.frameId === 'map')))
}
