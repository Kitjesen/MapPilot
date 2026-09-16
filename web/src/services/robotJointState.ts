export const GO2_JOINT_NAMES = [
  'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
  'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
  'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
  'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
] as const

export type Go2JointName = typeof GO2_JOINT_NAMES[number]

export interface Go2JointStateEvent {
  type: 'joint_state'
  stamp: number
  source_age_s: number
  robot_model: 'go2'
  names: string[]
  position: number[]
  velocity: number[]
  effort: number[]
}

export interface Go2JointSample {
  stamp: number
  sourceAgeS: number
  receivedAtMs: number
  positions: Readonly<Record<Go2JointName, number>>
}

export const GO2_JOINT_SOURCE_MAX_AGE_S = 0.5

export type Go2JointSampleStatus = 'fallback' | 'live' | 'stale'

export function go2JointSampleStatus(
  sample: Go2JointSample | null | undefined,
  nowMs: number,
  maxSourceAgeS = GO2_JOINT_SOURCE_MAX_AGE_S,
): Go2JointSampleStatus {
  if (!sample) return 'fallback'
  if (!Number.isFinite(nowMs) || !Number.isFinite(sample.receivedAtMs)
    || !Number.isFinite(sample.sourceAgeS)) return 'stale'
  const elapsedS = Math.max(0, nowMs - sample.receivedAtMs) / 1000
  return sample.sourceAgeS + elapsedS > maxSourceAgeS ? 'stale' : 'live'
}

const EXPECTED_NAMES = new Set<string>(GO2_JOINT_NAMES)

function record(value: unknown): Record<string, unknown> | null {
  return value !== null && typeof value === 'object' ? value as Record<string, unknown> : null
}

/**
 * Validate one complete Go2 joint snapshot. Source freshness comes from the
 * backend-provided age so browser and robot wall clocks are never compared.
 */
export function parseGo2JointState(
  value: unknown,
  receivedAtMs: number,
  maxSourceAgeS = GO2_JOINT_SOURCE_MAX_AGE_S,
): Go2JointSample | null {
  const event = record(value)
  if (!event || event.type !== 'joint_state' || event.robot_model !== 'go2') return null
  if (!Number.isFinite(receivedAtMs)) return null
  if (typeof event.stamp !== 'number' || !Number.isFinite(event.stamp) || event.stamp <= 0) return null
  if (typeof event.source_age_s !== 'number' || !Number.isFinite(event.source_age_s)
    || event.source_age_s < 0 || event.source_age_s > maxSourceAgeS) return null
  if (!Array.isArray(event.names) || !Array.isArray(event.position)
    || !Array.isArray(event.velocity) || !Array.isArray(event.effort)
    || event.names.length !== GO2_JOINT_NAMES.length
    || event.position.length !== GO2_JOINT_NAMES.length) return null

  const names = event.names as unknown[]
  const positions = event.position as unknown[]
  const seen = new Set<string>()
  const mapped = {} as Record<Go2JointName, number>
  for (let index = 0; index < names.length; index++) {
    const name = names[index]
    const position = positions[index]
    if (typeof name !== 'string' || !EXPECTED_NAMES.has(name) || seen.has(name)
      || typeof position !== 'number' || !Number.isFinite(position)) return null
    seen.add(name)
    mapped[name as Go2JointName] = position
  }
  if (seen.size !== GO2_JOINT_NAMES.length) return null
  return {
    stamp: event.stamp,
    sourceAgeS: event.source_age_s,
    receivedAtMs,
    positions: mapped,
  }
}
