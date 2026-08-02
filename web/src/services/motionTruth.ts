export const MotionAction = {
  CANCEL: 'cancel',
  EMERGENCY_STOP: 'emergency_stop',
  RESET_EMERGENCY_STOP: 'reset_emergency_stop',
  RESUME: 'resume',
  START: 'start',
  STOP: 'stop',
} as const

export type MotionAction = (typeof MotionAction)[keyof typeof MotionAction]

export const MotionGateReason = {
  ACTIVE_EMERGENCY_STOP_CONFIRMED: 'active_emergency_stop_confirmed',
  DISCONNECTED: 'disconnected',
  EMERGENCY_STOP_ACTIVE: 'emergency_stop_active',
  EMERGENCY_STOP_NOT_ACTIVE: 'emergency_stop_not_active',
  FRESH_AUTHORITATIVE_TRUTH: 'fresh_authoritative_truth',
  INITIAL_SNAPSHOT_MISSING: 'initial_snapshot_missing',
  INVALID_FRESHNESS_POLICY: 'invalid_freshness_policy',
  INVALID_TIMESTAMP: 'invalid_timestamp',
  REFRESH_ERROR: 'refresh_error',
  RISK_REDUCING_ACTION: 'risk_reducing_action',
  SNAPSHOT_NOT_AUTHORITATIVE: 'snapshot_not_authoritative',
  STALE_TIMESTAMP: 'stale_timestamp',
  UNSUPPORTED_ACTION: 'unsupported_action',
} as const

export type MotionGateReason = (typeof MotionGateReason)[keyof typeof MotionGateReason]

export interface MotionTruthSnapshot {
  readonly authoritative: boolean
  /** Timestamp in the same millisecond clock domain as `MotionTruthFreshness.nowMs`. */
  readonly timestampMs: number
  readonly emergencyStopActive: boolean
}

export interface MotionTruthState {
  readonly connected: boolean
  /** The latest refresh error; clear it only after a successful authoritative refresh. */
  readonly refreshError: string | null
  readonly snapshot: MotionTruthSnapshot | null
}

export interface MotionTruthFreshness {
  readonly nowMs: number
  /** The maximum accepted age, inclusive, in milliseconds. */
  readonly maxAgeMs: number
}

export interface MotionGateDecision {
  readonly allowed: boolean
  readonly reason: MotionGateReason
}

export function evaluateMotionAction(
  action: MotionAction,
  truth: MotionTruthState,
  freshness: MotionTruthFreshness,
): MotionGateDecision {
  if (
    action === MotionAction.STOP
    || action === MotionAction.CANCEL
    || action === MotionAction.EMERGENCY_STOP
  ) {
    return {
      allowed: true,
      reason: MotionGateReason.RISK_REDUCING_ACTION,
    }
  }
  if (
    action !== MotionAction.START
    && action !== MotionAction.RESUME
    && action !== MotionAction.RESET_EMERGENCY_STOP
  ) {
    return {
      allowed: false,
      reason: MotionGateReason.UNSUPPORTED_ACTION,
    }
  }
  if (
    !Number.isFinite(freshness.nowMs)
    || freshness.nowMs < 0
    || !Number.isFinite(freshness.maxAgeMs)
    || freshness.maxAgeMs < 0
  ) {
    return {
      allowed: false,
      reason: MotionGateReason.INVALID_FRESHNESS_POLICY,
    }
  }
  if (!truth.connected) {
    return {
      allowed: false,
      reason: MotionGateReason.DISCONNECTED,
    }
  }
  if (truth.refreshError !== null) {
    return {
      allowed: false,
      reason: MotionGateReason.REFRESH_ERROR,
    }
  }
  if (!truth.snapshot) {
    return {
      allowed: false,
      reason: MotionGateReason.INITIAL_SNAPSHOT_MISSING,
    }
  }
  if (!truth.snapshot.authoritative) {
    return {
      allowed: false,
      reason: MotionGateReason.SNAPSHOT_NOT_AUTHORITATIVE,
    }
  }
  if (
    !Number.isFinite(truth.snapshot.timestampMs)
    || truth.snapshot.timestampMs < 0
    || truth.snapshot.timestampMs > freshness.nowMs
  ) {
    return {
      allowed: false,
      reason: MotionGateReason.INVALID_TIMESTAMP,
    }
  }
  if (freshness.nowMs - truth.snapshot.timestampMs > freshness.maxAgeMs) {
    return {
      allowed: false,
      reason: MotionGateReason.STALE_TIMESTAMP,
    }
  }
  if (action === MotionAction.RESET_EMERGENCY_STOP) {
    if (!truth.snapshot.emergencyStopActive) {
      return {
        allowed: false,
        reason: MotionGateReason.EMERGENCY_STOP_NOT_ACTIVE,
      }
    }
    return {
      allowed: true,
      reason: MotionGateReason.ACTIVE_EMERGENCY_STOP_CONFIRMED,
    }
  }
  if (truth.snapshot.emergencyStopActive) {
    return {
      allowed: false,
      reason: MotionGateReason.EMERGENCY_STOP_ACTIVE,
    }
  }
  return {
    allowed: true,
    reason: MotionGateReason.FRESH_AUTHORITATIVE_TRUTH,
  }
}
