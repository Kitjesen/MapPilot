import assert from 'node:assert/strict'
import test from 'node:test'

import {
  MotionAction,
  MotionGateReason,
  evaluateMotionAction,
  type MotionTruthState,
} from '../src/services/motionTruth.ts'

const NOW_MS = 10_000
const MAX_AGE_MS = 1_000
const MOTION_INCREASING_ACTIONS = [MotionAction.START, MotionAction.RESUME] as const

function freshTruth(): MotionTruthState {
  return {
    connected: true,
    refreshError: null,
    snapshot: {
      authoritative: true,
      timestampMs: NOW_MS - 100,
      emergencyStopActive: false,
    },
  }
}

test('fresh connected authoritative truth allows motion to start or resume', () => {
  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(action, freshTruth(), {
        nowMs: NOW_MS,
        maxAgeMs: MAX_AGE_MS,
      }),
      {
        allowed: true,
        reason: MotionGateReason.FRESH_AUTHORITATIVE_TRUTH,
      },
    )
  }
})

test('disconnected truth blocks motion start and resume', () => {
  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(
        action,
        { ...freshTruth(), connected: false },
        { nowMs: NOW_MS, maxAgeMs: MAX_AGE_MS },
      ),
      {
        allowed: false,
        reason: MotionGateReason.DISCONNECTED,
      },
    )
  }
})

test('connected channel without its initial snapshot blocks motion start and resume', () => {
  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(
        action,
        { ...freshTruth(), snapshot: null },
        { nowMs: NOW_MS, maxAgeMs: MAX_AGE_MS },
      ),
      {
        allowed: false,
        reason: MotionGateReason.INITIAL_SNAPSHOT_MISSING,
      },
    )
  }
})

test('latest truth refresh error blocks motion start and resume despite a prior snapshot', () => {
  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(
        action,
        { ...freshTruth(), refreshError: 'snapshot refresh failed' },
        { nowMs: NOW_MS, maxAgeMs: MAX_AGE_MS },
      ),
      {
        allowed: false,
        reason: MotionGateReason.REFRESH_ERROR,
      },
    )
  }
})

test('non-authoritative snapshot blocks motion start and resume', () => {
  const fresh = freshTruth()
  const truth: MotionTruthState = {
    ...fresh,
    snapshot: { ...fresh.snapshot!, authoritative: false },
  }

  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(action, truth, {
        nowMs: NOW_MS,
        maxAgeMs: MAX_AGE_MS,
      }),
      {
        allowed: false,
        reason: MotionGateReason.SNAPSHOT_NOT_AUTHORITATIVE,
      },
    )
  }
})

test('snapshot older than the freshness limit blocks motion start and resume', () => {
  const fresh = freshTruth()
  const truth: MotionTruthState = {
    ...fresh,
    snapshot: {
      ...fresh.snapshot!,
      timestampMs: NOW_MS - MAX_AGE_MS - 1,
    },
  }

  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(action, truth, {
        nowMs: NOW_MS,
        maxAgeMs: MAX_AGE_MS,
      }),
      {
        allowed: false,
        reason: MotionGateReason.STALE_TIMESTAMP,
      },
    )
  }
})

test('stop cancel and emergency-stop are always allowed as risk-reducing actions', () => {
  const unavailableTruth: MotionTruthState = {
    connected: false,
    refreshError: 'truth channel unavailable',
    snapshot: null,
  }

  for (const action of [
    MotionAction.STOP,
    MotionAction.CANCEL,
    MotionAction.EMERGENCY_STOP,
  ]) {
    assert.deepEqual(
      evaluateMotionAction(action, unavailableTruth, {
        nowMs: NOW_MS,
        maxAgeMs: MAX_AGE_MS,
      }),
      {
        allowed: true,
        reason: MotionGateReason.RISK_REDUCING_ACTION,
      },
    )
  }
})

test('emergency-stop reset is blocked when fresh truth says no estop is active', () => {
  assert.equal(MotionAction.RESET_EMERGENCY_STOP, 'reset_emergency_stop')
  assert.deepEqual(
    evaluateMotionAction(MotionAction.RESET_EMERGENCY_STOP, freshTruth(), {
      nowMs: NOW_MS,
      maxAgeMs: MAX_AGE_MS,
    }),
    {
      allowed: false,
      reason: MotionGateReason.EMERGENCY_STOP_NOT_ACTIVE,
    },
  )
})

test('emergency-stop reset is allowed after fresh truth confirms an active estop', () => {
  const fresh = freshTruth()
  const truth: MotionTruthState = {
    ...fresh,
    snapshot: { ...fresh.snapshot!, emergencyStopActive: true },
  }

  assert.deepEqual(
    evaluateMotionAction(MotionAction.RESET_EMERGENCY_STOP, truth, {
      nowMs: NOW_MS,
      maxAgeMs: MAX_AGE_MS,
    }),
    {
      allowed: true,
      reason: MotionGateReason.ACTIVE_EMERGENCY_STOP_CONFIRMED,
    },
  )
})

test('active emergency stop blocks motion start and resume', () => {
  const fresh = freshTruth()
  const truth: MotionTruthState = {
    ...fresh,
    snapshot: { ...fresh.snapshot!, emergencyStopActive: true },
  }

  for (const action of MOTION_INCREASING_ACTIONS) {
    assert.deepEqual(
      evaluateMotionAction(action, truth, { nowMs: NOW_MS, maxAgeMs: MAX_AGE_MS }),
      {
        allowed: false,
        reason: MotionGateReason.EMERGENCY_STOP_ACTIVE,
      },
    )
  }
})

test('emergency-stop reset remains blocked when the active-estop snapshot is stale', () => {
  const fresh = freshTruth()
  const truth: MotionTruthState = {
    ...fresh,
    snapshot: {
      ...fresh.snapshot!,
      timestampMs: NOW_MS - MAX_AGE_MS - 1,
      emergencyStopActive: true,
    },
  }

  assert.deepEqual(
    evaluateMotionAction(MotionAction.RESET_EMERGENCY_STOP, truth, {
      nowMs: NOW_MS,
      maxAgeMs: MAX_AGE_MS,
    }),
    {
      allowed: false,
      reason: MotionGateReason.STALE_TIMESTAMP,
    },
  )
})

test('invalid or future snapshot timestamps never count as fresh truth', () => {
  for (const timestampMs of [Number.NaN, Number.POSITIVE_INFINITY, -1, NOW_MS + 1]) {
    const fresh = freshTruth()
    const truth: MotionTruthState = {
      ...fresh,
      snapshot: { ...fresh.snapshot!, timestampMs },
    }

    assert.deepEqual(
      evaluateMotionAction(MotionAction.START, truth, {
        nowMs: NOW_MS,
        maxAgeMs: MAX_AGE_MS,
      }),
      {
        allowed: false,
        reason: MotionGateReason.INVALID_TIMESTAMP,
      },
    )
  }
})

test('unknown runtime actions fail closed', () => {
  assert.deepEqual(
    evaluateMotionAction('teleport' as MotionAction, freshTruth(), {
      nowMs: NOW_MS,
      maxAgeMs: MAX_AGE_MS,
    }),
    {
      allowed: false,
      reason: MotionGateReason.UNSUPPORTED_ACTION,
    },
  )
})

test('invalid freshness policy fails closed for motion-increasing actions', () => {
  for (const freshness of [
    { nowMs: Number.NaN, maxAgeMs: MAX_AGE_MS },
    { nowMs: -1, maxAgeMs: MAX_AGE_MS },
    { nowMs: NOW_MS, maxAgeMs: Number.POSITIVE_INFINITY },
    { nowMs: NOW_MS, maxAgeMs: -1 },
  ]) {
    assert.deepEqual(
      evaluateMotionAction(MotionAction.START, freshTruth(), freshness),
      {
        allowed: false,
        reason: MotionGateReason.INVALID_FRESHNESS_POLICY,
      },
    )
  }
})
