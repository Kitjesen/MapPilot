import assert from 'node:assert/strict'
import test from 'node:test'

import {
  observeAuthoritativeTruth,
  type AuthoritativeTruthObservation,
} from '../src/services/authoritativeTruth.ts'

const previous: AuthoritativeTruthObservation = {
  authoritativeStateSeen: true,
  lastTruthAt: 1_000,
  truthError: null,
}

test('successful authoritative state refresh advances the truth timestamp', () => {
  assert.deepEqual(observeAuthoritativeTruth(previous, { ok: true }, 2_000), {
    authoritativeStateSeen: true,
    lastTruthAt: 2_000,
    truthError: null,
  })
})

test('failed authoritative state refresh preserves the last known timestamp', () => {
  assert.deepEqual(
    observeAuthoritativeTruth(previous, { ok: false, error: new Error('state offline') }, 2_000),
    {
      authoritativeStateSeen: true,
      lastTruthAt: 1_000,
      truthError: 'state offline',
    },
  )
})
