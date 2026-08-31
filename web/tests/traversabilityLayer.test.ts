import assert from 'node:assert/strict'
import test from 'node:test'

import { resolveNativeTraversabilityLayer } from '../src/components/scene3d/layers/traversabilityLayer.ts'
import type { NativeTraversabilityEvent } from '../src/types/index.ts'

const event: NativeTraversabilityEvent = {
  type: 'native_traversability',
  grid_b64: Buffer.from([0, 25, 75, 100]).toString('base64'),
  rows: 2,
  cols: 2,
  resolution: 0.2,
  origin: [1, -2, 0.4],
  yaw: 0,
  frame_id: 'map',
  stamp_s: 100,
  reset_epoch: 2,
  sequence: 8,
  source: 'native_nav_client',
  control_authority: true,
  value_semantics: 'control_risk_0_100',
  identity_verified: true,
}

test('native traversability accepts only fresh map-frame control risk', () => {
  const state = resolveNativeTraversabilityLayer(event, {
    nowS: 101,
    allowedFrameIds: ['map'],
  })
  assert.equal(state.status, 'ready')
  if (state.status === 'ready') assert.deepEqual(Array.from(state.values), [0, 25, 75, 100])
})

test('native traversability fails closed for rotated grids until the nav contract supports them', () => {
  const state = resolveNativeTraversabilityLayer(
    { ...event, yaw: 0.1 },
    { nowS: 101, allowedFrameIds: ['map'] },
  )
  assert.equal(state.status, 'error')
})

test('native traversability hides stale data instead of leaving an old risk mesh', () => {
  const state = resolveNativeTraversabilityLayer(event, {
    nowS: 104,
    allowedFrameIds: ['map'],
  })
  assert.equal(state.status, 'stale')
})
