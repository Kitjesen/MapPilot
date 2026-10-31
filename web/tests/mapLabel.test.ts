import assert from 'node:assert/strict'
import test from 'node:test'
import { mapLocalizationState, shortMapLabel } from '../src/services/mapLabel.ts'
import type { SSEState } from '../src/types/index.ts'

test('map labels preserve complete readable names so layout decides when to ellipsize', () => {
  assert.equal(shortMapLabel('903room_v6_corridor'), '903room · v6 · corridor')
  assert.equal(shortMapLabel('/maps/office_north.pcd'), 'office · north')
  assert.equal(shortMapLabel('D:\\maps\\仓库_二层.PCD'), '仓库 · 二层')
  assert.equal(shortMapLabel(''), null)
  assert.equal(shortMapLabel(null), null)
  assert.equal(shortMapLabel('office_north_corridor_mapping_20261019_180001'),
    'office · north · corridor · mapping · 20261019 · 180001')
})

function stream(localization: Record<string, unknown> | null, connected = true) {
  return {
    connected,
    stateSnapshot: { schema_version: 4, ts: 100, localization },
    stateSnapshotReceivedAt: 1000_000,
  } as Pick<SSEState, 'connected' | 'stateSnapshot' | 'stateSnapshotReceivedAt'>
}

test('localization follows fresh evidence in Gateway time, then ages without another event', () => {
  const state = stream({ state: 'ready', pose_fresh: true })
  assert.equal(mapLocalizationState(state, 1003), 'tracking')
  assert.equal(mapLocalizationState(state, 1008), 'stale')
  assert.equal(mapLocalizationState({ ...state, connected: false }, 1003), 'disconnected')
  assert.equal(mapLocalizationState(stream(null), 1003), 'waiting')
  assert.equal(mapLocalizationState(stream({ state: 'ready' }), 1003), 'unknown')
  assert.equal(mapLocalizationState(stream({ state: 'tracking', pose_fresh: false }), 1003), 'stale')
})

test('localization problems are preserved instead of becoming a normal status', () => {
  for (const state of ['initializing', 'relocalizing', 'degraded', 'lost', 'no_odometry'] as const) {
    assert.equal(mapLocalizationState(stream({ state, pose_fresh: true }), 1003), state)
  }
  assert.equal(mapLocalizationState(stream({ state: 'unexpected' }), 1003), 'unknown')
})
