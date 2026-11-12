import assert from 'node:assert/strict'
import test from 'node:test'
import { initialMapEntry } from '../src/services/mapEntry.ts'

test('saved-map links select the product page without activating a map', () => {
  assert.deepEqual(initialMapEntry('?tab=map&map=903room_v6_corridor'), {
    tab: 'map', map: '903room_v6_corridor',
  })
  assert.deepEqual(initialMapEntry('?map=warehouse.v2'), { tab: 'map', map: 'warehouse.v2' })
  assert.deepEqual(initialMapEntry('?tab=map'), { tab: 'map', map: null })
  assert.deepEqual(initialMapEntry('?observe=1'), { tab: 'scene', map: null })
})

test('business deep links select pages without requesting a Product switch', () => {
  for (const tab of ['mapping', 'scene']) {
    assert.deepEqual(initialMapEntry(`?tab=${tab}`), { tab, map: null })
  }
  for (const removed of ['console', 'inspection', 'planner']) {
    assert.deepEqual(initialMapEntry(`?tab=${removed}`), { tab: 'scene', map: null })
  }
})
