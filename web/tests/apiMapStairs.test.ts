import assert from 'node:assert/strict'
import test from 'node:test'

import { fetchMapStairs, saveMapStair } from '../src/services/api.ts'
import type { StairAnnotation } from '../src/types/stairs.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

test('a saved stair round trip writes only editable fields and retains native geometry in the response', async () => {
  const saved: StairAnnotation = {
    id: 'north-stair', name: 'North stairs', frame_id: 'map', content_epoch: 7,
    bounds: { min: { x: 0, y: 1, z: 0 }, max: { x: 4, y: 3, z: 3.1 } },
    lower: { x: 0.2, y: 2, z: 0 }, upper: { x: 3.8, y: 2, z: 3 },
    geometry: { recognized: true, reason: 'recognized', width_m: 1.2,
      mean_rise_m: 0.15, mean_run_m: 0.3,
      surfaces: [{ s_min: 0, s_max: 0.3, lateral_min: -0.6, lateral_max: 0.6, surface_z: 0.15, observed_points: 24 }] },
  }
  const original = structuredClone(saved)
  const requests: { path: string; method: string; body?: unknown }[] = []
  globalThis.fetch = async (input, init) => {
    const method = init?.method ?? 'GET'
    const body = init?.body ? JSON.parse(String(init.body)) : undefined
    requests.push({ path: String(input), method, body })
    if (method === 'PUT') {
      assert.deepEqual(Object.keys(body).sort(), ['bounds', 'content_epoch', 'frame_id', 'lower', 'name', 'upper'])
      assert.deepEqual(body, { name: saved.name, content_epoch: saved.content_epoch,
        frame_id: saved.frame_id, bounds: saved.bounds, lower: saved.lower, upper: saved.upper })
    }
    return Response.json({ success: true, map_id: 'field-map', content_epoch: 7, stairs: [saved] })
  }

  const loaded = await fetchMapStairs('field-map')
  const result = await saveMapStair('field-map', loaded.stairs[0])

  assert.deepEqual(requests.map(({ path, method }) => ({ path, method })), [
    { path: '/api/v1/maps/field-map/stairs', method: 'GET' },
    { path: '/api/v1/maps/field-map/stairs/north-stair', method: 'PUT' },
  ])
  assert.deepEqual(result.stairs[0].geometry, saved.geometry)
  assert.deepEqual(saved, original)
})
