import assert from 'node:assert/strict'
import test from 'node:test'

import { fetchSavedMapPointCloud } from '../src/services/api.ts'

const originalFetch = globalThis.fetch
const testGlobals = globalThis as typeof globalThis & {
  window?: { location: { origin: string } }
}
const originalWindow = testGlobals.window

testGlobals.window = { location: { origin: 'http://robot.test' } }

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

test.after(() => {
  if (originalWindow === undefined) delete testGlobals.window
  else testGlobals.window = originalWindow
})

test('legacy saved-map response without epoch remains displayable at full viewer density', async () => {
  const requestedUrls: string[] = []
  globalThis.fetch = async (input: string | URL | Request) => {
    requestedUrls.push(String(input))
    return new Response(JSON.stringify({
      schema_version: 1,
      count: 2,
      layout: 'xyz_rows',
      frame_id: '/map',
      source: 'saved_map',
      name: 'field-map',
      points: [[0, 1, 2], [3, 4, 5]],
    }), { status: 200 })
  }

  const cloud = await fetchSavedMapPointCloud('field-map')

  assert.deepEqual(cloud.points, [0, 1, 2, 3, 4, 5])
  assert.equal(cloud.frameId, 'map')
  assert.equal(cloud.epoch, null)
  assert.deepEqual(requestedUrls, ['/api/v1/maps/field-map/points?max_points=80000'])
})

test('present but invalid saved-map epoch is still rejected', async () => {
  globalThis.fetch = async () => new Response(JSON.stringify({
    schema_version: 1,
    count: 1,
    layout: 'flat_xyz',
    frame_id: 'map',
    epoch: 0,
    source: 'saved_map',
    name: 'field-map',
    points: [0, 1, 2],
  }), { status: 200 })

  await assert.rejects(
    fetchSavedMapPointCloud('field-map'),
    /missing a valid scene epoch/,
  )
})
