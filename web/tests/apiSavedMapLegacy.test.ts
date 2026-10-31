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

function fullCloudResponse() {
  return new Response('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nPOINTS 3\nDATA ascii\n0 1 2\n3 4 5\n6 7 8\n', {
    headers: {
      'X-Lingtu-Map-Id': 'field-map',
      'X-Lingtu-Content-Epoch': '7',
      'X-Lingtu-Frame-Id': 'map',
    },
  })
}

test('saved-map scene uses full PCD geometry instead of sampled JSON, including legacy scene epochs', async () => {
  const requestedUrls: string[] = []
  globalThis.fetch = async (input: string | URL | Request) => {
    requestedUrls.push(String(input))
    if (String(input).endsWith('/pcd')) return fullCloudResponse()
    return new Response(JSON.stringify({
      schema_version: 1,
      count: 1,
      layout: 'xyz_rows',
      frame_id: '/map',
      source: 'saved_map',
      name: 'field-map',
      points: [[0, 1, 2]],
    }), { status: 200 })
  }

  const cloud = await fetchSavedMapPointCloud('field-map')

  assert.deepEqual(cloud.points, [0, 1, 2, 3, 4, 5, 6, 7, 8])
  assert.equal(cloud.frameId, 'map')
  assert.equal(cloud.epoch, null)
  assert.equal(cloud.contentEpoch, 7)
  assert.deepEqual(requestedUrls, ['/api/v1/maps/field-map/points?max_points=1', '/api/v1/maps/field-map/pcd'])
})

test('full scene refuses a PCD from another content version instead of displaying sampled geometry', async () => {
  globalThis.fetch = async (input) => String(input).endsWith('/pcd') ? fullCloudResponse()
    : new Response(JSON.stringify({ name: 'field-map', frame_id: 'map', epoch: 2,
      content_epoch: 8, points: [[0, 1, 2]] }))
  await assert.rejects(fetchSavedMapPointCloud('field-map'), /地图内容已更新/)
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
