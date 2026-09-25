import assert from 'node:assert/strict'
import test from 'node:test'

import { globalRelocalize, relocalize } from '../src/services/api.ts'
import { parseInitialPose } from '../src/services/localizationInitialPose.ts'

const originalFetch = globalThis.fetch

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

function successfulResponse(mode: 'seeded' | 'global'): Response {
  return new Response(JSON.stringify({
    schema_version: 1,
    ok: true,
    success: true,
    map_name: 'factory-a',
    mode,
    ts: 1,
  }), { status: 200 })
}

test('seeded relocalization uses the localization operation contract', async () => {
  let request: { url: string; init?: RequestInit } | undefined
  globalThis.fetch = async (input, init) => {
    request = { url: String(input), init }
    return successfulResponse('seeded')
  }

  const pose = parseInitialPose({ x: '1.5', y: '-2', z: '0.3', yaw: '0.25' })
  await relocalize('factory-a', pose.x, pose.y, pose.yaw, pose.z)

  assert.equal(request?.url, '/api/v1/localization/relocalizations')
  assert.deepEqual(JSON.parse(String(request?.init?.body)), {
    map_name: 'factory-a',
    mode: 'seeded',
    initial_pose: { x: 1.5, y: -2, z: 0.3, yaw: 0.25 },
  })
})

test('missing or invalid initial pose fields are not silently sent as zero', () => {
  for (const z of ['', ' ', 'Infinity', 'NaN', '1abc']) {
    assert.throws(() => parseInitialPose({ x: '1', y: '2', z, yaw: '0' }), /有效/)
  }
  assert.equal(parseInitialPose({ x: '0', y: '0', z: '0', yaw: '0' }).z, 0)
})

test('global relocalization sends the selected Product map explicitly', async () => {
  let request: { url: string; init?: RequestInit } | undefined
  globalThis.fetch = async (input, init) => {
    request = { url: String(input), init }
    return successfulResponse('global')
  }

  await globalRelocalize('factory-a')

  assert.equal(request?.url, '/api/v1/localization/relocalizations')
  assert.deepEqual(JSON.parse(String(request?.init?.body)), {
    map_name: 'factory-a',
    mode: 'global',
  })
})
