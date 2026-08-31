import assert from 'node:assert/strict'
import test from 'node:test'

import { globalRelocalize, relocalize } from '../src/services/api.ts'

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

  await relocalize('factory-a', 1.5, -2, 0.25)

  assert.equal(request?.url, '/api/v1/localization/relocalizations')
  assert.deepEqual(JSON.parse(String(request?.init?.body)), {
    map_name: 'factory-a',
    mode: 'seeded',
    initial_pose: { x: 1.5, y: -2, yaw: 0.25 },
  })
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
