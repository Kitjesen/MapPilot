import assert from 'node:assert/strict'
import test from 'node:test'

import { fetchNavigationDdsSnapshot } from '../src/services/api.ts'

const originalFetch = globalThis.fetch

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

test('local-planner diagnostics use the existing read-only navigation DDS snapshot', async () => {
  const requestedUrls: string[] = []
  globalThis.fetch = async (input: string | URL | Request) => {
    requestedUrls.push(String(input))
    return new Response(JSON.stringify({
      schema_version: 'lingtu.navigation.dds_snapshot.v1',
      global_path: { schema_version: 1, path: [], robot: null, count: 0, frame_id: 'map', source: 'test' },
      local_path: { schema_version: 1, path: [], robot: null, count: 0, frame_id: 'map', source: 'test' },
      cmd_vel: null,
      nav_endpoint: { local_map: { enabled: false }, local_candidates: { valid: false } },
      traversability_endpoint: null,
      navigation: {},
      ts: 1,
      source: 'test',
    }), { status: 200 })
  }

  const snapshot = await fetchNavigationDdsSnapshot()

  assert.equal(snapshot.schema_version, 'lingtu.navigation.dds_snapshot.v1')
  assert.deepEqual(requestedUrls, ['/api/v1/navigation/dds_snapshot'])
})
