import assert from 'node:assert/strict'
import test from 'node:test'

import { deleteMap, fetchAppCapabilities, fetchMapList } from '../src/services/api.ts'

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

test('map listing defaults to the canonical maps collection', async () => {
  let requestedUrl = ''
  let requestedInit: RequestInit | undefined
  globalThis.fetch = async (input: string | URL | Request, init?: RequestInit) => {
    requestedUrl = String(input)
    requestedInit = init
    return new Response(JSON.stringify({ schema_version: 1, maps: [] }), { status: 200 })
  }

  await fetchMapList()

  assert.equal(requestedUrl, '/api/v1/slam/maps')
  assert.equal(requestedInit?.method, undefined)
})

test('map deletion follows the discovered named-map DELETE link', async () => {
  const calls: Array<{ url: string; init?: RequestInit }> = []
  globalThis.fetch = async (input: string | URL | Request, init?: RequestInit) => {
    const url = String(input)
    calls.push({ url, init })
    if (url === '/api/v1/app/capabilities') {
      return new Response(JSON.stringify({
        links: {
          map_delete: '/control/maps/{name}',
          map_build_occupancy: '/control/maps/{name}/build_occupancy',
        },
      }), { status: 200 })
    }
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      success: true,
      action: 'delete',
      name: 'field/map',
    }), { status: 200 })
  }

  await fetchAppCapabilities()
  await deleteMap('field/map')

  assert.deepEqual(calls.map(({ url }) => url), [
    '/api/v1/app/capabilities',
    '/control/maps/field%2Fmap',
  ])
  assert.equal(calls[1].init?.method, 'DELETE')
  assert.equal(calls[1].init?.body, undefined)
})
