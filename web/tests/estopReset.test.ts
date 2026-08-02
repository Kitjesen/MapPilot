import assert from 'node:assert/strict'
import test from 'node:test'

import * as api from '../src/services/api.ts'

test('estop reset uses the explicit reset control and an idempotent command identity', async (t) => {
  const originalFetch = globalThis.fetch
  let requestUrl = ''
  let requestInit: RequestInit | undefined
  t.after(() => {
    globalThis.fetch = originalFetch
  })
  globalThis.fetch = async (input, init) => {
    requestUrl = String(input)
    requestInit = init
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      status: 'estop_reset_requested',
      command: {
        name: 'estop_reset',
        request_id: 'reset-request',
        client_id: 'web-dashboard',
        accepted: true,
        replay: false,
        ts: 500,
      },
    }), {
      status: 200,
      headers: { 'Content-Type': 'application/json' },
    })
  }

  await api.resetEstop()

  assert.equal(requestUrl, '/api/v1/estop/reset')
  assert.equal(requestInit?.method, 'POST')
  const body = JSON.parse(String(requestInit?.body)) as Record<string, unknown>
  assert.equal(body.client_id, 'web-dashboard')
  assert.match(String(body.request_id), /^estop_reset-/)
})
