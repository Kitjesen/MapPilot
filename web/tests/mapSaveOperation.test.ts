import assert from 'node:assert/strict'
import test from 'node:test'

import {
  waitForMapSaveOperation,
  type SaveMapResult,
} from '../src/services/api.ts'

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

function admission(): SaveMapResult {
  return {
    schema_version: 1,
    ok: true,
    success: null,
    accepted: true,
    operation_id: 'save-17',
    name: 'warehouse',
    ts: 1,
  }
}

test('accepted SaveMap polls until the durable operation succeeds', async () => {
  const states = ['RUNNING', 'SUCCEEDED']
  const urls: string[] = []
  globalThis.fetch = async (input: string | URL | Request) => {
    urls.push(String(input))
    const state = states.shift()
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      success: true,
      operation_id: 'save-17',
      operation: { operation_id: 'save-17', state },
      ts: 2,
    }), { status: 200 })
  }

  const result = await waitForMapSaveOperation(admission(), {
    timeoutMs: 1_000,
    pollIntervalMs: 0,
  })

  assert.equal(result.success, true)
  assert.equal(result.operation?.state, 'SUCCEEDED')
  assert.deepEqual(urls, [
    '/api/v1/maps/operations/save-17',
    '/api/v1/maps/operations/save-17',
  ])
})

test('failed SaveMap operation never becomes a saved UI result', async () => {
  globalThis.fetch = async () => new Response(JSON.stringify({
    schema_version: 1,
    ok: true,
    success: true,
    operation_id: 'save-17',
    operation: {
      operation_id: 'save-17',
      state: 'FAILED',
      reason_code: 'artifact_failed',
      message: 'Map artifact generation failed.',
    },
    ts: 2,
  }), { status: 200 })

  await assert.rejects(
    waitForMapSaveOperation(admission(), { timeoutMs: 1_000, pollIntervalMs: 0 }),
    /artifact generation failed/,
  )
})

test('accepted SaveMap without operation identity fails closed', async () => {
  const value = admission()
  delete value.operation_id

  await assert.rejects(
    waitForMapSaveOperation(value),
    /without an operation_id/,
  )
})
