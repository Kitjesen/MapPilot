import assert from 'node:assert/strict'
import test from 'node:test'
import { startInspectionTask } from '../src/services/api.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

test('route submission retries preserve the caller request identity', async () => {
  const requests: Array<Record<string, unknown>> = []
  globalThis.fetch = async (_input, options) => {
    requests.push(JSON.parse(String(options?.body)))
    if (requests.length === 1) throw new TypeError('Response lost after submission')
    return Response.json({ accepted: true, task_id: 'native-task', request_id: 'same-start' })
  }
  const options = { map_id: 'hall', revision: 3, request_id: 'same-start' }
  await assert.rejects(startInspectionTask('scene-route', options), /Response lost/)
  assert.equal((await startInspectionTask('scene-route', options)).task_id, 'native-task')
  assert.deepEqual(requests, [
    { route_id: 'scene-route', map_id: 'hall', revision: 3, request_id: 'same-start' },
    { route_id: 'scene-route', map_id: 'hall', revision: 3, request_id: 'same-start' },
  ])
})

test('a new route submission gets a request identity when the caller omits it', async () => {
  const requests: Array<Record<string, unknown>> = []
  globalThis.fetch = async (_input, options) => {
    requests.push(JSON.parse(String(options?.body)))
    return Response.json({ accepted: true, task_id: 'native-task' })
  }
  await startInspectionTask('scene-route')
  await startInspectionTask('scene-route')
  assert.equal(typeof requests[0].request_id, 'string')
  assert.notEqual(requests[0].request_id, requests[1].request_id)
})
