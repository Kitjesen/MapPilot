import assert from 'node:assert/strict'
import test from 'node:test'

import {
  cancelFollowTask,
  getFollowTask,
  sendVisualServo,
} from '../src/services/api.ts'

const originalFetch = globalThis.fetch

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

test('follow cancellation sends the task reference captured by the caller', async () => {
  const requests: Array<{ url: string; body: Record<string, unknown> }> = []
  globalThis.fetch = async (input, init) => {
    const body = JSON.parse(String(init?.body)) as Record<string, unknown>
    requests.push({ url: String(input), body })
    return Response.json({
      ok: true,
      status: 'ok',
      command: {
        name: 'follow_cancel', client_id: 'web-dashboard', accepted: true, replay: false, ts: 1,
      },
      follow_task_id: 'follow-old',
      product_session_id: 'product-session-a',
      state: 'cancel_requested',
      terminal: false,
    })
  }

  const displayedTask = {
    follow_task_id: 'follow-old',
    product_session_id: 'product-session-a',
  }
  await cancelFollowTask(displayedTask)

  assert.equal(requests[0]?.url, '/api/v1/follow/follow-old/cancel')
  assert.equal(requests[0]?.body.product_session_id, 'product-session-a')
  assert.equal(requests[0]?.body.reason, 'web_operator_cancel')
  assert.equal(requests[0]?.body.client_id, 'web-dashboard')
})

test('follow status lookup is bound to both task and Product session', async () => {
  let received = ''
  globalThis.fetch = async input => {
    received = String(input)
    return Response.json({
      found: true,
      follow_task_id: 'follow/a',
      product_session_id: 'session b',
      state: 'following',
      terminal: false,
    })
  }

  await getFollowTask({ follow_task_id: 'follow/a', product_session_id: 'session b' })

  assert.equal(received, '/api/v1/follow/follow%2Fa?product_session_id=session+b')
})

test('legacy visual stop carries a Follow ref only when one is supplied', async () => {
  const bodies: Array<Record<string, unknown>> = []
  globalThis.fetch = async (_input, init) => {
    bodies.push(JSON.parse(String(init?.body)) as Record<string, unknown>)
    return Response.json({
      ok: true,
      status: 'ok',
      command: {
        name: 'visual_servo', client_id: 'web-dashboard', accepted: true, replay: false, ts: 1,
      },
    })
  }

  await sendVisualServo('stop')
  await sendVisualServo('stop', null, null, {
    follow_task_id: 'follow-old',
    product_session_id: 'product-session-a',
  })

  assert.equal('follow_task_id' in bodies[0]!, false)
  assert.equal('product_session_id' in bodies[0]!, false)
  assert.equal(bodies[1]?.follow_task_id, 'follow-old')
  assert.equal(bodies[1]?.product_session_id, 'product-session-a')
})
