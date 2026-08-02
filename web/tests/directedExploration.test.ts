import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import {
  fetchExplorationStatus,
  setDirectedExplorationTarget,
} from '../src/services/api.ts'

const sceneSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const originalFetch = globalThis.fetch

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

test('directed exploration checks status and posts only its schema fields', async () => {
  const calls: Array<{ url: string; method: string; body?: Record<string, unknown> }> = []
  globalThis.fetch = async (input: string | URL | Request, init?: RequestInit) => {
    const url = String(input)
    const body = typeof init?.body === 'string'
      ? JSON.parse(init.body) as Record<string, unknown>
      : undefined
    calls.push({ url, method: init?.method ?? 'GET', body })
    if (url === '/api/v1/explore/status') {
      return new Response(JSON.stringify({
        available: true, backend: 'tare', exploring: true, frontier_count: 1,
        can_start: false, blockers: [], advisories: [], navigation: {},
        tare: { runtime: 'native_dds' },
      }), { status: 200 })
    }
    return new Response(JSON.stringify({
      schema_version: 1, ok: true, accepted: true, status: 'accepted',
      intent: { active: true, session_id: 'session-1', frame_id: 'map', reason: 'web' }, native: {},
    }), { status: 200 })
  }

  const status = await fetchExplorationStatus()
  const response = await setDirectedExplorationTarget(1.25, -2.5, {
    ttl_s: 30,
    reason: 'web_scene_selected_point',
    request_id: 'directed-test-1',
  })

  assert.equal(status.tare?.runtime, 'native_dds')
  assert.equal(response.accepted, true)
  assert.deepEqual(calls.map(({ url, method }) => ({ url, method })), [
    { url: '/api/v1/explore/status', method: 'GET' },
    { url: '/api/v1/explore/directed', method: 'POST' },
  ])
  assert.deepEqual(calls[1].body, {
    x: 1.25, y: -2.5, ttl_s: 30,
    reason: 'web_scene_selected_point', request_id: 'directed-test-1',
  })
  assert.equal('client_id' in (calls[1].body ?? {}), false)
})

test('the selected-point action is explicit and never reuses navigation dispatch', () => {
  const handlerStart = sceneSource.indexOf('const handleDirectedExploration = useCallback')
  const handlerEnd = sceneSource.indexOf('const handleSaveCurrentLocation', handlerStart)
  const handlerSource = sceneSource.slice(handlerStart, handlerEnd)

  assert.ok(handlerStart >= 0)
  assert.match(handlerSource, /api\.fetchExplorationStatus\(\)/)
  assert.match(handlerSource, /api\.setDirectedExplorationTarget/)
  assert.doesNotMatch(handlerSource, /api\.sendGoal|api\.constructGoalCandidate/)
  assert.match(sceneSource, /onClick=\{handleDirectedExploration\}/)
  assert.match(sceneSource, /引导探索至此 \(30 秒\)/)
})
