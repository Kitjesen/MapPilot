import assert from 'node:assert/strict'
import test from 'node:test'
import { reviewInspectionEvidence, saveInspectionRoute } from '../src/services/api.ts'

const originalFetch = globalThis.fetch
test.afterEach(() => { globalThis.fetch = originalFetch })

test('manual evidence review saves the chosen result without a motion request', async () => {
  const requests: Array<{ url: string; method?: string; body: unknown }> = []
  globalThis.fetch = async (input, options) => {
    const body = JSON.parse(String(options?.body))
    requests.push({ url: String(input), method: options?.method, body })
    return Response.json({
      ok: true,
      evidence: { evidence_id: 'capture-1', review: { ...body, reviewed_at: '2026-10-12T09:00:00Z' } },
    })
  }

  for (const verdict of ['normal', 'anomaly', 'inconclusive'] as const) {
    const result = await reviewInspectionEvidence('capture-1', { verdict, note: '现场复核' })
    assert.equal(result.evidence.review?.verdict, verdict)
  }
  assert.deepEqual(requests, ['normal', 'anomaly', 'inconclusive'].map(verdict => ({
    url: '/api/v1/inspection/evidence/capture-1/review',
    method: 'POST', body: { verdict, note: '现场复核' },
  })))
})

test('review submission failure is returned to the operator rather than presented as saved', async () => {
  globalThis.fetch = async () => Response.json({
    error: 'inspection_evidence_unavailable', message: 'Evidence storage is unavailable',
  }, { status: 503 })
  await assert.rejects(
    reviewInspectionEvidence('capture-1', { verdict: 'normal' }),
    /Evidence storage is unavailable/,
  )
})

test('loop interval is submitted as seconds and survives route reload', async () => {
  let submitted: Record<string, unknown> = {}
  globalThis.fetch = async (_input, options) => {
    submitted = JSON.parse(String(options?.body))
    return Response.json({ ok: true, route: submitted })
  }
  const saved = await saveInspectionRoute({
    id: 'route-a', map_id: 'map-a', map_content_epoch: 2, revision: 3,
    points: [], loop_count: 4, loop_interval_s: 600,
    failure_policy: 'stop', max_retries: 0,
  })
  assert.equal(submitted.loop_interval_s, 600)
  assert.equal(saved.route.loop_interval_s, 600)
})
