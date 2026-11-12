import assert from 'node:assert/strict'
import test from 'node:test'
import { createRecordingPreviewFetch } from './fixtures/recordingPreview.ts'
import { appendLiveTrace } from '../src/services/recordingSeries.ts'
import { summarizeStreamUsage } from '../src/services/recordingStreams.ts'

test('preview provides fresh observations for five categories and keeps requests local', async () => {
  let timestamp = 1_800_000_000_000
  const fetch = createRecordingPreviewFetch(() => timestamp)
  const catalog = await (await fetch('/api/v1/recordings/catalog')).json()
  const first = await (await fetch('/api/v1/recordings/streams')).json()
  assert.equal(catalog.topics.length, 8)
  const usage = summarizeStreamUsage(catalog.topics, first.topics)
  assert.equal(usage.groups.length, 5)
  assert.equal(usage.observedCount, 8)
  assert.ok(usage.groups[0].bytesPerSecond! > usage.bytesPerSecond! / 2)
  let trace = appendLiveTrace(undefined, first.topics[0], first)
  timestamp += 1000
  const second = await (await fetch('/api/v1/recordings/streams')).json()
  assert.ok(second.topics[0].message_count > first.topics[0].message_count)
  assert.ok(second.topics[0].source_time_ns > first.topics[0].source_time_ns)
  trace = appendLiveTrace(trace, second.topics[0], second)
  assert.equal(trace?.samples.length, 2)
  assert.equal((await (await fetch('/api/v1/camera/status')).json()).status, 'not_loaded')
  assert.equal((await fetch('/api/v1/recordings/start', { method: 'POST' })).status, 405)
  assert.equal((await fetch('http://robot/api/v1/state')).status, 404)
})

test('preview history is explicitly synthetic and never provides downloadable files or robot actions', async () => {
  const fetch = createRecordingPreviewFetch()
  const list = await (await fetch('/api/v1/recordings')).json()
  assert.equal(list.sessions.length, 3)
  for (const session of list.sessions) {
    assert.match(session.session_id, /^demo-.*演示/)
    assert.equal(session.state, '演示记录')
    const path = `/api/v1/recordings/${encodeURIComponent(session.session_id)}`
    const detail = await (await fetch(path)).json()
    assert.equal(detail.session.context.demo, true)
    assert.deepEqual(detail.session.artifacts, [])
    const summary = await (await fetch(`${path}/summary`)).json()
    assert.equal(summary.summary.message_count, 3)
    const series = await (await fetch(`${path}/series?topic=/slam/odometry`)).json()
    assert.equal(series.series.samples.length, 3)
    assert.equal((await fetch(`${path}/archive`)).status, 404)
    assert.equal((await fetch(`${path}/files/demo.mcap`)).status, 404)
    assert.equal((await fetch(path, { method: 'DELETE' })).status, 405)
  }
  assert.equal((await fetch('/api/v1/recordings/unknown/summary')).status, 404)
  assert.equal((await fetch('/api/v1/robot/move', { method: 'POST' })).status, 405)
})
