import assert from 'node:assert/strict'
import test from 'node:test'
import { createElement } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import { createServer } from 'vite'
import { fetchRecordingSeries } from '../src/services/api.ts'
import { advancePlayhead, appendLiveTrace, canPlayRecordingWindow, formatSeriesAxis, formatSeriesValue, liveFieldSamples, numericValues, preferredNumericField, sampleAt, seriesSegments, shiftSamples } from '../src/services/recordingSeries.ts'
import type { RecordingLiveTopic, RecordingStreamsResponse } from '../src/types/index.ts'

test('numeric fields preserve nested native paths and array indices, and default to physical data', () => {
  const values = numericValues({ stamp_sec: 123, header: { stamp: { sec: 2 } }, position_rad: [.25, -.5], angular_velocity_rad_s: { x: 1.2 }, label: 'fixture', valid: true, unavailable: null, bad: NaN })
  assert.deepEqual(values, { stamp_sec: 123, 'header.stamp.sec': 2, 'position_rad[0]': .25, 'position_rad[1]': -.5, 'angular_velocity_rad_s.x': 1.2, bad: null })
  assert.equal(preferredNumericField(Object.keys(values)), 'position_rad[0]')
})

test('live curves append distinct actual messages, never cached or stale observations', () => {
  const item: RecordingLiveTopic = { topic: '/imu/raw', type_name: 'Imu', message_count: 10, bytes_received: 100, rate_hz: 20, bytes_per_second: 200, last_sample_age_ms: 10, source_time_ns: 50, preview: { angular_velocity_rad_s: { x: .5 } } }
  const snapshot: RecordingStreamsResponse = { ok: true, available: true, state: 'observing', observed_at_unix_ns: 100_000_000_000, snapshot_age_ms: 0, topics: [item] }
  const first = appendLiveTrace(undefined, item, snapshot)!
  assert.equal(first.samples.length, 1)
  assert.equal(appendLiveTrace(first, item, { ...snapshot, observed_at_unix_ns: 101_000_000_000 }), first)
  assert.equal(appendLiveTrace(first, { ...item, message_count: 11 }, { ...snapshot, state: 'stale' }), first)
  assert.equal(appendLiveTrace(first, { ...item, message_count: 11, last_sample_age_ms: 4000 }, snapshot), first)
  const next = appendLiveTrace(first, { ...item, message_count: 11, preview: { angular_velocity_rad_s: { x: -.5 } } }, { ...snapshot, observed_at_unix_ns: 101_000_000_000 })!
  assert.deepEqual(liveFieldSamples(next, 'angular_velocity_rad_s.x'), [{ t: 0, value: .5, gap_before: undefined }, { t: 1, value: -.5, gap_before: undefined }])
  const resumed = appendLiveTrace(next, { ...item, message_count: 12 }, { ...snapshot, observed_at_unix_ns: 110_000_000_000 })!
  assert.equal(resumed.samples.at(-1)?.gap_before, true)
  const restarted = appendLiveTrace(resumed, { ...item, message_count: 1 }, { ...snapshot, observed_at_unix_ns: 111_000_000_000 })!
  assert.equal(restarted.samples.length, 1)
  assert.equal(restarted.samples[0].t, 0)
})

test('comparison offsets change time only, never values, gaps or source samples', () => {
  const source = [{ t: 0, value: 2 }, { t: 1, value: 3 }, { t: 4, value: 8, gap_before: true }, { t: 5, value: null }]
  const shifted = shiftSamples(source, 2.5)
  assert.deepEqual(shifted, [{ t: 2.5, value: 2 }, { t: 3.5, value: 3 }, { t: 6.5, value: 8, gap_before: true }, { t: 7.5, value: null }])
  assert.equal(source[0].t, 0)
  assert.equal(sampleAt(shifted, 2), undefined)
  assert.equal(sampleAt(shifted, 8), undefined)
  assert.equal(sampleAt(shifted, 5), undefined)
  assert.equal(sampleAt(shifted, 7.5), undefined)
  assert.equal(sampleAt(shifted, 3)?.value, 2)
  assert.equal(sampleAt(shifted, 6.5)?.value, 8)
  assert.deepEqual(seriesSegments(source, 0, 5), [source.slice(0, 2), source.slice(2, 3)])
  assert.deepEqual(seriesSegments(source, 1, 4), [[source[1]], [source[2]]])
})

test('timeline respects playback speed and stops at window end without wrapping', () => {
  assert.equal(advancePlayhead(2, .25, 2, 0, 5), 2.5)
  assert.equal(advancePlayhead(4.9, .25, 4, 0, 5), 5)
  assert.equal(advancePlayhead(5, .5, 1, 0, 5), 5)
  assert.equal(advancePlayhead(-2, 0, 1, 1, 5), 1)
})

test('shared playback remains available in a B-only window but excludes incompatible and missing data', () => {
  const values = [{ t: 11.5, value: -.6 }]
  assert.equal(canPlayRecordingWindow([], values, true), true)
  assert.equal(canPlayRecordingWindow([], values, false), false)
  assert.equal(canPlayRecordingWindow([{ t: 1, value: null }], values, true), true)
  assert.equal(canPlayRecordingWindow([{ t: 1, value: null }], [], true), false)
  assert.equal(canPlayRecordingWindow(values, [], false), true)
  assert.equal(canPlayRecordingWindow([], values, true, 12, 15), false)
  assert.equal(canPlayRecordingWindow([], shiftSamples(values, 3), true, 14, 15), true)
})

test('compact axis labels stay short while legend values preserve precision', () => {
  assert.equal(formatSeriesAxis(-2.1084), '-2.11')
  assert.equal(formatSeriesAxis(8.1342), '8.13')
  assert.equal(formatSeriesAxis(-.12345), '-0.12')
  assert.equal(formatSeriesAxis(12000), '1.2e4')
  assert.equal(formatSeriesAxis(12180), '1.22e4')
  assert.equal(formatSeriesAxis(.00001234), '1.23e-5')
  assert.equal(formatSeriesValue(-2.1084), '-2.1084')
})

test('history field queries are read-only and preserve exact field paths and relative window', async () => {
  const original = globalThis.fetch
  const requests: Array<[string, string]> = []
  let receivedSignal: AbortSignal | null | undefined
  globalThis.fetch = async (url, options) => { requests.push([String(url), options?.method ?? 'GET']); receivedSignal = options?.signal; return Response.json({ ok: true, session_id: 'sample A', series: { samples: [] } }) }
  try {
    const controller = new AbortController()
    await fetchRecordingSeries('sample A', { topic: '/robot/joint_states', field: 'position_rad[0]', start: 1.25, end: 4, max_points: 2000 }, controller.signal)
    assert.equal(requests.length, 1)
    assert.equal(requests[0][1], 'GET')
    const url = new URL(requests[0][0], 'http://localhost')
    assert.equal(url.pathname, '/api/v1/recordings/sample%20A/series')
    assert.equal(url.searchParams.get('field'), 'position_rad[0]')
    assert.equal(url.searchParams.get('start'), '1.25')
    assert.equal(url.searchParams.get('end'), '4')
    assert.equal(receivedSignal, controller.signal)
    controller.abort()
    assert.equal(receivedSignal?.aborted, true)
  } finally { globalThis.fetch = original }
})

test('SVG plot keeps gaps separate, renders A/B values and keyboard-operable historical controls', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingPlot } = await server.ssrLoadModule('/src/components/RecordingPlot.tsx')
    const plot = renderToStaticMarkup(createElement(RecordingPlot, { traces: [{ label: 'A · original', samples: [{ t: 0, value: 1 }, { t: 1, value: 2 }, { t: 2, value: null }, { t: 3, value: 5 }, { t: 4, value: 6 }] }, { label: 'B · candidate', secondary: true, samples: [{ t: 1, value: 7 }, { t: 2, value: 8 }] }], start: 0, end: 4, cursor: 1, locale: 'zh', unit: 'rad' }))
    assert.equal((plot.match(/<polyline/g) ?? []).length, 3)
    assert.match(plot, /A · original/)
    assert.match(plot, /B · candidate/)
    assert.match(plot, /rad/)
    const { RecordingHistory } = await server.ssrLoadModule('/src/components/RecordingHistory.tsx')
    const history = renderToStaticMarkup(createElement(RecordingHistory, { sessionId: 'A', sessions: [{ session_id: 'A', state: 'completed' }, { session_id: 'B', state: 'failed' }], summary: { duration_ns: 5e9, topics: [{ topic: '/imu/raw', message_count: 10 }] }, topics: [], locale: 'zh' }))
    assert.match(history, /历史数值字段/)
    assert.match(history, /对比记录 B/)
    assert.match(history, /历史播放位置秒数/)
    assert.match(history, /录制回看 · 非实时/)
    assert.doesNotMatch(history, /ltplay/)
    assert.ok(history.indexOf('历史播放位置秒数') < history.indexOf('历史数值字段'))
    assert.equal((history.match(/aria-label="历史播放位置秒数"/g) ?? []).length, 1)
  } finally { await server.close() }
})
