import assert from 'node:assert/strict'
import test from 'node:test'
import { createElement } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import { createServer } from 'vite'
import { fetchRecordingStreams } from '../src/services/api.ts'
import { readRecordingPreferences, writeRecordingPreferences } from '../src/services/recordingPreferences.ts'
import { filterRecordingTopics, formatSampleAge, formatStreamBytes, formatStreamRate, streamFields } from '../src/services/recordingStreams.ts'

test('live inspection uses its read-only endpoint without creating a recording', async () => {
  const original = globalThis.fetch
  const requests: Array<[string, string]> = []
  globalThis.fetch = async (url, options) => {
    requests.push([String(url), options?.method ?? 'GET'])
    return Response.json({ ok: true, available: false, state: 'starting', topics: [] })
  }
  try {
    const result = await fetchRecordingStreams()
    assert.equal(result.state, 'starting')
    assert.deepEqual(requests, [['/api/v1/recordings/streams', 'GET']])
  } finally { globalThis.fetch = original }
})

test('recording choices survive reopening, including an intentional camera-only selection', () => {
  let retained: string | null = null
  const storage = { getItem: () => retained, setItem: (_key: string, value: string) => { retained = value } }
  assert.equal(readRecordingPreferences(storage), null)
  writeRecordingPreferences({ topics: ['/robot/joint_states', '/imu/raw'], camera: false }, storage)
  assert.deepEqual(readRecordingPreferences(storage), { topics: ['/robot/joint_states', '/imu/raw'], camera: false })
  writeRecordingPreferences({ topics: [], camera: true }, storage)
  assert.deepEqual(readRecordingPreferences(storage), { topics: [], camera: true })
  retained = '{unfinished'
  assert.equal(readRecordingPreferences(storage), null)
})

test('live inspector distinguishes absent observations from zero rates and renders actual fields', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingStreamInspector } = await server.ssrLoadModule('/src/components/RecordingStreams.tsx')
    assert.equal(formatStreamRate(undefined), '—')
    assert.equal(formatStreamRate(0), '0.0 Hz')
    assert.equal(formatStreamBytes(2048), '2.0 KiB/s')
    assert.equal(formatSampleAge(null, 'zh'), '未收到')
    const topics = [
      { topic: '/robot/joint_states', label: '电机关节', type_name: 'JointState', group: 'robot' },
      { topic: '/slam/odometry', label: '定位', type_name: 'Odometry', group: 'localization' },
    ]
    assert.deepEqual(filterRecordingTopics(topics, 'joint', 'all'), [topics[0]])
    assert.deepEqual(filterRecordingTopics(topics, '', 'localization'), [topics[1]])
    const observation = {
      topic: '/robot/joint_states', type_name: 'JointState', message_count: 123,
      bytes_received: 10000, rate_hz: 30, bytes_per_second: 2048,
      last_sample_age_ms: 12, source_time_ns: 1000,
      preview: { position: [0.25, -0.5], velocity: [1, 2], robot_model: 'fixture' },
    }
    const live = renderToStaticMarkup(createElement(RecordingStreamInspector, {
      topic: topics[0], observation, rates: [29, 30, 30], stale: false, locale: 'zh',
    }))
    assert.match(live, /30 Hz/)
    assert.match(live, /position/)
    assert.match(live, /0.25, -0.5/)
    assert.match(live, /123/)
    assert.match(live, /录制保留完整消息/)
    const waiting = renderToStaticMarkup(createElement(RecordingStreamInspector, {
      topic: topics[0], observation: undefined, rates: [], stale: false, locale: 'zh',
    }))
    assert.match(waiting, /等待这条数据流/)
    assert.doesNotMatch(waiting, /0.0 Hz/)
    const stale = renderToStaticMarkup(createElement(RecordingStreamInspector, {
      topic: topics[0], observation, rates: [], stale: true, locale: 'zh',
    }))
    assert.match(stale, /未更新/)
    assert.doesNotMatch(stale, /30 Hz/)
    assert.doesNotMatch(stale, /12 ms/)
    assert.deepEqual(streamFields({ pose: { position: { x: 1, y: 2 } } }), [['pose.position.x', '1'], ['pose.position.y', '2']])
    const { RecordingPanel } = await server.ssrLoadModule('/src/components/RecordingPanel.tsx')
    const workbench = renderToStaticMarkup(createElement(RecordingPanel, {
      locale: 'zh', status: { state: 'failed', recording: false, available: true, healthy: false },
      statusError: null, showToast: () => {}, refreshStatus: async () => {},
    }))
    assert.match(workbench, /数据工作区/)
    assert.doesNotMatch(workbench, />失败</)
  } finally { await server.close() }
})
