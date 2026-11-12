import assert from 'node:assert/strict'
import test from 'node:test'
import { createElement } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import { createServer } from 'vite'
import { recordingLiveCameraState } from '../src/services/recordingLiveCamera.ts'

const base = { hasFrame: false, lastFrameAt: null, now: 5000, status: null, statusError: false, stalled: false }

test('live video requires actual fresh decoded frames, not an available source or a connected transport', () => {
  assert.equal(recordingLiveCameraState({ ...base, status: { status: 'streaming' } }), 'waiting')
  assert.equal(recordingLiveCameraState({ ...base, hasFrame: true, lastFrameAt: 4900, status: { status: 'streaming' } }), 'live')
  assert.equal(recordingLiveCameraState({ ...base, hasFrame: true, lastFrameAt: 1000, status: { status: 'streaming' } }), 'paused')
  assert.equal(recordingLiveCameraState({ ...base, lastFrameAt: 1000, stalled: true }), 'paused')
})

test('a data gateway without a camera stays explicitly unavailable and status errors are distinguishable', () => {
  assert.equal(recordingLiveCameraState(base), 'loading')
  assert.equal(recordingLiveCameraState({ ...base, status: { status: 'not_loaded' } }), 'not_enabled')
  assert.equal(recordingLiveCameraState({ ...base, statusError: true }), 'error')
  assert.equal(recordingLiveCameraState({ ...base, status: { status: 'error' } }), 'error')
  assert.equal(recordingLiveCameraState({ ...base, status: { status: 'stale' } }), 'paused')
  assert.equal(recordingLiveCameraState({ ...base, status: { status: 'idle' } }), 'waiting')
  assert.equal(recordingLiveCameraState({ ...base, hasFrame: true, lastFrameAt: 4900, statusError: true }), 'live')
})

test('recording starts at capture controls with video selection visible and diagnostics collapsed', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingPanel } = await server.ssrLoadModule('/src/components/RecordingPanel.tsx')
    const panel = renderToStaticMarkup(createElement(RecordingPanel, { view: 'record', onViewChange: () => {}, locale: 'zh', status: null, statusError: null, showToast: () => {}, refreshStatus: async () => {} }))
    assert.match(panel, /data-view="record"/)
    assert.match(panel, /<button[^>]*>回看<\/button>/)
    assert.match(panel, /<label[^>]*><input type="checkbox"[^>]*disabled=""[^>]*>[\s\S]*?包含视频<\/span><\/label><button[^>]*disabled=""[^>]*>[\s\S]*?开始录制<\/button>/)
    assert.ok(panel.indexOf('包含视频') < panel.indexOf('采集配置'))
    assert.match(panel, /<details[^>]*aria-label="数据诊断"[^>]*><summary>/)
    assert.doesNotMatch(panel, /<details[^>]*\sopen(?:=|[\s>])|实时相机视频|运行概览|数据流量分析|role="tablist"/)
    assert.doesNotMatch(panel, /解除急停|Reset E-stop/)
    const sessions = renderToStaticMarkup(createElement(RecordingPanel, { view: 'history', onViewChange: () => {}, locale: 'zh', status: null, statusError: null, showToast: () => {}, refreshStatus: async () => {} }))
    assert.match(sessions, /data-view="history"/)
    assert.match(sessions, /录制文件/)
    assert.match(sessions, /<button[^>]*>录制<\/button>/)
    assert.doesNotMatch(sessions, /包含视频|运行概览/)
  } finally { await server.close() }
})

test('standalone live camera and recording history keep distinct video states', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingLiveCamera } = await server.ssrLoadModule('/src/components/RecordingLiveCamera.tsx')
    const camera = renderToStaticMarkup(createElement(RecordingLiveCamera, { locale: 'zh' }))
    assert.match(camera, /实时相机视频/)
    assert.match(camera, /等待画面/)
    assert.doesNotMatch(camera, /视频说明|此处仅连接/)
    assert.doesNotMatch(camera, /正在接收实时视频|data-live|<video/)
    const { RecordingHistory } = await server.ssrLoadModule('/src/components/RecordingHistory.tsx')
    const history = renderToStaticMarkup(createElement(RecordingHistory, { sessionId: 'saved', sessions: [], topics: [], locale: 'zh' }))
    assert.match(history, /录制回看 · 非实时/)
    assert.doesNotMatch(history, /实时相机视频/)
  } finally { await server.close() }
})
