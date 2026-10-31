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

test('the real data panel labels its live video and history separately without camera motion controls', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingPanel } = await server.ssrLoadModule('/src/components/RecordingPanel.tsx')
    const panel = renderToStaticMarkup(createElement(RecordingPanel, { locale: 'zh', status: null, statusError: null, showToast: () => {}, refreshStatus: async () => {} }))
    assert.match(panel, /实时相机视频/)
    assert.match(panel, /正在检查当前相机源/)
    assert.match(panel, /历史回看/)
    assert.match(panel, /此处仅连接当前相机视频流/)
    assert.doesNotMatch(panel, /解除急停|Reset E-stop/)
    const { RecordingHistory } = await server.ssrLoadModule('/src/components/RecordingHistory.tsx')
    const history = renderToStaticMarkup(createElement(RecordingHistory, { sessionId: 'saved', sessions: [], topics: [], locale: 'zh' }))
    assert.match(history, /录制回看 · 非实时/)
    assert.doesNotMatch(history, /实时相机视频/)
  } finally { await server.close() }
})
