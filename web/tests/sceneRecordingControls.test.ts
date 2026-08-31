import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import {
  RECORDING_STATUS_POLL_MS,
  recordingArtifactDownloadBlocked,
  recordingNeedsRecovery,
  recordingStatusIsActive,
} from '../src/services/recordingStatus.ts'

const sceneViewSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const appSource = readFileSync(new URL('../src/App.tsx', import.meta.url), 'utf8')
const topbarSource = readFileSync(new URL('../src/components/Topbar.tsx', import.meta.url), 'utf8')
const recordingPanelSource = readFileSync(
  new URL('../src/components/RecordingPanel.tsx', import.meta.url),
  'utf8',
)
const recordingApiSource = readFileSync(
  new URL('../src/services/api.ts', import.meta.url),
  'utf8',
)

test('recording is opened from the Scene workspace, not a primary navigation tab', () => {
  assert.match(sceneViewSource, /<CircleDot size=\{12\} \/>/)
  assert.match(sceneViewSource, /recordingPanelOpen \? '收起录制' : '录制'/)
  assert.match(sceneViewSource, /<FloatingWidget[\s\S]*id="scene-recording"/)
  assert.match(sceneViewSource, /<RecordingPanel[\s\S]*embedded/)
  assert.doesNotMatch(appSource, /RecordingPanel/)
  assert.doesNotMatch(topbarSource, /key: 'recording'/)
})

test('Scene owns one lightweight status poll even while the recording panel is closed', () => {
  const statusFetches = [sceneViewSource, recordingPanelSource]
    .flatMap(source => source.match(/api\.fetchRecordingStatus\(\)/g) ?? [])

  assert.equal(statusFetches.length, 1)
  assert.ok(RECORDING_STATUS_POLL_MS >= 2000)
  assert.match(sceneViewSource, /RECORDING_STATUS_POLL_MS/)
  assert.match(sceneViewSource, /window\.setInterval\([\s\S]*?refreshRecordingStatus\(\)/)
  assert.match(sceneViewSource, /status=\{recordingStatus\}/)
  assert.doesNotMatch(recordingPanelSource, /fetchRecordingStatus/)
})

test('a stale last-good status cannot leave recording start enabled', () => {
  assert.ok(sceneViewSource.includes('const recordingToolbarState = recordingStatusError'))
  assert.ok(sceneViewSource.includes('recordingStatus === null'))
  assert.ok(sceneViewSource.includes("? '状态未知'"))
  assert.ok(recordingPanelSource.includes('const recorderReady = statusError === null'))
  assert.ok(recordingPanelSource.includes('const visibleState = statusError'))
  assert.ok(recordingPanelSource.includes('disabled={busy !== null || !active}'))
})

test('an unhealthy active native state is recovery-required, not idle', () => {
  for (const state of ['preparing', 'recording', 'stopping']) {
    const unhealthyActive = {
      state,
      recording: false,
      healthy: false,
    }
    assert.equal(recordingStatusIsActive(unhealthyActive), true)
    assert.equal(recordingNeedsRecovery(unhealthyActive), true)
  }

  assert.equal(recordingNeedsRecovery({ state: 'idle', recording: false, healthy: false }), false)
  assert.match(sceneViewSource, /recordingNeedsRecovery\(recordingStatus\)/)
  assert.match(sceneViewSource, /\? '需恢复'/)
  assert.match(recordingPanelSource, /const active = recordingStatusIsActive\(status\)/)
  assert.match(recordingPanelSource, /disabled=\{busy !== null \|\| !active\}/)
  assert.match(recordingPanelSource, /Recovery required/)
})

test('artifacts remain non-downloadable until the native session is terminal', () => {
  for (const state of ['preparing', 'recording', 'stopping']) {
    assert.equal(recordingArtifactDownloadBlocked(state), true)
  }
  assert.equal(recordingArtifactDownloadBlocked('completed'), false)
  assert.equal(recordingArtifactDownloadBlocked('failed'), false)
  assert.match(recordingPanelSource, /aria-disabled="true"/)
  assert.match(recordingPanelSource, /Downloads are enabled only after the session stops/)
})

test('session detail races cannot redirect deletion to a different selection', () => {
  assert.match(recordingPanelSource, /const detailRequestIdRef = useRef\(0\)/)
  assert.match(recordingPanelSource, /requestId === detailRequestIdRef\.current/)
  assert.match(recordingPanelSource, /const sessionId = detail\?\.session\.session_id/)
  assert.match(recordingPanelSource, /api\.deleteRecording\(sessionId\)/)
  assert.doesNotMatch(recordingPanelSource, /api\.deleteRecording\(selectedId\)/)
  assert.match(recordingPanelSource, /value === undefined \|\| !Number\.isFinite\(value\)/)
})

test('scene recorder sends only bounded product-level configuration', () => {
  assert.match(recordingPanelSource, /minimumFreeGiB/)
  assert.match(recordingPanelSource, /camera/)
  assert.match(recordingPanelSource, /Native recorder is not ready/)
  assert.doesNotMatch(recordingPanelSource, /dds-topic|dds-domain|camera-color-shm/)
  assert.doesNotMatch(recordingPanelSource, /captureProfile|value="evidence"/)
  assert.match(recordingPanelSource, /capture_profile: 'sensors'/)
  assert.match(recordingApiSource, /capture_profile: config\.capture_profile \?\? 'sensors'/)
  assert.match(recordingApiSource, /task_id: config\.task_id/)
  assert.match(recordingApiSource, /minimum_free_gib: config\.minimum_free_gib \?\? 5/)
})
