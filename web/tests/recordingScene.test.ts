import assert from 'node:assert/strict'
import test from 'node:test'
import { createElement } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import { createServer } from 'vite'
import { fetchRecordingCameraFrame, fetchRecordingScene, fetchRecordingSceneInfo } from '../src/services/api.ts'
import { cameraCursorTime, cameraSegmentAt, pastPoseSegments, recordingCloudView, sceneFrameAt, sceneFrames, sceneWindowIndex, sceneWindowRange } from '../src/services/recordingScene.ts'
import type { RecordingCameraSegment, RecordingPoseSample, RecordingScene } from '../src/types/recordingScene.ts'

const pose = (t: number, x: number, frame_id = 'map', gap_before = false): RecordingPoseSample => ({ t, source_time_ns: String(1000000000 + t * 1e9), frame_id, position: [x, 0, .5], orientation: [0, 0, 0, 1], gap_before })
const scene: RecordingScene = {
  version: 1, ok: true, mode: 'scene', start_time_ns: '1000000000', duration_s: 14,
  window: { start_s: 0, end_s: 5 }, topics: [],
  cloud: { topic: '/lidar/raw_frame', original_count: 100, read_count: 32, returned_count: 2, frames: [{ t: 0, source_time_ns: '1000000000', frame_id: 'lidar', original_points: 10, points: [1, 2, 3] }, { t: 2, source_time_ns: '3000000000', frame_id: 'lidar', original_points: 10, points: [4, 5, 6], gap_before: true }] },
  trajectory: { topic: '/slam/odometry', original_count: 100, read_count: 4, samples: [pose(0, 0), pose(1, 1), pose(2, 2, 'map', true), pose(3, 3)] },
  paths: [{ topic: '/nav/global_path', read_count: 2, frames: [{ t: 0, source_time_ns: '1000000000', frame_id: 'map', original_poses: 2, points: [0, 0, .5, 4, 0, .5] }, { t: 3, source_time_ns: '4000000000', frame_id: 'map', original_poses: 0, points: [] }] }],
}

test('camera uses one shared sampled instant for A/B while paused seeks preserve exact cursor', () => {
  assert.equal(cameraCursorTime(1.29, 0, true), 1.2)
  assert.ok(Math.abs(cameraCursorTime(1.29, .35, true) + .35 - cameraCursorTime(1.29, 0, true)) < 1e-12)
  assert.equal(cameraCursorTime(1.29, 0, false), 1.29)
  assert.ok(Math.abs(cameraCursorTime(1.29, .35, false) - .94) < 1e-12)
})

test('scene requests are bounded five-second windows with lookbehind and file-origin timestamps', () => {
  assert.equal(sceneWindowIndex(9.9), 1)
  assert.deepEqual(sceneWindowRange(0, 14), [0, 5])
  assert.deepEqual(sceneWindowRange(1, 14), [4, 10])
  assert.deepEqual(sceneWindowRange(2, 14), [9, 14])
  assert.equal(sceneFrameAt(scene.cloud!.frames, -.1), undefined)
  assert.equal(sceneFrameAt(scene.cloud!.frames, 1.9)?.t, 0)
  assert.equal(sceneFrameAt(scene.cloud!.frames, 2)?.t, 2)
})

test('trajectory excludes future samples and never bridges gaps or different coordinate frames', () => {
  assert.deepEqual([...pastPoseSegments(scene.trajectory!.samples, 1.5, 'map')], [0, 0, .5, 1, 0, .5])
  assert.deepEqual([...pastPoseSegments(scene.trajectory!.samples, 2.5, 'map')], [0, 0, .5, 1, 0, .5])
  const mixed = [pose(0, 0), pose(1, 1, 'odom'), pose(2, 2), pose(3, 3)]
  assert.deepEqual([...pastPoseSegments(mixed, 3, 'map')], [2, 0, .5, 3, 0, .5])
})

test('cloud and path stay in their recorded frames, and an empty recorded path clears the prior plan', () => {
  assert.deepEqual(sceneFrames(scene, 1.5), ['lidar', 'map'])
  const lidar = recordingCloudView(scene, 1.5, 'lidar')
  assert.deepEqual([...lidar.points], [1, 2, 3])
  assert.equal(lidar.lines.length, 0)
  const map = recordingCloudView(scene, 1.5, 'map')
  assert.equal(map.points.length, 0)
  assert.deepEqual(map.lines.map(line => line.kind), ['trajectory', 'path'])
  const afterClear = recordingCloudView(scene, 3, 'map')
  assert.deepEqual(afterClear.lines.map(line => line.kind), ['trajectory'])
})

test('camera segment coverage does not fill missing time or choose another camera', () => {
  const segments = [{ index: 0, camera_id: 'color', start_s: 1, end_s: 2 }, { index: 0, camera_id: 'rear', start_s: 2, end_s: 3 }] as RecordingCameraSegment[]
  assert.equal(cameraSegmentAt(segments, .5, 'color'), undefined)
  assert.equal(cameraSegmentAt(segments, 2.5, 'color'), undefined)
  assert.equal(cameraSegmentAt(segments, 2.5, 'rear')?.camera_id, 'rear')
})

test('camera holds the last actual frame through nominal coverage, selects the next segment at its first frame, and preserves real gaps', () => {
  const segments = [
    { index: 0, camera_id: 'color', start_s: 0, end_s: .9, coverage_end_s: 1 },
    { index: 1, camera_id: 'color', start_s: 1, end_s: 1.9, coverage_end_s: 2 },
    { index: 2, camera_id: 'color', start_s: 3, end_s: 3.9, coverage_end_s: 4 },
  ] as RecordingCameraSegment[]
  assert.equal(cameraSegmentAt(segments, .902, 'color')?.index, 0)
  assert.equal(cameraSegmentAt(segments, .999, 'color')?.index, 0)
  assert.equal(cameraSegmentAt(segments, 1, 'color')?.index, 1)
  assert.equal(cameraSegmentAt(segments, 2.5, 'color'), undefined)
  assert.equal(segments[0].end_s, .9)
})

test('scene and indexed camera reads use only GET and preserve actual returned frame timing', async () => {
  const original = globalThis.fetch
  const requests: Array<[string, string]> = []
  const controller = new AbortController()
  globalThis.fetch = async (url, options) => {
    requests.push([String(url), options?.method ?? 'GET'])
    assert.equal(options?.signal, controller.signal)
    if (String(url).includes('camera-frame')) return new Response(new Blob(['frame'], { type: 'image/jpeg' }), { headers: { 'X-Recording-Time-S': '1.23', 'X-Recording-Media-Time-S': '.37', 'X-Recording-Timing': 'frame_timestamp_index', 'X-Recording-Frame-Index': '37', 'X-Recording-Source-Time-Ns': '1792891234567891234', 'X-Recording-Frame-Id': 'left_camera' } })
    return Response.json({ ok: true, session_id: 'sample', scene })
  }
  try {
    await fetchRecordingSceneInfo('sample', controller.signal)
    await fetchRecordingScene('sample', { start: 4, end: 10, cloud_topic: '/slam/registered_cloud', path_topic: ['/nav/global_path', '/nav/local_path'] }, controller.signal)
    const frame = await fetchRecordingCameraFrame('sample', 2, 1.29, 'rear', controller.signal)
    assert.equal(frame?.t, 1.23)
    assert.equal(frame?.mediaTime, .37)
    assert.equal(frame?.timing, 'frame_timestamp_index')
    assert.equal(frame?.frameIndex, 37)
    assert.equal(frame?.sourceTimeNs, '1792891234567891234')
    assert.equal(frame?.frameId, 'left_camera')
    assert.ok(requests.every(request => request[1] === 'GET'))
    const cloudUrl = new URL(requests[1][0], 'http://localhost')
    assert.equal(cloudUrl.searchParams.get('max_frames'), '32')
    assert.deepEqual(cloudUrl.searchParams.getAll('path_topic'), ['/nav/global_path', '/nav/local_path'])
    assert.equal(new URL(requests[2][0], 'http://localhost').searchParams.get('camera_id'), 'rear')
    globalThis.fetch = async () => new Response(null, { status: 204 })
    assert.equal(await fetchRecordingCameraFrame('sample', 2, 5, 'rear'), null)
    globalThis.fetch = async () => new Response(new Blob(['frame']))
    await assert.rejects(fetchRecordingCameraFrame('sample', 2, 1.29, 'rear'), /recording_camera_frame_time_missing/)
  } finally { globalThis.fetch = original }
})

test('video-only sessions retain the same historical playhead without requiring a DDS summary', async () => {
  const server = await createServer({ server: { middlewareMode: true, hmr: false, ws: false }, appType: 'custom' })
  try {
    const { RecordingHistory } = await server.ssrLoadModule('/src/components/RecordingHistory.tsx')
    const markup = renderToStaticMarkup(createElement(RecordingHistory, { sessionId: 'camera-only', sessions: [], topics: [], locale: 'zh' }))
    assert.match(markup, /历史播放位置秒数/)
    assert.match(markup, /历史回看倍速/)
    assert.doesNotMatch(markup, /历史数值字段/)
    const { RecordingVisualSource } = await server.ssrLoadModule('/src/components/RecordingVisuals.tsx')
    const pending = renderToStaticMarkup(createElement(RecordingVisualSource, { label: 'B', sessionId: 'camera-only', time: 1, cameraTime: 1, playing: false, locale: 'zh' }))
    assert.match(pending, /正在读取录制内容/)
    assert.doesNotMatch(pending, /未录制相机|未录制点云/)
  } finally { await server.close() }
})
