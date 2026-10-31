import assert from 'node:assert/strict'
import test from 'node:test'
import { cameraCursorTime } from '../src/services/recordingScene.ts'
import { createRecordingCameraLoader, recordingCameraCoverage, recordingCameraTarget, type RecordingCameraLoadState, type RecordingCameraTarget } from '../src/services/recordingCamera.ts'
import type { RecordingCameraFrame, RecordingCameraSegment } from '../src/types/recordingScene.ts'

class CameraClock {
  current = 0
  sequence = 0
  tasks = new Map<number, { at: number; callback: () => void }>()
  now = () => this.current
  setTimeout = (callback: () => void, delay: number) => { const id = ++this.sequence; this.tasks.set(id, { at: this.current + delay, callback }); return id }
  clearTimeout = (id: number) => { this.tasks.delete(id) }
  async flush() { for (let index = 0; index < 12; index++) await Promise.resolve() }
  async advance(milliseconds: number) {
    const end = this.current + milliseconds
    while (true) {
      const next = [...this.tasks.entries()].filter(([, task]) => task.at <= end).sort((a, b) => a[1].at - b[1].at)[0]
      if (!next) break
      this.current = next[1].at; this.tasks.delete(next[0]); next[1].callback(); await this.flush()
    }
    this.current = end
    await this.flush()
  }
}

function target(time: number, playing = true, extra: Partial<RecordingCameraTarget> = {}): RecordingCameraTarget {
  return { sessionId: 'sample', cameraId: 'color', segment: 0, time: cameraCursorTime(time, 0, playing), cursor: time, playing, ...extra }
}

function frame(time: number): RecordingCameraFrame { return { blob: new Blob(['frame']), t: time, mediaTime: time, timing: 'frame_timestamp_index', sourceTimeNs: String(1e9 + time * 1e9), frameIndex: Math.floor(time * 10), frameId: 'left_camera' } }

test('real API caller advances images under 20 Hz playback with 300 ms responses, one request in flight, and zero ordinary-play aborts', async () => {
  const original = globalThis.fetch
  const clock = new CameraClock()
  const accepted: Array<{ at: number; t: number }> = []
  const states: RecordingCameraLoadState[] = []
  let active = 0; let maximumActive = 0; let aborts = 0; let requests = 0
  globalThis.fetch = (url, options) => new Promise<Response>((resolve, reject) => {
    const time = Number(new URL(String(url), 'http://localhost').searchParams.get('t'))
    active++; requests++; maximumActive = Math.max(active, maximumActive)
    let done = false
    const timer = clock.setTimeout(() => {
      done = true; active--
      resolve(new Response(new Uint8Array([255, 216, 255]), { headers: { 'Content-Type': 'image/jpeg', 'X-Recording-Time-S': String(time), 'X-Recording-Timing': 'frame_timestamp_index', 'X-Recording-Frame-Id': 'left_camera' } }))
    }, 300)
    options?.signal?.addEventListener('abort', () => {
      if (done) return
      aborts++; active--; clock.clearTimeout(timer); reject(new DOMException('Aborted', 'AbortError'))
    }, { once: true })
  })
  const loader = createRecordingCameraLoader({ clock, onFrame: (_request, value) => { if (value) accepted.push({ at: clock.current, t: value.t }) }, onState: state => states.push(state) })
  try {
    loader.update(target(0))
    for (let index = 1; index <= 80; index++) { await clock.advance(50); loader.update(target(index / 20)); await clock.flush() }
    assert.ok(accepted.length >= 10, `completed ${accepted.length} frames over four seconds`)
    assert.ok(accepted[0].at <= 350)
    assert.ok(accepted.at(-1)!.t >= 3.2)
    assert.equal(maximumActive, 1)
    assert.equal(aborts, 0)
    assert.ok(requests <= 14, `coalesced requests: ${requests}`)
    assert.ok(states.some(state => state.phase === 'idle'))
  } finally { loader.dispose(); globalThis.fetch = original; await clock.flush() }
})

test('backward and paused seeks cancel obsolete work and cannot commit a late future frame', async () => {
  const clock = new CameraClock()
  const pending: Array<{ request: RecordingCameraTarget; signal: AbortSignal; resolve: (value: RecordingCameraFrame) => void }> = []
  const accepted: number[] = []
  const loader = createRecordingCameraLoader({ clock, onState: () => {}, onFrame: (_request, value) => { if (value) accepted.push(value.t) }, load: (request, signal) => new Promise(resolve => pending.push({ request, signal, resolve })) })
  loader.update(target(5)); await clock.advance(0)
  loader.update(target(2.123, false)); await clock.advance(0)
  assert.equal(pending[0].signal.aborted, true)
  assert.equal(pending[1].request.time, 2.123)
  pending[0].resolve(frame(5)); await clock.flush()
  assert.deepEqual(accepted, [])
  pending[1].resolve(frame(2.1)); await clock.flush()
  assert.deepEqual(accepted, [2.1])
  loader.update(target(8.765, false)); await clock.advance(0)
  assert.equal(pending[2].request.time, 8.765)
  pending[2].resolve(frame(8.7)); await clock.flush()
  assert.deepEqual(accepted, [2.1, 8.7])
  loader.dispose()
})

test('source and segment transitions abort the previous source and dispose ignores late responses', async () => {
  const clock = new CameraClock()
  const pending: Array<{ signal: AbortSignal; resolve: (value: RecordingCameraFrame) => void }> = []
  const accepted: string[] = []
  const loader = createRecordingCameraLoader({ clock, onState: () => {}, onFrame: request => accepted.push(`${request.cameraId}:${request.segment}`), load: (_request, signal) => new Promise(resolve => pending.push({ signal, resolve })) })
  loader.update(target(1)); await clock.advance(0)
  loader.update(target(1.1, true, { cameraId: 'rear' })); await clock.advance(0)
  assert.equal(pending[0].signal.aborted, true)
  loader.update(target(2, true, { cameraId: 'rear', segment: 1 })); await clock.advance(0)
  assert.equal(pending[1].signal.aborted, true)
  pending[2].resolve(frame(2)); await clock.flush()
  assert.deepEqual(accepted, ['rear:1'])
  loader.update(target(2.4, true, { cameraId: 'rear', segment: 1 })); await clock.advance(200)
  loader.dispose()
  assert.equal(pending[3].signal.aborted, true)
  for (const item of pending) item.resolve(frame(99))
  await clock.flush()
  assert.deepEqual(accepted, ['rear:1'])
})

test('204 is an explicit empty state, while errors stop auto-reading until retry uses the latest cursor', async () => {
  const clock = new CameraClock()
  const states: RecordingCameraLoadState[] = []
  const calls: number[] = []
  let behavior: 'empty' | 'error' | 'ok' = 'empty'
  const loader = createRecordingCameraLoader({ clock, onFrame: () => {}, onState: state => states.push(state), load: async request => {
    calls.push(request.time)
    if (behavior === 'error') throw new Error('decoder unavailable')
    return behavior === 'empty' ? null : frame(request.time)
  } })
  loader.update(target(0, false)); await clock.advance(0)
  assert.equal(states.at(-1)?.phase, 'empty')
  loader.update(target(0, false)); await clock.advance(1000)
  assert.deepEqual(calls, [0])
  behavior = 'error'; loader.update(target(1)); await clock.advance(0)
  assert.equal(states.at(-1)?.phase, 'error')
  loader.update(target(1.4)); await clock.advance(500)
  assert.deepEqual(calls, [0, 1])
  behavior = 'ok'; loader.retry(); await clock.advance(0)
  assert.deepEqual(calls, [0, 1, 1.4])
  assert.equal(states.at(-1)?.phase, 'idle')
  loader.dispose()
})

test('changing recording with the same camera and segment rejects the old recording response', async () => {
  const clock = new CameraClock()
  const pending: Array<{ signal: AbortSignal; resolve: (value: RecordingCameraFrame) => void }> = []
  const accepted: string[] = []
  const loader = createRecordingCameraLoader({ clock, onState: () => {}, onFrame: request => accepted.push(request.sessionId), load: (_request, signal) => new Promise(resolve => pending.push({ signal, resolve })) })
  loader.update(target(1)); await clock.advance(0)
  loader.update(target(1, true, { sessionId: 'other-recording' })); await clock.advance(0)
  assert.equal(pending[0].signal.aborted, true)
  pending[0].resolve(frame(1)); await clock.flush()
  assert.deepEqual(accepted, [])
  pending[1].resolve(frame(1)); await clock.flush()
  assert.deepEqual(accepted, ['other-recording'])
  loader.dispose()
})

test('actual coverage selects the current segment and clamps a sampled cursor that precedes its first frame', () => {
  const segments = [
    { index: 0, camera_id: 'color', start_s: .1, end_s: .9, coverage_end_s: 1 },
    { index: 1, camera_id: 'color', start_s: 1.05, end_s: 1.9, coverage_end_s: 2 },
  ] as RecordingCameraSegment[]
  const first = recordingCameraCoverage(segments, .11, 'color')
  assert.equal(recordingCameraTarget('s', first, .11, 0, true)?.time, .1)
  const next = recordingCameraCoverage(segments, 1.06, 'color')
  assert.equal(recordingCameraTarget('s', next, 1.06, 1, true)?.segment, 1)
  assert.equal(recordingCameraTarget('s', next, 1.06, 1, true)?.time, 1.05)
  assert.equal(recordingCameraTarget('s', next, 1.063, 1, false)?.time, 1.063)
})

test('coverage distinguishes before-start, real gaps and exact recording end with no endless-loading target', () => {
  const segments = [
    { index: 0, camera_id: 'color', start_s: 2.1, end_s: 3, coverage_end_s: 3.1 },
    { index: 1, camera_id: 'color', start_s: 4, end_s: 5, coverage_end_s: 5.1 },
  ] as RecordingCameraSegment[]
  assert.deepEqual(recordingCameraCoverage(segments, 1, 'color'), { kind: 'before', nextStart: 2.1 })
  assert.deepEqual(recordingCameraCoverage(segments, 3.5, 'color'), { kind: 'gap', nextStart: 4 })
  const ended = recordingCameraCoverage(segments, 5.1, 'color')
  assert.deepEqual(ended, { kind: 'ended', end: 5.1 })
  assert.equal(recordingCameraTarget('s', ended, 5.1, 5, true), null)
  assert.deepEqual(recordingCameraCoverage([], 1, 'color'), { kind: 'unrecorded' })
})
