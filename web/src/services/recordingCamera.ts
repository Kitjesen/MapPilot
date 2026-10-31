import { fetchRecordingCameraFrame } from './api.ts'
import { CAMERA_PREVIEW_HZ, cameraSegmentAt } from './recordingScene.ts'
import type { RecordingCameraFrame, RecordingCameraSegment } from '../types/recordingScene.ts'

export interface RecordingCameraTarget {
  sessionId: string
  cameraId: string
  segment: number
  time: number
  cursor: number
  playing: boolean
}

export type RecordingCameraCoverage =
  | { kind: 'covered'; segment: RecordingCameraSegment }
  | { kind: 'before' | 'gap'; nextStart: number }
  | { kind: 'ended'; end: number }
  | { kind: 'unrecorded' }

export type RecordingCameraLoadState = { phase: 'idle' | 'loading' | 'empty' | 'error'; error?: string }

export function recordingCameraCoverage(segments: RecordingCameraSegment[], time: number, cameraId: string | undefined): RecordingCameraCoverage {
  const available = segments.filter(segment => !cameraId || segment.camera_id === cameraId).sort((a, b) => a.start_s - b.start_s)
  if (!available.length) return { kind: 'unrecorded' }
  const segment = cameraSegmentAt(available, time, cameraId)
  if (segment) return { kind: 'covered', segment }
  const next = available.find(item => item.start_s > time)
  if (next) return { kind: time < available[0].start_s ? 'before' : 'gap', nextStart: next.start_s }
  const last = available.at(-1)!
  return { kind: 'ended', end: last.coverage_end_s ?? last.end_s }
}

export function recordingCameraTarget(sessionId: string, coverage: RecordingCameraCoverage, cursor: number, sampledTime: number, playing: boolean): RecordingCameraTarget | null {
  if (coverage.kind !== 'covered') return null
  return { sessionId, cameraId: coverage.segment.camera_id, segment: coverage.segment.index,
    time: playing ? Math.min(cursor, Math.max(coverage.segment.start_s, sampledTime)) : cursor, cursor, playing }
}

interface CameraClock { now(): number; setTimeout(callback: () => void, delay: number): number; clearTimeout(timer: number): void }
interface CameraLoaderOptions {
  onFrame(target: RecordingCameraTarget, frame: RecordingCameraFrame | null): void
  onState(state: RecordingCameraLoadState): void
  clock?: CameraClock
  load?: (target: RecordingCameraTarget, signal: AbortSignal) => Promise<RecordingCameraFrame | null>
}

function targetKey(target: RecordingCameraTarget) { return `${target.sessionId}\n${target.cameraId}\n${target.segment}\n${target.time}` }
function sameSource(a: RecordingCameraTarget, b: RecordingCameraTarget) { return a.sessionId === b.sessionId && a.cameraId === b.cameraId && a.segment === b.segment }

/** One decoder request runs at a time; forward playback coalesces into the latest pending target. */
export function createRecordingCameraLoader(options: CameraLoaderOptions) {
  const clock = options.clock ?? { now: () => performance.now(), setTimeout: (callback, delay) => globalThis.setTimeout(callback, delay), clearTimeout: timer => globalThis.clearTimeout(timer) }
  const load = options.load ?? ((target, signal) => fetchRecordingCameraFrame(target.sessionId, target.segment, target.time, target.cameraId, signal))
  let latest: RecordingCameraTarget | null = null
  let active: { target: RecordingCameraTarget; controller: AbortController } | undefined
  let timer: number | undefined
  let completed = ''
  let failed = false
  let disposed = false
  let lastStart = -Infinity

  const cancel = () => {
    if (timer !== undefined) clock.clearTimeout(timer)
    timer = undefined
    active?.controller.abort()
    active = undefined
  }
  const schedule = () => {
    if (disposed || failed || active || timer !== undefined || !latest || targetKey(latest) === completed) return
    const delay = latest.playing ? Math.max(0, 1000 / CAMERA_PREVIEW_HZ - (clock.now() - lastStart)) : 0
    timer = clock.setTimeout(() => {
      timer = undefined
      if (disposed || failed || active || !latest || targetKey(latest) === completed) return
      const request = { target: latest, controller: new AbortController() }
      active = request
      lastStart = clock.now()
      options.onState({ phase: 'loading' })
      void load(request.target, request.controller.signal).then(frame => {
        if (disposed || request.controller.signal.aborted || active !== request) return
        completed = targetKey(request.target)
        options.onFrame(request.target, frame)
        options.onState({ phase: frame ? 'idle' : 'empty' })
      }).catch((cause: unknown) => {
        if (disposed || request.controller.signal.aborted || active !== request) return
        failed = true
        options.onState({ phase: 'error', error: cause instanceof Error ? cause.message : String(cause) })
      }).finally(() => {
        if (active !== request) return
        active = undefined
        schedule()
      })
    }, delay)
  }

  return {
    update(target: RecordingCameraTarget | null) {
      if (disposed) return
      const previous = latest
      if (!target) {
        if (!latest && !active && timer === undefined) return
        latest = null; completed = ''; failed = false; cancel()
        options.onState({ phase: 'idle' })
        return
      }
      const movedSource = !!previous && !sameSource(previous, target)
      const seekBack = !!previous && target.cursor < previous.cursor
      const pausedSeek = !!previous && !target.playing && target.time !== previous.time
      latest = target
      if (movedSource || seekBack || pausedSeek) {
        cancel(); completed = ''; failed = false; lastStart = -Infinity
      }
      schedule()
    },
    retry() { if (disposed) return; failed = false; completed = ''; schedule() },
    dispose() { disposed = true; latest = null; cancel() },
  }
}
