import type { CameraMediaStatus } from '../types/index.ts'

export type RecordingLiveCameraState = 'live' | 'loading' | 'not_enabled' | 'waiting' | 'paused' | 'error'

export function recordingLiveCameraState({ hasFrame, lastFrameAt, now, status, statusError, stalled }: {
  hasFrame: boolean
  lastFrameAt: number | null
  now: number | null
  status: Pick<CameraMediaStatus, 'status'> | null
  statusError: boolean
  stalled: boolean
}): RecordingLiveCameraState {
  if (hasFrame && lastFrameAt !== null && (now === null || now - lastFrameAt <= 3000)) return 'live'
  if (status?.status === 'not_loaded') return 'not_enabled'
  if ((!status && statusError) || status?.status === 'error') return 'error'
  if (lastFrameAt !== null || stalled || status?.status === 'stale') return 'paused'
  return status === null ? 'loading' : 'waiting'
}
