import type { RecordingStatusResponse } from '../types'

export const RECORDING_STATUS_POLL_MS = 2500

const ACTIVE_RECORDING_STATES = new Set(['preparing', 'recording', 'stopping'])
const DOWNLOADABLE_RECORDING_STATES = new Set(['completed', 'failed'])

type RecordingStatusTruth = Pick<
  RecordingStatusResponse,
  'state' | 'recording' | 'healthy'
>

function normalizedState(state: string | null | undefined): string {
  return state?.trim().toLowerCase() ?? ''
}

export function formatRecordingDuration(seconds: number | undefined): string {
  if (seconds === undefined || !Number.isFinite(seconds) || seconds < 0) return '—'
  if (seconds > 0 && seconds < 1) return seconds < 0.05 ? '<0.1s' : `${seconds.toFixed(1)}s`
  const total = Math.floor(seconds)
  if (total < 60) return `${total}s`
  const hours = Math.floor(total / 3600)
  const minutes = Math.floor((total % 3600) / 60)
  const remainder = total % 60
  return hours > 0
    ? `${hours}h ${String(minutes).padStart(2, '0')}m`
    : `${minutes}m ${String(remainder).padStart(2, '0')}s`
}

export function recordingStopHasIncompleteData(errorCode: string | undefined, detail: unknown): boolean {
  return errorCode === 'native_recorder_stop_failed'
    && typeof detail === 'object' && detail !== null
    && 'state' in detail && detail.state === 'failed'
}

export function recordingStatusIsActive(
  status: RecordingStatusTruth | null | undefined,
): boolean {
  return status?.recording === true
    || ACTIVE_RECORDING_STATES.has(normalizedState(status?.state))
}

export function recordingNeedsRecovery(
  status: RecordingStatusTruth | null | undefined,
): boolean {
  return status?.healthy === false
    && ACTIVE_RECORDING_STATES.has(normalizedState(status.state))
}

export function recordingStatusCanStart(
  status: (RecordingStatusTruth & Pick<RecordingStatusResponse, 'available'>) | null | undefined,
): boolean {
  return status?.available === true
    && !recordingStatusIsActive(status)
    && (status.healthy !== false || normalizedState(status.state) === 'failed')
}

export function recordingArtifactDownloadBlocked(
  state: string | null | undefined,
): boolean {
  return !DOWNLOADABLE_RECORDING_STATES.has(normalizedState(state))
}
