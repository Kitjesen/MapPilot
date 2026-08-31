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

export function recordingArtifactDownloadBlocked(
  state: string | null | undefined,
): boolean {
  return !DOWNLOADABLE_RECORDING_STATES.has(normalizedState(state))
}
