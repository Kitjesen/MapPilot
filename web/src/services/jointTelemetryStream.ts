import { parseGo2JointState, type Go2JointSample } from './robotJointState.ts'

export interface JointTelemetrySnapshot {
  sample: Go2JointSample | null
  connected: boolean
}

export const EMPTY_JOINT_TELEMETRY: JointTelemetrySnapshot = { sample: null, connected: false }

/** High-rate display telemetry stays outside the application-wide SSE state. */
export function createJointTelemetryStream() {
  let snapshot = EMPTY_JOINT_TELEMETRY
  const listeners = new Set<() => void>()
  const publish = (next: JointTelemetrySnapshot) => {
    snapshot = next
    listeners.forEach(listener => listener())
  }
  return {
    getSnapshot: () => snapshot,
    subscribe(listener: () => void) {
      listeners.add(listener)
      return () => { listeners.delete(listener) }
    },
    setConnected(connected: boolean) {
      if (connected !== snapshot.connected) publish({ ...snapshot, connected })
    },
    ingest(event: unknown, receivedAtMs: number) {
      const sample = parseGo2JointState(event, receivedAtMs)
      if (!sample || (snapshot.sample && sample.stamp <= snapshot.sample.stamp)) return false
      publish({ ...snapshot, sample })
      return true
    },
  }
}

export type JointTelemetryStream = ReturnType<typeof createJointTelemetryStream>
