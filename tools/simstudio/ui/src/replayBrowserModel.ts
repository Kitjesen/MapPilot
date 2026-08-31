import type {
  RecordingFrame,
  RecordingStatus,
  RecordingTimeline,
} from "./api.ts";

export const RECORDING_TIMELINE_PAGE_SIZE = 100;

export interface RecordingFrameDetails {
  frameIndex: number;
  relativeTimeNs: number;
  simTimeNs: number;
  sequence: number;
  physicsStep: number;
  modelGeneration: number;
  resetGeneration: number;
  counts: {
    bodies: number;
    joints: number;
    actuators: number;
    sensors: number;
    scenarioEvents: number;
    sensorMetadata: number;
    sensorPayloads: number;
    sensorPayloadBytes: number;
    lifecycleEvidence: number;
  };
  hasCommand: boolean;
  basePose: {
    stableId: string;
    positionM: number[] | null;
    quaternionWxyz: number[] | null;
  } | null;
}

type RecordingReadModelIdentity = Pick<
  RecordingFrame | RecordingTimeline,
  "run_id" | "recording_id" | "session_id"
>;

export function recordingReadModelMatches(
  recording: Pick<
    RecordingStatus,
    "run_id" | "recording_id" | "session_id"
  >,
  readModel: RecordingReadModelIdentity,
): boolean {
  return recording.recording_id !== null
    && recording.session_id !== null
    && readModel.run_id === recording.run_id
    && readModel.recording_id === recording.recording_id
    && readModel.session_id === recording.session_id;
}

export function boundedFrameIndex(
  value: number,
  frameCount: number,
): number | null {
  if (!Number.isFinite(value) || !Number.isSafeInteger(frameCount) || frameCount <= 0) {
    return null;
  }
  return Math.min(Math.max(Math.trunc(value), 0), frameCount - 1);
}

export function timelineOffsetForFrame(
  frameIndex: number,
  pageSize = RECORDING_TIMELINE_PAGE_SIZE,
): number {
  return Math.floor(Math.max(0, frameIndex) / pageSize) * pageSize;
}

export function summarizeRecordingFrame(
  frame: RecordingFrame,
): RecordingFrameDetails {
  const baseBody = frame.snapshot.bodies.find(
    (body) => body.frame_id === "base_link" || body.stable_id.endsWith("/base_link"),
  );
  return {
    frameIndex: frame.frame_index,
    relativeTimeNs: frame.relative_time_ns,
    simTimeNs: frame.snapshot.sim_time_ns,
    sequence: frame.snapshot.sequence,
    physicsStep: frame.snapshot.physics_step,
    modelGeneration: frame.snapshot.model_generation,
    resetGeneration: frame.snapshot.reset_generation,
    counts: {
      bodies: frame.snapshot.bodies.length,
      joints: frame.snapshot.joints?.length ?? 0,
      actuators: frame.snapshot.actuators?.length ?? 0,
      sensors: frame.snapshot.sensors?.length ?? 0,
      scenarioEvents: frame.scenario_events.length,
      sensorMetadata: frame.sensor_metadata.length,
      sensorPayloads: frame.sensor_payloads.length,
      sensorPayloadBytes: frame.sensor_payloads.reduce(
        (total, payload) => total + payload.bytes,
        0,
      ),
      lifecycleEvidence: frame.lifecycle_evidence.length,
    },
    hasCommand: frame.command !== null,
    basePose: baseBody
      ? {
          stableId: baseBody.stable_id,
          positionM: baseBody.position_m ?? null,
          quaternionWxyz: baseBody.quaternion_wxyz ?? null,
        }
      : null,
  };
}
