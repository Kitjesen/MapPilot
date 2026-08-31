import assert from "node:assert/strict";
import test from "node:test";

import {
  boundedFrameIndex,
  recordingReadModelMatches,
  RECORDING_TIMELINE_PAGE_SIZE,
  summarizeRecordingFrame,
  timelineOffsetForFrame,
} from "../src/replayBrowserModel.ts";
import type { RecordingFrame, RecordingStatus, RecordingTimeline } from "../src/api.ts";

test("frame scrubbing stays inside the recording and opens the containing timeline page", () => {
  assert.equal(boundedFrameIndex(-8, 240), 0);
  assert.equal(boundedFrameIndex(17.9, 240), 17);
  assert.equal(boundedFrameIndex(999, 240), 239);
  assert.equal(boundedFrameIndex(0, 0), null);

  assert.equal(timelineOffsetForFrame(0), 0);
  assert.equal(timelineOffsetForFrame(99), 0);
  assert.equal(timelineOffsetForFrame(100), RECORDING_TIMELINE_PAGE_SIZE);
  assert.equal(timelineOffsetForFrame(239), 200);
});

test("a replacement recording for the same run never reuses stale timeline or frame data", () => {
  const selected = {
    run_id: "run01",
    recording_id: "new-recording",
    session_id: "new-session",
  } as RecordingStatus;
  const staleTimeline = {
    run_id: "run01",
    recording_id: "old-recording",
    session_id: "old-session",
  } as RecordingTimeline;
  const staleFrame = {
    run_id: "run01",
    recording_id: "old-recording",
    session_id: "old-session",
  } as RecordingFrame;

  assert.equal(recordingReadModelMatches(selected, staleTimeline), false);
  assert.equal(recordingReadModelMatches(selected, staleFrame), false);
  assert.equal(
    recordingReadModelMatches(selected, {
      ...staleTimeline,
      recording_id: "new-recording",
      session_id: "new-session",
    }),
    true,
  );
});

test("selected frame details expose timestamps, snapshot counts, and the recorded base pose", () => {
  const frame: RecordingFrame = {
    schema: "lingtu.sim.studio.recording-frame.v1",
    recording_id: "a".repeat(32),
    run_id: "run01",
    session_id: "studio-session",
    frame_index: 17,
    relative_time_ns: 340_000_000,
    snapshot: {
      sequence: 27,
      physics_step: 216,
      sim_time_ns: 1_340_000_000,
      model_generation: 2,
      reset_generation: 3,
      bodies: [
        {
          stable_id: "robot_01/arm",
          frame_id: "arm",
          position_m: [0, 0, 1],
          quaternion_wxyz: [1, 0, 0, 0],
        },
        {
          stable_id: "robot_01/base_link",
          frame_id: "base_link",
          position_m: [1.25, -0.5, 0.62],
          quaternion_wxyz: [0.99, 0, 0, 0.14],
        },
      ],
      joints: [{ stable_id: "robot_01/hip" }],
      actuators: [{ stable_id: "robot_01/hip_motor" }],
      sensors: [{ stable_id: "robot_01/imu" }],
    },
    command: { linear_x: 0.3 },
    metadata: {},
    scenario_events: [{ event: "checkpoint" }],
    sensor_metadata: [{ sensor: "imu" }],
    sensor_payloads: [
      {
        schema: "lingtu.sim.sensor-payload-ref.v1",
        payload_index: 0,
        session_id: "studio-session",
        model_generation: 2,
        reset_generation: 3,
        sensor_id: "robot_01.front_rgb",
        stream_kind: "rgb",
        encoding: "rgb8",
        media_type: "application/vnd.lingtu.rgb8",
        sample_sequence: 8,
        sample_time_ns: 1_340_000_000,
        sha256: "c".repeat(64),
        bytes: 6,
        metadata: { width: 2, height: 1, stride_bytes: 6 },
      },
    ],
    lifecycle_evidence: [],
  };

  assert.deepEqual(summarizeRecordingFrame(frame), {
    frameIndex: 17,
    relativeTimeNs: 340_000_000,
    simTimeNs: 1_340_000_000,
    sequence: 27,
    physicsStep: 216,
    modelGeneration: 2,
    resetGeneration: 3,
    counts: {
      bodies: 2,
      joints: 1,
      actuators: 1,
      sensors: 1,
      scenarioEvents: 1,
      sensorMetadata: 1,
      sensorPayloads: 1,
      sensorPayloadBytes: 6,
      lifecycleEvidence: 0,
    },
    hasCommand: true,
    basePose: {
      stableId: "robot_01/base_link",
      positionM: [1.25, -0.5, 0.62],
      quaternionWxyz: [0.99, 0, 0, 0.14],
    },
  });
});
