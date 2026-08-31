import {
  ChevronLeft,
  ChevronRight,
  Clock3,
  Database,
  LoaderCircle,
  RefreshCw,
} from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import type {
  RecordingFrame,
  RecordingFrameSummary,
  RecordingStatus,
  RecordingTimeline,
  SimStudioClient,
} from "../api.ts";
import { errorMessage, shortDigest } from "../format.ts";
import {
  boundedFrameIndex,
  recordingReadModelMatches,
  RECORDING_TIMELINE_PAGE_SIZE,
  summarizeRecordingFrame,
  timelineOffsetForFrame,
} from "../replayBrowserModel.ts";
import {
  EmptyPanel,
  ErrorPanel,
  SectionHeading,
  SkeletonRows,
  StatusBadge,
} from "./Common.tsx";

interface ReplayBrowserProps {
  client: SimStudioClient;
}

interface TimelineCursor {
  runId: string;
  offset: number;
}

interface FrameSelection {
  runId: string;
  frameIndex: number;
}

interface ScopedError {
  key: string;
  message: string;
}

function timelineKey(runId: string, offset: number): string {
  return `${runId}:${offset}`;
}

function frameKey(runId: string, frameIndex: number): string {
  return `${runId}:${frameIndex}`;
}

function formatNanoseconds(value: number): string {
  return `${value.toLocaleString()} ns · ${(value / 1_000_000_000).toFixed(6)} s`;
}

function formatDuration(value: number): string {
  return `${(value / 1_000_000_000).toFixed(3)} s`;
}

function formatBytes(value: number): string {
  if (value < 1024) return `${value.toLocaleString()} B`;
  if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KiB`;
  return `${(value / (1024 * 1024)).toFixed(1)} MiB`;
}

function formatVector(value: number[] | null): string {
  if (!value || value.length === 0) return "未记录";
  return value.map((entry) => entry.toFixed(4)).join(", ");
}

function summaryCounts(frame: RecordingFrameSummary): string {
  const payloads = frame.sensor_payload_count > 0
    ? ` · ${frame.sensor_payload_count} payloads (${formatBytes(frame.sensor_payload_bytes)})`
    : "";
  return `${frame.body_count} bodies · ${frame.joint_count} joints · ${frame.sensor_count} sensors${payloads}`;
}

export function ReplayBrowser({ client }: ReplayBrowserProps) {
  const [recordings, setRecordings] = useState<RecordingStatus[]>([]);
  const [recordingsLoading, setRecordingsLoading] = useState(true);
  const [recordingsError, setRecordingsError] = useState<string | null>(null);
  const [selectedRunId, setSelectedRunId] = useState("");
  const [timelineCursor, setTimelineCursor] = useState<TimelineCursor | null>(null);
  const [timeline, setTimeline] = useState<RecordingTimeline | null>(null);
  const [timelineLoadingKey, setTimelineLoadingKey] = useState<string | null>(null);
  const [timelineError, setTimelineError] = useState<ScopedError | null>(null);
  const [frameSelection, setFrameSelection] = useState<FrameSelection | null>(null);
  const [frame, setFrame] = useState<RecordingFrame | null>(null);
  const [frameLoadingKey, setFrameLoadingKey] = useState<string | null>(null);
  const [frameError, setFrameError] = useState<ScopedError | null>(null);
  const recordingsRequest = useRef(0);
  const timelineRequest = useRef(0);
  const frameRequest = useRef(0);

  const refreshRecordings = useCallback(async () => {
    const requestId = ++recordingsRequest.current;
    setRecordingsLoading(true);
    setRecordingsError(null);
    try {
      const result = await client.listRecordings();
      if (requestId !== recordingsRequest.current) return;
      const visible = result.recordings.filter(
        (recording) => recording.state === "VALID" || recording.state === "INVALID",
      );
      setRecordings(visible);
      setSelectedRunId((current) => {
        if (visible.some((recording) => recording.run_id === current)) return current;
        return visible.find((recording) => recording.state === "VALID")?.run_id
          ?? visible[0]?.run_id
          ?? "";
      });
    } catch (reason) {
      if (requestId === recordingsRequest.current) {
        setRecordingsError(errorMessage(reason));
      }
    } finally {
      if (requestId === recordingsRequest.current) {
        setRecordingsLoading(false);
      }
    }
  }, [client]);

  useEffect(() => {
    const initialLoad = window.setTimeout(() => {
      void refreshRecordings();
    }, 0);
    return () => window.clearTimeout(initialLoad);
  }, [refreshRecordings]);

  const selectedRecording = useMemo(
    () => recordings.find((recording) => recording.run_id === selectedRunId) ?? null,
    [recordings, selectedRunId],
  );
  const activeOffset = timelineCursor?.runId === selectedRunId
    ? timelineCursor.offset
    : 0;
  const activeTimelineKey = selectedRunId
    ? timelineKey(selectedRunId, activeOffset)
    : "";
  const currentTimeline = selectedRecording
    && timeline
    && recordingReadModelMatches(selectedRecording, timeline)
    && timeline.page.offset === activeOffset
    ? timeline
    : null;
  const selectedFrameIndex = frameSelection?.runId === selectedRunId
    ? frameSelection.frameIndex
    : null;
  const activeFrameKey = selectedFrameIndex === null
    ? ""
    : frameKey(selectedRunId, selectedFrameIndex);
  const currentFrame = selectedRecording
    && frame
    && recordingReadModelMatches(selectedRecording, frame)
    && frame.frame_index === selectedFrameIndex
    ? frame
    : null;
  const currentTimelineError = timelineError?.key === activeTimelineKey
    ? timelineError.message
    : null;
  const currentFrameError = frameError?.key === activeFrameKey
    ? frameError.message
    : null;
  const timelineLoading = timelineLoadingKey === activeTimelineKey;
  const frameLoading = frameLoadingKey === activeFrameKey;

  const loadTimeline = useCallback(async (runId: string, offset: number) => {
    const key = timelineKey(runId, offset);
    const requestId = ++timelineRequest.current;
    setTimelineLoadingKey(key);
    setTimelineError(null);
    try {
      const result = await client.getRecordingTimeline(
        runId,
        offset,
        RECORDING_TIMELINE_PAGE_SIZE,
      );
      if (requestId !== timelineRequest.current) return;
      setTimeline(result);
      setFrameSelection((current) => {
        const currentIndex = current?.runId === runId
          ? boundedFrameIndex(current.frameIndex, result.frame_count)
          : null;
        const nextIndex = currentIndex
          ?? result.frames[0]?.frame_index
          ?? boundedFrameIndex(0, result.frame_count);
        return nextIndex === null ? null : { runId, frameIndex: nextIndex };
      });
    } catch (reason) {
      if (requestId === timelineRequest.current) {
        setTimelineError({ key, message: errorMessage(reason) });
      }
    } finally {
      if (requestId === timelineRequest.current) {
        setTimelineLoadingKey(null);
      }
    }
  }, [client]);

  useEffect(() => {
    if (!selectedRecording || selectedRecording.state !== "VALID") {
      timelineRequest.current += 1;
      return;
    }
    const initialLoad = window.setTimeout(() => {
      void loadTimeline(selectedRecording.run_id, activeOffset);
    }, 0);
    return () => window.clearTimeout(initialLoad);
  }, [activeOffset, loadTimeline, selectedRecording]);

  const loadFrame = useCallback(async (runId: string, frameIndex: number) => {
    const key = frameKey(runId, frameIndex);
    const requestId = ++frameRequest.current;
    setFrameLoadingKey(key);
    setFrameError(null);
    try {
      const result = await client.getRecordingFrame(runId, frameIndex);
      if (requestId === frameRequest.current) {
        setFrame(result);
      }
    } catch (reason) {
      if (requestId === frameRequest.current) {
        setFrameError({ key, message: errorMessage(reason) });
      }
    } finally {
      if (requestId === frameRequest.current) {
        setFrameLoadingKey(null);
      }
    }
  }, [client]);

  useEffect(() => {
    if (
      !selectedRecording
      || selectedRecording.state !== "VALID"
      || selectedFrameIndex === null
    ) {
      frameRequest.current += 1;
      return;
    }
    const initialLoad = window.setTimeout(() => {
      void loadFrame(selectedRecording.run_id, selectedFrameIndex);
    }, 0);
    return () => window.clearTimeout(initialLoad);
  }, [loadFrame, selectedFrameIndex, selectedRecording]);

  function selectRecording(recording: RecordingStatus) {
    timelineRequest.current += 1;
    frameRequest.current += 1;
    setSelectedRunId(recording.run_id);
    setTimelineCursor({ runId: recording.run_id, offset: 0 });
    const initialFrame = boundedFrameIndex(0, recording.frame_count);
    setFrameSelection(
      initialFrame === null
        ? null
        : { runId: recording.run_id, frameIndex: initialFrame },
    );
  }

  function selectFrame(nextValue: number) {
    if (!selectedRecording || selectedRecording.state !== "VALID") return;
    const frameCount = currentTimeline?.frame_count ?? selectedRecording.frame_count;
    const nextFrameIndex = boundedFrameIndex(nextValue, frameCount);
    if (nextFrameIndex === null) return;
    setFrameSelection({ runId: selectedRecording.run_id, frameIndex: nextFrameIndex });
    setTimelineCursor({
      runId: selectedRecording.run_id,
      offset: timelineOffsetForFrame(nextFrameIndex),
    });
  }

  function selectPage(offset: number) {
    if (!selectedRecording || selectedRecording.state !== "VALID") return;
    const nextFrameIndex = boundedFrameIndex(offset, selectedRecording.frame_count);
    setTimelineCursor({ runId: selectedRecording.run_id, offset });
    if (nextFrameIndex !== null) {
      setFrameSelection({
        runId: selectedRecording.run_id,
        frameIndex: nextFrameIndex,
      });
    }
  }

  const frameCount = currentTimeline?.frame_count ?? selectedRecording?.frame_count ?? 0;
  const frameDetails = currentFrame ? summarizeRecordingFrame(currentFrame) : null;
  const pageStart = currentTimeline && currentTimeline.page.returned > 0
    ? currentTimeline.page.offset + 1
    : 0;
  const pageEnd = currentTimeline
    ? currentTimeline.page.offset + currentTimeline.page.returned
    : 0;

  return (
    <section aria-labelledby="replay-title">
      <SectionHeading
        id="replay-title"
        title="Replay Browser"
        description="只读浏览已提交的仿真录制、真值时间线和单帧证据。此界面不会启动仿真或视觉回放。"
        action={
          <button
            className="button button--quiet"
            type="button"
            disabled={recordingsLoading}
            onClick={() => void refreshRecordings()}
          >
            {recordingsLoading ? (
              <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
            ) : (
              <RefreshCw aria-hidden="true" size={16} strokeWidth={1.7} />
            )}
            刷新 Recordings
          </button>
        }
      />

      <div className="replay-contract-note" role="note">
        <Database aria-hidden="true" size={17} strokeWidth={1.65} />
        <span>
          数据来自后端校验后的 Run-owned recording read model；前端不接收 artifact 路径、可执行文件或启动参数。
        </span>
      </div>

      {recordingsError ? (
        <ErrorPanel
          title="Recording 列表读取失败"
          message={recordingsError}
          onRetry={() => void refreshRecordings()}
        />
      ) : null}

      <div className="replay-workbench">
        <aside className="replay-recording-pane" aria-label="仿真录制列表">
          <div className="pane-title">
            <h2>Recordings</h2>
            <span>{recordings.length}</span>
          </div>
          {recordingsLoading && recordings.length === 0 ? <SkeletonRows count={5} /> : null}
          {!recordingsLoading && !recordingsError && recordings.length === 0 ? (
            <EmptyPanel title="暂无已提交录制">
              Run 提交录制后，VALID 或 INVALID 的完整性状态会显示在这里。
            </EmptyPanel>
          ) : null}
          {recordings.length > 0 ? (
            <div role="listbox" aria-label="Recordings">
              {recordings.map((recording) => (
                <button
                  type="button"
                  role="option"
                  aria-selected={selectedRunId === recording.run_id}
                  className={`replay-recording-row${selectedRunId === recording.run_id ? " replay-recording-row--selected" : ""}`}
                  key={recording.run_id}
                  onClick={() => selectRecording(recording)}
                >
                  <span className="replay-recording-row__identity">
                    <strong title={recording.run_id}>{recording.run_id}</strong>
                    <small title={recording.recording_id ?? ""}>
                      {recording.recording_id
                        ? `rec ${shortDigest(recording.recording_id, 14)}`
                        : "recording identity unavailable"}
                    </small>
                  </span>
                  <span className="replay-recording-row__meta">
                    <StatusBadge value={recording.state} />
                    <small>
                      {recording.state === "VALID"
                        ? `${recording.frame_count.toLocaleString()} frames · ${formatDuration(recording.duration_ns)}`
                        : "integrity check failed"}
                    </small>
                  </span>
                </button>
              ))}
            </div>
          ) : null}
        </aside>

        <div className="replay-detail-pane">
          {!selectedRecording && !recordingsLoading ? (
            <EmptyPanel title="选择 Recording">
              从左侧选择一条录制以检查时间线和单帧真值。
            </EmptyPanel>
          ) : null}

          {selectedRecording ? (
            <>
              <header className="replay-detail-header">
                <div>
                  <span className="kind-label">Validated recording</span>
                  <h2 title={selectedRecording.run_id}>{selectedRecording.run_id}</h2>
                  <p>
                    Recording <code>{shortDigest(selectedRecording.recording_id, 20)}</code>
                    {selectedRecording.session_id
                      ? <> · Session <code>{shortDigest(selectedRecording.session_id, 18)}</code></>
                      : null}
                  </p>
                </div>
                <StatusBadge value={selectedRecording.state} />
              </header>

              {selectedRecording.state === "INVALID" ? (
                <ErrorPanel
                  title="Recording 不可浏览"
                  message={selectedRecording.diagnostics[0]?.message ?? "录制完整性校验失败。"}
                />
              ) : null}

              {selectedRecording.state === "VALID" ? (
                <>
                  <dl className="replay-recording-facts">
                    <div>
                      <dt>Frames</dt>
                      <dd>{selectedRecording.frame_count.toLocaleString()}</dd>
                    </div>
                    <div>
                      <dt>Duration</dt>
                      <dd>{formatDuration(selectedRecording.duration_ns)}</dd>
                    </div>
                    <div>
                      <dt>Clock authority</dt>
                      <dd>{selectedRecording.replay.clock_authority ?? "未记录"}</dd>
                    </div>
                    <div>
                      <dt>Reset generation</dt>
                      <dd>
                        {selectedRecording.reset_generation
                          ? `${selectedRecording.reset_generation.start} → ${selectedRecording.reset_generation.end}`
                          : "未记录"}
                      </dd>
                    </div>
                  </dl>

                  <section className="replay-timeline-panel" aria-labelledby="timeline-title">
                    <div className="pane-title">
                      <h3 id="timeline-title">Timeline</h3>
                      <span>{frameCount.toLocaleString()} frames</span>
                    </div>

                    <div className="replay-scrubber">
                      <div className="replay-scrubber__header">
                        <label htmlFor="replay-frame-number">Frame index</label>
                        <span id="replay-frame-help">0 – {Math.max(frameCount - 1, 0).toLocaleString()}</span>
                      </div>
                      <input
                        id="replay-frame-range"
                        aria-label="Frame scrubber"
                        aria-describedby="replay-frame-help"
                        type="range"
                        min={0}
                        max={Math.max(frameCount - 1, 0)}
                        step={1}
                        value={selectedFrameIndex ?? 0}
                        disabled={frameCount === 0}
                        onChange={(event) => selectFrame(Number(event.currentTarget.value))}
                      />
                      <div className="replay-frame-number">
                        <input
                          id="replay-frame-number"
                          aria-describedby="replay-frame-help"
                          type="number"
                          inputMode="numeric"
                          min={0}
                          max={Math.max(frameCount - 1, 0)}
                          step={1}
                          value={selectedFrameIndex ?? 0}
                          disabled={frameCount === 0}
                          onChange={(event) => selectFrame(Number(event.currentTarget.value))}
                        />
                        <span>/ {Math.max(frameCount - 1, 0).toLocaleString()}</span>
                      </div>
                    </div>

                    <div className="replay-pagination" aria-label="Timeline page navigation">
                      <button
                        className="button button--quiet"
                        type="button"
                        disabled={activeOffset === 0 || timelineLoading}
                        onClick={() => selectPage(Math.max(0, activeOffset - RECORDING_TIMELINE_PAGE_SIZE))}
                      >
                        <ChevronLeft aria-hidden="true" size={16} strokeWidth={1.8} />
                        Previous
                      </button>
                      <span aria-live="polite">
                        {currentTimeline
                          ? `Frames ${pageStart.toLocaleString()}–${pageEnd.toLocaleString()} of ${currentTimeline.frame_count.toLocaleString()}`
                          : "Timeline page unavailable"}
                      </span>
                      <button
                        className="button button--quiet"
                        type="button"
                        disabled={currentTimeline?.page.next_offset == null || timelineLoading}
                        onClick={() => {
                          if (currentTimeline?.page.next_offset != null) {
                            selectPage(currentTimeline.page.next_offset);
                          }
                        }}
                      >
                        Next
                        <ChevronRight aria-hidden="true" size={16} strokeWidth={1.8} />
                      </button>
                    </div>

                    {currentTimelineError ? (
                      <ErrorPanel
                        title="Timeline 读取失败"
                        message={currentTimelineError}
                        onRetry={() => void loadTimeline(selectedRecording.run_id, activeOffset)}
                      />
                    ) : null}
                    {timelineLoading && !currentTimeline ? <SkeletonRows count={5} /> : null}
                    {!timelineLoading && !currentTimelineError && currentTimeline?.frames.length === 0 ? (
                      <EmptyPanel title="Timeline 为空">
                        此 Recording 没有可浏览的真值帧。
                      </EmptyPanel>
                    ) : null}
                    {currentTimeline && currentTimeline.frames.length > 0 ? (
                      <div
                        className="replay-frame-list"
                        role="listbox"
                        aria-label={`Timeline frames ${pageStart} through ${pageEnd}`}
                        aria-busy={timelineLoading}
                      >
                        {currentTimeline.frames.map((summary) => (
                          <button
                            type="button"
                            role="option"
                            aria-selected={selectedFrameIndex === summary.frame_index}
                            className={`replay-frame-row${selectedFrameIndex === summary.frame_index ? " replay-frame-row--selected" : ""}`}
                            key={summary.frame_index}
                            onClick={() => selectFrame(summary.frame_index)}
                          >
                            <span className="replay-frame-row__index">#{summary.frame_index}</span>
                            <span>
                              <strong>{formatNanoseconds(summary.relative_time_ns)}</strong>
                              <small>{summaryCounts(summary)}</small>
                            </span>
                            <span>
                              <small>step {summary.physics_step.toLocaleString()}</small>
                              {summary.has_command ? <StatusBadge value="command" /> : null}
                            </span>
                          </button>
                        ))}
                      </div>
                    ) : null}
                  </section>

                  <section className="replay-frame-detail" aria-labelledby="frame-detail-title">
                    <div className="pane-title">
                      <h3 id="frame-detail-title">
                        <Clock3 aria-hidden="true" size={16} strokeWidth={1.65} />
                        Selected frame
                      </h3>
                      {selectedFrameIndex !== null ? <span>#{selectedFrameIndex}</span> : null}
                    </div>
                    {currentFrameError ? (
                      <ErrorPanel
                        title="Frame 读取失败"
                        message={currentFrameError}
                        onRetry={selectedFrameIndex === null
                          ? undefined
                          : () => void loadFrame(selectedRecording.run_id, selectedFrameIndex)}
                      />
                    ) : null}
                    {frameLoading && !currentFrame ? <SkeletonRows count={4} /> : null}
                    {!frameLoading && !currentFrameError && selectedFrameIndex === null ? (
                      <EmptyPanel title="暂无 Frame">
                        Timeline 包含帧后，可通过滑块、数字输入或列表选择单帧。
                      </EmptyPanel>
                    ) : null}
                    {frameDetails && currentFrame ? (
                      <div className="replay-frame-evidence">
                        <dl className="replay-timestamp-grid">
                          <div>
                            <dt>Relative time</dt>
                            <dd>{formatNanoseconds(frameDetails.relativeTimeNs)}</dd>
                          </div>
                          <div>
                            <dt>Simulation time</dt>
                            <dd>{formatNanoseconds(frameDetails.simTimeNs)}</dd>
                          </div>
                          <div>
                            <dt>Sequence</dt>
                            <dd>{frameDetails.sequence.toLocaleString()}</dd>
                          </div>
                          <div>
                            <dt>Physics step</dt>
                            <dd>{frameDetails.physicsStep.toLocaleString()}</dd>
                          </div>
                          <div>
                            <dt>Model generation</dt>
                            <dd>{frameDetails.modelGeneration}</dd>
                          </div>
                          <div>
                            <dt>Reset generation</dt>
                            <dd>{frameDetails.resetGeneration}</dd>
                          </div>
                        </dl>

                        <div className="replay-evidence-grid">
                          <section className="replay-pose-card" aria-labelledby="base-pose-title">
                            <h4 id="base-pose-title">Recorded base pose</h4>
                            {frameDetails.basePose ? (
                              <dl>
                                <div>
                                  <dt>Stable ID</dt>
                                  <dd>{frameDetails.basePose.stableId}</dd>
                                </div>
                                <div>
                                  <dt>Position xyz (m)</dt>
                                  <dd>{formatVector(frameDetails.basePose.positionM)}</dd>
                                </div>
                                <div>
                                  <dt>Quaternion wxyz</dt>
                                  <dd>{formatVector(frameDetails.basePose.quaternionWxyz)}</dd>
                                </div>
                              </dl>
                            ) : (
                              <p>此帧没有记录 <code>base_link</code> body pose。</p>
                            )}
                          </section>

                          <section className="replay-count-card" aria-labelledby="snapshot-counts-title">
                            <h4 id="snapshot-counts-title">Snapshot counts</h4>
                            <dl>
                              {Object.entries(frameDetails.counts).map(([label, value]) => (
                                <div key={label}>
                                  <dt>{label.replace(/([A-Z])/g, " $1")}</dt>
                                  <dd>{value.toLocaleString()}</dd>
                                </div>
                              ))}
                              <div>
                                <dt>Command</dt>
                                <dd>{frameDetails.hasCommand ? "present" : "none"}</dd>
                              </div>
                            </dl>
                          </section>
                        </div>

                        <section
                          className="replay-sensor-payload-card"
                          aria-labelledby="sensor-payload-title"
                        >
                          <header>
                            <div>
                              <h4 id="sensor-payload-title">Sensor payload evidence</h4>
                              <p>内容摘要已由后端校验；此处只呈现安全引用，不暴露存储路径。</p>
                            </div>
                            <span>
                              {frameDetails.counts.sensorPayloads.toLocaleString()} refs · {formatBytes(frameDetails.counts.sensorPayloadBytes)}
                            </span>
                          </header>
                          {currentFrame.sensor_payloads.length === 0 ? (
                            <p className="replay-sensor-payload-card__empty">
                              此帧没有 RGB、Depth、LiDAR 或其他传感器 payload。
                            </p>
                          ) : (
                            <div className="replay-sensor-payload-list">
                              {currentFrame.sensor_payloads.map((payload) => (
                                <article
                                  className="replay-sensor-payload-row"
                                  key={`${payload.payload_index}:${payload.sha256}`}
                                >
                                  <header>
                                    <strong>{payload.sensor_id}</strong>
                                    <StatusBadge value={payload.stream_kind} />
                                  </header>
                                  <dl>
                                    <div>
                                      <dt>Encoding</dt>
                                      <dd>{payload.encoding}</dd>
                                    </div>
                                    <div>
                                      <dt>Media type</dt>
                                      <dd>{payload.media_type}</dd>
                                    </div>
                                    <div>
                                      <dt>Sample</dt>
                                      <dd>#{payload.sample_sequence.toLocaleString()}</dd>
                                    </div>
                                    <div>
                                      <dt>Timestamp</dt>
                                      <dd>{formatNanoseconds(payload.sample_time_ns)}</dd>
                                    </div>
                                    <div>
                                      <dt>Payload</dt>
                                      <dd>{formatBytes(payload.bytes)}</dd>
                                    </div>
                                    <div>
                                      <dt>SHA-256</dt>
                                      <dd title={payload.sha256}>{shortDigest(payload.sha256, 18)}</dd>
                                    </div>
                                  </dl>
                                  {Object.keys(payload.metadata).length > 0 ? (
                                    <details>
                                      <summary>Capture metadata</summary>
                                      <pre>{JSON.stringify(payload.metadata, null, 2)}</pre>
                                    </details>
                                  ) : null}
                                </article>
                              ))}
                            </div>
                          )}
                        </section>

                        <details className="replay-payload-disclosure">
                          <summary>Recorded command, metadata, payload references, and event evidence</summary>
                          <pre>{JSON.stringify({
                            command: currentFrame.command,
                            metadata: currentFrame.metadata,
                            scenario_events: currentFrame.scenario_events,
                            sensor_metadata: currentFrame.sensor_metadata,
                            sensor_payloads: currentFrame.sensor_payloads,
                            lifecycle_evidence: currentFrame.lifecycle_evidence,
                          }, null, 2)}</pre>
                        </details>
                      </div>
                    ) : null}
                  </section>
                </>
              ) : null}
            </>
          ) : null}
        </div>
      </div>
    </section>
  );
}
