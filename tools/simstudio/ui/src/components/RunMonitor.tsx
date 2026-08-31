import {
  Activity,
  Archive,
  Clapperboard,
  File,
  Folder,
  LoaderCircle,
  Pause,
  Play,
  Plus,
  RefreshCw,
  RotateCcw,
  Square,
  Wrench,
} from "lucide-react";
import { useCallback, useEffect, useMemo, useState } from "react";
import type { ComponentType } from "react";

import type {
  ArtifactRecord,
  ArtifactPreview,
  RunOperation,
  RunReadiness,
  RecordingOperation,
  RecordingStatus,
  SimStudioClient,
  StudioRecord,
} from "../api.ts";
import {
  errorMessage,
  formatBytes,
  formatDate,
  operationKey,
  shortDigest,
  stringField,
} from "../format.ts";
import { availableRunOperations } from "../runPolicy.ts";
import type { ComposedBundle } from "./SessionComposer.tsx";
import {
  EmptyPanel,
  ErrorPanel,
  SectionHeading,
  SkeletonRows,
  StatusBadge,
} from "./Common.tsx";

interface RunMonitorProps {
  client: SimStudioClient;
  pendingBundle: ComposedBundle | null;
  maxPreviewBytes?: number;
}

const OPERATION_LABELS: Record<RunOperation, string> = {
  prepare: "Prepare",
  start: "Start",
  pause: "Pause",
  reset: "Reset",
  stop: "Stop",
};

const OPERATION_ICONS: Record<RunOperation, ComponentType<{ size?: number; strokeWidth?: number }>> = {
  prepare: Wrench,
  start: Play,
  pause: Pause,
  reset: RotateCcw,
  stop: Square,
};

const POLLED_STATES = new Set(["CREATED", "READY", "RUNNING", "PAUSED"]);
const DEFAULT_PREVIEW_BYTES = 65_536;

export function RunMonitor({ client, pendingBundle, maxPreviewBytes }: RunMonitorProps) {
  const [runs, setRuns] = useState<StudioRecord[]>([]);
  const [runsLoading, setRunsLoading] = useState(true);
  const [runsError, setRunsError] = useState<string | null>(null);
  const [selectedId, setSelectedId] = useState("");
  const [readiness, setReadiness] = useState<RunReadiness | null>(null);
  const [artifacts, setArtifacts] = useState<ArtifactRecord[]>([]);
  const [recording, setRecording] = useState<RecordingStatus | null>(null);
  const [artifactDetail, setArtifactDetail] = useState<ArtifactRecord | null>(null);
  const [artifactPreview, setArtifactPreview] = useState<ArtifactPreview | null>(null);
  const [previewLoading, setPreviewLoading] = useState(false);
  const [previewError, setPreviewError] = useState<string | null>(null);
  const [dataRunId, setDataRunId] = useState("");
  const [detailLoading, setDetailLoading] = useState(false);
  const [detailError, setDetailError] = useState<string | null>(null);
  const [creating, setCreating] = useState(false);
  const [operation, setOperation] = useState<RunOperation | null>(null);
  const [recordingOperation, setRecordingOperation] = useState<RecordingOperation | null>(null);

  const refreshRuns = useCallback(async () => {
    await Promise.resolve();
    setRunsLoading(true);
    setRunsError(null);
    try {
      const result = await client.listRuns();
      setRuns(result);
      setSelectedId((current) =>
        result.some((item) => item.id === current) ? current : (result.at(-1)?.id ?? ""),
      );
    } catch (reason) {
      setRunsError(errorMessage(reason));
    } finally {
      setRunsLoading(false);
    }
  }, [client]);

  useEffect(() => {
    const initialLoad = window.setTimeout(() => {
      void refreshRuns();
    }, 0);
    return () => window.clearTimeout(initialLoad);
  }, [refreshRuns]);

  const selectedRun = useMemo(
    () => runs.find((item) => item.id === selectedId) ?? null,
    [runs, selectedId],
  );

  const loadRunData = useCallback(
    async (runId: string, quiet = false) => {
      await Promise.resolve();
      if (!quiet) setDetailLoading(true);
      setDetailError(null);
      try {
        const [record, nextReadiness, nextArtifacts, nextRecording] = await Promise.all([
          client.getRun(runId),
          client.getRunReadiness(runId),
          client.listArtifacts(runId),
          client.inspectRecording(runId),
        ]);
        setRuns((current) => current.map((item) => (item.id === record.id ? record : item)));
        setReadiness(nextReadiness);
        setArtifacts(nextArtifacts);
        setRecording(nextRecording);
        setDataRunId(runId);
        setArtifactDetail((current) =>
          current && nextArtifacts.some((item) => item.artifact_id === current.artifact_id)
            ? current
            : null,
        );
        setArtifactPreview((current) =>
          current && nextArtifacts.some((item) => item.artifact_id === current.artifact_id)
            ? current
            : null,
        );
      } catch (reason) {
        setDetailError(errorMessage(reason));
      } finally {
        if (!quiet) setDetailLoading(false);
      }
    },
    [client],
  );

  useEffect(() => {
    if (!selectedId) return;
    const initialLoad = window.setTimeout(() => {
      void loadRunData(selectedId);
    }, 0);
    if (!selectedRun || !POLLED_STATES.has(selectedRun.status)) {
      return () => window.clearTimeout(initialLoad);
    }
    const timer = window.setInterval(() => {
      void loadRunData(selectedId, true);
    }, 2500);
    return () => {
      window.clearTimeout(initialLoad);
      window.clearInterval(timer);
    };
  }, [loadRunData, selectedId, selectedRun]);

  const hasCurrentRunData = Boolean(selectedId && selectedId === dataRunId);
  const currentReadiness = hasCurrentRunData ? readiness : null;
  const currentArtifacts = hasCurrentRunData ? artifacts : [];
  const currentRecording = hasCurrentRunData ? recording : null;
  const currentArtifactDetail = hasCurrentRunData ? artifactDetail : null;
  const currentArtifactPreview = hasCurrentRunData ? artifactPreview : null;
  const currentDetailError = hasCurrentRunData ? detailError : null;
  const currentDetailLoading = Boolean(selectedRun && (!hasCurrentRunData || detailLoading));

  async function createRun() {
    if (!pendingBundle) return;
    setCreating(true);
    setRunsError(null);
    try {
      const record = await client.createRun(
        pendingBundle.record.id,
        pendingBundle.launchProfile,
        operationKey("run"),
      );
      setRuns((current) => [...current.filter((item) => item.id !== record.id), record]);
      setSelectedId(record.id);
    } catch (reason) {
      setRunsError(errorMessage(reason));
    } finally {
      setCreating(false);
    }
  }

  async function operate(nextOperation: RunOperation) {
    if (!selectedRun) return;
    setOperation(nextOperation);
    setDetailError(null);
    try {
      const record = await client.operateRun(
        selectedRun.id,
        nextOperation,
        selectedRun.revision,
        operationKey(`${nextOperation}-${selectedRun.id}`),
      );
      setRuns((current) => current.map((item) => (item.id === record.id ? record : item)));
      await loadRunData(record.id, true);
    } catch (reason) {
      setDetailError(errorMessage(reason));
    } finally {
      setOperation(null);
    }
  }

  async function inspectArtifact(artifact: ArtifactRecord) {
    setDetailError(null);
    setPreviewError(null);
    setArtifactPreview(null);
    try {
      const detail = await client.getArtifact(artifact.artifact_id);
      setArtifactDetail(detail);
      if (detail.kind !== "file") return;
      setPreviewLoading(true);
      const previewLimit = Math.min(maxPreviewBytes ?? DEFAULT_PREVIEW_BYTES, DEFAULT_PREVIEW_BYTES);
      setArtifactPreview(await client.previewArtifact(detail.run_id, detail.artifact_id, previewLimit));
    } catch (reason) {
      if (artifact.kind === "file") {
        setPreviewError(errorMessage(reason));
      } else {
        setDetailError(errorMessage(reason));
      }
    } finally {
      setPreviewLoading(false);
    }
  }

  async function operateRecording(nextOperation: RecordingOperation) {
    if (!selectedRun) return;
    setRecordingOperation(nextOperation);
    setDetailError(null);
    try {
      const record = await client.operateRecording(
        selectedRun.id,
        nextOperation,
        selectedRun.revision,
        operationKey(`recording-${nextOperation}-${selectedRun.id}`),
      );
      setRuns((current) => current.map((item) => (item.id === record.id ? record : item)));
      await loadRunData(record.id, true);
    } catch (reason) {
      setDetailError(errorMessage(reason));
    } finally {
      setRecordingOperation(null);
    }
  }

  const operations = selectedRun ? availableRunOperations(selectedRun.status) : [];
  const runRecordingValue = selectedRun?.payload.recording;
  const runRecordingState =
    runRecordingValue !== null
    && typeof runRecordingValue === "object"
    && !Array.isArray(runRecordingValue)
    && typeof (runRecordingValue as Record<string, unknown>).state === "string"
      ? String((runRecordingValue as Record<string, unknown>).state)
      : "IDLE";
  const recordingEligibleState = selectedRun?.status === "RUNNING" || selectedRun?.status === "PAUSED";
  const canStartRecording = Boolean(
    recordingEligibleState
    && runRecordingState !== "CAPTURING"
    && currentRecording?.state === "MISSING",
  );
  const canStopRecording = Boolean(
    recordingEligibleState && runRecordingState === "CAPTURING",
  );

  return (
    <section aria-labelledby="runs-title">
      <SectionHeading
        id="runs-title"
        title="Run Monitor"
        description="创建并控制本地仿真会话，持续读取后端 Readiness 和运行制品。"
        action={
          <button className="button button--quiet" type="button" onClick={() => void refreshRuns()}>
            <RefreshCw aria-hidden="true" size={16} strokeWidth={1.7} />
            刷新 Runs
          </button>
        }
      />

      {pendingBundle ? (
        <div className="bundle-launchbar">
          <div>
            <Archive aria-hidden="true" size={20} strokeWidth={1.6} />
            <span>
              Bundle <code>{pendingBundle.record.id}</code>
            </span>
            <StatusBadge value={pendingBundle.launchProfile} />
          </div>
          <button className="button button--primary" type="button" disabled={creating} onClick={() => void createRun()}>
            {creating ? (
              <LoaderCircle className="spin" aria-hidden="true" size={17} strokeWidth={1.8} />
            ) : (
              <Plus aria-hidden="true" size={17} strokeWidth={1.8} />
            )}
            {creating ? "正在创建" : "创建 Run"}
          </button>
        </div>
      ) : (
        <div className="contract-note">
          当前没有新编译的 Bundle。可查看历史 Run，或先在 Session Composer 中编译会话。
        </div>
      )}

      {runsError ? <ErrorPanel title="Run 列表操作失败" message={runsError} onRetry={() => void refreshRuns()} /> : null}

      <div className="run-workbench">
        <aside className="run-list" aria-label="仿真运行列表">
          <div className="pane-title">
            <h2>Runs</h2>
            <span>{runs.length}</span>
          </div>
          {runsLoading ? <SkeletonRows count={5} /> : null}
          {!runsLoading && !runsError && runs.length === 0 ? (
            <EmptyPanel title="还没有 Run">编译 Bundle 后，在上方创建第一个仿真运行。</EmptyPanel>
          ) : null}
          {!runsLoading ? (
            <div role="listbox" aria-label="Runs">
              {[...runs].reverse().map((run) => (
                <button
                  type="button"
                  role="option"
                  aria-selected={selectedId === run.id}
                  className={`run-row${selectedId === run.id ? " run-row--selected" : ""}`}
                  key={run.id}
                  onClick={() => setSelectedId(run.id)}
                >
                  <span>
                    <strong>{stringField(run.payload.session_id, run.id.slice(0, 10))}</strong>
                    <small>{formatDate(run.updated_at)}</small>
                  </span>
                  <StatusBadge value={run.status} />
                </button>
              ))}
            </div>
          ) : null}
        </aside>

        <div className="run-detail">
          {!selectedRun && !runsLoading ? (
            <EmptyPanel title="选择一个 Run">查看生命周期、Readiness、传感器和制品。</EmptyPanel>
          ) : null}
          {selectedRun ? (
            <>
              <header className="run-detail__header">
                <div>
                  <span className="kind-label">run</span>
                  <h2>{selectedRun.id}</h2>
                  <p>
                    Revision {selectedRun.revision}，更新于 {formatDate(selectedRun.updated_at)}
                  </p>
                </div>
                <StatusBadge value={selectedRun.status} />
              </header>

              <div className="run-actions" aria-label="Run 生命周期操作">
                {operations.map((item) => {
                  const Icon = OPERATION_ICONS[item];
                  return (
                    <button
                      className={`button ${item === "stop" ? "button--danger" : "button--quiet"}`}
                      type="button"
                      disabled={operation !== null}
                      onClick={() => void operate(item)}
                      key={item}
                    >
                      {operation === item ? (
                        <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
                      ) : (
                        <Icon aria-hidden="true" size={16} strokeWidth={1.8} />
                      )}
                      {OPERATION_LABELS[item]}
                    </button>
                  );
                })}
                {POLLED_STATES.has(selectedRun.status) ? (
                  <span className="polling-label">
                    <Activity aria-hidden="true" size={15} strokeWidth={1.7} />
                    2.5 秒自动刷新
                  </span>
                ) : null}
              </div>

              {currentDetailError ? <ErrorPanel title="Run 数据读取失败" message={currentDetailError} /> : null}
              {currentDetailLoading ? <SkeletonRows count={3} /> : null}

              {!currentDetailLoading ? (
                <div className="monitor-grid">
                  <section className="monitor-panel">
                    <div className="pane-title">
                      <h3>Readiness</h3>
                      {currentReadiness ? <StatusBadge value={currentReadiness.ready ? "ready" : currentReadiness.status} /> : null}
                    </div>
                    {currentReadiness && Object.keys(currentReadiness.readiness).length > 0 ? (
                      <dl className="readiness-grid">
                        {Object.entries(currentReadiness.readiness).map(([name, value]) => (
                          <div key={name}>
                            <dt>{name}</dt>
                            <dd><StatusBadge value={value} /></dd>
                          </div>
                        ))}
                      </dl>
                    ) : (
                      <p className="muted-copy">后端尚未报告 Runtime Readiness。</p>
                    )}
                  </section>

                  <section className="monitor-panel">
                    <div className="pane-title">
                      <h3>Sensors</h3>
                      <span>{currentReadiness ? Object.keys(currentReadiness.sensors).length : 0}</span>
                    </div>
                    {currentReadiness && Object.keys(currentReadiness.sensors).length > 0 ? (
                      <dl className="readiness-grid">
                        {Object.entries(currentReadiness.sensors).map(([name, value]) => (
                          <div key={name}>
                            <dt>{name}</dt>
                            <dd><StatusBadge value={value} /></dd>
                          </div>
                        ))}
                      </dl>
                    ) : (
                      <p className="muted-copy">当前 Run 没有传感器就绪信息。</p>
                    )}
                  </section>
                </div>
              ) : null}

              {!currentDetailLoading ? (
                <section className="recording-panel">
                  <div className="pane-title">
                    <h3>
                      <Clapperboard aria-hidden="true" size={17} strokeWidth={1.6} />
                      Recording &amp; Replay
                    </h3>
                    <StatusBadge value={runRecordingState === "CAPTURING" ? "CAPTURING" : (currentRecording?.state ?? "unknown")} />
                  </div>
                  {canStartRecording || canStopRecording ? (
                    <div className="recording-actions">
                      {canStartRecording ? (
                        <button
                          className="button button--quiet"
                          type="button"
                          disabled={recordingOperation !== null}
                          onClick={() => void operateRecording("start")}
                        >
                          {recordingOperation === "start" ? (
                            <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
                          ) : (
                            <Clapperboard aria-hidden="true" size={16} strokeWidth={1.8} />
                          )}
                          开始真值录制
                        </button>
                      ) : null}
                      {canStopRecording ? (
                        <button
                          className="button button--danger"
                          type="button"
                          disabled={recordingOperation !== null}
                          onClick={() => void operateRecording("stop")}
                        >
                          {recordingOperation === "stop" ? (
                            <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
                          ) : (
                            <Square aria-hidden="true" size={16} strokeWidth={1.8} />
                          )}
                          停止并提交
                        </button>
                      ) : null}
                      <span>录制窗口由 MuJoCo 仿真时钟定序。</span>
                    </div>
                  ) : null}
                  {runRecordingState === "CAPTURING" ? (
                    <p className="recording-live-note">
                      正在写入有界真值时间线；停止录制或停止 Run 时才会原子发布 manifest。
                    </p>
                  ) : null}
                  {currentRecording?.state === "VALID" ? (
                    <>
                      <dl className="recording-grid">
                        <div>
                          <dt>Frames</dt>
                          <dd>{currentRecording.frame_count.toLocaleString()}</dd>
                        </div>
                        <div>
                          <dt>Simulation duration</dt>
                          <dd>{(currentRecording.duration_ns / 1_000_000_000).toFixed(3)} s</dd>
                        </div>
                        <div>
                          <dt>Model generation</dt>
                          <dd>{currentRecording.model_generation}</dd>
                        </div>
                        <div>
                          <dt>Reset generation</dt>
                          <dd>
                            {currentRecording.reset_generation?.start}
                            {currentRecording.reset_generation?.end !== currentRecording.reset_generation?.start
                              ? ` → ${currentRecording.reset_generation?.end}`
                              : ""}
                          </dd>
                        </div>
                        <div>
                          <dt>Clock authority</dt>
                          <dd>{currentRecording.replay.clock_authority}</dd>
                        </div>
                        <div>
                          <dt>Replay input</dt>
                          <dd>
                            {currentRecording.replay.deterministic ? "deterministic" : "unavailable"}
                            {currentRecording.replay.visual ? " + RobotSimUE" : ""}
                          </dd>
                        </div>
                      </dl>
                      <p className="recording-note">
                        Manifest、timeline、Run 与 SessionBundle 身份已完整校验；该录制可作为后续受控回放作业的输入。
                      </p>
                    </>
                  ) : null}
                  {currentRecording?.state === "MISSING" ? (
                    <p className="muted-copy">
                      {runRecordingState === "CAPTURING"
                        ? "录制尚未提交，因此当前没有可回放 manifest。"
                        : "此 Run 没有已提交的真值时间线；运行或暂停状态下可开始一个受控录制窗口。"}
                    </p>
                  ) : null}
                  {currentRecording?.state === "INVALID" ? (
                    <div className="recording-diagnostic">
                      <strong>录制不可回放</strong>
                      <p>{currentRecording.diagnostics[0]?.message ?? "录制完整性校验失败。"}</p>
                    </div>
                  ) : null}
                </section>
              ) : null}

              <section className="artifact-panel">
                <div className="pane-title">
                  <h3>Artifacts</h3>
                  <span>{currentArtifacts.length}</span>
                </div>
                {currentArtifacts.length === 0 ? (
                  <EmptyPanel title="暂无制品">运行推进后，日志、证据和结果文件会出现在这里。</EmptyPanel>
                ) : (
                  <div className="artifact-layout">
                    <div className="artifact-list">
                      {currentArtifacts.map((artifact) => {
                        const Icon = artifact.kind === "directory" ? Folder : File;
                        return (
                          <button
                            type="button"
                            className={currentArtifactDetail?.artifact_id === artifact.artifact_id ? "is-selected" : ""}
                            onClick={() => void inspectArtifact(artifact)}
                            key={artifact.artifact_id}
                          >
                            <Icon aria-hidden="true" size={16} strokeWidth={1.6} />
                            <span>
                              <strong>{artifact.path}</strong>
                              <small>{artifact.kind === "file" ? formatBytes(artifact.size) : "目录"}</small>
                            </span>
                          </button>
                        );
                      })}
                    </div>
                    <aside className="artifact-detail">
                      {currentArtifactDetail ? (
                        <dl>
                          <div>
                            <dt>Path</dt>
                            <dd>{currentArtifactDetail.path}</dd>
                          </div>
                          <div>
                            <dt>Size</dt>
                            <dd>{formatBytes(currentArtifactDetail.size)}</dd>
                          </div>
                          <div>
                            <dt>SHA-256</dt>
                            <dd title={currentArtifactDetail.sha256 ?? ""}>
                              <code>{shortDigest(currentArtifactDetail.sha256, 18)}</code>
                            </dd>
                          </div>
                        </dl>
                      ) : (
                        <p>选择制品后，前端会通过受控 artifact ID 再次向后端读取元数据。</p>
                      )}
                      <div className="artifact-preview">
                        <div className="artifact-preview__header">
                          <strong>Preview</strong>
                          {previewLoading ? <StatusBadge value="loading" /> : null}
                          {currentArtifactPreview ? <StatusBadge value={currentArtifactPreview.format ?? "preview"} /> : null}
                        </div>
                        {previewError ? <p className="artifact-preview__error">{previewError}</p> : null}
                        {!previewError && previewLoading ? (
                          <p>正在通过后端受控 preview 接口读取片段…</p>
                        ) : null}
                        {!previewError && !previewLoading && currentArtifactDetail?.kind === "directory" ? (
                          <p>目录不提供内容预览。</p>
                        ) : null}
                        {!previewError && !previewLoading && currentArtifactPreview?.previewable ? (
                          <pre>{currentArtifactPreview.content}</pre>
                        ) : null}
                        {!previewError && !previewLoading && currentArtifactPreview && !currentArtifactPreview.previewable ? (
                          <p>{currentArtifactPreview.reason ?? "该制品不是可预览的 UTF-8 文本。"}</p>
                        ) : null}
                        {!previewError && !previewLoading && !currentArtifactDetail ? (
                          <p>选择文件制品后显示 JSON/text 预览；前端不使用原始路径。</p>
                        ) : null}
                      </div>
                    </aside>
                  </div>
                )}
              </section>
            </>
          ) : null}
        </div>
      </div>
    </section>
  );
}
