import {
  Archive,
  Boxes,
  CheckCircle2,
  FileInput,
  LoaderCircle,
  RefreshCw,
  Rocket,
  Search,
  UploadCloud,
} from "lucide-react";
import { useCallback, useEffect, useMemo, useState } from "react";
import type { FormEvent } from "react";

import type {
  ImportKind,
  ImportContract,
  InboxSource,
  SimStudioClient,
  SourceInspection,
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
import { applySourceRecommendations } from "../importDraft.ts";
import {
  EmptyPanel,
  ErrorPanel,
  SectionHeading,
  SkeletonRows,
  StatusBadge,
} from "./Common.tsx";
import { ImportRequestForm } from "./ImportRequestForm.tsx";

interface ImportWorkbenchProps {
  client: SimStudioClient;
  onCatalogChanged: () => void;
}

function parseRequest(value: string): Record<string, unknown> {
  const trimmed = value.trim();
  if (!trimmed) return {};
  const parsed = JSON.parse(trimmed) as unknown;
  if (parsed === null || typeof parsed !== "object" || Array.isArray(parsed)) {
    throw new Error("Import request 必须是 JSON object。");
  }
  return parsed as Record<string, unknown>;
}

function importSummary(record: StudioRecord): string {
  const payload = record.payload;
  return [
    stringField(payload.kind),
    stringField(payload.source_entry ?? payload.source),
    stringField(payload.package_ref ?? payload.reference),
  ].filter((value) => value !== "未提供").join(" · ") || record.id;
}

export function ImportWorkbench({ client, onCatalogChanged }: ImportWorkbenchProps) {
  const [imports, setImports] = useState<StudioRecord[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedId, setSelectedId] = useState("");
  const [sources, setSources] = useState<InboxSource[]>([]);
  const [sourceFile, setSourceFile] = useState<File | null>(null);
  const [uploadedSource, setUploadedSource] = useState<InboxSource | null>(null);
  const [uploading, setUploading] = useState(false);
  const [inspection, setInspection] = useState<SourceInspection | null>(null);
  const [inspecting, setInspecting] = useState(false);
  const [kind, setKind] = useState<ImportKind>("world");
  const [sourceEntry, setSourceEntry] = useState("");
  const [requestText, setRequestText] = useState("");
  const [importContract, setImportContract] = useState<ImportContract | null>(null);
  const [submitting, setSubmitting] = useState(false);
  const [promoting, setPromoting] = useState(false);

  const selected = useMemo(
    () => imports.find((record) => record.id === selectedId) ?? imports[0] ?? null,
    [imports, selectedId],
  );
  const contractLoading = importContract?.kind !== kind;
  const requestDocument = useMemo(() => {
    try {
      return parseRequest(requestText);
    } catch {
      return null;
    }
  }, [requestText]);

  const refreshImports = useCallback(async () => {
    await Promise.resolve();
    setLoading(true);
    setError(null);
    try {
      const result = await client.listImports();
      setImports(result);
      setSelectedId((current) =>
        result.some((record) => record.id === current) ? current : (result.at(-1)?.id ?? ""),
      );
    } catch (reason) {
      setError(errorMessage(reason));
    } finally {
      setLoading(false);
    }
  }, [client]);

  const refreshSources = useCallback(async () => {
    const result = await client.listSources();
    setSources(result.sources);
  }, [client]);

  useEffect(() => {
    const initialLoad = window.setTimeout(() => {
      void Promise.all([refreshImports(), refreshSources()]).catch((reason: unknown) => {
        setError(errorMessage(reason));
      });
    }, 0);
    return () => window.clearTimeout(initialLoad);
  }, [refreshImports, refreshSources]);

  useEffect(() => {
    let active = true;
    void client.getImportContract(kind).then(
      (contract) => {
        if (!active) return;
        setImportContract(contract);
        setRequestText(`${JSON.stringify(contract.request_template, null, 2)}\n`);
      },
      (reason: unknown) => {
        if (!active) return;
        setImportContract(null);
        setError(errorMessage(reason));
      },
    );
    return () => {
      active = false;
    };
  }, [client, kind]);

  async function uploadSelectedSource() {
    if (!sourceFile) {
      setError("请先选择 ZIP、TAR、TAR.GZ 或 TGZ 归档。");
      return;
    }
    setUploading(true);
    setError(null);
    try {
      const source = await client.uploadSource(sourceFile);
      setUploadedSource(source);
      setSourceEntry(source.entry);
      setSources((current) => [
        ...current.filter((item) => item.entry !== source.entry),
        source,
      ]);
      await inspectSource(source);
    } catch (reason) {
      setError(errorMessage(reason));
    } finally {
      setUploading(false);
    }
  }

  function applyInspection(result: SourceInspection) {
    const base = importContract?.kind === kind
      ? importContract.request_template
      : parseRequest(requestText);
    const recommended = applySourceRecommendations(kind, base, result);
    setRequestText(`${JSON.stringify(recommended, null, 2)}\n`);
  }

  async function inspectSource(source?: InboxSource) {
    const selectedSource = source ?? sources.find((item) => item.entry === sourceEntry);
    if (!selectedSource) {
      setError("请先上传或选择一个受管 Source Archive。");
      return;
    }
    setInspecting(true);
    setError(null);
    try {
      const result = await client.inspectSource(selectedSource.source_id);
      setInspection(result);
      applyInspection(result);
    } catch (reason) {
      setInspection(null);
      setError(errorMessage(reason));
    } finally {
      setInspecting(false);
    }
  }

  async function createImport(event: FormEvent<HTMLFormElement>) {
    event.preventDefault();
    setError(null);
    setSubmitting(true);
    try {
      const request = parseRequest(requestText);
      const record = await client.createImport(
        kind,
        sourceEntry.trim(),
        request,
        operationKey("import"),
      );
      setImports((current) => [...current.filter((item) => item.id !== record.id), record]);
      setSelectedId(record.id);
    } catch (reason) {
      setError(errorMessage(reason));
    } finally {
      setSubmitting(false);
    }
  }

  async function promoteSelected() {
    if (!selected) return;
    setPromoting(true);
    setError(null);
    try {
      const record = await client.promoteImport(selected.id, operationKey(`promote-${selected.id}`));
      setImports((current) => current.map((item) => (item.id === record.id ? record : item)));
      setSelectedId(record.id);
      onCatalogChanged();
    } catch (reason) {
      setError(errorMessage(reason));
    } finally {
      setPromoting(false);
    }
  }

  return (
    <section aria-labelledby="imports-title">
      <SectionHeading
        id="imports-title"
        title="Import Workbench"
        description="从 Studio 受控 inbox 暂存 robot/world 包，完成校验后 promote 到本地 Catalog。"
        action={
          <button className="button button--quiet" type="button" onClick={() => void refreshImports()}>
            <RefreshCw aria-hidden="true" size={16} strokeWidth={1.7} />
            刷新 Imports
          </button>
        }
      />

      {error ? <ErrorPanel title="Import 操作失败" message={error} /> : null}

      <div className="import-workbench">
        <form className="import-form" onSubmit={createImport}>
          <fieldset
            className="form-section source-upload-section"
            disabled={uploading || inspecting || submitting}
          >
            <legend>
              <UploadCloud aria-hidden="true" size={18} strokeWidth={1.7} />
              Source Archive
            </legend>
            <div className="source-upload-grid">
              <label className="field source-file-field">
                <span>本地归档</span>
                <input
                  type="file"
                  accept=".zip,.tar,.tar.gz,.tgz,application/zip,application/x-tar,application/gzip"
                  onChange={(event) => {
                    setSourceFile(event.target.files?.item(0) ?? null);
                    setUploadedSource(null);
                  }}
                />
                <small>归档需包含模型/高度图、引用资产及许可证文件。</small>
              </label>
              <button
                className="button button--quiet source-upload-button"
                type="button"
                disabled={!sourceFile || uploading}
                onClick={() => void uploadSelectedSource()}
              >
                {uploading ? (
                  <LoaderCircle className="spin" aria-hidden="true" size={17} strokeWidth={1.8} />
                ) : (
                  <UploadCloud aria-hidden="true" size={17} strokeWidth={1.8} />
                )}
                {uploading ? "正在上传" : "上传到 Inbox"}
              </button>
            </div>
            {uploadedSource ? (
              <div className="source-upload-result" role="status">
                <CheckCircle2 aria-hidden="true" size={17} strokeWidth={1.7} />
                <span>
                  <strong>{uploadedSource.original_name ?? "Source archive"}</strong>
                  <small>
                    {formatBytes(uploadedSource.bytes)} · Source {uploadedSource.source_id}
                  </small>
                </span>
              </div>
            ) : null}
            {sourceEntry && !inspection ? (
              <button
                className="button button--quiet source-inspect-button"
                type="button"
                disabled={inspecting}
                onClick={() => void inspectSource()}
              >
                {inspecting ? (
                  <LoaderCircle className="spin" aria-hidden="true" size={17} strokeWidth={1.8} />
                ) : (
                  <Search aria-hidden="true" size={17} strokeWidth={1.8} />
                )}
                {inspecting ? "正在预检" : "预检当前 Source"}
              </button>
            ) : null}
            {inspection ? (
              <div className="source-inspection" aria-label="Source archive inspection">
                <header>
                  <span>
                    <Search aria-hidden="true" size={17} strokeWidth={1.7} />
                    <strong>Archive inspection</strong>
                  </span>
                  <small>
                    {inspection.summary.files} files · {formatBytes(inspection.summary.total_bytes)} extracted
                  </small>
                </header>
                <dl className="inspection-groups">
                  <div>
                    <dt>Robot model</dt>
                    <dd>
                      {inspection.candidates.robot_models.map((item) => (
                        <code key={item.path}>{item.format}: {item.path}</code>
                      ))}
                      {inspection.candidates.robot_models.length === 0 ? <span>未发现</span> : null}
                    </dd>
                  </div>
                  <div>
                    <dt>Heightmap</dt>
                    <dd>
                      {inspection.candidates.heightmaps.map((item) => <code key={item.path}>{item.path}</code>)}
                      {inspection.candidates.heightmaps.length === 0 ? <span>未发现</span> : null}
                    </dd>
                  </div>
                  <div>
                    <dt>Mesh / License</dt>
                    <dd>
                      {inspection.candidates.meshes.slice(0, 3).map((item) => <code key={item.path}>{item.path}</code>)}
                      {inspection.candidates.licenses.map((item) => <code key={item.path}>{item.path}</code>)}
                      {inspection.candidates.meshes.length + inspection.candidates.licenses.length === 0
                        ? <span>未发现</span>
                        : null}
                    </dd>
                  </div>
                </dl>
                <button
                  className="button button--quiet"
                  type="button"
                  onClick={() => applyInspection(inspection)}
                >
                  <CheckCircle2 aria-hidden="true" size={16} strokeWidth={1.7} />
                  应用 {kind} 文件建议
                </button>
              </div>
            ) : null}
          </fieldset>

          <fieldset className="form-section" disabled={submitting || contractLoading}>
            <legend>
              <FileInput aria-hidden="true" size={18} strokeWidth={1.7} />
              受控导入
            </legend>
            <div className="form-grid form-grid--two">
              <label className="field">
                <span>Package Kind</span>
                <select value={kind} onChange={(event) => setKind(event.target.value as ImportKind)}>
                  <option value="world">world</option>
                  <option value="robot">robot</option>
                </select>
              </label>
              <label className="field">
                <span>Inbox Entry</span>
                <input
                  required
                  list="simstudio-inbox-sources"
                  value={sourceEntry}
                  onChange={(event) => {
                    const entry = event.target.value;
                    setSourceEntry(entry);
                    setUploadedSource(sources.find((item) => item.entry === entry) ?? null);
                    setInspection(null);
                  }}
                  placeholder="incoming/factory_park_hf.zip"
                />
                <datalist id="simstudio-inbox-sources">
                  {sources.map((source) => (
                    <option value={source.entry} key={source.entry}>
                      {source.archive_format} · {formatBytes(source.bytes)} · {source.source_id}
                    </option>
                  ))}
                </datalist>
                <small>上传后自动填写；也可选择已有的受管归档。</small>
              </label>
            </div>
            <div className="import-request-field">
              {requestDocument ? (
                <ImportRequestForm
                  kind={kind}
                  request={requestDocument}
                  disabled={submitting || contractLoading}
                  onChange={(request) => setRequestText(`${JSON.stringify(request, null, 2)}\n`)}
                />
              ) : (
                <div className="contract-note">
                  Advanced JSON 当前不是有效对象；修正后结构化编辑器会恢复。
                </div>
              )}
              <details className="json-disclosure import-json-editor">
                <summary>Advanced JSON</summary>
                <label className="field">
                  <span>{contractLoading ? "正在加载 Import Contract" : "完整 Import Request"}</span>
                  <textarea
                    value={requestText}
                    onChange={(event) => setRequestText(event.target.value)}
                    spellCheck={false}
                  />
                  <small>
                    {importContract
                      ? `模板来自 ${importContract.request_schema.path} · ${shortDigest(importContract.request_schema.sha256, 16)}`
                      : "请求模板不可用。"}
                    {" "}source path 始终由 Studio 注入，不接受绝对路径。
                  </small>
                </label>
              </details>
            </div>
          </fieldset>
          <div className="form-actions">
            <button
              className="button button--primary"
              type="submit"
              disabled={submitting || inspecting || contractLoading || !sourceEntry || !requestText.trim()}
            >
              {submitting ? (
                <LoaderCircle className="spin" aria-hidden="true" size={17} strokeWidth={1.8} />
              ) : (
                <Archive aria-hidden="true" size={17} strokeWidth={1.8} />
              )}
              {submitting ? "正在暂存" : "创建 Import Job"}
            </button>
          </div>
        </form>

        <aside className="import-history" aria-label="Import jobs">
          <div className="pane-title">
            <h2>Import Jobs</h2>
            <span>{imports.length}</span>
          </div>
          {loading ? <SkeletonRows count={5} /> : null}
          {!loading && imports.length === 0 ? (
            <EmptyPanel title="暂无 Import Job">创建导入后，这里会显示 job 状态与 promote 操作。</EmptyPanel>
          ) : null}
          {!loading && imports.length > 0 ? (
            <div className="import-job-list" role="listbox" aria-label="Import jobs">
              {[...imports].reverse().map((record) => (
                <button
                  type="button"
                  className={selected?.id === record.id ? "import-job import-job--selected" : "import-job"}
                  role="option"
                  aria-selected={selected?.id === record.id}
                  onClick={() => setSelectedId(record.id)}
                  key={record.id}
                >
                  <Boxes aria-hidden="true" size={17} strokeWidth={1.6} />
                  <span>
                    <strong>{importSummary(record)}</strong>
                    <small>{formatDate(record.updated_at)}</small>
                  </span>
                  <StatusBadge value={record.status} />
                </button>
              ))}
            </div>
          ) : null}
        </aside>

        <article className="import-detail">
          {!selected ? (
            <EmptyPanel title="选择 Import Job">查看后端保存的 payload，并将合格导入 promote 到 Catalog。</EmptyPanel>
          ) : (
            <>
              <header className="detail-header">
                <div>
                  <span className="kind-label">import</span>
                  <h2>{selected.id}</h2>
                  <p>Revision {selected.revision}，更新于 {formatDate(selected.updated_at)}</p>
                </div>
                <StatusBadge value={selected.status} />
              </header>
              <dl className="fact-grid">
                <div>
                  <dt>Kind</dt>
                  <dd>{selected.kind}</dd>
                </div>
                <div>
                  <dt>Status</dt>
                  <dd>{selected.status}</dd>
                </div>
                <div>
                  <dt>Created</dt>
                  <dd>{formatDate(selected.created_at)}</dd>
                </div>
              </dl>
              <div className="import-actions">
                <button
                  className="button button--primary"
                  type="button"
                  disabled={promoting || selected.status.toLowerCase() === "promoted"}
                  onClick={() => void promoteSelected()}
                >
                  {promoting ? (
                    <LoaderCircle className="spin" aria-hidden="true" size={17} strokeWidth={1.8} />
                  ) : (
                    <Rocket aria-hidden="true" size={17} strokeWidth={1.8} />
                  )}
                  {promoting ? "正在 Promote" : "Promote 到 Catalog"}
                </button>
                {selected.status.toLowerCase() === "promoted" ? (
                  <span className="inline-success">
                    <CheckCircle2 aria-hidden="true" size={16} strokeWidth={1.7} />
                    Catalog refresh requested
                  </span>
                ) : null}
              </div>
              <details className="json-disclosure" open>
                <summary>查看 Import Payload</summary>
                <pre>{JSON.stringify(selected.payload, null, 2)}</pre>
              </details>
            </>
          )}
        </article>
      </div>
    </section>
  );
}
