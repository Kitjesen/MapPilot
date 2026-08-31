import {
  Box,
  CheckCircle2,
  Download,
  FilePlus2,
  LoaderCircle,
  MapPinned,
  PackagePlus,
  Plus,
  RefreshCw,
  Save,
  Trash2,
} from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import type {
  SceneDraftRecord,
  SceneElementPlacement,
  ScenePublication,
  SceneToolCatalog,
  SceneToolValidation,
  SimStudioClient,
} from "../api.ts";
import { errorMessage, operationKey, shortDigest } from "../format.ts";
import {
  buildSceneElementBatch,
  createScenePlacement,
  eligibleSceneSurfaces,
  sceneBounds,
} from "../sceneDraft.ts";
import {
  EmptyPanel,
  ErrorPanel,
  SectionHeading,
  SkeletonRows,
  StatusBadge,
} from "./Common.tsx";

interface SceneWorkbenchProps {
  client: SimStudioClient;
  onUsePublishedWorldForNewSession?: (publication: ScenePublication) => void | Promise<void>;
}

interface ElementDraft extends SceneElementPlacement {
  editorId: string;
}

export function SceneWorkbench({ client, onUsePublishedWorldForNewSession }: SceneWorkbenchProps) {
  const [catalog, setCatalog] = useState<SceneToolCatalog | null>(null);
  const [loading, setLoading] = useState(true);
  const [loadError, setLoadError] = useState<string | null>(null);
  const [batchId, setBatchId] = useState("studio_layout");
  const [description, setDescription] = useState("");
  const [elements, setElements] = useState<ElementDraft[]>([]);
  const [selectedId, setSelectedId] = useState("");
  const [validation, setValidation] = useState<SceneToolValidation | null>(null);
  const [validationError, setValidationError] = useState<string | null>(null);
  const [validating, setValidating] = useState(false);
  const [sceneDrafts, setSceneDrafts] = useState<SceneDraftRecord[]>([]);
  const [activeDraft, setActiveDraft] = useState<SceneDraftRecord | null>(null);
  const [draftsLoading, setDraftsLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);
  const [packageId, setPackageId] = useState("studio_factory_park");
  const [packageVersion, setPackageVersion] = useState("1.0.0");
  const [publishing, setPublishing] = useState(false);
  const [publishError, setPublishError] = useState<string | null>(null);
  const [publication, setPublication] = useState<ScenePublication | null>(null);
  const nextOrdinal = useRef(1);

  const loadCatalog = useCallback(async () => {
    setLoading(true);
    setLoadError(null);
    try {
      setCatalog(await client.getFactoryParkSceneCatalog());
    } catch (reason) {
      setLoadError(errorMessage(reason));
    } finally {
      setLoading(false);
    }
  }, [client]);

  const loadSceneDrafts = useCallback(async () => {
    setDraftsLoading(true);
    try {
      setSceneDrafts(await client.listSceneDrafts());
    } catch (reason) {
      setSaveError(errorMessage(reason));
    } finally {
      setDraftsLoading(false);
    }
  }, [client]);

  useEffect(() => {
    const timer = window.setTimeout(
      () => void Promise.all([loadCatalog(), loadSceneDrafts()]),
      0,
    );
    return () => window.clearTimeout(timer);
  }, [loadCatalog, loadSceneDrafts]);

  const selected = useMemo(
    () => elements.find((element) => element.editorId === selectedId) ?? null,
    [elements, selectedId],
  );
  const bounds = useMemo(() => catalog ? sceneBounds(catalog) : null, [catalog]);

  function invalidateValidation() {
    setValidation(null);
    setValidationError(null);
    setSaveError(null);
    setPublishError(null);
    setPublication(null);
  }

  function startNewDraft() {
    setActiveDraft(null);
    setBatchId("studio_layout");
    setDescription("");
    setElements([]);
    setSelectedId("");
    setValidation(null);
    setValidationError(null);
    setPackageId("studio_layout_world");
    setPackageVersion("1.0.0");
    setPublishError(null);
    setPublication(null);
    setSaveError(null);
    nextOrdinal.current = 1;
  }

  function openDraft(record: SceneDraftRecord) {
    const batch = record.payload.batch;
    const hydrated = batch.elements.map((element, index) => ({
      editorId: `scene-element-${index + 1}`,
      instance_key: element.instance_key,
      element_type: element.element_type,
      surface_id: element.surface_id,
      position_xy_m: [...element.position_xy_m] as [number, number],
      yaw_deg: element.yaw_deg,
    }));
    setActiveDraft(record);
    setBatchId(batch.batch_id);
    setDescription(batch.description);
    setElements(hydrated);
    setSelectedId(hydrated[0]?.editorId ?? "");
    nextOrdinal.current = hydrated.length + 1;
    setPackageId(`${batch.batch_id}_world`);
    setPackageVersion("1.0.0");
    setPublishError(null);
    setPublication(null);
    setValidationError(null);
    if (catalog && record.payload.layout_digest !== catalog.layout_digest) {
      setValidation(null);
      setSaveError("该草稿基于旧版 FactoryPark 布局；重新保存前必须再次通过当前几何校验。");
    } else {
      setValidation(record.payload.validation);
      setSaveError(null);
    }
  }

  function addElement() {
    if (!catalog) return;
    const placement = createScenePlacement(catalog, nextOrdinal.current);
    const editorId = `scene-element-${nextOrdinal.current}`;
    nextOrdinal.current += 1;
    const created = { editorId, ...placement };
    setElements((current) => [...current, created]);
    setSelectedId(editorId);
    invalidateValidation();
  }

  function removeElement(editorId: string) {
    setElements((current) => current.filter((element) => element.editorId !== editorId));
    setSelectedId((current) => current === editorId ? "" : current);
    invalidateValidation();
  }

  function updateElement(editorId: string, update: Partial<ElementDraft>) {
    setElements((current) => current.map((element) =>
      element.editorId === editorId ? { ...element, ...update } : element));
    invalidateValidation();
  }

  function changeElementType(element: ElementDraft, elementType: string) {
    if (!catalog) return;
    const surfaces = eligibleSceneSurfaces(catalog, elementType);
    const currentSurface = surfaces.find((surface) => surface.surface_id === element.surface_id);
    const surface = currentSurface
      ?? surfaces.find((candidate) => candidate.semantic_class === "parking_area")
      ?? surfaces[0];
    if (!surface) return;
    updateElement(element.editorId, {
      element_type: elementType,
      surface_id: surface.surface_id,
      position_xy_m: [...surface.position_xy_m],
    });
  }

  function changeSurface(element: ElementDraft, surfaceId: string) {
    const surface = catalog?.surfaces.find((candidate) => candidate.surface_id === surfaceId);
    if (!surface) return;
    updateElement(element.editorId, {
      surface_id: surface.surface_id,
      position_xy_m: [...surface.position_xy_m],
    });
  }

  function draftBatch() {
    if (!catalog) throw new Error("scene catalog is unavailable");
    return buildSceneElementBatch(
      catalog,
      batchId,
      description,
      elements.map((element) => ({
        instance_key: element.instance_key,
        element_type: element.element_type,
        surface_id: element.surface_id,
        position_xy_m: [...element.position_xy_m],
        yaw_deg: element.yaw_deg,
      })),
    );
  }

  async function validateBatch() {
    setValidating(true);
    setValidation(null);
    setValidationError(null);
    try {
      setValidation(await client.validateFactoryParkElementBatch(draftBatch()));
    } catch (reason) {
      setValidationError(errorMessage(reason));
    } finally {
      setValidating(false);
    }
  }

  async function saveDraft() {
    setSaving(true);
    setSaveError(null);
    try {
      const batch = draftBatch();
      const saved = activeDraft
        ? await client.updateSceneDraft(
            activeDraft.id,
            activeDraft.revision,
            batch,
            operationKey("scene-draft-update"),
          )
        : await client.createSceneDraft(batch, operationKey("scene-draft-create"));
      setActiveDraft(saved);
      setPublication(null);
      setValidation(saved.payload.validation);
      setSceneDrafts((current) => {
        const withoutSaved = current.filter((item) => item.id !== saved.id);
        return [...withoutSaved, saved].sort((left, right) =>
          left.created_at.localeCompare(right.created_at) || left.id.localeCompare(right.id));
      });
    } catch (reason) {
      setSaveError(errorMessage(reason));
    } finally {
      setSaving(false);
    }
  }

  async function publishDraft() {
    if (!activeDraft) return;
    setPublishing(true);
    setPublishError(null);
    try {
      const result = await client.publishSceneDraft(
        activeDraft.id,
        activeDraft.revision,
        {
          id: packageId,
          version: packageVersion,
          description: description.trim() || `FactoryPark layout ${batchId}`,
        },
        operationKey(`scene-publish-${activeDraft.id}`),
      );
      setPublication(result);
      setActiveDraft(result.scene_draft);
      setSceneDrafts((current) => current.map((item) =>
        item.id === result.scene_draft.id ? result.scene_draft : item));
    } catch (reason) {
      setPublishError(errorMessage(reason));
    } finally {
      setPublishing(false);
    }
  }

  function downloadBatch() {
    try {
      const batch = draftBatch();
      const blob = new Blob([`${JSON.stringify(batch, null, 2)}\n`], {
        type: "application/json",
      });
      const url = URL.createObjectURL(blob);
      const anchor = document.createElement("a");
      anchor.href = url;
      anchor.download = `${batch.batch_id}.elements.json`;
      anchor.click();
      URL.revokeObjectURL(url);
    } catch (reason) {
      setValidationError(errorMessage(reason));
    }
  }

  const viewBox = bounds
    ? `${bounds.minX} ${-bounds.maxY} ${bounds.width} ${bounds.height}`
    : "0 0 1 1";

  return (
    <section aria-labelledby="scene-workbench-title">
      <SectionHeading
        id="scene-workbench-title"
        title="Scene Layout"
        description="在真实 FactoryPark_HF 几何上编辑静态场景元素，并通过同一世界编译器验证碰撞与放置约束。"
        action={
          <button
            className="button button--quiet"
            type="button"
            onClick={() => void Promise.all([loadCatalog(), loadSceneDrafts()])}
          >
            <RefreshCw aria-hidden="true" size={16} strokeWidth={1.7} />
            刷新布局
          </button>
        }
      />

      <div className="contract-note">
        编辑内容先保存为可修订 SceneDraft；发布时生成新的不可变 WorldPackage，绝不覆盖基础包，也不会在编辑请求中启动 Blender、Unreal 或 MuJoCo。
      </div>
      {loadError ? <ErrorPanel title="场景工具不可用" message={loadError} onRetry={() => void loadCatalog()} /> : null}
      {loading ? <SkeletonRows count={6} /> : null}

      {!loading && catalog && bounds ? (
        <div className="scene-workbench">
          <section className="scene-map-panel">
            <header className="pane-title">
              <div>
                <h2>{catalog.world_package}</h2>
                <span title={catalog.layout_digest}>{shortDigest(catalog.layout_digest, 14)}</span>
              </div>
              <StatusBadge value="authoritative layout" />
            </header>
            <div className="scene-canvas-wrap">
              <svg
                className="scene-canvas"
                viewBox={viewBox}
                role="img"
                aria-label="FactoryPark_HF 顶视场景布局"
              >
                <rect
                  className="scene-canvas__background"
                  x={bounds.minX}
                  y={-bounds.maxY}
                  width={bounds.width}
                  height={bounds.height}
                />
                {catalog.surfaces.map((surface) => {
                  const [x, y] = surface.position_xy_m;
                  const [width, height] = surface.size_xy_m;
                  return (
                    <g
                      className={`scene-surface scene-surface--${surface.semantic_class}`}
                      transform={`rotate(${-surface.yaw_deg} ${x} ${-y})`}
                      key={surface.surface_id}
                    >
                      <rect x={x - width / 2} y={-y - height / 2} width={width} height={height}>
                        <title>{surface.surface_id} · {surface.semantic_class}</title>
                      </rect>
                    </g>
                  );
                })}
                <circle
                  className="scene-spawn-clearance"
                  cx={catalog.spawn.position_xy_m[0]}
                  cy={-catalog.spawn.position_xy_m[1]}
                  r={catalog.spawn.clearance_radius_m}
                >
                  <title>Robot spawn clearance</title>
                </circle>
                {elements.map((element) => {
                  const template = catalog.element_types[element.element_type];
                  if (!template) return null;
                  const [x, y] = element.position_xy_m;
                  const selectedClass = selectedId === element.editorId ? " is-selected" : "";
                  return (
                    <g
                      className={`scene-element scene-element--${template.authority}${selectedClass}`}
                      transform={`rotate(${-element.yaw_deg} ${x} ${-y})`}
                      onClick={() => setSelectedId(element.editorId)}
                      key={element.editorId}
                    >
                      {template.shape === "cylinder" ? (
                        <circle cx={x} cy={-y} r={Math.max(template.radius_m ?? 0.2, 0.45)} />
                      ) : (
                        <rect
                          x={x - Math.max(template.size_m?.[0] ?? 0.5, 0.7) / 2}
                          y={-y - Math.max(template.size_m?.[1] ?? 0.5, 0.5) / 2}
                          width={Math.max(template.size_m?.[0] ?? 0.5, 0.7)}
                          height={Math.max(template.size_m?.[1] ?? 0.5, 0.5)}
                        />
                      )}
                      <title>{element.instance_key} · {element.element_type}</title>
                    </g>
                  );
                })}
              </svg>
              <div className="scene-legend" aria-label="场景图例">
                <span><i className="physics" /> PhysicsShared</span>
                <span><i className="visual" /> VisualOnly</span>
                <span><i className="spawn" /> Spawn clearance</span>
              </div>
            </div>
          </section>

          <aside className="scene-editor-panel">
            <div className="scene-draft-bar">
              <label className="field">
                <span>Scene draft</span>
                <select
                  value={activeDraft?.id ?? ""}
                  disabled={draftsLoading}
                  onChange={(event) => {
                    const record = sceneDrafts.find((item) => item.id === event.target.value);
                    if (record) openDraft(record);
                    else startNewDraft();
                  }}
                >
                  <option value="">Unsaved draft</option>
                  {sceneDrafts.map((record) => (
                    <option value={record.id} key={record.id}>
                      {record.payload.batch.batch_id} · r{record.revision}
                    </option>
                  ))}
                </select>
              </label>
              <button className="button button--quiet" type="button" onClick={startNewDraft}>
                <FilePlus2 aria-hidden="true" size={16} strokeWidth={1.7} />
                新建
              </button>
              <button
                className="button button--primary"
                type="button"
                disabled={saving || elements.length === 0}
                onClick={() => void saveDraft()}
              >
                {saving ? (
                  <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
                ) : (
                  <Save aria-hidden="true" size={16} strokeWidth={1.8} />
                )}
                {activeDraft ? `保存 r${activeDraft.revision + 1}` : "保存草稿"}
              </button>
            </div>
            {activeDraft ? (
              <div className="scene-draft-identity">
                <span>SceneDraft</span>
                <code title={activeDraft.id}>{shortDigest(activeDraft.id, 12)}</code>
                <StatusBadge value={`revision ${activeDraft.revision}`} />
              </div>
            ) : null}
            {saveError ? <ErrorPanel title="场景草稿操作失败" message={saveError} /> : null}
            {activeDraft ? (
              <section className="scene-publish-panel" aria-label="发布世界包">
                <div className="scene-publish-heading">
                  <div>
                    <strong>Publish WorldPackage</strong>
                    <span>生成新版本；不会覆盖 FactoryPark_HF 基础包。</span>
                  </div>
                  <StatusBadge value={activeDraft.status} />
                </div>
                <div className="scene-publish-fields">
                  <label className="field">
                    <span>Package ID</span>
                    <input
                      value={packageId}
                      pattern="[A-Za-z0-9][A-Za-z0-9_.-]*"
                      onChange={(event) => setPackageId(event.target.value)}
                    />
                  </label>
                  <label className="field field--compact">
                    <span>Version</span>
                    <input
                      value={packageVersion}
                      pattern="[A-Za-z0-9][A-Za-z0-9+_.-]*"
                      onChange={(event) => setPackageVersion(event.target.value)}
                    />
                  </label>
                  <button
                    className="button button--primary"
                    type="button"
                    disabled={
                      publishing
                      || validation?.digest !== activeDraft.payload.batch_digest
                      || packageId.length === 0
                      || packageVersion.length === 0
                    }
                    onClick={() => void publishDraft()}
                  >
                    {publishing ? (
                      <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
                    ) : (
                      <PackagePlus aria-hidden="true" size={16} strokeWidth={1.8} />
                    )}
                    发布不可变版本
                  </button>
                </div>
                {publication ? (
                  <div className="scene-publish-success">
                    <p>
                      已发布 <code>{publication.publication.package.ref}</code>
                      <span>{shortDigest(publication.publication.content_digest, 16)}</span>
                    </p>
                    {onUsePublishedWorldForNewSession ? (
                      <button
                        className="button button--quiet"
                        type="button"
                        onClick={() => void onUsePublishedWorldForNewSession(publication)}
                      >
                        用于新会话
                      </button>
                    ) : null}
                  </div>
                ) : null}
                {publishError ? <ErrorPanel title="世界包发布失败" message={publishError} /> : null}
              </section>
            ) : null}
            <div className="scene-batch-fields">
              <label className="field">
                <span>Batch ID</span>
                <input
                  value={batchId}
                  pattern="[a-z][a-z0-9_]{0,63}"
                  onChange={(event) => {
                    setBatchId(event.target.value);
                    invalidateValidation();
                  }}
                />
              </label>
              <label className="field">
                <span>Description</span>
                <input
                  value={description}
                  onChange={(event) => {
                    setDescription(event.target.value);
                    invalidateValidation();
                  }}
                  placeholder="安全设施布置"
                />
              </label>
            </div>
            <div className="scene-element-toolbar">
              <div>
                <strong>Elements</strong>
                <span>{elements.length}</span>
              </div>
              <button className="button button--quiet" type="button" onClick={addElement}>
                <Plus aria-hidden="true" size={16} strokeWidth={1.8} />
                添加元素
              </button>
            </div>

            {elements.length === 0 ? (
              <EmptyPanel title="尚无场景元素">添加元素后，在真实支撑面上编辑位置和朝向。</EmptyPanel>
            ) : (
              <div className="scene-element-list">
                {elements.map((element) => {
                  const template = catalog.element_types[element.element_type];
                  return (
                    <button
                      type="button"
                      className={selectedId === element.editorId ? "is-selected" : ""}
                      onClick={() => setSelectedId(element.editorId)}
                      key={element.editorId}
                    >
                      <Box aria-hidden="true" size={16} strokeWidth={1.6} />
                      <span>
                        <strong>{element.instance_key}</strong>
                        <small>{element.element_type} · {element.surface_id}</small>
                      </span>
                      <StatusBadge value={template?.authority ?? "unknown"} />
                    </button>
                  );
                })}
              </div>
            )}

            {selected ? (
              <section className="scene-element-editor">
                <header>
                  <strong>{selected.instance_key}</strong>
                  <button
                    className="icon-button icon-button--danger"
                    type="button"
                    aria-label={`删除 ${selected.instance_key}`}
                    onClick={() => removeElement(selected.editorId)}
                  >
                    <Trash2 aria-hidden="true" size={16} strokeWidth={1.7} />
                  </button>
                </header>
                <div className="form-grid form-grid--two">
                  <label className="field">
                    <span>Instance key</span>
                    <input
                      value={selected.instance_key}
                      pattern="[a-z][a-z0-9_]{0,63}"
                      onChange={(event) => updateElement(selected.editorId, { instance_key: event.target.value })}
                    />
                  </label>
                  <label className="field">
                    <span>Element type</span>
                    <select
                      value={selected.element_type}
                      onChange={(event) => changeElementType(selected, event.target.value)}
                    >
                      {Object.keys(catalog.element_types).sort().map((elementType) => (
                        <option value={elementType} key={elementType}>{elementType}</option>
                      ))}
                    </select>
                  </label>
                  <label className="field scene-field--wide">
                    <span>Support surface</span>
                    <select
                      value={selected.surface_id}
                      onChange={(event) => changeSurface(selected, event.target.value)}
                    >
                      {eligibleSceneSurfaces(catalog, selected.element_type).map((surface) => (
                        <option value={surface.surface_id} key={surface.surface_id}>
                          {surface.surface_id} · {surface.semantic_class}
                        </option>
                      ))}
                    </select>
                  </label>
                </div>
                <div className="scene-pose-fields">
                  {([0, 1] as const).map((axis) => (
                    <label className="field" key={axis}>
                      <span>{axis === 0 ? "X (m)" : "Y (m)"}</span>
                      <input
                        type="number"
                        step="0.05"
                        value={selected.position_xy_m[axis]}
                        onChange={(event) => {
                          const position: [number, number] = [...selected.position_xy_m];
                          position[axis] = event.target.valueAsNumber;
                          updateElement(selected.editorId, { position_xy_m: position });
                        }}
                      />
                    </label>
                  ))}
                  <label className="field">
                    <span>Yaw (°)</span>
                    <input
                      type="number"
                      step="1"
                      value={selected.yaw_deg}
                      onChange={(event) => updateElement(selected.editorId, { yaw_deg: event.target.valueAsNumber })}
                    />
                  </label>
                </div>
              </section>
            ) : null}

            {validationError ? <ErrorPanel title="元素批次校验失败" message={validationError} /> : null}
            {validation ? (
              <div className="scene-validation-result">
                <CheckCircle2 aria-hidden="true" size={19} strokeWidth={1.7} />
                <div>
                  <strong>几何与权限校验通过</strong>
                  <span>Digest {shortDigest(validation.digest, 18)} · {validation.elements.length} elements</span>
                </div>
              </div>
            ) : null}
            <div className="scene-editor-actions">
              <button
                className="button button--primary"
                type="button"
                disabled={validating || elements.length === 0}
                onClick={() => void validateBatch()}
              >
                {validating ? (
                  <LoaderCircle className="spin" aria-hidden="true" size={16} strokeWidth={1.8} />
                ) : (
                  <MapPinned aria-hidden="true" size={16} strokeWidth={1.8} />
                )}
                验证布局
              </button>
              <button
                className="button button--quiet"
                type="button"
                disabled={elements.length === 0}
                onClick={downloadBatch}
              >
                <Download aria-hidden="true" size={16} strokeWidth={1.8} />
                导出 JSON
              </button>
            </div>
          </aside>
        </div>
      ) : null}
    </section>
  );
}
