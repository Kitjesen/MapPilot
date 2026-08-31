export const API_PREFIX = "/api/sim/v1" as const;

export type FetchLike = (
  input: RequestInfo | URL,
  init?: RequestInit,
) => Promise<Response>;

export interface PackageIdentity {
  kind: string;
  id: string;
  version: string;
  ref: string;
}

export interface PackageSummary {
  package: PackageIdentity;
  manifest: { path: string };
  description: string | null;
  declared_capabilities: Record<string, unknown>;
}

export interface CatalogList {
  schema: "lingtu.sim.catalog-list.v1";
  packages: PackageSummary[];
}

export interface HealthStatus {
  service: "simstudio";
  api_version: "v1";
  status: string;
  field_isolated: boolean;
  runtime_bound: boolean;
}

export interface PackageDetail extends PackageSummary {
  schema: "lingtu.sim.catalog-package.v1";
  manifest_spec: JsonObject;
  dependencies: {
    packages: PackageSummary[];
    edges: Array<{
      source: PackageIdentity;
      field: string;
      target: PackageIdentity;
    }>;
  };
  qualification: {
    state: string;
    checks: Array<{ id: string; status: string; evidence?: unknown[] }>;
    diagnostics: Array<{ code: string; message: string }>;
    qualified_capabilities: Record<string, unknown>;
  };
}

export type JsonObject = Record<string, unknown>;
export type LaunchProfile = "headless" | "visual";
export type RunOperation = "prepare" | "start" | "pause" | "reset" | "stop";
export type RecordingOperation = "start" | "stop";
export type ImportKind = "robot" | "world";

export interface InboxSource {
  schema: "lingtu.sim.studio.inbox-source.v1";
  source_id: string;
  entry: string;
  original_name?: string;
  bytes: number;
  archive_format: "zip" | "tar" | "tar.gz";
}

export interface InboxSourceList {
  schema: "lingtu.sim.studio.inbox-source-list.v1";
  sources: InboxSource[];
}

export interface SourceFileRecord {
  path: string;
  size: number;
  sha256: string;
}

export interface SourceInspection {
  schema: "lingtu.sim.studio.source-inspection.v1";
  source: Omit<InboxSource, "schema" | "original_name">;
  summary: { files: number; total_bytes: number };
  candidates: {
    robot_models: Array<SourceFileRecord & { format: "mjcf" | "urdf" }>;
    licenses: SourceFileRecord[];
    heightmaps: SourceFileRecord[];
    meshes: SourceFileRecord[];
    textures: SourceFileRecord[];
  };
  recommendations: {
    robot: {
      source_format: "mjcf" | "urdf";
      source_model: string;
      license_file: string | null;
    } | null;
    world: {
      heightmap: string;
      mesh: string | null;
      license_file: string | null;
    } | null;
  };
  files: SourceFileRecord[];
}

export interface ImportContract {
  schema: "lingtu.sim.studio.import-contract.v1";
  kind: ImportKind;
  source_entry_owned_by: "simstudio";
  request_schema: {
    id: string;
    path: string;
    sha256: string;
  };
  request_template: JsonObject;
}

export interface StudioRecord<TPayload extends JsonObject = JsonObject> {
  schema: string;
  kind: string;
  id: string;
  revision: number;
  created_at: string;
  updated_at: string;
  status: string;
  payload: TPayload;
}

export interface RunReadiness {
  run_id: string;
  status: string;
  revision: number;
  ready: boolean;
  readiness: Record<string, string>;
  sensors: Record<string, string>;
}

export interface ArtifactRecord {
  artifact_id: string;
  run_id: string;
  path: string;
  kind: "file" | "directory";
  size: number;
  sha256: string | null;
}

export interface ArtifactPreview {
  artifact_id: string;
  run_id: string;
  path: string;
  size: number;
  sha256: string | null;
  mime_type: string;
  previewable: boolean;
  encoding: string | null;
  content: string | null;
  truncated: boolean;
  format?: "json" | "text" | string;
  reason?: string;
}

export interface RecordingStatus {
  schema: "lingtu.sim.studio.recording.v1";
  recording_id: string | null;
  run_id: string;
  state: "MISSING" | "INVALID" | "VALID";
  source_run_id: string | null;
  session_id: string | null;
  model_generation: number | null;
  reset_generation: { start: number; end: number } | null;
  frame_count: number;
  duration_ns: number;
  terminal_state: string | null;
  event_order: string[];
  replay: {
    deterministic: boolean;
    visual: boolean;
    clock_authority: "recorded_mujoco" | null;
  };
  artifacts: {
    manifest: "simulation-recording.json";
    timeline: "simulation-timeline.jsonl";
  } | null;
  diagnostics: Array<{ code: string; message: string }>;
}

export interface RecordingList {
  schema: "lingtu.sim.studio.recording-list.v1";
  recordings: RecordingStatus[];
}

export interface RecordingFrameSummary {
  frame_index: number;
  relative_time_ns: number;
  sim_time_ns: number;
  sequence: number;
  physics_step: number;
  model_generation: number;
  reset_generation: number;
  body_count: number;
  joint_count: number;
  actuator_count: number;
  sensor_count: number;
  has_command: boolean;
  scenario_event_count: number;
  sensor_metadata_count: number;
  sensor_payload_count: number;
  sensor_payload_bytes: number;
  lifecycle_evidence_count: number;
}

export interface RecordingTimeline {
  schema: "lingtu.sim.studio.recording-timeline.v1";
  recording_id: string;
  run_id: string;
  session_id: string;
  frame_count: number;
  duration_ns: number;
  page: {
    offset: number;
    limit: number;
    returned: number;
    next_offset: number | null;
  };
  frames: RecordingFrameSummary[];
}

export interface RecordingBodySnapshot extends JsonObject {
  stable_id: string;
  instance_id?: string;
  frame_id?: string;
  position_m?: number[];
  quaternion_wxyz?: number[];
}

export interface RecordingSnapshot extends JsonObject {
  sequence: number;
  physics_step: number;
  sim_time_ns: number;
  model_generation: number;
  reset_generation: number;
  bodies: RecordingBodySnapshot[];
  joints?: JsonObject[];
  actuators?: JsonObject[];
  sensors?: JsonObject[];
}

export interface RecordingSensorPayloadReference extends JsonObject {
  schema: "lingtu.sim.sensor-payload-ref.v1";
  payload_index: number;
  session_id: string;
  model_generation: number;
  reset_generation: number;
  sensor_id: string;
  stream_kind: string;
  encoding: string;
  media_type: string;
  sample_sequence: number;
  sample_time_ns: number;
  sha256: string;
  bytes: number;
  metadata: JsonObject;
}

export interface RecordingFrame {
  schema: "lingtu.sim.studio.recording-frame.v1";
  recording_id: string;
  run_id: string;
  session_id: string;
  frame_index: number;
  relative_time_ns: number;
  snapshot: RecordingSnapshot;
  command: JsonObject | null;
  metadata: JsonObject;
  scenario_events: JsonObject[];
  sensor_metadata: JsonObject[];
  sensor_payloads: RecordingSensorPayloadReference[];
  lifecycle_evidence: JsonObject[];
}

export interface SceneSurface {
  surface_id: string;
  semantic_class: string;
  position_xy_m: [number, number];
  size_xy_m: [number, number];
  yaw_deg: number;
}

export interface SceneElementTemplate {
  semantic_class: string;
  shape: "box" | "cylinder";
  material: string;
  authority: "PhysicsShared" | "VisualOnly";
  allowed_surface_classes: string[];
  size_m?: [number, number, number];
  radius_m?: number;
  half_height_m?: number;
}

export interface SceneToolCatalog {
  schema: "lingtu.sim.studio.scene-tool-catalog.v1";
  scene_tool: "factory-park-hf";
  world_package: string;
  element_batch_schema: "lingtu.sim.factory-park-element-batch.v1";
  read_only: true;
  executes_runtime: false;
  layout_digest: string;
  element_types: Record<string, SceneElementTemplate>;
  surfaces: SceneSurface[];
  spawn: {
    position_xy_m: [number, number];
    clearance_radius_m: number;
  };
}

export interface SceneElementPlacement extends JsonObject {
  instance_key: string;
  element_type: string;
  surface_id: string;
  position_xy_m: [number, number];
  yaw_deg: number;
}

export interface SceneElementBatch extends JsonObject {
  schema: "lingtu.sim.factory-park-element-batch.v1";
  batch_id: string;
  description: string;
  elements: SceneElementPlacement[];
}

export interface SceneToolValidation {
  schema: "lingtu.sim.studio.scene-tool-validation.v1";
  scene_tool: "factory-park-hf";
  world_package: string;
  valid: true;
  batch_id: string;
  digest: string;
  layout_digest: string;
  stable_ids: string[];
  elements: Array<{
    stable_id: string;
    element_type: string;
    authority: "PhysicsShared" | "VisualOnly";
  }>;
  diagnostics: Array<{ code: string; message: string }>;
}

export interface SceneDraftPayload extends JsonObject {
  schema: "lingtu.sim.studio.scene-draft-payload.v1";
  scene_tool: "factory-park-hf";
  world_package: string;
  layout_digest: string;
  batch_digest: string;
  batch: SceneElementBatch;
  validation: SceneToolValidation;
  publications?: WorldPublication[];
}

export type SceneDraftRecord = StudioRecord<SceneDraftPayload>;

export interface PublishedWorldPackage extends JsonObject {
  schema: "lingtu.sim.studio.world-publication.v1";
  package: PackageIdentity;
  package_root: string;
  qualification_path: string;
  content_digest: string;
}

export interface WorldPublication extends PublishedWorldPackage {
  source?: JsonObject;
}

export interface ScenePublication extends JsonObject {
  schema: "lingtu.sim.studio.scene-publication.v1";
  scene_draft: SceneDraftRecord;
  publication: PublishedWorldPackage;
  source: {
    scene_draft_id: string;
    scene_draft_revision: number;
    batch_digest: string;
    base_layout_digest: string;
  };
}

export interface SimStudioCapabilities {
  schema: "lingtu.sim.studio.capabilities.v1";
  api_version: "v1";
  api_prefix: typeof API_PREFIX;
  field_isolated: boolean;
  read_models: {
    source_inbox?: { list: string; upload: string; inspect: string };
    imports?: { list: string };
    import_contracts?: { get: string };
    session_drafts?: { list: string };
    bundles?: { list: string };
    run_artifacts?: { list: string; preview: string };
    recordings?: {
      list: string;
      inspect: string;
      timeline: string;
      frame: string;
      start: string;
      stop: string;
    };
    scene_drafts?: { list: string; get: string; create: string; update: string; publish?: string };
  };
  artifact_preview: {
    addressing: "opaque_artifact_id" | string;
    raw_path_input: boolean;
    max_bytes: number;
  };
  schema_document: {
    href: string;
    format: string;
  };
}

interface SuccessEnvelope<T> {
  ok: true;
  result: T;
}

interface ErrorEnvelope {
  ok: false;
  error: {
    code: string;
    message: string;
    details?: unknown;
  };
}

export class SimStudioApiError extends Error {
  readonly code: string;
  readonly status: number;
  readonly details?: unknown;

  constructor(
    message: string,
    options: { code?: string; status?: number; details?: unknown } = {},
  ) {
    super(message);
    this.name = "SimStudioApiError";
    this.code = options.code ?? "SIMSTUDIO_CLIENT_ERROR";
    this.status = options.status ?? 0;
    this.details = options.details;
  }
}

export function simApiPath(endpoint: string): string {
  if (
    typeof endpoint !== "string" ||
    !endpoint.startsWith("/") ||
    endpoint.startsWith("//") ||
    endpoint.includes("://") ||
    endpoint.startsWith("/api/")
  ) {
    throw new SimStudioApiError("SimStudio requests must use a relative v1 endpoint");
  }

  const candidate = new URL(`${API_PREFIX}${endpoint}`, "http://simstudio.local");
  if (
    candidate.origin !== "http://simstudio.local" ||
    !candidate.pathname.startsWith(`${API_PREFIX}/`)
  ) {
    throw new SimStudioApiError("SimStudio request escaped the v1 interface");
  }
  return `${candidate.pathname}${candidate.search}`;
}

export class SimStudioClient {
  private readonly fetcher: FetchLike;

  constructor(fetcher: FetchLike = globalThis.fetch.bind(globalThis)) {
    this.fetcher = fetcher;
  }

  listPackages(): Promise<CatalogList> {
    return this.request<CatalogList>("/packages");
  }

  health(): Promise<HealthStatus> {
    return this.request<HealthStatus>("/health");
  }

  capabilities(): Promise<SimStudioCapabilities> {
    return this.request<SimStudioCapabilities>("/capabilities");
  }

  inspectPackage(kind: string, reference: string): Promise<PackageDetail> {
    return this.request<PackageDetail>(
      `/packages/${this.segment(kind)}/${this.segment(reference)}`,
    );
  }

  listSources(): Promise<InboxSourceList> {
    return this.request<InboxSourceList>("/inbox/sources");
  }

  uploadSource(file: File): Promise<InboxSource> {
    if (!(file instanceof Blob) || typeof file.name !== "string" || file.name.length === 0) {
      throw new SimStudioApiError("请选择一个本地归档文件");
    }
    const headers = new Headers({
      "Content-Type": "application/octet-stream",
      "X-SimStudio-Filename": file.name,
    });
    return this.request<InboxSource>("/inbox/uploads", {
      method: "POST",
      headers,
      body: file,
    });
  }

  inspectSource(sourceId: string): Promise<SourceInspection> {
    return this.request<SourceInspection>(
      `/inbox/sources/${this.segment(sourceId)}/inspection`,
    );
  }

  getImportContract(kind: ImportKind): Promise<ImportContract> {
    return this.request<ImportContract>(
      `/import-contracts/${this.segment(kind)}`,
    );
  }

  listImports(): Promise<StudioRecord[]> {
    return this.request<StudioRecord[]>("/imports");
  }

  createImport(
    kind: ImportKind,
    sourceEntry: string,
    request: JsonObject = {},
    idempotencyKey?: string,
  ): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      "/imports",
      { kind, source: { entry: sourceEntry }, request },
      idempotencyKey,
    );
  }

  getImport(importId: string): Promise<StudioRecord> {
    return this.request<StudioRecord>(`/imports/${this.segment(importId)}`);
  }

  promoteImport(importId: string, idempotencyKey?: string): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      `/imports/${this.segment(importId)}/promote`,
      undefined,
      idempotencyKey,
    );
  }

  createDraft(intent: JsonObject, idempotencyKey?: string): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      "/session-drafts",
      { intent },
      idempotencyKey,
    );
  }

  listDrafts(): Promise<StudioRecord[]> {
    return this.request<StudioRecord[]>("/session-drafts");
  }

  composeDraft(
    draftId: string,
    revision: number,
    idempotencyKey?: string,
  ): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      `/session-drafts/${this.segment(draftId)}/compose`,
      { revision },
      idempotencyKey,
    );
  }

  listBundles(): Promise<StudioRecord[]> {
    return this.request<StudioRecord[]>("/bundles");
  }

  createRun(
    bundleId: string,
    launchProfile: LaunchProfile,
    idempotencyKey?: string,
  ): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      "/runs",
      { bundle_id: bundleId, launch_profile: launchProfile },
      idempotencyKey,
    );
  }

  listRuns(): Promise<StudioRecord[]> {
    return this.request<StudioRecord[]>("/runs");
  }

  getRun(runId: string): Promise<StudioRecord> {
    return this.request<StudioRecord>(`/runs/${this.segment(runId)}`);
  }

  operateRun(
    runId: string,
    operation: RunOperation,
    revision: number,
    idempotencyKey?: string,
  ): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      `/runs/${this.segment(runId)}/${operation}`,
      { revision },
      idempotencyKey,
    );
  }

  getRunReadiness(runId: string): Promise<RunReadiness> {
    return this.request<RunReadiness>(
      `/runs/${this.segment(runId)}/readiness`,
    );
  }

  listArtifacts(runId: string): Promise<ArtifactRecord[]> {
    return this.request<ArtifactRecord[]>(
      `/runs/${this.segment(runId)}/artifacts`,
    );
  }

  inspectRecording(runId: string): Promise<RecordingStatus> {
    return this.request<RecordingStatus>(
      `/runs/${this.segment(runId)}/recording`,
    );
  }

  listRecordings(): Promise<RecordingList> {
    return this.request<RecordingList>("/recordings");
  }

  getRecordingTimeline(
    runId: string,
    offset = 0,
    limit = 100,
  ): Promise<RecordingTimeline> {
    const safeOffset = this.boundedInteger(offset, "Timeline offset", 0);
    const safeLimit = this.boundedInteger(limit, "Timeline limit", 1, 200);
    return this.request<RecordingTimeline>(
      `/runs/${this.segment(runId)}/recording/timeline?offset=${safeOffset}&limit=${safeLimit}`,
    );
  }

  getRecordingFrame(runId: string, frameIndex: number): Promise<RecordingFrame> {
    const safeFrameIndex = this.boundedInteger(frameIndex, "Frame index", 0);
    return this.request<RecordingFrame>(
      `/runs/${this.segment(runId)}/recording/frames/${safeFrameIndex}`,
    );
  }

  operateRecording(
    runId: string,
    operation: RecordingOperation,
    revision: number,
    idempotencyKey?: string,
  ): Promise<StudioRecord> {
    return this.post<StudioRecord>(
      `/runs/${this.segment(runId)}/recording/${operation}`,
      { revision },
      idempotencyKey,
    );
  }

  getFactoryParkSceneCatalog(): Promise<SceneToolCatalog> {
    return this.request<SceneToolCatalog>(
      "/scene-tools/factory-park-hf/catalog",
    );
  }

  validateFactoryParkElementBatch(
    batch: SceneElementBatch,
  ): Promise<SceneToolValidation> {
    return this.post<SceneToolValidation>(
      "/scene-tools/factory-park-hf/element-batches/validate",
      batch,
    );
  }

  createSceneDraft(
    batch: SceneElementBatch,
    idempotencyKey?: string,
  ): Promise<SceneDraftRecord> {
    return this.post<SceneDraftRecord>(
      "/scene-drafts",
      { scene_tool: "factory-park-hf", batch },
      idempotencyKey,
    );
  }

  listSceneDrafts(): Promise<SceneDraftRecord[]> {
    return this.request<SceneDraftRecord[]>("/scene-drafts");
  }

  getSceneDraft(sceneDraftId: string): Promise<SceneDraftRecord> {
    return this.request<SceneDraftRecord>(
      `/scene-drafts/${this.segment(sceneDraftId)}`,
    );
  }

  updateSceneDraft(
    sceneDraftId: string,
    revision: number,
    batch: SceneElementBatch,
    idempotencyKey?: string,
  ): Promise<SceneDraftRecord> {
    const headers = new Headers();
    if (idempotencyKey !== undefined) {
      headers.set("Idempotency-Key", idempotencyKey);
    }
    return this.request<SceneDraftRecord>(
      `/scene-drafts/${this.segment(sceneDraftId)}`,
      {
        method: "PUT",
        headers,
        body: JSON.stringify({ revision, batch }),
      },
    );
  }

  publishSceneDraft(
    sceneDraftId: string,
    revision: number,
    packageIdentity: { id: string; version: string; description: string },
    idempotencyKey?: string,
  ): Promise<ScenePublication> {
    return this.post<ScenePublication>(
      `/scene-drafts/${this.segment(sceneDraftId)}/publish`,
      { revision, package: packageIdentity },
      idempotencyKey,
    );
  }

  getArtifact(artifactId: string): Promise<ArtifactRecord> {
    return this.request<ArtifactRecord>(
      `/artifacts/${this.segment(artifactId)}`,
    );
  }

  previewArtifact(
    runId: string,
    artifactId: string,
    maxBytes?: number,
  ): Promise<ArtifactPreview> {
    const query = maxBytes === undefined
      ? ""
      : `?max_bytes=${encodeURIComponent(String(maxBytes))}`;
    return this.request<ArtifactPreview>(
      `/runs/${this.segment(runId)}/artifacts/${this.segment(artifactId)}/preview${query}`,
    );
  }

  private post<T>(
    endpoint: string,
    body?: JsonObject,
    idempotencyKey?: string,
  ): Promise<T> {
    const headers = new Headers();
    if (idempotencyKey !== undefined) {
      headers.set("Idempotency-Key", idempotencyKey);
    }
    const init: RequestInit = {
      method: "POST",
      headers,
    };
    if (body !== undefined) {
      init.body = JSON.stringify(body);
    }
    return this.request<T>(endpoint, init);
  }

  private segment(value: string): string {
    if (typeof value !== "string" || value.length === 0) {
      throw new SimStudioApiError("SimStudio resource identifiers must be non-empty");
    }
    return encodeURIComponent(value);
  }

  private boundedInteger(
    value: number,
    label: string,
    minimum: number,
    maximum = Number.MAX_SAFE_INTEGER,
  ): number {
    if (!Number.isSafeInteger(value) || value < minimum || value > maximum) {
      throw new SimStudioApiError(
        `${label} must be an integer between ${minimum} and ${maximum}`,
      );
    }
    return value;
  }

  private async request<T>(endpoint: string, init?: RequestInit): Promise<T> {
    const headers = new Headers(init?.headers);
    headers.set("accept", "application/json");
    if (init?.body !== undefined && !headers.has("content-type")) {
      headers.set("content-type", "application/json");
    }
    let response: Response;
    try {
      response = await this.fetcher(simApiPath(endpoint), {
        ...init,
        headers,
      });
    } catch (error) {
      throw new SimStudioApiError("无法连接本地 SimStudio 服务", {
        code: "SIMSTUDIO_NETWORK_ERROR",
        details: error,
      });
    }

    let envelope: SuccessEnvelope<T> | ErrorEnvelope;
    try {
      envelope = (await response.json()) as SuccessEnvelope<T> | ErrorEnvelope;
    } catch (error) {
      throw new SimStudioApiError("SimStudio returned an unreadable response", {
        code: "SIMSTUDIO_INVALID_RESPONSE",
        status: response.status,
        details: error,
      });
    }

    if (!response.ok || envelope.ok !== true) {
      const failure = envelope.ok === false ? envelope.error : undefined;
      throw new SimStudioApiError(
        failure?.message ?? `SimStudio request failed with HTTP ${response.status}`,
        {
          code: failure?.code ?? "SIMSTUDIO_HTTP_ERROR",
          status: response.status,
          details: failure?.details,
        },
      );
    }
    return envelope.result;
  }
}
