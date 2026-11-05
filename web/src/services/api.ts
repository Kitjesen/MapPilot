import { dashboardFetch } from './observationMode.ts'
import type { RecordingSeriesQuery, RecordingSeriesResponse } from '../types/recordingSeries.ts'
import type { RecordingCameraFrame, RecordingSceneInfoResponse, RecordingSceneQuery, RecordingSceneResponse } from '../types/recordingScene.ts'
import type { StairAnnotation, StairListResponse } from '../types/stairs.ts'
import type { MapEditOperation, MapEditSource } from '../types/mapEdit.ts'
import { MapEditSaveUnconfirmed, mapEditWireOperations } from './mapEdit.ts'
import { fetchMapPreview } from './mapPreview.ts'
import { parsePcd } from './pcdPreview.ts'
// Centralized API service layer for LingTu web dashboard
// All fetch() calls in one place.

import type {
  CameraMediaStatus,
  AppBootstrapResponse,
  AppCapabilitiesResponse,
  AppTrafficResponse,
  AgentTaskCancelResponse,
  AgentTaskStatusResponse,
  RecordingOperationResponse,
  RecordingStartConfig,
  RecordingDetailResponse,
  RecordingListResponse,
  RecordingStatusResponse,
  RecordingCatalogResponse,
  RecordingStreamsResponse,
  RecordingSummaryResponse,
  RecordingValidationResponse,
  ClientLinks,
  CommandReceipt,
  ControlCommandResponse,
  DirectedExplorationResponse,
  DirectedExplorationTargetRequest,
  DynamicFilterResult,
  ExplorationStatusResponse,
  FollowTaskCommandResponse,
  FollowTaskRef,
  FollowTaskStatusResponse,
  GatewayErrorResponse,
  GoalCandidateRequest,
  GoalCandidateResponse,
  GoalSource,
  GoalTargetType,
  HealthResponse,
  InspectionCommandResponse,
  InspectionEvidenceDetailResponse,
  InspectionEvidenceListResponse,
  InspectionRoute,
  InspectionRouteListResponse,
  InspectionRouteRequest,
  InspectionRouteResponse,
  InspectionStatusResponse,
  InspectionTaskCommandResponse,
  InspectionTaskListResponse,
  InspectionTaskReportResponse,
  InspectionTaskStatusResponse,
  LeaseAction,
  LeaseResponse,
  LocalizationOperationResponse,
  LocationOperationResponse,
  LocationUpsertRequest,
  LocationsResponse,
  MapInfo,
  MapLifecycleResponse,
  MapListResponse,
  MapPointsResponse,
  NavigationStatusResponse,
  NavigationDdsSnapshotResponse,
  NavigationTaskStatusQueryResponse,
  PathResponse,
  PlanPreviewRequest,
  PlanPreviewResponse,
  ProductFieldCheckRequest,
  ProductFieldCheckResponse,
  ReadinessResponse,
  RealRuntimeEvidenceLatestResponse,
  RoutecheckLatestResponse,
  RuntimeDataflowResponse,
  RuntimeDataflowSubscribeRequest,
  RuntimeDataflowSubscribeResponse,
  RuntimeDataflowTopicDetailResponse,
  SceneGraphResponse,
  SessionEvent,
  SlamStatusResponse,
  StateResponse,
  VisualServoMode,
  VisualServoRequest,
} from '../types'
import { currentNavigationTaskStore } from './currentNavigationTask.ts'

const WEB_CLIENT_ID = 'web-dashboard'

export function fetchMapStairs(map: string): Promise<StairListResponse> {
  return fetchJson(`/api/v1/maps/${encodeURIComponent(map)}/stairs`)
}

export async function saveMapStair(map: string, stair: StairAnnotation): Promise<StairListResponse> {
  const { id, name, content_epoch, frame_id, bounds, lower, upper } = stair
  const body = { name, content_epoch, frame_id, bounds, lower, upper }
  return readJsonResponse(await dashboardFetch(`/api/v1/maps/${encodeURIComponent(map)}/stairs/${encodeURIComponent(id)}`, {
    method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  }))
}

export async function deleteMapStair(map: string, id: string, epoch: number): Promise<StairListResponse> {
  return readJsonResponse(await dashboardFetch(
    `/api/v1/maps/${encodeURIComponent(map)}/stairs/${encodeURIComponent(id)}?content_epoch=${epoch}`, { method: 'DELETE' },
  ))
}

type CommandResponse = ControlCommandResponse | LeaseResponse

export interface SavedMapPointCloud {
  points: number[]
  frameId: string
  epoch: number | null
  mapName: string
  contentEpoch: number | null
  timestamp: number
}

let clientLinks: ClientLinks = {}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function detailStrings(detail: unknown, key: string): string[] {
  if (!isRecord(detail)) return []
  const value = detail[key]
  if (!Array.isArray(value)) return []
  return value.filter((item): item is string => typeof item === 'string' && item.length > 0)
}

function commandSuffix(command?: CommandReceipt): string {
  const parts: string[] = []
  if (command?.replay) parts.push('重复请求已确认')
  return parts.length ? `（${parts.join(' ')}）` : ''
}

export class GatewayApiError extends Error {
  readonly statusCode: number
  readonly body?: GatewayErrorResponse
  readonly command?: CommandReceipt
  readonly errorCode?: string
  readonly detail?: unknown

  constructor(statusCode: number, body?: GatewayErrorResponse) {
    super(body?.message || body?.error || `HTTP ${statusCode}`)
    this.name = 'GatewayApiError'
    this.statusCode = statusCode
    this.body = body
    this.command = body?.command
    this.errorCode = body?.error
    this.detail = body?.detail
  }
}

export function isGatewayApiError(error: unknown): error is GatewayApiError {
  return error instanceof GatewayApiError
}

export function formatCommandAck(response: CommandResponse, label = '命令'): string {
  if (!response.ok) {
    return `${label}未接受${commandSuffix(response.command)}`
  }
  return `${label}已提交${commandSuffix(response.command)}`
}

export function formatCommandError(error: unknown, label = '命令失败'): string {
  if (isGatewayApiError(error)) {
    if (!error.body && [502, 503, 504].includes(error.statusCode)) {
      return `${label}: 未收到机器人服务的确认（HTTP ${error.statusCode}）。请检查服务连接，并核对执行结果后再操作。`
    }
    const message = error.body?.message || error.body?.error || error.message || `HTTP ${error.statusCode}`
    const blockers = detailStrings(error.detail, 'blockers')
    const advisories = detailStrings(error.detail, 'advisories')
    const reasonParts = [...blockers, ...advisories].slice(0, 3)
    const reason = reasonParts.length ? `；原因：${reasonParts.join('，')}` : ''
    const rejected = error.command?.accepted === false ? '（未接受）' : ''
    return `${label}${rejected}: ${message}${reason}${commandSuffix(error.command)}`
  }
  if (error instanceof TypeError && /failed to fetch|networkerror|network request failed|load failed/i.test(error.message)) {
    return `${label}: 网络请求未完成，请检查机器人服务连接，并核对执行结果后再操作。`
  }
  if (error instanceof Error) {
    return `${label}: ${error.message}`
  }
  return `${label}: ${String(error)}`
}

async function readJsonResponse<T>(res: Response): Promise<T> {
  const text = await res.text()
  let data: unknown
  try {
    data = text ? JSON.parse(text) : undefined
  } catch {
    data = undefined
  }
  if (!res.ok) {
    const error = isRecord(data) ? data as unknown as GatewayErrorResponse : undefined
    throw new GatewayApiError(res.status, error)
  }
  return data as T
}

async function fetchJson<T>(url: string): Promise<T> {
  return readJsonResponse<T>(await dashboardFetch(url))
}

function setClientLinks(links?: ClientLinks | null): void {
  if (!links) return
  clientLinks = { ...clientLinks, ...links }
}

function apiPath(linkName: keyof ClientLinks, fallback: string): string {
  const link = clientLinks[linkName]
  if (!link) return fallback
  try {
    const url = new URL(link, window.location.origin)
    if (url.protocol === 'http:' || url.protocol === 'https:') {
      return `${url.pathname}${url.search}`
    }
  } catch {
    return link
  }
  return link
}

function mapPointsPath(name: string): string {
  const encoded = encodeURIComponent(name)
  const template =
    clientLinks.saved_map_points ??
    clientLinks.map_points ??
    '/api/v1/maps/{name}/points?max_points=30000'
  const withName = template.includes('{name}')
    ? template.replace('{name}', encoded)
    : `/api/v1/maps/${encoded}/points?max_points=30000`
  const url = new URL(withName, window.location.origin)
  // Only scene identity comes from JSON; geometry is loaded from the full PCD.
  url.searchParams.set('max_points', '1')
  return `${url.pathname}${url.search}`
}

function locationDetailPath(name: string): string {
  const encoded = encodeURIComponent(name)
  const template = apiPath('location_detail', '/api/v1/locations/{name}')
  return template.includes('{name}')
    ? template.replace('{name}', encoded)
    : `/api/v1/locations/${encoded}`
}

function inspectionRouteDetailPath(routeId: string): string {
  const encoded = encodeURIComponent(routeId)
  const template = apiPath('inspection_route_detail', '/api/v1/inspection/routes/{route_id}')
  return template.includes('{route_id}')
    ? template.replace('{route_id}', encoded)
    : `/api/v1/inspection/routes/${encoded}`
}

function inspectionTaskPath(taskId: string): string {
  const encoded = encodeURIComponent(taskId)
  const template = apiPath('inspection_task_status', '/api/v1/inspection/tasks/{task_id}')
  return template.includes('{task_id}')
    ? template.replace('{task_id}', encoded)
    : `/api/v1/inspection/tasks/${encoded}`
}

function inspectionTaskReportPath(taskId: string): string {
  const encoded = encodeURIComponent(taskId)
  const template = apiPath(
    'inspection_task_report',
    '/api/v1/inspection/tasks/{task_id}/report',
  )
  return template.includes('{task_id}')
    ? template.replace('{task_id}', encoded)
    : '/api/v1/inspection/tasks/' + encoded + '/report'
}

function inspectionTaskActionPath(
  taskId: string,
  linkName: 'inspection_task_pause' | 'inspection_task_resume' | 'inspection_task_cancel',
  fallback: string,
): string {
  const encoded = encodeURIComponent(taskId)
  const template = apiPath(linkName, fallback)
  return template.includes('{task_id}')
    ? template.replace('{task_id}', encoded)
    : fallback.replace('{task_id}', encoded)
}

function inspectionEvidenceDetailPath(evidenceId: string): string {
  return `/api/v1/inspection/evidence/${encodeURIComponent(evidenceId)}`
}

export function inspectionEvidenceArtifactUrl(
  evidenceId: string,
  kind: 'rgb' | 'pose' | 'detections',
): string {
  return `${inspectionEvidenceDetailPath(evidenceId)}/artifacts/${encodeURIComponent(kind)}`
}

function mapNamedPath(linkName: keyof ClientLinks, fallback: string, name: string): string {
  const encoded = encodeURIComponent(name)
  const template = apiPath(linkName, fallback).replace(/%7Bname%7D/gi, '{name}')
  return template.includes('{name}')
    ? template.replace('{name}', encoded)
    : fallback.replace('{name}', encoded)
}

export function makeRequestId(prefix: string): string {
  if (globalThis.crypto?.randomUUID) {
    return `${prefix}-${globalThis.crypto.randomUUID()}`
  }
  return `${prefix}-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`
}

function commandBody<T extends Record<string, unknown>>(
  prefix: string,
  body: T,
): T & { client_id: string; request_id: string } {
  return {
    ...body,
    client_id: WEB_CLIENT_ID,
    request_id: makeRequestId(prefix),
  }
}

async function postJson<T>(url: string, body?: unknown): Promise<T> {
  const res = await dashboardFetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  })
  return readJsonResponse<T>(res)
}

async function readMapLifecycle(res: Response): Promise<MapLifecycleResponse> {
  const text = await res.text()
  let data: Record<string, unknown> = {}
  try {
    data = text ? JSON.parse(text) as Record<string, unknown> : {}
  } catch {
    data = {}
  }
  if (!res.ok || data.ok === false || data.success === false) {
    const message = typeof data.message === 'string'
      ? data.message
      : typeof data.error === 'string'
        ? data.error
        : `HTTP ${res.status}`
    throw new Error(message)
  }
  return data as MapLifecycleResponse
}

// --- App bootstrap / read APIs ---

export async function fetchAppBootstrap(): Promise<AppBootstrapResponse> {
  const data = await fetchJson<AppBootstrapResponse>(apiPath('bootstrap', '/api/v1/app/bootstrap'))
  setClientLinks(data.links)
  return data
}

export async function fetchCameraStatus(): Promise<CameraMediaStatus> {
  return fetchJson<CameraMediaStatus>('/api/v1/camera/status')
}

export async function fetchAppCapabilities(): Promise<AppCapabilitiesResponse> {
  const data = await fetchJson<AppCapabilitiesResponse>(apiPath('capabilities', '/api/v1/app/capabilities'))
  setClientLinks(data.links)
  return data
}

export async function fetchAppTraffic(url = '/api/v1/app/traffic'): Promise<AppTrafficResponse> {
  return fetchJson<AppTrafficResponse>(url)
}

export async function fetchState(): Promise<StateResponse> {
  return fetchJson<StateResponse>(apiPath('state', '/api/v1/state'))
}

export async function fetchHealth(): Promise<HealthResponse> {
  return fetchJson<HealthResponse>(apiPath('health', '/api/v1/health'))
}

export async function fetchReadiness(): Promise<ReadinessResponse> {
  return fetchJson<ReadinessResponse>(apiPath('readiness', '/api/v1/readiness'))
}

export async function fetchRoutecheckLatest(): Promise<RoutecheckLatestResponse> {
  return fetchJson<RoutecheckLatestResponse>(
    apiPath('routecheck_latest', '/api/v1/diagnostics/routecheck/latest'),
  )
}

export async function fetchRealRuntimeEvidenceLatest(): Promise<RealRuntimeEvidenceLatestResponse> {
  return fetchJson<RealRuntimeEvidenceLatestResponse>(
    apiPath('real_runtime_evidence_latest', '/api/v1/diagnostics/real-runtime-evidence/latest'),
  )
}

export async function fetchRuntimeDataflow(): Promise<RuntimeDataflowResponse> {
  return fetchJson<RuntimeDataflowResponse>(
    apiPath('runtime_dataflow', '/api/v1/runtime/dataflow'),
  )
}

export async function fetchRuntimeDataflowTopic(
  topic: string,
): Promise<RuntimeDataflowTopicDetailResponse> {
  const path = apiPath('runtime_dataflow_topic', '/api/v1/runtime/dataflow/topic')
  const sep = path.includes('?') ? '&' : '?'
  return fetchJson<RuntimeDataflowTopicDetailResponse>(
    `${path}${sep}${new URLSearchParams({ topic }).toString()}`,
  )
}

export async function subscribeRuntimeDataflow(
  request: RuntimeDataflowSubscribeRequest,
): Promise<RuntimeDataflowSubscribeResponse> {
  return postJson<RuntimeDataflowSubscribeResponse>(
    apiPath('runtime_dataflow_subscribe', '/api/v1/runtime/dataflow/subscribe'),
    {
      transport: 'gateway_sse',
      ...request,
    },
  )
}

export async function runProductFieldCheck(
  request: ProductFieldCheckRequest = {},
): Promise<ProductFieldCheckResponse> {
  return postJson<ProductFieldCheckResponse>(
    apiPath('field_check', '/api/v1/diagnostics/field-check'),
    {
      mode: 'simulation',
      ...request,
    },
  )
}

export async function fetchSceneGraph(): Promise<SceneGraphResponse> {
  return fetchJson<SceneGraphResponse>(apiPath('scene_graph', '/api/v1/scene_graph'))
}

export async function fetchPath(): Promise<PathResponse> {
  return fetchJson<PathResponse>(apiPath('path', '/api/v1/path'))
}

export async function fetchNavigationStatus(): Promise<NavigationStatusResponse> {
  return fetchJson<NavigationStatusResponse>(apiPath('navigation_status', '/api/v1/navigation/status'))
}

export async function fetchNavigationTaskStatus(
  taskId: string,
): Promise<NavigationTaskStatusQueryResponse> {
  const normalized = taskId.trim()
  if (!normalized) throw new Error('navigation_task_id_required')
  const encoded = encodeURIComponent(normalized)
  const template = apiPath(
    'navigation_task_status',
    '/api/v1/navigation/tasks/{task_id}',
  )
  return fetchJson<NavigationTaskStatusQueryResponse>(
    template.includes('{task_id}')
      ? template.replace('{task_id}', encoded)
      : `/api/v1/navigation/tasks/${encoded}`,
  )
}

export async function fetchNavigationDdsSnapshot(): Promise<NavigationDdsSnapshotResponse> {
  return fetchJson<NavigationDdsSnapshotResponse>(
    apiPath('navigation_dds_snapshot', '/api/v1/navigation/dds_snapshot'),
  )
}

export async function fetchLocations(): Promise<LocationsResponse> {
  return fetchJson<LocationsResponse>(apiPath('locations', '/api/v1/locations'))
}

export async function fetchInspectionRoutes(mapId?: string | null): Promise<InspectionRouteListResponse> {
  const path = apiPath('inspection_routes', '/api/v1/inspection/routes')
  const query = mapId ? `?${new URLSearchParams({ map_id: mapId }).toString()}` : ''
  const data = await fetchJson<InspectionRouteListResponse>(`${path}${query}`)
  return {
    ...data,
    routes: data.routes.map(normalizeInspectionRoute),
  }
}

export async function fetchInspectionRoute(
  routeId: string,
  mapId?: string | null,
): Promise<InspectionRouteResponse> {
  const path = inspectionRouteDetailPath(routeId)
  const query = mapId ? `?${new URLSearchParams({ map_id: mapId }).toString()}` : ''
  const data = await fetchJson<InspectionRouteResponse>(`${path}${query}`)
  return {
    ...data,
    route: normalizeInspectionRoute(data.route),
  }
}

export async function saveInspectionRoute(
  request: InspectionRouteRequest,
): Promise<InspectionRouteResponse> {
  const data = await postJson<InspectionRouteResponse>(
    apiPath('inspection_routes', '/api/v1/inspection/routes'),
    request,
  )
  return {
    ...data,
    route: normalizeInspectionRoute(data.route),
  }
}

export async function deleteInspectionRoute(
  routeId: string,
  mapId?: string | null,
): Promise<InspectionCommandResponse> {
  const path = inspectionRouteDetailPath(routeId)
  const query = mapId ? `?${new URLSearchParams({ map_id: mapId }).toString()}` : ''
  const res = await dashboardFetch(`${path}${query}`, { method: 'DELETE' })
  return readJsonResponse<InspectionCommandResponse>(res)
}

export async function fetchInspectionTasks(
  options: {
    mapId?: string | null
    routeId?: string | null
    includeTerminal?: boolean
    limit?: number
  } = {},
): Promise<InspectionTaskListResponse> {
  const query = new URLSearchParams()
  if (options.mapId) query.set('map_id', options.mapId)
  if (options.routeId) query.set('route_id', options.routeId)
  if (options.includeTerminal === true) query.set('include_terminal', 'true')
  if (options.limit !== undefined) query.set('limit', String(options.limit))
  const path = apiPath('inspection_tasks', '/api/v1/inspection/tasks')
  return fetchJson<InspectionTaskListResponse>(
    query.size > 0 ? `${path}?${query.toString()}` : path,
  )
}

export async function fetchInspectionTask(
  taskId: string,
): Promise<InspectionTaskStatusResponse> {
  return fetchJson<InspectionTaskStatusResponse>(inspectionTaskPath(taskId))
}

export async function fetchInspectionTaskReport(
  taskId: string,
): Promise<InspectionTaskReportResponse> {
  return fetchJson<InspectionTaskReportResponse>(inspectionTaskReportPath(taskId))
}

export async function startInspectionTask(
  routeId: string,
  options: { map_id?: string | null; revision?: number | null; request_id?: string } = {},
): Promise<InspectionTaskCommandResponse> {
  return postJson<InspectionTaskCommandResponse>(
    apiPath('inspection_tasks', '/api/v1/inspection/tasks'),
    {
      route_id: routeId,
      map_id: options.map_id ?? null,
      revision: options.revision ?? 0,
      request_id: options.request_id ?? makeRequestId('inspection_start'),
    },
  )
}

export async function pauseInspectionTask(
  taskId: string,
): Promise<InspectionTaskCommandResponse> {
  return postJson<InspectionTaskCommandResponse>(
    inspectionTaskActionPath(
      taskId,
      'inspection_task_pause',
      '/api/v1/inspection/tasks/{task_id}/pause',
    ),
    {
      reason: 'operator_pause',
      request_id: makeRequestId('inspection_pause'),
    },
  )
}

export async function resumeInspectionTask(
  taskId: string,
): Promise<InspectionTaskCommandResponse> {
  return postJson<InspectionTaskCommandResponse>(
    inspectionTaskActionPath(
      taskId,
      'inspection_task_resume',
      '/api/v1/inspection/tasks/{task_id}/resume',
    ),
    {
      reason: 'operator_resume',
      request_id: makeRequestId('inspection_resume'),
    },
  )
}

export async function cancelInspectionTask(
  taskId: string,
): Promise<InspectionTaskCommandResponse> {
  return postJson<InspectionTaskCommandResponse>(
    inspectionTaskActionPath(
      taskId,
      'inspection_task_cancel',
      '/api/v1/inspection/tasks/{task_id}/cancel',
    ),
    {
      reason: 'operator_cancel',
      request_id: makeRequestId('inspection_cancel'),
    },
  )
}

export async function fetchInspectionStatus(): Promise<InspectionStatusResponse> {
  return fetchJson<InspectionStatusResponse>(
    apiPath('inspection_status', '/api/v1/inspection/status'),
  )
}

export async function fetchInspectionEvidence(limit = 12): Promise<InspectionEvidenceListResponse> {
  const boundedLimit = Math.min(100, Math.max(1, Math.trunc(limit)))
  return fetchJson<InspectionEvidenceListResponse>(
    `/api/v1/inspection/evidence?${new URLSearchParams({ limit: String(boundedLimit) }).toString()}`,
  )
}

export async function fetchInspectionEvidenceDetail(
  evidenceId: string,
): Promise<InspectionEvidenceDetailResponse> {
  return fetchJson<InspectionEvidenceDetailResponse>(inspectionEvidenceDetailPath(evidenceId))
}

export async function reviewInspectionEvidence(
  evidenceId: string,
  review: { verdict: 'normal' | 'anomaly' | 'inconclusive'; note?: string },
): Promise<InspectionEvidenceDetailResponse> {
  return postJson<InspectionEvidenceDetailResponse>(
    `${inspectionEvidenceDetailPath(evidenceId)}/review`, review,
  )
}

export async function saveLocation(body: LocationUpsertRequest): Promise<LocationOperationResponse> {
  return postJson<LocationOperationResponse>(
    apiPath('locations', '/api/v1/locations'),
    {
      ...body,
      source: body.source ?? 'web',
      client_id: body.client_id ?? WEB_CLIENT_ID,
      request_id: body.request_id ?? makeRequestId('location'),
    },
  )
}

export async function updateLocation(
  name: string,
  body: LocationUpsertRequest,
): Promise<LocationOperationResponse> {
  const res = await dashboardFetch(locationDetailPath(name), {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      ...body,
      name,
      source: body.source ?? 'web',
      client_id: body.client_id ?? WEB_CLIENT_ID,
      request_id: body.request_id ?? makeRequestId('location'),
    }),
  })
  return readJsonResponse<LocationOperationResponse>(res)
}

export async function deleteLocation(name: string): Promise<LocationOperationResponse> {
  const res = await dashboardFetch(locationDetailPath(name), { method: 'DELETE' })
  return readJsonResponse<LocationOperationResponse>(res)
}

// --- Navigation ---

export async function sendInstruction(text: string, runId?: string, controlOnly = false): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('instruction', '/api/v1/instruction'),
    commandBody('instruction', { text, control_only: controlOnly, ...(runId ? { run_id: runId } : {}) }),
  )
}

export async function getAgentTask(runId: string): Promise<AgentTaskStatusResponse> {
  return readJsonResponse<AgentTaskStatusResponse>(await dashboardFetch(
    `/api/v1/agent/runs/${encodeURIComponent(runId)}`,
    { signal: AbortSignal.timeout(5_000) },
  ))
}

export async function cancelAgentTask(runId: string): Promise<AgentTaskCancelResponse> {
  const body = commandBody('agent-cancel', {})
  return readJsonResponse<AgentTaskCancelResponse>(await dashboardFetch(
    `/api/v1/agent/runs/${encodeURIComponent(runId)}/cancel`,
    {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(5_000),
    },
  ))
}

export interface SendGoalOptions {
  z?: number
  yaw?: number
  source?: GoalSource
  target_type?: GoalTargetType
  label?: string | null
  acceptance_radius_m?: number | null
  max_speed_mps?: number | null
  metadata?: Record<string, unknown>
}

export const DIRECTED_EXPLORATION_TTL_S = 30

export interface DirectedExplorationTargetOptions {
  ttl_s?: number
  reason?: string
  request_id?: string | null
}

export async function sendGoal(
  x: number,
  y: number,
  options: SendGoalOptions = {},
): Promise<ControlCommandResponse> {
  const response = await postJson<ControlCommandResponse>(
    apiPath('goal', '/api/v1/goal'),
    commandBody('goal', { x, y, ...options }),
  )
  currentNavigationTaskStore.trackAccepted(response)
  return response
}

export async function fetchExplorationStatus(): Promise<ExplorationStatusResponse> {
  return fetchJson<ExplorationStatusResponse>(apiPath('explore_status', '/api/v1/explore/status'))
}

export async function setDirectedExplorationTarget(
  x: number,
  y: number,
  options: DirectedExplorationTargetOptions = {},
): Promise<DirectedExplorationResponse> {
  // The Gateway schema forbids client_id here. Keep this body limited to the
  // explicit directed-exploration contract rather than using commandBody().
  const body: DirectedExplorationTargetRequest = {
    x,
    y,
    ttl_s: options.ttl_s ?? DIRECTED_EXPLORATION_TTL_S,
    reason: options.reason ?? 'web_scene_directed_explore',
    request_id: options.request_id ?? makeRequestId('directed-explore'),
  }
  return postJson<DirectedExplorationResponse>(apiPath('explore_directed', '/api/v1/explore/directed'), body)
}

export async function navigateClick(
  x: number,
  y: number,
  options: SendGoalOptions = {},
): Promise<ControlCommandResponse> {
  const response = await postJson<ControlCommandResponse>(
    apiPath('navigate_click', '/api/v1/navigate/click'),
    commandBody('navigate_click', { x, y, ...options }),
  )
  currentNavigationTaskStore.trackAccepted(response)
  return response
}

export async function constructGoalCandidate(
  request: GoalCandidateRequest,
): Promise<GoalCandidateResponse> {
  return postJson<GoalCandidateResponse>(
    apiPath('navigation_goal_candidate', '/api/v1/navigation/goal_candidate'),
    {
      preview: true,
      ...request,
    },
  )
}

export async function previewNavigationPlan(
  x: number,
  y: number,
  z = 0,
  acceptanceRadiusM: number | null = null,
): Promise<PlanPreviewResponse> {
  const body: PlanPreviewRequest = {
    x,
    y,
    z,
    acceptance_radius_m: acceptanceRadiusM,
  }
  return postJson<PlanPreviewResponse>(apiPath('navigation_plan', '/api/v1/navigation/plan'), body)
}

export async function sendStop(): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('stop', '/api/v1/stop'),
    commandBody('stop', {}),
  )
}

export async function resetEstop(): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('estop_reset', '/api/v1/estop/reset'),
    commandBody('estop_reset', {}),
  )
}

export async function cancelNavigation(reason = 'client_cancel'): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('navigation_cancel', '/api/v1/navigation/cancel'),
    commandBody('navigation_cancel', { reason }),
  )
}

export async function cancelNavigationTask(
  taskId: string,
  reason = 'web_operator_cancel',
): Promise<ControlCommandResponse> {
  const normalized = taskId.trim()
  if (!normalized) throw new Error('navigation_task_id_required')
  const encoded = encodeURIComponent(normalized)
  const template = apiPath(
    'navigation_task_cancel',
    '/api/v1/navigation/tasks/{task_id}/cancel',
  )
  const path = template.includes('{task_id}')
    ? template.replace('{task_id}', encoded)
    : `/api/v1/navigation/tasks/${encoded}/cancel`
  return postJson<ControlCommandResponse>(
    path,
    commandBody('navigation_task_cancel', { task_id: normalized, reason }),
  )
}

export async function pauseNavigationTask(
  taskId: string,
  reason = 'web_operator_pause',
): Promise<ControlCommandResponse> {
  const normalized = taskId.trim()
  if (!normalized) throw new Error('navigation_task_id_required')
  const encoded = encodeURIComponent(normalized)
  const template = apiPath(
    'navigation_task_pause',
    '/api/v1/navigation/tasks/{task_id}/pause',
  )
  const path = template.includes('{task_id}')
    ? template.replace('{task_id}', encoded)
    : `/api/v1/navigation/tasks/${encoded}/pause`
  return postJson<ControlCommandResponse>(
    path,
    commandBody('navigation_task_pause', { task_id: normalized, reason }),
  )
}

export async function resumeNavigationTask(
  taskId: string,
  reason = 'web_operator_resume',
): Promise<ControlCommandResponse> {
  const normalized = taskId.trim()
  if (!normalized) throw new Error('navigation_task_id_required')
  const encoded = encodeURIComponent(normalized)
  const template = apiPath(
    'navigation_task_resume',
    '/api/v1/navigation/tasks/{task_id}/resume',
  )
  const path = template.includes('{task_id}')
    ? template.replace('{task_id}', encoded)
    : `/api/v1/navigation/tasks/${encoded}/resume`
  return postJson<ControlCommandResponse>(
    path,
    commandBody('navigation_task_resume', { task_id: normalized, reason }),
  )
}

export async function sendMode(mode: 'manual' | 'autonomous' | 'estop'): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('mode', '/api/v1/mode'),
    commandBody('mode', { mode }),
  )
}

export async function sendVisualServo(
  mode: VisualServoMode,
  target?: string | null,
  targetId?: string | null,
  followTask?: FollowTaskRef | null,
): Promise<ControlCommandResponse | FollowTaskCommandResponse> {
  const body: VisualServoRequest = {
    mode,
    target: mode === 'stop' ? null : target,
    target_id: mode === 'follow' ? targetId : null,
    ...(mode === 'stop' && followTask
      ? {
          follow_task_id: followTask.follow_task_id,
          product_session_id: followTask.product_session_id,
        }
      : {}),
  }
  return postJson<ControlCommandResponse | FollowTaskCommandResponse>(
    apiPath('visual_servo', '/api/v1/visual_servo'),
    commandBody('visual_servo', body as unknown as Record<string, unknown>),
  )
}

export async function getFollowTask(task: FollowTaskRef): Promise<FollowTaskStatusResponse> {
  const followTaskId = task.follow_task_id.trim()
  const productSessionId = task.product_session_id.trim()
  if (!followTaskId) throw new Error('follow_task_id_required')
  if (!productSessionId) throw new Error('product_session_id_required')
  const query = new URLSearchParams({ product_session_id: productSessionId })
  return fetchJson<FollowTaskStatusResponse>(
    `/api/v1/follow/${encodeURIComponent(followTaskId)}?${query.toString()}`,
  )
}

export async function cancelFollowTask(
  task: FollowTaskRef,
  reason = 'web_operator_cancel',
): Promise<FollowTaskCommandResponse> {
  const followTaskId = task.follow_task_id.trim()
  const productSessionId = task.product_session_id.trim()
  if (!followTaskId) throw new Error('follow_task_id_required')
  if (!productSessionId) throw new Error('product_session_id_required')
  return postJson<FollowTaskCommandResponse>(
    `/api/v1/follow/${encodeURIComponent(followTaskId)}/cancel`,
    commandBody('follow_cancel', {
      product_session_id: productSessionId,
      reason,
    }),
  )
}

export async function updateLease(action: LeaseAction, ttl = 30): Promise<LeaseResponse> {
  return postJson<LeaseResponse>(
    apiPath('lease', '/api/v1/lease'),
    commandBody('lease', { action, ttl }),
  )
}

// --- SLAM ---

export async function fetchSlamStatus(): Promise<SlamStatusResponse> {
  return fetchJson<SlamStatusResponse>(apiPath('slam_status', '/api/v1/slam/status'))
}

// --- Maps ---

export async function fetchMapList(): Promise<MapListResponse> {
  return readJsonResponse<MapListResponse>(await dashboardFetch(apiPath('maps', '/api/v1/slam/maps'), {
    cache: 'no-store',
  }))
}

export async function fetchMaps(): Promise<MapInfo[]> {
  const data = await fetchMapList()
  return data.maps
}

export async function deleteMap(name: string): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(mapNamedPath('map_delete', '/api/v1/maps/{name}', name), {
    method: 'DELETE',
  })
  return readMapLifecycle(res)
}

export async function renameMap(oldName: string, newName: string): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(apiPath('map_rename', '/api/v1/map/rename'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ old_name: oldName, new_name: newName }),
  })
  return readMapLifecycle(res)
}

export interface SaveMapResult extends MapLifecycleResponse {
  success: boolean | null
  name: string
  accepted?: boolean | null
  operation_id?: string | null
  operation?: MapSaveOperationStatus | null
  status?: string | null
  path?: string | null
  size?: string | null
  slam_profile?: string | null
  source?: string | null
  map_save_source?: string | null
  relocalization_supported?: boolean | null
  saved_map_relocalization_supported?: boolean | null
  restart_recovery_supported?: boolean | null
  recovery_method?: string | null
  warnings?: unknown[] | null
  dynamic_filter?: DynamicFilterResult | null
}

async function readLocalizationOperation(res: Response): Promise<LocalizationOperationResponse> {
  const text = await res.text()
  let data: Record<string, unknown> = {}
  try {
    data = text ? JSON.parse(text) as Record<string, unknown> : {}
  } catch {
    data = {}
  }
  if (!res.ok || data.ok === false || data.success === false) {
    const message = typeof data.message === 'string'
      ? data.message
      : typeof data.error === 'string'
        ? data.error
        : `HTTP ${res.status}`
    throw new Error(message)
  }
  return data as unknown as LocalizationOperationResponse
}

export interface MapSaveOperationStatus {
  created_at_ns?: number | null
  completed_at_ns?: number | null
  operation_id?: string | null
  map_id?: string | null
  name?: string | null
  state?: string | null
  phase?: string | null
  progress?: number | null
  reason_code?: string | null
  message?: string | null
  cancel_requested?: boolean | null
  source_saved?: boolean | null
  processing?: {
    optimization?: MapSaveProcessingStep
    cleanup?: MapSaveProcessingStep
  } | null
}

export interface MapSaveProcessingStep {
  performed?: boolean
  success?: boolean
  reason_code?: string
}

export type MapSaveWaitResult =
  | { state: 'saved'; result: SaveMapResult }
  | { state: 'source_saved'; result: SaveMapResult & { operation_id: string } }
  | { state: 'pending'; result: SaveMapResult & { operation_id: string }; reason: 'timeout' | 'status_unavailable' }

export interface WaitForMapSaveOptions {
  until?: 'source' | 'ready'
  timeoutMs?: number
  pollIntervalMs?: number
  signal?: AbortSignal
  onProgress?: (result: SaveMapResult) => void
}

function mapSaveOperationPath(operationId: string, action?: 'retry' | 'cancel'): string {
  const encoded = encodeURIComponent(operationId)
  const fallback = `/api/v1/maps/operations/{operation_id}${action ? `/${action}` : ''}`
  const template = apiPath(
    action === 'retry' ? 'map_operation_retry' : action === 'cancel' ? 'map_operation_cancel' : 'map_operation_status',
    fallback,
  ).replace(/%7Boperation_id%7D/gi, '{operation_id}')
  return template.includes('{operation_id}')
    ? template.replace('{operation_id}', encoded)
    : fallback.replace('{operation_id}', encoded)
}

function mapSaveOperation(response: SaveMapResult): MapSaveOperationStatus {
  return isRecord(response.operation)
    ? response.operation as MapSaveOperationStatus
    : response
}

function mapSaveOperationId(response: SaveMapResult): string {
  const operation = mapSaveOperation(response)
  const value = response.operation_id ?? operation.operation_id
  return typeof value === 'string' ? value.trim() : ''
}

function mapSaveState(response: SaveMapResult): string {
  const operation = mapSaveOperation(response)
  const value = operation.state ?? response.status
  return typeof value === 'string' ? value.trim().toUpperCase() : ''
}

export class MapSaveError extends Error {
  result: SaveMapResult

  constructor(message: string, result: SaveMapResult) {
    super(message)
    this.result = result
  }
}

function mapSaveFailure(response: SaveMapResult, state: string): MapSaveError {
  const operation = mapSaveOperation(response)
  const message = operation.message ?? response.message
  const reason = operation.reason_code ?? response.reason_code
  return new MapSaveError(
    typeof message === 'string' && message.trim()
      ? message
      : `Map save ${state.toLowerCase()}${reason ? `: ${String(reason)}` : ''}`,
    response,
  )
}

async function readMapSaveResponse(response: Response): Promise<SaveMapResult> {
  let result: SaveMapResult
  try {
    result = await response.json() as SaveMapResult
  } catch {
    throw new Error(`地图保存响应无法解析（HTTP ${response.status}）`)
  }
  if (!response.ok || result.ok === false || result.success === false) {
    throw mapSaveFailure(result, mapSaveState(result) || 'FAILED')
  }
  return result
}

function waitDelay(delayMs: number, signal?: AbortSignal): Promise<void> {
  if (signal?.aborted) return Promise.reject(new DOMException('Map save wait aborted', 'AbortError'))
  return new Promise((resolve, reject) => {
    const onAbort = () => {
      globalThis.clearTimeout(timer)
      reject(new DOMException('Map save wait aborted', 'AbortError'))
    }
    const timer = globalThis.setTimeout(() => {
      signal?.removeEventListener('abort', onAbort)
      resolve()
    }, delayMs)
    signal?.addEventListener('abort', onAbort, { once: true })
  })
}

export async function saveMap(name: string, autoProcess = true): Promise<SaveMapResult> {
  const res = await dashboardFetch(apiPath('map_save', '/api/v1/map/save'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, auto_process: autoProcess }),
  })
  return readMapSaveResponse(res)
}

export async function fetchMapSaveOperation(operationId: string, signal?: AbortSignal): Promise<SaveMapResult> {
  return readJsonResponse<SaveMapResult>(await dashboardFetch(mapSaveOperationPath(operationId), { signal }))
}

export interface MapSaveOperationList {
  operations: MapSaveOperationStatus[]
  ts: number
}

export async function fetchMapSaveOperations(signal?: AbortSignal): Promise<MapSaveOperationList> {
  return readJsonResponse<MapSaveOperationList>(
    await dashboardFetch(apiPath('map_operations', '/api/v1/maps/operations'), { signal }),
  )
}

export async function retryMapSaveOperation(operationId: string): Promise<SaveMapResult> {
  return readMapSaveResponse(await dashboardFetch(mapSaveOperationPath(operationId, 'retry'), {
    method: 'POST',
  }))
}

export async function cancelMapSaveOperation(operationId: string): Promise<SaveMapResult> {
  return readMapSaveResponse(await dashboardFetch(mapSaveOperationPath(operationId, 'cancel'), {
    method: 'POST',
  }))
}

export async function processMapSaveOperation(operationId: string): Promise<SaveMapResult> {
  return readMapSaveResponse(await dashboardFetch(`/api/v1/maps/operations/${encodeURIComponent(operationId)}/process`, {
    method: 'POST',
  }))
}

export function mapSaveSourceUrl(operationId: string): string {
  return `/api/v1/maps/operations/${encodeURIComponent(operationId)}/source/pcd`
}

export async function waitForMapSaveOperation(
  admission: SaveMapResult,
  options: WaitForMapSaveOptions = {},
): Promise<MapSaveWaitResult> {
  const initialState = mapSaveState(admission)
  if (['FAILED', 'CANCELLED', 'CANCELED'].includes(initialState)
    && !(options.until === 'source' && mapSaveOperation(admission).source_saved === true)) {
    throw mapSaveFailure(admission, initialState)
  }
  if (initialState === 'SUCCEEDED') return { state: 'saved', result: { ...admission, success: true } }
  const operationId = mapSaveOperationId(admission)
  if (!operationId && admission.success === true) return { state: 'saved', result: admission }
  if (!operationId) {
    throw new Error('Map save was accepted without an operation_id')
  }
  if (options.until === 'source' && mapSaveOperation(admission).source_saved === true) {
    return { state: 'source_saved', result: { ...admission, operation_id: operationId } }
  }
  const timeoutMs = Math.max(1, options.timeoutMs ?? 360_000)
  const pollIntervalMs = Math.max(0, options.pollIntervalMs ?? 750)
  const deadline = Date.now() + timeoutMs
  let last = admission
  const pending = (reason: 'timeout' | 'status_unavailable'): MapSaveWaitResult => ({
    state: 'pending', reason,
    result: { ...last, name: last.name || admission.name, success: null, operation_id: operationId },
  })
  options.onProgress?.(last)

  while (Date.now() <= deadline) {
    if (options.signal?.aborted) {
      throw new DOMException('Map save wait aborted', 'AbortError')
    }
    try {
      last = { ...admission, ...await fetchMapSaveOperation(operationId, options.signal) }
    } catch (error) {
      if (options.signal?.aborted) throw error
      return pending('status_unavailable')
    }
    options.onProgress?.(last)
    const state = mapSaveState(last)
    if (state === 'SUCCEEDED') {
      return { state: 'saved', result: {
        ...admission,
        ...last,
        success: true,
        accepted: true,
        operation_id: operationId,
        name: last.name || admission.name,
      } }
    }
    if (options.until === 'source' && mapSaveOperation(last).source_saved === true) {
      return { state: 'source_saved', result: { ...last, name: last.name || admission.name, operation_id: operationId } }
    }
    if (state === 'FAILED' || state === 'CANCELLED' || state === 'CANCELED') {
      throw mapSaveFailure(last, state)
    }
    if (state && state !== 'WAITING_SNAPSHOT' && state !== 'QUEUED' && state !== 'RUNNING' && state !== 'CAPTURED') {
      throw new Error(`Map save returned unknown operation state: ${state}`)
    }
    if (Date.now() >= deadline) break
    await waitDelay(Math.min(pollIntervalMs, Math.max(0, deadline - Date.now())), options.signal)
  }
  return pending('timeout')
}

export async function importPcdMap(
  name: string,
  sourcePath: string,
  voxelSize = 0,
): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(apiPath('map_import_pcd', '/api/v1/maps/import_pcd'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, source_path: sourcePath, voxel_size: voxelSize }),
  })
  return readMapLifecycle(res)
}

export async function cropMap(
  name: string,
  bounds: Record<string, unknown>,
): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(mapNamedPath('map_crop', '/api/v1/maps/{name}/crop', name), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ bounds }),
  })
  return readMapLifecycle(res)
}

export async function markMapZone(
  name: string,
  body: Record<string, unknown>,
): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(mapNamedPath('map_mark_zone', '/api/v1/maps/{name}/mark_zone', name), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  return readMapLifecycle(res)
}

export interface EditedMapResponse extends MapLifecycleResponse {
  build_options?: Record<string, unknown>
  content_epoch?: number
  saved_rays_preserved?: boolean
  point_count?: number
  removed_points?: number
  /** Local review responses explicitly distinguish sampled-only counts. */
  point_count_scope?: 'preview_sample'
}

export async function saveEditedMap(source: Pick<MapEditSource, 'name' | 'contentEpoch'>, name: string, operations: readonly MapEditOperation[]): Promise<EditedMapResponse> {
  let res: Response
  try {
    res = await dashboardFetch(mapNamedPath('map_save_as', '/api/v1/maps/{name}/save_as', source.name), {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, expected_source_content_epoch: source.contentEpoch,
        operations: mapEditWireOperations(operations) }),
    })
  } catch { throw new MapEditSaveUnconfirmed() }
  try { return await readMapLifecycle(res) as EditedMapResponse }
  catch (reason) { if (res.ok) throw new MapEditSaveUnconfirmed(); throw reason }
}

export async function buildMapOctomap(name: string, options?: Record<string, unknown>): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(mapNamedPath('map_build_octomap', '/api/v1/maps/{name}/build_octomap', name), {
    method: 'POST',
    ...(options ? { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(options) } : {}),
  })
  return readMapLifecycle(res)
}

export async function validateMapPlan(
  name: string,
  x: number,
  y: number,
  z = 0,
): Promise<Record<string, unknown>> {
  return postJson<Record<string, unknown>>(
    mapNamedPath('map_validate_plan', '/api/v1/maps/{name}/validate_plan', name),
    { x, y, z, client_id: WEB_CLIENT_ID },
  )
}

// --- Session status ---

export type SessionState = SessionEvent['data']

function normalizeInspectionRoute(route: InspectionRoute): InspectionRoute {
  return {
    ...route,
    points: (route.points ?? []).map(point => {
      const raw = point as InspectionRoute['points'][number] & {
        position_tolerance_m?: number
        dwell_s?: number
      }
      return {
        ...point,
        tolerance: Number.isFinite(Number(raw.tolerance))
          ? Number(raw.tolerance)
          : Number(raw.position_tolerance_m ?? 0.35),
        dwell: Number.isFinite(Number(raw.dwell))
          ? Number(raw.dwell)
          : Number(raw.dwell_s ?? 0),
      }
    }),
  }
}

export async function resumeNavigation(): Promise<ControlCommandResponse> {
  const response = await postJson<ControlCommandResponse>(
    apiPath('navigation_resume', '/api/v1/navigation/resume'),
    commandBody('navigation_resume', {}),
  )
  if (!response.ok) throw new Error(response.status || '恢复请求被拒绝')
  for (let attempt = 0; attempt < 12; attempt += 1) {
    const status = await fetchNavigationStatus()
    if (status.control?.resume_required === false) {
      const authority = status.control?.authority
      const permission = status.motion?.permission
      if (authority === 'UNKNOWN' || permission === 'UNKNOWN') {
        if (attempt < 11) await new Promise(resolve => setTimeout(resolve, 150))
        continue
      }
      if (permission === 'ESTOPPED') {
        throw new Error('控制暂停未恢复：急停仍在生效')
      }
      // A task pause or input hold can remain after the control hold is released.
      // Motion permission is reported separately; this command never starts a task.
      return response
    }
    if (attempt < 11) await new Promise(resolve => setTimeout(resolve, 150))
  }
  throw new Error('恢复请求已接受，但控制状态仍未确认；请查看控制状态')
}

export async function fetchSession(): Promise<SessionState> {
  return fetchJson<SessionState>(apiPath('session', '/api/v1/session'))
}

export async function resetMapCloud(): Promise<MapLifecycleResponse> {
  const res = await dashboardFetch(apiPath('map_cloud_reset', '/api/v1/map_cloud/reset'), { method: 'POST' })
  const data = await readMapLifecycle(res)
  if (typeof window !== 'undefined') {
    window.dispatchEvent(new CustomEvent('lingtu:cloud-reset'))
  }
  return data
}

export async function fetchSavedMapPointsResponse(name: string): Promise<MapPointsResponse> {
  return fetchJson<MapPointsResponse>(mapPointsPath(name))
}

export async function fetchSavedMapPointCloud(name: string): Promise<SavedMapPointCloud> {
  const data = await fetchSavedMapPointsResponse(name)
  const frameId = String(data.frame_id || '').trim().replace(/^\/+/, '')
  if (!frameId) throw new Error('Saved map response is missing frame_id')
  let epoch: number | null = null
  if (data.epoch != null) {
    epoch = Number(data.epoch)
    if (!Number.isInteger(epoch) || epoch < 1 || epoch > 0xffffffff) {
      throw new Error('Saved map response is missing a valid scene epoch')
    }
  }
  const mapName = String(data.name || name).trim()
  if (mapName !== name) {
    throw new Error(`Saved map response name mismatch: expected ${name}, got ${mapName}`)
  }
  let contentEpoch: number | null = null
  if (data.content_epoch != null) {
    contentEpoch = Number(data.content_epoch)
    if (!Number.isSafeInteger(contentEpoch) || contentEpoch < 1) {
      throw new Error('Saved map response is missing a valid content_epoch')
    }
  }
  const preview = await fetchMapPreview(name, contentEpoch)
  if (preview.frameId.replace(/^\/+/, '') !== frameId) {
    throw new Error('Saved map frame changed while loading; retry')
  }
  const points = parsePcd(preview.buffer)
  if (!points || points.length < 3) throw new Error('地图点云为空或格式不支持')
  return {
    points: Array.from(points),
    frameId,
    epoch,
    mapName,
    contentEpoch: preview.contentEpoch,
    timestamp: Number(data.ts) || 0,
  }
}

export async function fetchSavedMapPoints(name: string): Promise<number[]> {
  return (await fetchSavedMapPointCloud(name)).points
}

export async function relocalize(
  mapName: string,
  x: number,
  y: number,
  yaw: number,
  z: number = 0,
): Promise<LocalizationOperationResponse> {
  const res = await dashboardFetch(apiPath('localization_relocalize', '/api/v1/localization/relocalizations'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      map_name: mapName,
      mode: 'seeded',
      initial_pose: { x, y, z, yaw },
    }),
  })
  return readLocalizationOperation(res)
}

export async function globalRelocalize(mapName: string): Promise<LocalizationOperationResponse> {
  const res = await dashboardFetch(apiPath('localization_relocalize', '/api/v1/localization/relocalizations'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ map_name: mapName, mode: 'global' }),
  })
  return readLocalizationOperation(res)
}

// --- Diagnostics / bag recording ---

export async function fetchRecordingStatus(): Promise<RecordingStatusResponse> {
  return fetchJson<RecordingStatusResponse>(apiPath('recording_status', '/api/v1/recordings/status'))
}

export async function fetchRecordingCatalog(): Promise<RecordingCatalogResponse> {
  return fetchJson<RecordingCatalogResponse>('/api/v1/recordings/catalog')
}

export async function fetchRecordingStreams(): Promise<RecordingStreamsResponse> {
  return fetchJson<RecordingStreamsResponse>('/api/v1/recordings/streams')
}

export async function fetchRecordingSeries(sessionId: string, query: RecordingSeriesQuery, signal?: AbortSignal): Promise<RecordingSeriesResponse> {
  const params = new URLSearchParams({ topic: query.topic })
  if (query.field) params.set('field', query.field)
  if (query.start !== undefined) params.set('start', String(query.start))
  if (query.end !== undefined) params.set('end', String(query.end))
  if (query.max_points !== undefined) params.set('max_points', String(query.max_points))
  return readJsonResponse<RecordingSeriesResponse>(await dashboardFetch(`/api/v1/recordings/${encodeURIComponent(sessionId)}/series?${params}`, { signal }))
}

export async function fetchRecordingSceneInfo(sessionId: string, signal?: AbortSignal): Promise<RecordingSceneInfoResponse> {
  return readJsonResponse<RecordingSceneInfoResponse>(await dashboardFetch(`/api/v1/recordings/${encodeURIComponent(sessionId)}/scene-info`, { signal }))
}

export async function fetchRecordingScene(sessionId: string, query: RecordingSceneQuery, signal?: AbortSignal): Promise<RecordingSceneResponse> {
  const params = new URLSearchParams({ start: String(query.start), end: String(query.end), max_frames: '32', max_points: '2048', max_poses: '2000' })
  if (query.cloud_topic) params.set('cloud_topic', query.cloud_topic)
  if (query.pose_topic) params.set('pose_topic', query.pose_topic)
  for (const topic of query.path_topic ?? []) params.append('path_topic', topic)
  return readJsonResponse<RecordingSceneResponse>(await dashboardFetch(`/api/v1/recordings/${encodeURIComponent(sessionId)}/scene?${params}`, { signal }))
}

export async function fetchRecordingCameraFrame(sessionId: string, segment: number, time: number, cameraId: string, signal?: AbortSignal): Promise<RecordingCameraFrame | null> {
  const params = new URLSearchParams({ segment: String(segment), t: String(time), camera_id: cameraId })
  const response = await dashboardFetch(`/api/v1/recordings/${encodeURIComponent(sessionId)}/camera-frame?${params}`, { signal })
  if (response.status === 204) return null
  if (!response.ok) { await readJsonResponse(response); return null }
  const timestamp = response.headers.get('X-Recording-Time-S')
  const frameTime = timestamp === null ? NaN : Number(timestamp)
  if (!Number.isFinite(frameTime)) throw new Error('recording_camera_frame_time_missing')
  const mediaTime = response.headers.get('X-Recording-Media-Time-S')
  const frameIndex = response.headers.get('X-Recording-Frame-Index')
  return { blob: await response.blob(), t: frameTime, mediaTime: mediaTime === null ? null : Number(mediaTime), timing: response.headers.get('X-Recording-Timing') ?? 'segment_cfr_estimate', sourceTimeNs: response.headers.get('X-Recording-Source-Time-Ns'), frameIndex: frameIndex === null ? null : Number(frameIndex), frameId: response.headers.get('X-Recording-Frame-Id') }
}

export async function fetchRecordingList(limit = 100): Promise<RecordingListResponse> {
  const bounded = Math.max(1, Math.min(256, Math.trunc(limit)))
  return fetchJson<RecordingListResponse>(`/api/v1/recordings?limit=${bounded}`)
}

export async function fetchRecordingDetail(sessionId: string): Promise<RecordingDetailResponse> {
  return fetchJson<RecordingDetailResponse>(
    `/api/v1/recordings/${encodeURIComponent(sessionId)}`,
  )
}

export async function fetchRecordingSummary(sessionId: string): Promise<RecordingSummaryResponse> {
  return fetchJson<RecordingSummaryResponse>(`/api/v1/recordings/${encodeURIComponent(sessionId)}/summary`)
}

export async function validateRecording(sessionId: string): Promise<RecordingValidationResponse> {
  return postJson<RecordingValidationResponse>(`/api/v1/recordings/${encodeURIComponent(sessionId)}/validate`)
}

export function recordingArtifactUrl(sessionId: string, artifactPath: string): string {
  const encodedPath = artifactPath.split('/').map(part => encodeURIComponent(part)).join('/')
  return `/api/v1/recordings/${encodeURIComponent(sessionId)}/files/${encodedPath}`
}

export function recordingArchiveUrl(sessionId: string): string {
  return `/api/v1/recordings/${encodeURIComponent(sessionId)}/archive`
}

export async function deleteRecording(sessionId: string): Promise<{ ok: boolean; session_id: string }> {
  return readJsonResponse<{ ok: boolean; session_id: string }>(await dashboardFetch(
    `/api/v1/recordings/${encodeURIComponent(sessionId)}`,
    { method: 'DELETE' },
  ))
}

export async function startRecording(
  config: RecordingStartConfig = {},
): Promise<RecordingOperationResponse> {
  return postJson<RecordingOperationResponse>(
    apiPath('recording_start', '/api/v1/recordings/start'),
    {
      duration: config.duration ?? 600,
      prefix: config.prefix ?? 'web',
      capture_profile: config.capture_profile ?? 'sensors',
      ...(config.task_id !== undefined ? { task_id: config.task_id } : {}),
      camera: config.camera ?? false,
      minimum_free_gib: config.minimum_free_gib ?? 5,
      ...(config.topics !== undefined ? { topics: config.topics } : {}),
    },
  )
}

export async function stopRecording(): Promise<RecordingOperationResponse> {
  return postJson<RecordingOperationResponse>(apiPath('recording_stop', '/api/v1/recordings/stop'))
}
