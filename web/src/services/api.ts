// Centralized API service layer for LingTu web dashboard
// All fetch() calls in one place.

import type {
  AlgorithmBenchmarkLatestResponse,
  AppBootstrapResponse,
  AppCapabilitiesResponse,
  AppTrafficResponse,
  AuthCheckResponse,
  AuthLoginResponse,
  RecordingOperationResponse,
  RecordingStatusResponse,
  ClientLinks,
  CommandReceipt,
  ControlCommandResponse,
  DevicesResponse,
  EnvName,
  DirectedExplorationResponse,
  DirectedExplorationTargetRequest,
  DynamicFilterResult,
  ExplorationStatusResponse,
  GatewayErrorResponse,
  GoalCandidateRequest,
  GoalCandidateResponse,
  GoalSource,
  GoalTargetType,
  HealthResponse,
  InspectionAcceptanceRequest,
  InspectionAcceptanceResponse,
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
  ProductName,
  ReadinessResponse,
  RealRuntimeEvidenceLatestResponse,
  RoutecheckLatestResponse,
  RuntimeDataflowResponse,
  RuntimeDataflowSubscribeRequest,
  RuntimeDataflowSubscribeResponse,
  RuntimeDataflowTopicDetailResponse,
  RuntimeSwitchPlanRequest,
  RuntimeSwitchPlanResponse,
  SceneGraphResponse,
  SessionEvent,
  SessionTransitionResponse,
  SlamOperationResponse,
  SlamProfile,
  SlamStatusResponse,
  StateResponse,
  VisualServoMode,
  VisualServoRequest,
} from '../types'
import { currentNavigationTaskStore } from './currentNavigationTask.ts'

const WEB_CLIENT_ID = 'web-dashboard'

type CommandResponse = ControlCommandResponse | LeaseResponse

export interface SavedMapPointCloud {
  points: number[]
  frameId: string
  epoch: number | null
  mapName: string
  versionId: string | null
  mapPcdSha256: string | null
  timestamp: number
}

let clientLinks: ClientLinks = {}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function shortRequestId(id?: string | null): string {
  return id ? id.slice(0, 8) : ''
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
  const requestId = shortRequestId(command?.request_id)
  if (requestId) parts.push(`#${requestId}`)
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
    const message = error.body?.message || error.body?.error || error.message || `HTTP ${error.statusCode}`
    const blockers = detailStrings(error.detail, 'blockers')
    const advisories = detailStrings(error.detail, 'advisories')
    const reasonParts = [...blockers, ...advisories].slice(0, 3)
    const reason = reasonParts.length ? `；原因：${reasonParts.join('，')}` : ''
    const rejected = error.command?.accepted === false ? '（未接受）' : ''
    return `${label}${rejected}: ${message}${reason}${commandSuffix(error.command)}`
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
  return readJsonResponse<T>(await fetch(url))
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
    '/api/v1/maps/{name}/points?max_points=80000'
  const withName = template.includes('{name}')
    ? template.replace('{name}', encoded)
    : `/api/v1/maps/${encoded}/points?max_points=80000`
  const url = new URL(withName, window.location.origin)
  url.searchParams.set('max_points', '80000')
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

function makeRequestId(prefix: string): string {
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

function flattenPointArray(points: MapPointsResponse['points']): number[] {
  const first = points[0]
  if (Array.isArray(first)) {
    return (points as Array<[number, number, number]>).flatMap(([x, y, z]) => [x, y, z])
  }
  return points as number[]
}

async function postJson<T>(url: string, body?: unknown): Promise<T> {
  const res = await fetch(url, {
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

async function readSlamOperation(res: Response): Promise<SlamOperationResponse> {
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
  return data as SlamOperationResponse
}

// --- App bootstrap / read APIs ---

export async function fetchAppBootstrap(): Promise<AppBootstrapResponse> {
  const data = await fetchJson<AppBootstrapResponse>(apiPath('bootstrap', '/api/v1/app/bootstrap'))
  setClientLinks(data.links)
  return data
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

export async function fetchAlgorithmBenchmarkLatest(): Promise<AlgorithmBenchmarkLatestResponse> {
  return fetchJson<AlgorithmBenchmarkLatestResponse>(
    apiPath('algorithm_benchmark_latest', '/api/v1/diagnostics/algorithm-benchmark/latest'),
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

export async function runRuntimeSwitchPlan(
  request: RuntimeSwitchPlanRequest,
): Promise<RuntimeSwitchPlanResponse> {
  return postJson<RuntimeSwitchPlanResponse>(
    apiPath('runtime_switch_plan', '/api/v1/runtime/switch-plan'),
    request,
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

export async function fetchDevices(): Promise<DevicesResponse> {
  return fetchJson<DevicesResponse>(apiPath('devices', '/api/v1/devices'))
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

export async function runInspectionAcceptance(
  request: InspectionAcceptanceRequest = {},
): Promise<InspectionAcceptanceResponse> {
  return postJson<InspectionAcceptanceResponse>(
    apiPath('inspection_acceptance', '/api/v1/inspection/acceptance'),
    {
      mode: 'simulation',
      client_id: WEB_CLIENT_ID,
      ...request,
    },
  )
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
  const res = await fetch(`${path}${query}`, { method: 'DELETE' })
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
  options: { map_id?: string | null; revision?: number | null } = {},
): Promise<InspectionTaskCommandResponse> {
  return postJson<InspectionTaskCommandResponse>(
    apiPath('inspection_tasks', '/api/v1/inspection/tasks'),
    {
      route_id: routeId,
      map_id: options.map_id ?? null,
      revision: options.revision ?? 0,
      request_id: makeRequestId('inspection_start'),
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
  const res = await fetch(locationDetailPath(name), {
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
  const res = await fetch(locationDetailPath(name), { method: 'DELETE' })
  return readJsonResponse<LocationOperationResponse>(res)
}

// --- Navigation ---

export async function sendInstruction(text: string): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('instruction', '/api/v1/instruction'),
    commandBody('instruction', { text }),
  )
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
      client_id: WEB_CLIENT_ID,
      ...request,
    },
  )
}

export async function previewNavigationPlan(
  x: number,
  y: number,
  z = 0,
): Promise<PlanPreviewResponse> {
  const body: PlanPreviewRequest = {
    x,
    y,
    z,
    client_id: WEB_CLIENT_ID,
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
): Promise<ControlCommandResponse> {
  const body: VisualServoRequest = {
    mode,
    target: mode === 'stop' ? null : target,
  }
  return postJson<ControlCommandResponse>(
    apiPath('visual_servo', '/api/v1/visual_servo'),
    commandBody('visual_servo', body as unknown as Record<string, unknown>),
  )
}

export async function updateLease(action: LeaseAction, ttl = 30): Promise<LeaseResponse> {
  return postJson<LeaseResponse>(
    apiPath('lease', '/api/v1/lease'),
    commandBody('lease', { action, ttl }),
  )
}

// --- SLAM ---

export async function switchSlamMode(profile: SlamProfile): Promise<SlamOperationResponse> {
  const res = await fetch(apiPath('slam_switch', '/api/v1/slam/switch'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ profile }),
  })
  return readSlamOperation(res)
}

export async function fetchSlamStatus(): Promise<SlamStatusResponse> {
  return fetchJson<SlamStatusResponse>(apiPath('slam_status', '/api/v1/slam/status'))
}

// --- Maps ---

export async function fetchMapList(): Promise<MapListResponse> {
  return fetchJson<MapListResponse>(apiPath('maps', '/api/v1/slam/maps'))
}

export async function fetchMaps(): Promise<MapInfo[]> {
  const data = await fetchMapList()
  return data.maps
}

export async function activateMap(name: string): Promise<MapLifecycleResponse> {
  const res = await fetch(apiPath('map_activate', '/api/v1/map/activate'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  })
  return readMapLifecycle(res)
}

export async function deleteMap(name: string): Promise<MapLifecycleResponse> {
  const res = await fetch(mapNamedPath('map_delete', '/api/v1/maps/{name}', name), {
    method: 'DELETE',
  })
  return readMapLifecycle(res)
}

export async function renameMap(oldName: string, newName: string): Promise<MapLifecycleResponse> {
  const res = await fetch(apiPath('map_rename', '/api/v1/map/rename'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ old_name: oldName, new_name: newName }),
  })
  return readMapLifecycle(res)
}

export interface SaveMapResult extends MapLifecycleResponse {
  success: boolean
  name: string
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
  map_optimization?: Record<string, unknown> | null
  map_optimization_ok?: boolean | null
}

export async function saveMap(name: string, optimization?: 'pgo' | 'hba' | 'none'): Promise<SaveMapResult> {
  // Save can take up to ~2 min on a busy robot because map optimization + DUFOMap
  // run synchronously. Default fetch has no timeout which is what we want.
  const res = await fetch(apiPath('map_save', '/api/v1/map/save'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name, optimization }),
  })
  return readMapLifecycle(res) as Promise<SaveMapResult>
}

export async function importPcdMap(
  name: string,
  sourcePath: string,
  voxelSize = 0,
): Promise<MapLifecycleResponse> {
  const res = await fetch(apiPath('map_import_pcd', '/api/v1/maps/import_pcd'), {
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
  const res = await fetch(mapNamedPath('map_crop', '/api/v1/maps/{name}/crop', name), {
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
  const res = await fetch(mapNamedPath('map_mark_zone', '/api/v1/maps/{name}/mark_zone', name), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  })
  return readMapLifecycle(res)
}

export async function buildMapOctomap(name: string): Promise<MapLifecycleResponse> {
  const res = await fetch(mapNamedPath('map_build_octomap', '/api/v1/maps/{name}/build_octomap', name), {
    method: 'POST',
  })
  return readMapLifecycle(res)
}

export async function validateMapPlan(
  name: string,
  x: number,
  y: number,
  z = 0,
  plannerConstraints: Record<string, unknown> = {},
): Promise<Record<string, unknown>> {
  return postJson<Record<string, unknown>>(
    mapNamedPath('map_validate_plan', '/api/v1/maps/{name}/validate_plan', name),
    { x, y, z, client_id: WEB_CLIENT_ID, planner_constraints: plannerConstraints },
  )
}

// --- Session state machine ---

export type SessionState = SessionEvent['data']
export type SessionMode = 'mapping' | 'navigating' | 'exploring'
export interface StartSessionOptions {
  mapName?: string
  slamProfile?: Exclude<SlamProfile, 'stop'>
}

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
  return postJson<ControlCommandResponse>(
    apiPath('navigation_resume', '/api/v1/navigation/resume'),
    commandBody('navigation_resume', {}),
  )
}

export interface ProductSessionSwitchOptions {
  currentProduct?: ProductName | null
  mapName?: string | null
  relocalize?: boolean
  initialPose?: [number, number, number] | null
}

export interface ProductControlHandoff {
  plan: RuntimeSwitchPlanResponse
  command: string
}

const SAVED_MAP_SWITCH_PRODUCTS = new Set<ProductName>(['nav', 'tracking', 'inspection'])

async function copyOperatorCommand(command: string): Promise<void> {
  if (typeof navigator === 'undefined' || !navigator.clipboard?.writeText) {
    throw new Error('Clipboard access is unavailable; use the ProductControl command shown in the preflight result')
  }
  await navigator.clipboard.writeText(command)
}

export async function prepareProductSwitch(
  targetProduct: ProductName,
  options: ProductSessionSwitchOptions = {},
): Promise<ProductControlHandoff> {
  const mapName = options.mapName?.trim() || null
  const requiresSavedMap = SAVED_MAP_SWITCH_PRODUCTS.has(targetProduct)
  if (requiresSavedMap && !mapName) {
    throw new Error(`${targetProduct} Product switch requires a saved map`)
  }
  const usesSavedMap = requiresSavedMap || (targetProduct === 'explore' && mapName !== null)
  const relocalize = usesSavedMap && options.relocalize !== false
  const plan = await runRuntimeSwitchPlan({
    current_product: options.currentProduct ?? null,
    target_product: targetProduct,
    map_name: usesSavedMap ? mapName : null,
    relocalize,
    initial_pose: usesSavedMap ? options.initialPose : null,
  })
  if (!plan.ok || !plan.read_only || !plan.dry_run || plan.motion) {
    const blocker = plan.blockers?.filter(Boolean).join('; ')
    throw new Error(blocker || plan.error || 'Product switch preflight was rejected')
  }
  const command = plan.operator_command?.trim()
  if (!command) {
    throw new Error('Gateway did not return a ProductControl command')
  }
  return { plan, command }
}

export async function copyProductSwitchCommand(
  targetProduct: ProductName,
  options: ProductSessionSwitchOptions = {},
): Promise<ProductControlHandoff> {
  const handoff = await prepareProductSwitch(targetProduct, options)
  await copyOperatorCommand(handoff.command)
  return handoff
}

export function productControlStopCommand(env: EnvName): string {
  return `python -m lingtu.control stop-session --env ${env}`
}

export async function copyProductControlStopCommand(env: EnvName): Promise<string> {
  const command = productControlStopCommand(env)
  await copyOperatorCommand(command)
  return command
}

export function productControlRestartCommand(env: EnvName, process: 'slam'): string {
  return `python -m lingtu.control restart --process ${process} --env ${env}`
}

export async function copyProductControlRestartCommand(
  env: EnvName,
  process: 'slam',
): Promise<string> {
  const command = productControlRestartCommand(env, process)
  await copyOperatorCommand(command)
  return command
}

export async function fetchSession(): Promise<SessionState> {
  return fetchJson<SessionState>(apiPath('session', '/api/v1/session'))
}

export async function startSession(mode: SessionMode, mapNameOrOptions?: string | StartSessionOptions): Promise<SessionState> {
  const options: StartSessionOptions = typeof mapNameOrOptions === 'string'
    ? { mapName: mapNameOrOptions }
    : (mapNameOrOptions ?? {})
  const body: Record<string, string> = {
    mode,
    map_name: options.mapName ?? '',
  }
  if (options.slamProfile) {
    body.slam_profile = options.slamProfile
  }
  const data = await postJson<SessionTransitionResponse>(apiPath('session_start', '/api/v1/session/start'), body)
  if (!data.success) throw new Error(data.message || 'Session start failed')
  return data.session as SessionState
}

export async function endSession(): Promise<SessionState> {
  const data = await postJson<SessionTransitionResponse>(apiPath('session_end', '/api/v1/session/end'))
  if (!data.success) throw new Error(data.message || 'Session end failed')
  return data.session as SessionState
}

export async function resetMapCloud(): Promise<MapLifecycleResponse> {
  const res = await fetch(apiPath('map_cloud_reset', '/api/v1/map_cloud/reset'), { method: 'POST' })
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
  return {
    points: flattenPointArray(data.points),
    frameId,
    epoch,
    mapName,
    versionId: String(data.version_id || '').trim() || null,
    mapPcdSha256: String(data.map_pcd_sha256 || '').trim() || null,
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
): Promise<SlamOperationResponse> {
  const res = await fetch(apiPath('slam_relocalize', '/api/v1/slam/relocalize'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ map_name: mapName, x, y, yaw }),
  })
  return readSlamOperation(res)
}

// Global (no-guess) relocalize via 3D-BBS. Fires the worker in localizer;
// the 2-4 s scan runs async, so the response is immediate. Fitness appears
// on /localization_quality after the worker finishes.
export async function autoRelocalize(): Promise<SlamOperationResponse> {
  const res = await fetch(apiPath('slam_auto_relocalize', '/api/v1/slam/auto_relocalize'), { method: 'POST' })
  return readSlamOperation(res)
}

// --- Diagnostics / bag recording ---

export async function fetchRecordingStatus(): Promise<RecordingStatusResponse> {
  return fetchJson<RecordingStatusResponse>(apiPath('recording_status', '/api/v1/recordings/status'))
}

export async function startRecording(
  duration = 600,
  prefix = 'web',
): Promise<RecordingOperationResponse> {
  return postJson<RecordingOperationResponse>(
    apiPath('recording_start', '/api/v1/recordings/start'),
    { duration, prefix },
  )
}

export async function stopRecording(): Promise<RecordingOperationResponse> {
  return postJson<RecordingOperationResponse>(apiPath('recording_stop', '/api/v1/recordings/stop'))
}

// --- Auth ---

export async function login(key: string): Promise<AuthLoginResponse> {
  return postJson<AuthLoginResponse>(apiPath('auth_login', '/api/v1/auth/login'), { key })
}

export async function checkAuth(): Promise<AuthCheckResponse> {
  return fetchJson<AuthCheckResponse>(apiPath('auth_check', '/api/v1/auth/check'))
}
