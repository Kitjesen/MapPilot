// Centralized API service layer for LingTu web dashboard
// All fetch() calls in one place.

import type {
  AlgorithmBenchmarkLatestResponse,
  AppBootstrapResponse,
  AppCapabilitiesResponse,
  AppTrafficResponse,
  AuthCheckResponse,
  AuthLoginResponse,
  BagOperationResponse,
  BagStatusResponse,
  ClientLinks,
  CommandReceipt,
  ControlCommandResponse,
  DevicesResponse,
  DynamicFilterResult,
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
  PathResponse,
  PlanPreviewRequest,
  PlanPreviewResponse,
  ProductFieldCheckRequest,
  ProductFieldCheckResponse,
  ProductModeProfile,
  ReadinessResponse,
  RealRuntimeEvidenceLatestResponse,
  RoutecheckLatestResponse,
  RuntimeDataflowResponse,
  RuntimeDataflowSubscribeRequest,
  RuntimeDataflowSubscribeResponse,
  RuntimeDataflowTopicDetailResponse,
  RuntimeSwitchRequest,
  RuntimeSwitchResponse,
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

const WEB_CLIENT_ID = 'web-dashboard'

type CommandResponse = ControlCommandResponse | LeaseResponse

interface RestartSlamOptions {
  currentProfile?: string | null
  targetProfile?: ProductModeProfile
  mapName?: string | null
}

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

function inspectionRouteStartPath(routeId: string): string {
  const encoded = encodeURIComponent(routeId)
  const template = apiPath('inspection_route_start', '/api/v1/inspection/routes/{route_id}/start')
  return template.includes('{route_id}')
    ? template.replace('{route_id}', encoded)
    : `/api/v1/inspection/routes/${encoded}/start`
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
  const template = apiPath(linkName, fallback)
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
  request: RuntimeSwitchPlanRequest = {},
): Promise<RuntimeSwitchPlanResponse> {
  return postJson<RuntimeSwitchPlanResponse>(
    apiPath('runtime_switch_plan', '/api/v1/runtime/switch-plan'),
    {
      target_profile: 'explore',
      ...request,
    },
  )
}

export async function runRuntimeSwitch(
  request: RuntimeSwitchRequest,
): Promise<RuntimeSwitchResponse> {
  return postJson<RuntimeSwitchResponse>(
    apiPath('runtime_switch', '/api/v1/runtime/switch'),
    {
      relocalize: true,
      strategy: 'auto',
      execute: false,
      allow_restart: false,
      client_id: WEB_CLIENT_ID,
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

export async function startInspectionRoute(
  routeId: string,
  options: { map_id?: string | null; revision?: number | null } = {},
): Promise<InspectionCommandResponse> {
  return postJson<InspectionCommandResponse>(
    inspectionRouteStartPath(routeId),
    {
      map_id: options.map_id ?? null,
      revision: options.revision ?? 0,
      request_id: makeRequestId('inspection_start'),
    },
  )
}

export async function pauseInspectionRun(
  mapId?: string | null,
): Promise<InspectionCommandResponse> {
  return postJson<InspectionCommandResponse>(
    apiPath('inspection_pause', '/api/v1/inspection/run/pause'),
    {
      map_id: mapId ?? null,
      reason: 'operator_pause',
      request_id: makeRequestId('inspection_pause'),
    },
  )
}

export async function resumeInspectionRun(
  mapId?: string | null,
): Promise<InspectionCommandResponse> {
  return postJson<InspectionCommandResponse>(
    apiPath('inspection_resume', '/api/v1/inspection/run/resume'),
    {
      map_id: mapId ?? null,
      reason: 'operator_resume',
      request_id: makeRequestId('inspection_resume'),
    },
  )
}

export async function cancelInspectionRun(
  mapId?: string | null,
): Promise<InspectionCommandResponse> {
  return postJson<InspectionCommandResponse>(
    apiPath('inspection_cancel', '/api/v1/inspection/run/cancel'),
    {
      map_id: mapId ?? null,
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

export async function sendGoal(
  x: number,
  y: number,
  options: SendGoalOptions = {},
): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('goal', '/api/v1/goal'),
    commandBody('goal', { x, y, ...options }),
  )
}

export async function navigateClick(
  x: number,
  y: number,
  options: SendGoalOptions = {},
): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('navigate_click', '/api/v1/navigate/click'),
    commandBody('navigate_click', { x, y, ...options }),
  )
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

export async function cancelNavigation(reason = 'client_cancel'): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    apiPath('navigation_cancel', '/api/v1/navigation/cancel'),
    commandBody('navigation_cancel', { reason }),
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

export async function restartSlam(options: RestartSlamOptions = {}): Promise<SlamOperationResponse> {
  const res = await fetch(apiPath('slam_restart', '/api/v1/slam/restart'), { method: 'POST' })
  if (res.status === 404) {
    if (!clientLinks.runtime_switch) {
      throw new Error('Gateway does not expose the localization restart endpoint yet. Deploy the updated Gateway and retry.')
    }
    const targetProfile = options.targetProfile ?? (options.mapName ? 'nav' : 'map')
    const switched = await runRuntimeSwitch({
      current_profile: options.currentProfile ?? null,
      target_profile: targetProfile,
      map_name: targetProfile === 'nav' ? options.mapName ?? null : null,
      relocalize: targetProfile === 'nav',
      strategy: 'cold',
      execute: true,
      allow_restart: true,
    })
    if (!switched.ok && !switched.accepted) {
      const blockers = switched.blockers?.filter(Boolean).join('；')
      throw new Error(blockers || switched.error || switched.status || 'Runtime restart request was not accepted')
    }
    return {
      schema_version: 1,
      ok: true,
      success: true,
      profile: 'native_dds',
      message: switched.accepted
        ? 'Localization restart accepted'
        : 'Localization pipeline restarted',
      ts: Date.now() / 1000,
      accepted: switched.accepted,
      status: switched.status,
      command_id: switched.command_id,
    }
  }
  return readSlamOperation(res)
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
  const res = await fetch(apiPath('map_lifecycle', '/api/v1/maps'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'delete', name }),
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
  currentProfile?: string | null
  mapName?: string | null
  initialPose?: [number, number, number] | null
  endpoint?: string | null
}

const PRODUCT_PROFILE_BY_SESSION_MODE: Record<SessionMode, ProductModeProfile> = {
  mapping: 'map',
  navigating: 'nav',
  exploring: 'tare_explore',
}

export async function switchProductSession(
  mode: SessionMode,
  options: ProductSessionSwitchOptions = {},
): Promise<RuntimeSwitchResponse> {
  const mapName = options.mapName?.trim() || null
  if (mode === 'navigating' && !mapName) {
    throw new Error('Navigation product switch requires a saved map')
  }
  const response = await runRuntimeSwitch({
    current_profile: options.currentProfile ?? null,
    target_profile: PRODUCT_PROFILE_BY_SESSION_MODE[mode],
    target_endpoint: options.endpoint ?? 'thunder_field',
    endpoint: options.endpoint ?? 'thunder_field',
    map_name: mode === 'navigating' ? mapName : null,
    relocalize: mode === 'navigating',
    initial_pose: mode === 'navigating' ? options.initialPose : null,
    strategy: 'cold',
    execute: true,
    allow_restart: true,
  })
  if (!response.ok || !response.accepted) {
    const blocker = response.blockers?.filter(Boolean).join('; ')
    throw new Error(blocker || response.error || response.status || 'Product mode switch was rejected')
  }
  return response
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

export async function fetchBagStatus(): Promise<BagStatusResponse> {
  return fetchJson<BagStatusResponse>(apiPath('bag_status', '/api/v1/bag/status'))
}

export async function startBagRecording(
  duration = 600,
  prefix = 'web',
): Promise<BagOperationResponse> {
  return postJson<BagOperationResponse>(apiPath('bag_start', '/api/v1/bag/start'), { duration, prefix })
}

export async function stopBagRecording(): Promise<BagOperationResponse> {
  return postJson<BagOperationResponse>(apiPath('bag_stop', '/api/v1/bag/stop'))
}

// --- Auth ---

export async function login(key: string): Promise<AuthLoginResponse> {
  return postJson<AuthLoginResponse>(apiPath('auth_login', '/api/v1/auth/login'), { key })
}

export async function checkAuth(): Promise<AuthCheckResponse> {
  return fetchJson<AuthCheckResponse>(apiPath('auth_check', '/api/v1/auth/check'))
}
