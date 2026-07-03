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
  RuntimeSwitchRequest,
  RuntimeSwitchResponse,
  RuntimeSwitchPlanRequest,
  RuntimeSwitchPlanResponse,
  SceneGraphResponse,
  SessionEvent,
  SessionTransitionResponse,
  SlamOperationResponse,
  SlamProfile,
  StateResponse,
} from '../types'

const WEB_CLIENT_ID = 'web-dashboard'

type CommandResponse = ControlCommandResponse | LeaseResponse

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
    clientLinks.map_points ??
    clientLinks.saved_map_points ??
    '/api/v1/maps/{name}/points?max_points=30000'
  const withName = template.includes('{name}')
    ? template.replace('{name}', encoded)
    : `/api/v1/maps/${encoded}/points?max_points=30000`
  const url = new URL(withName, window.location.origin)
  return `${url.pathname}${url.search || '?max_points=30000'}`
}

function locationDetailPath(name: string): string {
  const encoded = encodeURIComponent(name)
  const template = apiPath('location_detail', '/api/v1/locations/{name}')
  return template.includes('{name}')
    ? template.replace('{name}', encoded)
    : `/api/v1/locations/${encoded}`
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
}

export async function saveMap(name: string): Promise<SaveMapResult> {
  // Save can take up to ~2 min on a busy robot because PGO + DUFOMap
  // run synchronously. Default fetch has no timeout which is what we want.
  const res = await fetch(apiPath('map_save', '/api/v1/map/save'), {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
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
  return readMapLifecycle(res)
}

export async function fetchSavedMapPointsResponse(name: string): Promise<MapPointsResponse> {
  return fetchJson<MapPointsResponse>(mapPointsPath(name))
}

export async function fetchSavedMapPoints(name: string): Promise<number[]> {
  const data = await fetchSavedMapPointsResponse(name)
  return flattenPointArray(data.points)
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
