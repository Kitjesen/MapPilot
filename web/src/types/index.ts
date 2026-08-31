// Shared TypeScript interfaces for LingTu web dashboard

export interface MapInfo {
  name: string
  has_pcd: boolean
  has_occupancy?: boolean
  has_octomap?: boolean
  activation_ready?: boolean
  state?: string | null
  is_active: boolean
  size_mb?: number
  patch_count?: number
}

export interface MapListResponse {
  schema_version: number
  maps: MapInfo[]
  count: number
  active: string
  map_dir: string
  ts: number
}

export interface MapPointsResponse {
  schema_version: number
  protocol_version?: number | null
  count: number
  layout: 'flat_xyz' | 'xyz_rows'
  frame_id: string
  epoch?: number | null
  sequence?: number | null
  stamp_s?: number | null
  stream_kind?: 'cloud' | 'map' | 'scan' | 'reset' | null
  source: string
  name?: string | null
  content_epoch?: number | null
  points: number[] | Array<[number, number, number]>
  bounds?: Record<string, number[]> | null
  ts: number
}

export interface ServerInfo {
  api_version: string
  time: number
}

export interface EndpointSpec {
  method: string
  path: string
  operation_id?: string | null
  request_schema?: string | null
  response_schema?: string | null
  response_content_types: string[]
  status_codes: string[]
}

export interface SceneGraphObject {
  id?: string | null
  label: string
  x?: number | null
  y?: number | null
  z?: number | null
  confidence?: number | null
  distance?: number | null
  bbox?: unknown
  metadata: Record<string, unknown>
}

export interface SceneGraphRelation {
  source?: string | null
  target?: string | null
  relation?: string | null
  confidence?: number | null
  metadata: Record<string, unknown>
}

export interface SceneGraphRegion {
  id?: string | null
  name?: string | null
  label?: string | null
  x?: number | null
  y?: number | null
  z?: number | null
  polygon?: unknown
  metadata: Record<string, unknown>
}

export interface SceneGraphResponse {
  schema_version: number
  frame_id: string
  ts?: number | null
  objects: SceneGraphObject[]
  relations: SceneGraphRelation[]
  regions: SceneGraphRegion[]
  count: number
  scene_graph?: unknown
}

export interface LocationEntry {
  name: string
  x: number
  y: number
  z: number
  yaw?: number | null
  tags: string[]
  source?: string | null
  ts?: number | null
  map_id?: string | null
  map_content_epoch?: number | null
  frame_id?: string | null
  metadata?: Record<string, unknown>
}

export interface LocationsResponse {
  schema_version: number
  locations: LocationEntry[]
  count: number
  frame_id: string
  ts?: number | null
  source: string
}

export interface LocationUpsertRequest {
  name: string
  x?: number | null
  y?: number | null
  z?: number
  yaw?: number | null
  tags?: string[]
  source?: string
  metadata?: Record<string, unknown>
  use_current_pose?: boolean
  request_id?: string | null
  client_id?: string
}

export interface LocationOperationResponse {
  schema_version: number
  ok: boolean
  status: 'saved' | 'deleted' | 'not_found' | 'unavailable' | 'invalid' | 'error'
  action: 'create' | 'update' | 'delete'
  location?: LocationEntry | null
  locations: LocationsResponse
  message?: string | null
  error?: string | null
  request_id?: string | null
  client_id: string
  ts: number
}

export interface StateResponse {
  schema_version: number
  ts: number
  server?: ServerInfo
  lease?: Record<string, unknown> | null
  teleop?: Record<string, unknown> | null
  session?: SessionEvent['data'] | Record<string, unknown> | null
  localization?: Record<string, unknown> | null
  navigation?: NavigationStatusResponse | null
  visual_servo?: VisualServoStatus | null
  map?: Record<string, unknown> | null
  scene?: Record<string, unknown> | null
  path?: Record<string, unknown> | null
  media?: AppMediaLinks | null
  links?: Record<string, string>
}

export interface HealthResponse {
  status: string
  modules_ok: number
  modules_fail: number
  gateway: Record<string, unknown>
  teleop: {
    active: boolean
    clients: number
  }
  sensors: Record<string, unknown>
  slam_hz: number
  map_points: number
  has_odom: boolean
  modules: Record<string, string>
  brainstem: Record<string, unknown>
  [key: string]: unknown
}

export interface ReadinessModuleStatus {
  ok: boolean
  detail?: Record<string, unknown> | null
  error?: string | null
}

export interface ReadinessResponse {
  schema_version: number
  status: string
  ready: boolean
  data_ready: boolean
  motion_ready: boolean
  non_motion_safe: boolean
  modules: Record<string, ReadinessModuleStatus>
  module_count: number
  failed_modules: string[]
  reasons: string[]
  advisories?: string[]
  runtime: Record<string, unknown>
  ts: number
}

export interface RoutecheckPhaseSummary {
  selected_planner?: string | null
  planner?: string | null
  outcome?: string | null
  ok?: boolean | null
  feasible?: boolean | null
  fallback_reason?: string | null
  error?: string | null
  reasons?: string[]
  [key: string]: unknown
}

export interface RoutecheckSummary {
  schema_version?: number
  mode?: string | null
  outcome?: string | null
  non_motion?: boolean | null
  phases?: Record<string, RoutecheckPhaseSummary>
  generated_at?: string | null
  ts?: number | null
  [key: string]: unknown
}

export interface RoutecheckLatestResponse {
  schema_version: number
  ok: boolean
  artifacts_root: string
  count: number
  artifact_dir?: string | null
  summary_path?: string | null
  latest?: RoutecheckSummary | null
  reason?: string | null
  ts: number
}

export interface RealRuntimeEvidenceLatestResponse {
  schema_version?: string | number
  ok?: boolean
  runtime_evidence_ok?: boolean
  report_age_s?: number | null
  max_age_s?: number | null
  runtime_contract?: string | null
  simulation_only?: boolean | null
  real_robot_motion?: boolean | null
  cmd_vel_sent_to_hardware?: boolean | null
  reason?: string | null
  blockers?: string[]
  ts?: number | null
  [key: string]: unknown
}

export interface AuthLoginResponse {
  ok: boolean
  message?: string | null
  [key: string]: unknown
}

export interface AuthCheckResponse {
  auth_required: boolean
  [key: string]: unknown
}

export interface RobotPoseSummary {
  x: number
  y: number
  z: number
  yaw?: number | null
  vx?: number | null
  vy?: number | null
  wz?: number | null
  frame_id?: string | null
  ts?: number | null
}

export interface PathResponse {
  schema_version: number
  path: PathPoint[]
  robot: RobotPoseSummary | null
  count: number
  frame_id: string
  ts?: number | null
  source: string
}

export interface NativeLocalPlannerCandidate {
  rotation_index?: number
  rotation_deg?: number
  group_id?: number
  state?: string
  dominant_state?: string
  selected?: boolean
  score?: number
  terrain_risk?: number
  path?: number[][]
}

export interface NativeLocalPlannerCandidates {
  valid?: boolean
  frame_id?: string
  timestamp_s?: number
  candidates?: NativeLocalPlannerCandidate[]
}

export interface NativeLocalTraversabilityDebug {
  rows?: number
  cols?: number
  resolution_m?: number
  origin_xy?: number[]
  fresh?: boolean
  cells_total?: number
  risk_cells_total?: number
  risk_cells_returned?: number
  complete?: boolean
  truncated?: boolean
  sampling?: string
  default_cost?: number | null
  unreported_cells?: 'zero_cost' | 'not_serialized' | string
  risk_cells?: number[][]
}

export interface NativeLocalMapDebug {
  enabled?: boolean
  frame_id?: string
  obstacle_points_fresh?: boolean
  obstacle_points?: number[][]
  traversability?: NativeLocalTraversabilityDebug
}

export interface NativeNavigationEndpointStatus extends Record<string, unknown> {
  stamp_s?: number
  local_map?: NativeLocalMapDebug
  local_candidates?: NativeLocalPlannerCandidates
}

export interface NavigationDdsSnapshotResponse {
  schema_version: 'lingtu.navigation.dds_snapshot.v1'
  global_path: PathResponse
  local_path: PathResponse
  cmd_vel: {
    frame_id?: string
    linear?: Record<string, number>
    angular?: Record<string, number>
    active_source?: string
    ts?: number | null
  } | null
  nav_endpoint: NativeNavigationEndpointStatus | null
  traversability_endpoint: Record<string, unknown> | null
  navigation: Record<string, unknown>
  ts: number
  source: string
}

export interface PlanPreviewRequest {
  x: number
  y: number
  z?: number
  frame_id?: 'map'
}

export interface PlanPreviewResponse {
  schema_version: number
  ok: boolean
  feasible: boolean
  frame_id: string
  start: PathPoint | null
  goal: PathPoint
  path: PathPoint[]
  count: number
  distance_m?: number | null
  plan_ms?: number | null
  planner?: string | null
  source: string
  reasons: string[]
  error?: string | null
  ts: number
}

export type GoalSource =
  | 'coordinate'
  | 'map_click'
  | 'saved_location'
  | 'semantic'
  | 'frontier'
  | 'api'

export type GoalTargetType =
  | 'coordinate'
  | 'map_point'
  | 'saved_location'
  | 'semantic_target'
  | 'frontier'

export interface GoalCandidateRequest {
  x?: number | null
  y?: number | null
  z?: number
  yaw?: number | null
  frame_id?: 'map'
  source?: GoalSource
  target_type?: GoalTargetType
  label?: string | null
  location_name?: string | null
  acceptance_radius_m?: number | null
  max_speed_mps?: number | null
  metadata?: Record<string, unknown>
  preview?: boolean
}

export interface ConstructedGoalTarget {
  schema_version: number
  x: number
  y: number
  z: number
  yaw: number
  frame_id: string
  source: string
  target_type: string
  label?: string | null
  location_name?: string | null
  acceptance_radius_m?: number | null
  max_speed_mps?: number | null
  metadata: Record<string, unknown>
  ts?: number | null
}

export interface GoalCandidateResponse {
  schema_version: number
  ok: boolean
  status: string
  target?: ConstructedGoalTarget | null
  preview?: PlanPreviewResponse | null
  reasons: string[]
  error?: string | null
  ts: number
}

export type InspectionAcceptanceMode = 'non_motion' | 'simulation' | 'field'

export type InspectionFailurePolicy = 'stop' | 'retry' | 'skip'

export interface InspectionRoutePoint {
  id: string
  x: number
  y: number
  z: number
  yaw?: number | null
  tolerance: number
  dwell: number
  action: string
  enabled: boolean
}

export interface InspectionRoute {
  id: string
  name?: string | null
  map_id?: string | null
  map_content_epoch?: number | null
  revision?: number | null
  point_count?: number | null
  points: InspectionRoutePoint[]
  loop_count?: number | null
  failure_policy?: InspectionFailurePolicy | string | null
  max_retries?: number | null
  [key: string]: unknown
}

export interface InspectionRouteRequest {
  id: string
  name?: string | null
  map_id: string
  map_content_epoch: number
  revision: number
  points: InspectionRoutePoint[]
  loop_count: number
  failure_policy: InspectionFailurePolicy
  max_retries: number
}

export interface InspectionRouteResponse {
  schema_version: 'lingtu.inspection.v1'
  ok: boolean
  route: InspectionRoute
  ts: number
}

export interface InspectionRouteListResponse {
  schema_version: 'lingtu.inspection.v1'
  ok: boolean
  map_id: string
  routes: InspectionRoute[]
  count: number
  ts: number
}

export interface InspectionCommandResponse {
  schema_version: 'lingtu.inspection.v1'
  ok: boolean
  accepted: boolean
  action: 'start' | 'pause' | 'resume' | 'cancel' | 'delete'
  route_id?: string | null
  map_id?: string | null
  revision?: number | null
  request_id?: string | null
  ts: number
}

export interface InspectionTaskCommandResponse {
  schema_version: 'lingtu.inspection.task.v1'
  ok: boolean
  accepted: boolean
  action: 'start' | 'pause' | 'resume' | 'cancel'
  task_id: string
  request_id: string
  route_id?: string | null
  map_id?: string | null
  revision?: number | null
  lifecycle: 'submission_accepted'
  terminal: false
  ts: number
}

export interface InspectionTaskProgress {
  known: boolean
  completed_points: number
  point_count: number
  current_point_number?: number | null
  current_point_id: string
  loop_number?: number | null
  retry_count: number
  action: string
  evidence_id: string
}

export interface InspectionTaskStatusResponse {
  schema_version: 'lingtu.inspection.task.v1'
  found: boolean
  task_id: string
  current_state: string
  state_source: 'native_task_event' | 'business_ack_only' | 'continuity_monitor' | 'none' | string
  execution_confirmed: boolean
  terminal: boolean
  terminal_source: string
  reason: string
  progress: InspectionTaskProgress
  available_actions: Array<'pause' | 'resume' | 'cancel' | string>
  can_pause: boolean
  can_resume: boolean
  can_cancel: boolean
  identity: {
    task_id: string
    route_id?: string | null
    map_id?: string | null
    map_content_epoch?: number | null
    route_revision?: number | null
  }
  last_submission?: Record<string, unknown> | null
  latest_event?: Record<string, unknown> | null
  timeline: Array<Record<string, unknown>>
  delivery: {
    continuity?: string
    history_complete?: boolean
    reason?: string
    retention?: string
    boot_id?: string
    event_sequence?: number
  }
  updated_at: number
}

export interface InspectionTaskListResponse {
  schema_version: 'lingtu.inspection.task.v1'
  retention: 'process_local_gateway_projection' | 'durable_gateway_projection'
  count: number
  tasks: InspectionTaskStatusResponse[]
  ts: number
}

export interface InspectionTaskReportPoint {
  loop_index: number
  point_index: number
  point_id: string
  action: string
  status:
    | 'PENDING'
    | 'IN_PROGRESS'
    | 'COMPLETED'
    | 'MISSING_EVIDENCE'
    | 'INVALID_EVIDENCE'
    | 'UNAVAILABLE_EVIDENCE'
    | 'UNKNOWN'
  evidence_status:
    | 'NOT_REQUIRED'
    | 'PENDING'
    | 'VERIFIED'
    | 'MISSING'
    | 'INVALID'
    | 'UNAVAILABLE'
    | 'UNKNOWN'
  evidence_id: string
  reason: string
}

export interface InspectionTaskReportResponse {
  schema_version: 'lingtu.inspection.report.v1'
  task_id: string
  report_status:
    | 'IN_PROGRESS'
    | 'COMPLETE'
    | 'PARTIAL'
    | 'FAILED'
    | 'CANCELLED'
    | 'UNKNOWN'
  acceptance: 'PENDING' | 'ACCEPTABLE' | 'REVIEW_REQUIRED' | 'NOT_ACCEPTABLE' | 'UNKNOWN'
  terminal: boolean
  execution: {
    state: string
    terminal: boolean
    confirmed: boolean
    reason: string
    history_complete: boolean
    history_reason: string
  }
  identity: {
    route_id: string
    route_revision: number
    map_id: string
    map_content_epoch: number
  }
  coverage: {
    required_points: number
    completed_points: number
    required_evidence: number
    verified_evidence: number
    missing_evidence: number
    invalid_evidence: number
    unavailable_evidence: number
    unknown_evidence: number
  }
  points: InspectionTaskReportPoint[]
  issues: Array<{
    code: string
    reason?: string
    loop_index?: number
    point_index?: number
    point_id?: string
    action?: string
    evidence_id?: string
  }>
}

export interface InspectionEvidenceWorkerStatus {
  ready: boolean
  reason?: string | null
  supported_actions: string[]
  heartbeat_ts?: number | null
  heartbeat_age_s?: number | null
  max_heartbeat_age_s?: number | null
  worker_id?: string | null
  status?: string | null
  state?: string | null
  readiness_reason?: string | null
  analyzers?: Record<string, string>
  last_error?: string | null
  error?: string | null
  [key: string]: unknown
}

export interface InspectionEvidenceArtifact {
  kind: 'rgb' | 'pose' | 'detections' | string
  media_type?: string | null
  bytes?: number | null
}

export interface InspectionEvidenceSummary {
  evidence_id: string
  request: {
    run_id?: string
    route_id?: string
    route_revision?: number
    map_id?: string
    map_content_epoch?: number
    point_id?: string
    point_index?: number
    request_id?: string
    action?: string
    requested_at_s?: number
    deadline_s?: number
    [key: string]: unknown
  }
  analysis: {
    support?: string
    verdict?: string
    reason?: string
    [key: string]: unknown
  }
  persistence?: Record<string, unknown>
  artifacts: InspectionEvidenceArtifact[]
}

export interface InspectionEvidenceListResponse {
  schema_version: 'lingtu.inspection.evidence.list.v1'
  ok: boolean
  count: number
  limit: number
  integrity_failures: number
  evidence: InspectionEvidenceSummary[]
  ts: number
}

export interface InspectionEvidenceDetailResponse {
  schema_version: 'lingtu.inspection.evidence.detail.v1'
  ok: boolean
  evidence: InspectionEvidenceSummary
  ts: number
}

export interface InspectionStatusResponse {
  schema_version: 'lingtu.inspection.v1'
  ok: boolean
  status: Record<string, unknown> & {
    evidence_worker?: InspectionEvidenceWorkerStatus
  }
  ts: number
}

export interface ProductFieldCheckRequest {
  mode?: InspectionAcceptanceMode
  map_dir?: string | null
  require_occupancy?: boolean
  expected_data_source?: string | null
  expected_source_profile?: string | null
  expected_frame_id?: string | null
}

export interface ProductFieldCheckResponse {
  schema_version: string
  ok: boolean
  mode: string
  summary: string
  map: Record<string, unknown>
  runtime: Record<string, unknown>
  navigation: Record<string, unknown>
  evidence: Record<string, unknown>
  blockers: string[]
  advisories: string[]
}

export interface NavigationPathSummary {
  points: number
  endpoint: string
}

export interface NavigationControlActiveSource {
  name: string
  label: string
  category: string
  owner: string
  priority?: number | null
  active?: boolean | null
  age_ms?: number | null
}

export interface NavigationControlSummary {
  mode: string
  lease: Record<string, unknown>
  active_cmd_source: string
  command_owner: string
  source_category: string
  manual_override: boolean
  autonomy_requested: boolean
  preempting_autonomy: boolean
  active_source: NavigationControlActiveSource
  sources: Record<string, unknown>
  [key: string]: unknown
}

export interface NavigationLocalizationSummary {
  state?: string | null
  ready?: boolean | null
  degraded: boolean
  algorithm_healthy?: boolean | null
  pose_fresh?: boolean | null
  pose_freshness?: string | null
  degeneracy?: string | null
  speed_scale?: number | null
  reasons: string[]
}

export interface NavigationReadinessSummary {
  can_accept_goal: boolean
  can_execute_autonomy: boolean
  blockers: string[]
  advisories: string[]
  localization_ready: boolean
  control_owner: string
  session_mode?: string | null
}

export interface NavigationProgressSummary {
  wp_index: number
  wp_total: number
  fraction: number
  path_points: number
  replan_count: number
  active: boolean
  terminal: boolean
}

export interface NavigationFrameMismatch {
  source: string
  expected_frame: string
  received_frame: string
}

export interface NavigationFrameSummary {
  planning_frame_id: string
  odom_frame_id: string
  costmap_frame_id: string
  goal_frame_id?: string | null
  ok: boolean
  mismatches: NavigationFrameMismatch[]
}

export interface NavigationDiagnosticsSummary {
  reason_codes: string[]
  failure_reason: string
  localization_reasons: string[]
  frame_mismatches: NavigationFrameMismatch[]
  safety?: Record<string, unknown> | null
  plan_safety_policy?: string | null
  last_plan_report?: Record<string, unknown>
}

export interface NavigationMissionSummary {
  state: string
  raw: Record<string, unknown>
}

export interface NavigationTargetSummary {
  goal?: PathPoint | null
  current_waypoint?: PathPoint | null
  distance_to_goal_m?: number | null
  active_waypoint_distance_m?: number | null
  remaining_waypoints?: number | null
}

export type NavigationSpeedPolicyMode = 'normal' | 'cautious' | 'restricted' | 'hold' | 'unknown'

export interface NavigationSpeedPolicy {
  scale?: number | null
  mode: NavigationSpeedPolicyMode
  reason?: string | null
  source: string
  applied?: boolean | null
}

export interface NavigationMotionSummary {
  current_speed_mps?: number | null
  speed_scale?: number | null
  speed_policy: NavigationSpeedPolicy
  active_cmd_source: string
  command_owner: string
}

export interface NavigationFeedbackSummary {
  next_action: string
  primary: string
  blockers: string[]
  advisories: string[]
  reason_codes: string[]
}

export interface NavigationStatusResponse {
  schema_version: number
  state: string
  has_odometry: boolean
  can_accept_goal: boolean
  wp_index: number
  wp_total: number
  replan_count: number
  speed_scale?: number | null
  failure_reason: string
  reason_codes: string[]
  readiness: NavigationReadinessSummary
  progress: NavigationProgressSummary
  path: NavigationPathSummary
  frames: NavigationFrameSummary
  control: NavigationControlSummary
  localization: NavigationLocalizationSummary
  target: NavigationTargetSummary
  motion: NavigationMotionSummary
  feedback: NavigationFeedbackSummary
  diagnostics: NavigationDiagnosticsSummary
  mission: NavigationMissionSummary
  ts: number
}

/** Explicit TARE direction intent. This is never a navigation goal. */
export interface DirectedExplorationTargetRequest {
  x: number
  y: number
  ttl_s?: number
  reason?: string
  request_id?: string | null
}

export interface DirectedExplorationIntent {
  active: boolean
  x?: number | null
  y?: number | null
  ttl_s?: number | null
  session_id: string
  frame_id: string
  reason: string
  request_id?: string | null
}

export interface DirectedExplorationResponse {
  schema_version: number
  ok: true
  accepted: boolean
  status: 'accepted' | 'cleared'
  intent: DirectedExplorationIntent
  native: Record<string, unknown>
}

export interface ExplorationStatusResponse {
  available: boolean
  backend: 'none' | 'frontier' | 'tare' | string
  exploring: boolean
  frontier_count: number
  can_start: boolean
  blockers: string[]
  advisories: string[]
  navigation: Record<string, unknown>
  reason?: string | null
  required_profile?: string | null
  supported_profiles?: string[] | null
  action?: string | null
  tare?: Record<string, unknown> | null
  supervisor?: Record<string, unknown> | null
}

export interface ClientLinks {
  bootstrap?: string
  capabilities?: string
  traffic?: string
  state?: string
  scene_graph?: string
  locations?: string
  location_detail?: string
  path?: string
  localization_status?: string
  navigation_status?: string
  navigation_dds_snapshot?: string
  navigation_goal_status?: string
  navigation_task_status?: string
  runtime_dataflow?: string
  runtime_dataflow_topic?: string
  runtime_dataflow_subscribe?: string
  devices?: string
  readiness?: string
  metrics?: string
  auth_login?: string
  auth_check?: string
  events?: string
  teleop_ws?: string
  camera_ws?: string
  cloud_ws?: string
  scan_ws?: string
  camera_snapshot?: string
  webrtc_whep?: string
  go2rtc_status?: string
  health?: string
  session?: string
  navigation_goal_candidate?: string
  navigation_plan?: string
  inspection_acceptance?: string
  inspection_routes?: string
  inspection_route_detail?: string
  inspection_tasks?: string
  inspection_task_status?: string
  inspection_task_report?: string
  inspection_task_pause?: string
  inspection_task_resume?: string
  inspection_task_cancel?: string
  inspection_status?: string
  navigation_cancel?: string
  navigation_task_cancel?: string
  navigation_task_pause?: string
  navigation_task_resume?: string
  navigation_resume?: string
  goal?: string
  navigate_click?: string
  stop?: string
  estop_reset?: string
  instruction?: string
  visual_servo?: string
  mode?: string
  lease?: string
  maps?: string
  map_delete?: string
  map_rename?: string
  map_save?: string
  map_operations?: string
  map_operation_status?: string
  map_operation_cancel?: string
  map_operation_retry?: string
  map_import_pcd?: string
  map_crop?: string
  map_mark_zone?: string
  map_build_octomap?: string
  map_build_occupancy?: string
  map_validate_plan?: string
  map_cloud_reset?: string
  map_points?: string
  saved_map_points?: string
  explore_status?: string
  explore_directed?: string
  explore_directed_clear?: string
  explore_start?: string
  explore_stop?: string
  slam_status?: string
  localization_relocalize?: string
  localization_map_tracking?: string
  recording_start?: string
  recording_stop?: string
  recording_status?: string
  recording_list?: string
  memory_temporal?: string
  memory_temporal_semantic?: string
  diagnostic_pack?: string
  field_check?: string
  routecheck_latest?: string
  real_runtime_evidence_latest?: string
  runtime_contract?: string
  [key: string]: string | undefined
}

export type CameraMediaRuntimeStatus = 'streaming' | 'idle' | 'stale' | 'error' | 'not_loaded'

export interface CameraPortStatus {
  frames: number
  fps: number
  stale_ms?: number | null
}

export interface CameraInfoStatus {
  frames: number
  active_topic?: string | null
  preferred_topic?: string | null
  topics: string[]
}

export interface CameraJpegStatus {
  cached: boolean
  seq: number
  bytes: number
}

export interface CameraMediaStatus {
  schema_version: number
  available: boolean
  status: CameraMediaRuntimeStatus
  reason?: string | null
  backend?: string | null
  fps: number
  frames: number
  color: CameraPortStatus
  depth: CameraPortStatus
  camera_info: CameraInfoStatus
  reconnect_count: number
  service_recovery_allowed: boolean
  service_recovery_suppressed: boolean
  jpeg: CameraJpegStatus
  ts: number
  error?: string | null
  [key: string]: unknown
}

export interface WHEPMediaStatus {
  supported: boolean
  endpoint: string
  go2rtc_status: string
  [key: string]: unknown
}

export interface AppMediaLinks {
  events: string
  teleop_ws: string
  camera_ws: string
  cloud_ws: string
  scan_ws?: string
  camera_snapshot: string
  webrtc_whep?: string
  go2rtc_status?: string
  camera: CameraMediaStatus
  whep: WHEPMediaStatus
  [key: string]: unknown
}

export interface RealtimeEventsCapability {
  path: string
  transport: 'sse'
  initial_snapshot: boolean
  heartbeat_s: number
  schema_version?: number
  event_schema?: string
  event_id_field?: string
  timestamp_field?: string
  heartbeat_type?: string
  snapshot_type?: string
  event_types?: string[]
  diagnostic_event_types?: string[]
  named_events?: boolean
  browser_handler?: string
  retry_ms?: number
  replay_supported?: boolean
  last_event_id_header?: string
  drop_policy?: string | null
  large_event_policy?: Record<string, unknown>
}

export interface RealtimeTeleopCapability {
  path: string
  transport: 'websocket'
  control_messages: string[]
  binary_camera_frames: boolean
}

export interface RealtimeCameraCapability {
  path: string
  transport: 'websocket'
  binary_camera_frames: boolean
  explicit_subscription: boolean
}

export interface RealtimeCloudCapability {
  path: string
  transport: 'websocket'
  binary_point_cloud_frames: boolean
  drop_policy?: string | null
}

export interface AppRealtimeCapabilities {
  events: RealtimeEventsCapability
  teleop: RealtimeTeleopCapability
  camera: RealtimeCameraCapability
  cloud: RealtimeCloudCapability
  scan?: RealtimeCloudCapability
}

export interface TrafficSSEStats {
  clients: number
  queue_maxsize?: number | null
  queue_depths: number[]
  max_depth_seen: number
  latest_event_id: number
  published_events: number
  dropped_events: number
  suppressed_events: Record<string, number>
  raster_min_interval_s?: number | null
  drop_policy?: string | null
  [key: string]: unknown
}

export interface TrafficCloudStats {
  clients: number
  queue_maxsize?: number | null
  queue_depths: number[]
  max_depth_seen: number
  published_frames: number
  dropped_frames: number
  drop_policy?: string | null
  latest_seq: number
  [key: string]: unknown
}

export interface AppTrafficResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  status: 'ok' | 'degraded'
  sse: TrafficSSEStats
  cloud: TrafficCloudStats
  scan?: TrafficCloudStats
  recommended_client_rates_hz: Record<string, number>
  client_policy: {
    usage: string
    poll_rates_hz: Record<string, number>
    events_endpoint: string
    traffic_endpoint: string
    cloud_endpoint: string
    large_event_policy: Record<string, unknown>
    backpressure: Record<string, unknown>
    [key: string]: unknown
  }
  warnings: string[]
  links: ClientLinks
}

export interface RuntimeDataflowPortSummary {
  module: string
  port: string
  direction: 'in' | 'out'
  type?: string | null
  msg_count: number
  rate_hz: number
  stale_ms?: number | null
  connected?: boolean | null
  callbacks?: number | null
}

export interface RuntimeDataflowObservability {
  observable: boolean
  observable_via: string[]
  module_port_candidates: RuntimeDataflowPortSummary[]
  gateway_channels: Array<Record<string, unknown>>
  live_module_samples: boolean
  has_fresh_module_sample: boolean
  fresh_stale_ms_limit?: number | null
}

export interface RuntimeDataflowCommunication {
  allowed: boolean
  interfaces: Array<Record<string, unknown>>
  arbitrary_publish_supported: boolean
  policy: string
}

export interface RuntimeDataflowTokenEvidence {
  token: string
  kind: string
  observable: boolean
  live: boolean
  reason: string
  module_ports: RuntimeDataflowPortSummary[]
  gateway_channels: Array<Record<string, unknown>>
}

export interface RuntimeDataflowStageEvidence {
  name: string
  owner?: string | null
  frame_role?: string | null
  map_dependency?: string | null
  inputs: string[]
  outputs: string[]
  input_evidence: RuntimeDataflowTokenEvidence[]
  output_evidence: RuntimeDataflowTokenEvidence[]
  observable: boolean
  live: boolean
  status: string
  missing_inputs: string[]
  missing_outputs: string[]
  not_live_inputs: string[]
  not_live_outputs: string[]
}

export interface RuntimeDataflowTopicInspection {
  observable?: boolean
  observation_level?: string
  live?: boolean
  module_stats_available?: boolean
  module_stats?: RuntimeDataflowPortSummary[]
  payload_available?: boolean
  payload_interfaces?: Array<Record<string, unknown>>
  stream_interfaces?: Array<Record<string, unknown>>
  communicate?: boolean
  write_interfaces?: Array<Record<string, unknown>>
  arbitrary_publish_supported?: boolean
  policy?: string
  [key: string]: unknown
}

export interface RuntimeDataflowTopicSummary {
  topic: string
  message_formats: string[]
  default_frame_id?: string | null
  allowed_frame_ids: string[]
  required_for_real_runtime_frame_evidence: boolean
  data_flow_stages: Array<Record<string, unknown>>
  observability: RuntimeDataflowObservability
  communication: RuntimeDataflowCommunication
  inspection: RuntimeDataflowTopicInspection
}

export interface RuntimeDataflowResponse {
  schema_version: number
  ts: number
  runtime_contract?: string | null
  runtime_boundary: RuntimeIdentity
  transport_layers: Record<string, Record<string, unknown>>
  motion_path: Record<string, unknown>
  module_ports: Record<string, unknown>
  topics: RuntimeDataflowTopicSummary[]
  stage_evidence: RuntimeDataflowStageEvidence[]
  control_boundary: {
    arbitrary_publish_supported?: boolean
    policy?: string
    command_interfaces?: Array<Record<string, unknown>>
    [key: string]: unknown
  }
  links: ClientLinks
}

export interface RuntimeDataflowTopicDetailResponse {
  schema_version: number
  ok: boolean
  ts: number
  selector: string
  topic?: RuntimeDataflowTopicSummary | null
  runtime_contract?: string | null
  runtime_boundary: RuntimeIdentity
  inspection: RuntimeDataflowTopicInspection
  control_boundary: RuntimeDataflowResponse['control_boundary']
  available_topics: string[]
  links: ClientLinks
  error?: string | null
}

export interface RuntimeDataflowSubscribeRequest {
  selector: string
  transport?: 'gateway_sse'
  max_rate_hz?: number | null
}

export interface RuntimeDataflowSubscribeResponse {
  schema_version: 'lingtu.runtime_dataflow_subscription.v1'
  ok: boolean
  ts: number
  read_only: boolean
  arbitrary_publish_supported: boolean
  publishes: string[]
  selector: string
  topic?: string | null
  transport: 'gateway_sse'
  stream_url: string
  event_types: string[]
  stream_interfaces: Array<Record<string, unknown>>
  blockers: string[]
  links: ClientLinks
}

export type EnvName = 'real' | 'sim'

export interface RuntimeIdentity {
  env: EnvName
  product: ProductName | null
  state?: string | null
  product_session_id?: string | null
  [key: string]: unknown
}

export type ProductName =
  | 'teleop'
  | 'teleop_avoid'
  | 'map'
  | 'explore'
  | 'nav'
  | 'tracking'
  | 'inspection'

export interface RuntimeProductCapability {
  product: ProductName
  available: boolean
  reason?: string | null
  variants?: string[]
}

export interface RuntimeProductCapabilities {
  env: EnvName
  backend?: string | null
  product_session_id?: string | null
  availability_source?: string | null
  products: Partial<Record<ProductName, RuntimeProductCapability>>
}

export type VisualServoMode = 'find' | 'follow' | 'stop'

export interface VisualServoRequest {
  mode: VisualServoMode
  target?: string | null
  target_id?: string | null
  client_id?: string
  request_id?: string | null
}

export interface VisualServoPersonStatus {
  id: string | null
  position: number[]
  velocity: number[]
  last_seen: number
  confidence: number
}

export interface VisualServoStatus {
  mode: 'idle' | 'find' | 'follow'
  target: string | null
  target_id: string | null
  select: string | null
  follow_available: boolean
  target_visible: boolean
  state: string
  navigation_state: string
  navigation_task_id: string | null
  navigation_reason: string
  goal_rate_hz: number
  robot_position: number[]
  goal_position: number[] | null
  distance_m: number | null
  desired_distance_m: number
  person: VisualServoPersonStatus | null
}

export interface VisualServoStatusEvent {
  type: 'visual_servo_status'
  data: VisualServoStatus
}

export interface AppBootstrapResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  robot: Record<string, unknown>
  session: SessionEvent['data'] | Record<string, unknown>
  mission: Record<string, unknown>
  safety: Record<string, unknown>
  localization: Record<string, unknown>
  navigation: NavigationStatusResponse
  control: Record<string, unknown>
  map: Record<string, unknown>
  scene: { available: boolean; endpoint: string }
  path: { points: number; endpoint: string }
  media: AppMediaLinks
  traffic: Record<string, unknown> & {
    client_policy?: {
      usage: string
      poll_rates_hz: Record<string, number>
      events_endpoint: string
      traffic_endpoint?: string
      cloud_endpoint?: string
      large_event_policy?: Record<string, unknown>
      backpressure?: Record<string, unknown>
    }
  }
  capabilities: Record<string, boolean>
  runtime_products?: RuntimeProductCapabilities
  capabilities_endpoint: string
  links: ClientLinks
}

export interface AppCapabilitiesResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  auth: Record<string, unknown>
  features: Record<string, boolean>
  runtime_products?: RuntimeProductCapabilities
  endpoints: Record<string, Record<string, EndpointSpec>>
  probes: Record<string, EndpointSpec>
  realtime: AppRealtimeCapabilities
  client_policy: Record<string, unknown>
  links: ClientLinks
}

export interface CommandReceipt {
  name: string
  task_id?: string | null
  request_id?: string | null
  native_request_id?: string | null
  client_id: string
  accepted: boolean
  replay: boolean
  ts: number
}

export interface GatewayCommandErrorDetail {
  reason_code: string
  reason?: string | null
  source?: string | null
  path?: string | null
  blockers?: string[]
  advisories?: string[]
  safety?: Record<string, unknown> | null
  preview?: PlanPreviewResponse | Record<string, unknown> | null
  lease?: Record<string, unknown> | null
  state?: string | null
  has_odometry?: boolean | null
  session_mode?: string | null
  localization?: Record<string, unknown> | null
  error?: string | null
  [key: string]: unknown
}

export interface GatewayErrorResponse {
  schema_version?: number
  ok?: false
  error: string
  message?: string | null
  detail?: GatewayCommandErrorDetail | Record<string, unknown> | null
  command?: CommandReceipt
}

export interface ControlCommandResponse {
  schema_version: number
  ok: boolean
  status: string
  command: CommandReceipt
  task_id?: string | null
  request_id?: string | null
  native_request_id?: string | null
  stage?: string | null
  execution_confirmed?: boolean | null
  goal?: number[] | null
  yaw?: number | null
  frame_id?: string | null
  instruction?: string | null
  mode?: string | null
  reason?: string | null
  target?: ConstructedGoalTarget | null
  [key: string]: unknown
}

export interface NavigationGoalStatus {
  task_id: string
  request_id: string
  boot_id?: string
  sequence?: number
  event_sequence?: number
  state?: number
  state_name?: string
  goal_epoch?: number
  reason?: string
  terminal?: boolean
  ts?: number
  [key: string]: unknown
}

export interface NavigationTaskStatusQueryResponse {
  schema_version: number
  found: boolean
  task_id: string
  request_id: string
  status: NavigationGoalStatus | null
  reason: string
  ts: number
}

export type LeaseAction = 'acquire' | 'release' | 'renew'

export interface LeaseRequest {
  action: LeaseAction
  client_id?: string
  request_id?: string | null
  ttl?: number
}

export interface LeaseResponse {
  schema_version: number
  ok: boolean
  status: string
  command: CommandReceipt
  holder?: string | null
  active?: boolean | null
  expires_in?: number | null
}

export interface OdometryEvent {
  type: 'odometry'
  x: number
  y: number
  z?: number | null
  yaw: number
  vx: number
  wz?: number | null
  frame_id?: string | null
  child_frame_id?: string | null
  ts?: number | null
}

export interface MissionStatusEvent {
  type: 'mission_status'
  state: string
  goal: string | null
  progress: number
}

export interface SafetyStateEvent {
  type: 'safety_state'
  estop: boolean
  level: string
}

export interface SceneGraphEvent {
  type: 'scene_graph'
  frame_id?: string | null
  stamp_s?: number | null
  objects: Array<{ id: string; label: string; x: number; y: number; z?: number; confidence: number }>
}

export interface PingEvent {
  type: 'ping'
}

export interface SnapshotEventData {
  odometry?: Record<string, unknown>
  safety?: Record<string, unknown>
  mission?: Record<string, unknown>
  mode?: string
  lease?: Record<string, unknown>
  session?: Record<string, unknown>
  navigation?: NavigationStatusResponse
  visual_servo?: VisualServoStatus | null
}

export interface SnapshotEvent {
  type: 'snapshot'
  data?: SnapshotEventData
}

export interface MissionEvent {
  type: 'mission'
  data?: Record<string, unknown>
}

export interface SafetyEvent {
  type: 'safety'
  data?: Record<string, unknown>
}

export interface NavigationStatusEvent {
  type: 'navigation_status'
  data?: NavigationStatusResponse
}

export interface InspectionTaskEvent {
  type: 'inspection_task_event'
  schema_version?: number
  event_id?: number
  ts?: number
  data?: {
    event_id: string
    task_id: string
    request_id: string
    boot_id: string
    event_sequence: number
    kind: number
    kind_name: string
    state: number
    state_name: string
    terminal: boolean
    map_id: string
    map_content_epoch: number
    route_id: string
    route_revision: number
    point_index: number
    point_count: number
    loop_index: number
    retry_count: number
    point_id: string
    action: string
    action_request_id: string
    evidence_id: string
    reason: string
    ts: number
    [key: string]: unknown
  }
}

export interface LeaseEvent {
  type: 'lease'
  data?: LeaseResponse | Record<string, unknown>
}

export interface LocationEvent {
  type: 'location'
  data?: LocationOperationResponse | Record<string, unknown>
}

export interface LocationsEvent {
  type: 'locations'
  data?: LocationsResponse | Record<string, unknown>
}

export interface CommandAckEvent {
  type: 'command_ack'
  data?: (ControlCommandResponse & { status_code?: number | null; detail?: unknown }) | Record<string, unknown>
}

export interface EvalEvent {
  type: 'eval'
  data?: Record<string, unknown>
}

export interface DialogueEvent {
  type: 'dialogue'
  data?: Record<string, unknown>
}

export interface GnssFusionEvent {
  type: 'gnss_fusion'
  data?: Record<string, unknown>
}

export interface SlamDiagnosticEvent {
  type: 'slam_diag'
  data?: Record<string, unknown>
}

export interface SlamDriftEvent {
  type: 'slam_drift'
  level?: string
  xy?: number
  v?: number
  action?: string
  count?: number
  dump_path?: string
  data?: Record<string, unknown>
}

export interface TareStatsEvent {
  type: 'tare_stats'
  data?: Record<string, unknown>
}

export interface ExplorationSupervisorEvent {
  type: 'exploration_supervisor'
  data?: Record<string, unknown>
}

export interface ExploringEvent {
  type: 'exploring'
  active?: boolean
}

export interface SlamStatusEvent {
  type: 'slam_status'
  slam_hz: number
  mode: string
  map_points: number
  degeneracy_count: number
}

export interface RobotStatusEvent {
  type: 'robot_status'
  battery: number
  temperature: number
}

export interface PathPoint {
  x: number
  y: number
  z: number
  yaw?: number | null
  frame_id?: string | null
  ts?: number | null
  metadata?: Record<string, unknown>
}

export interface GlobalPathEvent {
  type: 'global_path'
  points: PathPoint[]
  frame_id?: string | null
  stamp_s?: number | null
  receive_sequence?: number | null
}

export interface LocalPathEvent {
  type: 'local_path'
  points: PathPoint[]
  frame_id?: string | null
  stamp_s?: number | null
  receive_sequence?: number | null
}

export interface MapCloudEvent {
  type: 'map_cloud'
  points?: number[]  // flat [x,y,z, x,y,z, ...] when streamed inline
  count: number
  seq?: number
  bytes?: number
  source?: string
  clean_layer?: boolean
  reset?: boolean
}

export interface MapScenePaletteEntry {
  color?: string
  name?: string
}

export interface MapSceneLayer {
  id?: string
  type?: string
  layer_type?: string
  source?: string
  topic?: string
  frame_id?: string
  ts?: number
  point_count?: number
  has_labels?: boolean
  label_count?: number
  labels?: number[]
  confidence?: number[]
  palette?: Record<string, MapScenePaletteEntry>
  taxonomy?: string | null
  taxonomy_version?: string | number | null
  metadata?: Record<string, unknown>
  [key: string]: unknown
}

export interface ElevationMapSceneLayer extends MapSceneLayer {
  id: string
  type: 'grid'
  frame_id: string
  producer_boot_id: string
  stamp_s: number
  generation: number
  reset_epoch: number
  observation_sequence: number
  live: boolean
  grid_b64: string
  rows: number
  cols: number
  resolution: number
  origin: [number, number, number]
  yaw: number
  encoding: 'float32_le'
  valid_count: number
  min_z: number | null
  max_z: number | null
  downsample_factor: number
  value_semantics: 'min_observed_z_not_ground'
  payload_retained?: boolean
  payload_generation?: number
  payload_observation_sequence?: number
  payload_stamp_s?: number
  retained_for_generation?: number
}

export interface MapSceneEvent {
  type: 'map_scene'
  schema_version?: string | number
  ts?: number
  source?: string
  frame_id?: string
  map_id?: string | null
  sequence?: number
  metadata?: Record<string, unknown>
  layers: MapSceneLayer[]
  consumed_pointcloud_layers?: number
}

export interface SessionEvent {
  type: 'session'
  data: {
    mode: 'idle' | 'mapping' | 'navigating' | 'exploring'
    env: EnvName
    product?: ProductName | null
    product_session_id?: string | null
    slam_profile?: string | null
    localization_backend?: string | null
    health_source?: string | null
    active_map: string | null
    saved_active_map?: string | null
    map_has_pcd: boolean
    map_has_octomap?: boolean
    since: number
    icp_quality: number
    localizer_ready: boolean
    localizer_algorithm_healthy?: boolean
    pose_fresh?: boolean | null
    pose_freshness?: string | null
    map_state?: string | null
    map_save_supported?: boolean
    map_save_source?: string | null
    relocalization_supported?: boolean
    saved_map_relocalization_supported?: boolean
    restart_recovery_supported?: boolean
    recovery_method?: string | null
    relocalization_state?: string | null
    recovery_signal?: string | null
    recovery_action?: string | null
    explorer_available: boolean
    explorer_unavailable_reason?: string | null
    explorer_required_product?: string | null
  }
}

export interface DynamicFilterResult {
  success: boolean
  orig_count?: number
  clean_count?: number
  dropped?: number
  elapsed_s?: number
  error?: string
  skipped?: boolean
  [key: string]: unknown
}

export interface MapLifecycleResponse {
  schema_version: number
  ok: boolean
  success?: boolean | null
  message?: string | null
  name?: string | null
  active?: string | null
  old_name?: string | null
  new_name?: string | null
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
  errors?: unknown[] | null
  dynamic_filter?: DynamicFilterResult | null
  ts: number
  [key: string]: unknown
}

export type NativeTraversabilityValueSemantics = 'control_risk_0_100'

export interface NativeTraversabilityEvent {
  type: 'native_traversability'
  grid_b64: string
  rows: number
  cols: number
  resolution: number
  origin: [number, number, number]
  yaw: number
  frame_id: string
  stamp_s: number
  reset_epoch: number
  sequence: number
  source: 'native_nav_client'
  control_authority: true
  value_semantics: NativeTraversabilityValueSemantics
  identity_verified: true
}

export interface AgentMessageEvent {
  type: 'agent_message'
  role: 'thinking' | 'assistant' | 'tool'
  text: string
  phase?: string
  ts: number
}

export interface SSEEnvelopeFields {
  schema_version?: number
  event_id?: number
  ts?: number
}

export type SSEEvent = SSEEnvelopeFields & (
  | OdometryEvent
  | MissionStatusEvent
  | SafetyStateEvent
  | SceneGraphEvent
  | PingEvent
  | SnapshotEvent
  | MissionEvent
  | SafetyEvent
  | NavigationStatusEvent
  | InspectionTaskEvent
  | LeaseEvent
  | LocationEvent
  | LocationsEvent
  | CommandAckEvent
  | EvalEvent
  | DialogueEvent
  | GnssFusionEvent
  | SlamDiagnosticEvent
  | SlamDriftEvent
  | TareStatsEvent
  | ExplorationSupervisorEvent
  | ExploringEvent
  | SlamStatusEvent
  | RobotStatusEvent
  | GlobalPathEvent
  | MapCloudEvent
  | MapSceneEvent
  | SessionEvent
  | NativeTraversabilityEvent
  | AgentMessageEvent
  | VisualServoStatusEvent
  | LocalPathEvent
)

export interface SSEState {
  odometry: OdometryEvent | null
  missionStatus: MissionStatusEvent | null
  safetyState: SafetyStateEvent | null
  sceneGraph: SceneGraphEvent | null
  slamStatus: SlamStatusEvent | null
  robotStatus: RobotStatusEvent | null
  globalPath: GlobalPathEvent | null
  localPath: LocalPathEvent | null
  mapCloud: MapCloudEvent | null
  mapScene: MapSceneEvent | null
  session: SessionEvent['data'] | null
  navigationStatus: NavigationStatusResponse | null
  inspectionTaskEvent: InspectionTaskEvent | null
  lease: LeaseResponse | Record<string, unknown> | null
  commandAck: CommandAckEvent['data'] | null
  locations: LocationsResponse | null
  stateSnapshot: StateResponse | null
  traffic: AppTrafficResponse | null
  nativeTraversability: NativeTraversabilityEvent | null
  agentMessage: AgentMessageEvent | null  // latest agent chat message (ts dedups)
  visualServoStatus: VisualServoStatus | null
  gnssFusion: GnssFusionEvent | null
  slamDiag: SlamDiagnosticEvent | null
  slamDrift: SlamDriftEvent | null
  tareStats: TareStatsEvent | null
  explorationSupervisor: ExplorationSupervisorEvent | null
  exploring: ExploringEvent | null
  evalEvent: EvalEvent | null
  dialogue: DialogueEvent | null
  lastHeartbeat: number | null
  lastEventId: number | null
  missedEvents: number
  reconnects: number
  lastError: string | null
  lastRefreshAt: number | null
  lastRefreshReason: string | null
  refreshError: string | null
  authoritativeStateSeen: boolean
  lastTruthAt: number | null
  truthError: string | null
  connected: boolean
  events: SSEEvent[]
}

export type ToastKind = 'success' | 'error' | 'info'

export interface Toast {
  id: number
  message: string
  kind: ToastKind
}

export type Tab = 'console' | 'scene' | 'map' | 'slam' | 'dataflow' | 'inspection' | 'planner'

export type SlamRuntimeProfile = 'none' | 'native_dds'
export interface SlamServiceDetail {
  status: string
  canonical_unit?: string
  selected_unit?: string
  installed_units?: string[]
  active_units?: string[]
  candidate_units?: string[]
  [key: string]: unknown
}

export interface SlamServiceMetadata {
  role: string
  group: string
  product_default: boolean
  experimental: boolean
  description?: string
  [key: string]: unknown
}

export interface SlamStatusResponse {
  mode: string
  native_mode?: string | null
  services: Record<string, string>
  service_details: Record<string, SlamServiceDetail>
  service_groups: Record<string, string[]>
  service_metadata: Record<string, SlamServiceMetadata>
  product_runtime: string
  manual_systemctl_required: boolean
  [key: string]: unknown
}

export interface RecordingStatusResponse {
  available?: boolean
  healthy?: boolean
  backend?: string
  state?: 'idle' | 'preparing' | 'recording' | 'stopping' | 'completed' | 'failed' | string
  recording: boolean
  session_id?: string | null
  path?: string | null
  duration_s: number
  size_bytes: number
  size_truncated?: boolean
  pid?: number | null
  exit_code?: number | null
  disk_free: number
  disk_total: number
  error?: string | null
}

export interface LocalizationOperationResponse {
  schema_version: number
  ok: boolean
  success: boolean
  map_name: string
  mode: 'seeded' | 'global' | 'tracking'
  request_id?: string | null
  message?: string | null
  activation_ready?: boolean | null
  quality?: number | null
  details?: Record<string, unknown>
  ts: number
  [key: string]: unknown
}

export interface RecordingOperationResponse {
  status?: string | null
  state?: string | null
  backend?: string | null
  session_id?: string | null
  path?: string | null
  pid?: number | null
  duration?: number | null
  prefix?: string | null
  capture_profile?: 'sensors' | 'evidence' | string | null
  camera?: boolean | null
  minimum_free_gib?: number | null
  error?: string | null
  detail?: unknown
}

/** Safe, product-level recording choices. Raw DDS and device arguments stay native-only. */
export interface RecordingStartConfig {
  duration?: number
  prefix?: string
  capture_profile?: 'sensors' | 'evidence'
  task_id?: string
  camera?: boolean
  minimum_free_gib?: number
}

export interface RecordingSessionSummary {
  session_id: string
  state: string
  manager_pid?: number | null
}

export interface RecordingListResponse {
  ok: boolean
  sessions: RecordingSessionSummary[]
  truncated: boolean
  disk_free: number
  disk_total: number
}

export interface RecordingArtifact {
  path: string
  download: string
}

export interface RecordingDetailResponse {
  ok: boolean
  session: {
    version?: number
    session_id: string
    state: string
    created_at_unix_ns?: number
    started_at_unix_ns?: number
    ended_at_unix_ns?: number
    context?: Record<string, unknown>
    error?: string | null
    children: Array<{
      name?: string
      required?: boolean
      artifacts: string[]
      selected_topics: string[]
    }>
    artifacts: RecordingArtifact[]
  }
}
