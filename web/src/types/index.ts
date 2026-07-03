// Shared TypeScript interfaces for LingTu web dashboard

export interface MapInfo {
  name: string
  has_pcd: boolean
  has_tomogram: boolean
  has_occupancy?: boolean
  has_octomap?: boolean
  navigation_ready?: boolean
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
  count: number
  layout: 'flat_xyz' | 'xyz_rows'
  frame_id: string
  source: string
  name?: string | null
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
  odometry?: Record<string, unknown> | null
  safety?: Record<string, unknown> | null
  mission?: Record<string, unknown> | null
  eval?: Record<string, unknown> | null
  dialogue?: Record<string, unknown> | null
  mode?: string | null
  lease?: Record<string, unknown> | null
  teleop?: Record<string, unknown> | null
  session?: SessionEvent['data'] | Record<string, unknown> | null
  localization?: Record<string, unknown> | null
  navigation?: NavigationStatusResponse | null
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

export interface DeviceEntry {
  [key: string]: unknown
}

export interface DevicesResponse {
  devices: DeviceEntry[]
  manager: string
  spec_count: number
  opened_count: number
  error?: string | null
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

export interface PlanPreviewRequest {
  x: number
  y: number
  z?: number
  frame_id?: 'map'
  client_id?: string
  planner_constraints?: Record<string, unknown>
}

export interface PlanPreviewResponse {
  schema_version: number
  ok: boolean
  feasible: boolean
  frame_id: string
  start: PathPoint | null
  goal: PathPoint
  adjusted_goal?: PathPoint | null
  path: PathPoint[]
  count: number
  distance_m?: number | null
  plan_ms?: number | null
  planner?: string | null
  selected_planner?: string | null
  reached_goal?: boolean
  global_plan?: Record<string, unknown> | null
  planner_diagnostics?: Record<string, unknown> | null
  plan_safety_policy?: string | null
  path_safety?: Record<string, unknown> | null
  fallback_reason?: string
  rejected_plans?: Record<string, unknown>[]
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
  client_id?: string
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

export interface InspectionAcceptanceRequest {
  mode?: InspectionAcceptanceMode
  points?: string[]
  tag?: string | null
  map_dir?: string | null
  require_tomogram?: boolean
  require_occupancy?: boolean
  expected_data_source?: string | null
  expected_source_profile?: string | null
  expected_frame_id?: string | null
  client_id?: string
}

export interface InspectionAcceptanceTargetResult {
  name: string
  status: string
  ok: boolean
  target_type?: string | null
  source?: string | null
  location_name?: string | null
  preview_feasible: boolean
  preview_count?: number | null
  planner?: string | null
  distance_m?: number | null
  non_motion: boolean
  command_published: boolean
  reasons: string[]
  error?: string | null
}

export interface InspectionAcceptanceResponse {
  schema_version: string
  ok: boolean
  summary: string
  gateway_url?: string | null
  mode: string
  field_ready: boolean
  field_summary: string
  target_count: number
  pass_count: number
  fail_count: number
  locations_count?: number | null
  motion_safety: Record<string, unknown>
  frontier_preview: Record<string, unknown>
  runtime_switch: Record<string, unknown>
  targets: InspectionAcceptanceTargetResult[]
  blockers: string[]
  advisories: string[]
  evidence: Record<string, unknown>
  commands: Record<string, string>
  ts: number
}

export interface ProductFieldCheckRequest {
  mode?: InspectionAcceptanceMode
  map_dir?: string | null
  require_tomogram?: boolean
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
  stage_evidence: Record<string, unknown>
  navigation: Record<string, unknown>
  frontier_preview: Record<string, unknown>
  runtime_switch: Record<string, unknown>
  evidence: Record<string, unknown>
  algorithm: Record<string, unknown>
  blockers: string[]
  advisories: string[]
  commands: Record<string, string>
}

export interface AlgorithmBenchmarkLatestResponse {
  schema_version: 'lingtu.algorithm_benchmark_latest.v1'
  ok: boolean
  read_only: boolean
  ros2_topic_required: boolean
  publishes: string[]
  artifacts_root: string
  count: number
  summary_path?: string | null
  report_mtime?: number | null
  report_age_s?: number | null
  max_age_s: number
  preset: string
  source: string
  summary_schema_version?: string | null
  claim_allowed: boolean
  missing_or_failed: string[]
  required_gate_sequence: string[]
  validation_flow: Record<string, unknown>[]
  claim_boundary: Record<string, unknown>
  blocking_categories: Record<string, string[]>
  blockers: string[]
  reason?: string | null
  latest?: Record<string, unknown> | null
  ts: number
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
  mux_available: boolean
  active_source: NavigationControlActiveSource
  sources: Record<string, unknown>
  cmd_vel_mux: Record<string, unknown>
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
  cmd_vel_mux_available: boolean
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
  runtime_dataflow?: string
  runtime_dataflow_topic?: string
  runtime_dataflow_subscribe?: string
  runtime_switch_plan?: string
  runtime_switch?: string
  algorithm_benchmark_latest?: string
  devices?: string
  readiness?: string
  auth_login?: string
  auth_check?: string
  events?: string
  teleop_ws?: string
  camera_ws?: string
  cloud_ws?: string
  camera_snapshot?: string
  webrtc_stats?: string
  webrtc_offer?: string
  webrtc_bitrate?: string
  webrtc_whep?: string
  go2rtc_status?: string
  health?: string
  session?: string
  session_start?: string
  session_end?: string
  navigation_goal_candidate?: string
  navigation_plan?: string
  inspection_acceptance?: string
  navigation_cancel?: string
  goal?: string
  navigate_click?: string
  stop?: string
  instruction?: string
  mode?: string
  lease?: string
  maps?: string
  map_lifecycle?: string
  map_activate?: string
  map_rename?: string
  map_save?: string
  map_import_pcd?: string
  map_crop?: string
  map_mark_zone?: string
  map_build_octomap?: string
  map_validate_plan?: string
  map_restore_predufo?: string
  map_cloud_reset?: string
  map_points?: string
  saved_map_points?: string
  explore_status?: string
  explore_start?: string
  explore_stop?: string
  slam_status?: string
  slam_switch?: string
  slam_auto_relocalize?: string
  slam_relocalize?: string
  bag_start?: string
  bag_stop?: string
  bag_status?: string
  memory_temporal?: string
  memory_temporal_semantic?: string
  diagnostic_pack?: string
  field_check?: string
  routecheck_latest?: string
  real_runtime_evidence_latest?: string
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
  teleop_stream_clients: number
  ts: number
  error?: string | null
  [key: string]: unknown
}

export interface WebRTCMediaStatus {
  available: boolean
  stats: string
  offer: string
  bitrate: string
  whep: string
  go2rtc_status: string
  [key: string]: unknown
}

export interface AppMediaLinks {
  events: string
  teleop_ws: string
  camera_ws: string
  cloud_ws: string
  camera_snapshot: string
  webrtc_available: boolean
  webrtc_stats?: string
  webrtc_offer?: string
  webrtc_bitrate?: string
  webrtc_whep?: string
  go2rtc_status?: string
  camera: CameraMediaStatus
  webrtc: WebRTCMediaStatus
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
  legacy_event_types?: string[]
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
  legacy_camera_query?: string
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
  slope_grid_inline: boolean
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
  ros2_topic_required: boolean
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
  ros2_topic_required?: boolean
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
  runtime_boundary: Record<string, unknown>
  transport_layers: Record<string, Record<string, unknown>>
  ros2_topic_required: boolean
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
  runtime_boundary: Record<string, unknown>
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
  ros2_topic_required: boolean
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

export interface RuntimeSwitchPlanRequest {
  current_profile?: string | null
  target_profile?: string | null
  current_endpoint?: string | null
  target_endpoint?: string | null
  endpoint?: string | null
}

export interface RuntimeSwitchValidationSummary {
  ok: boolean
  blockers: string[]
  warnings: string[]
}

export interface RuntimeSwitchPlanResponse {
  schema_version: 'lingtu.runtime_switch_plan.v1'
  ok: boolean
  ts: number
  read_only: boolean
  dry_run: boolean
  motion: boolean
  publishes: string[]
  lifecycle: string
  inputs: Record<string, unknown>
  from: Record<string, unknown>
  to: Record<string, unknown>
  changed: string[]
  current_validation: RuntimeSwitchValidationSummary
  target_validation: RuntimeSwitchValidationSummary
  blockers: string[]
  links: ClientLinks
  error?: string | null
}

export type ProductModeProfile =
  | 'teleop'
  | 'teleop_avoid'
  | 'map'
  | 'tracking'
  | 'nav'
  | 'inspection'

export interface RuntimeSwitchRequest {
  current_profile?: string | null
  target_profile: ProductModeProfile
  current_endpoint?: string | null
  target_endpoint?: string | null
  endpoint?: string | null
  map_name?: string | null
  relocalize?: boolean
  initial_pose?: [number, number, number] | null
  allow_restart?: boolean
  client_id?: string
  request_id?: string | null
}

export interface RuntimeSwitchResponse {
  schema_version: 'lingtu.runtime_switch.v1'
  ok: boolean
  ts: number
  accepted: boolean
  status: string
  read_only: boolean
  dry_run: boolean
  motion: boolean
  lifecycle: string
  current_profile?: string | null
  target_profile: string
  map_name?: string | null
  relocalize: boolean
  plan: Record<string, unknown>
  command: string[]
  command_id?: string | null
  pid?: number | null
  log_path?: string | null
  blockers: string[]
  links: ClientLinks
  error?: string | null
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
  capabilities_endpoint: string
  links: ClientLinks
}

export interface AppCapabilitiesResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  auth: Record<string, unknown>
  features: Record<string, boolean>
  endpoints: Record<string, Record<string, EndpointSpec>>
  probes: Record<string, EndpointSpec>
  realtime: AppRealtimeCapabilities
  client_policy: Record<string, unknown>
  links: ClientLinks
}

export interface CommandReceipt {
  name: string
  request_id?: string | null
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
  goal?: number[] | null
  yaw?: number | null
  frame_id?: string | null
  instruction?: string | null
  mode?: string | null
  reason?: string | null
  target?: ConstructedGoalTarget | null
  [key: string]: unknown
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
  yaw: number
  vx: number
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
  objects: Array<{ id: string; label: string; x: number; y: number; confidence: number }>
}

export interface HeartbeatEvent {
  type: 'heartbeat'
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
}

export interface LocalPathEvent {
  type: 'local_path'
  points: PathPoint[]
}

export interface MapCloudEvent {
  type: 'map_cloud'
  points?: number[]  // flat [x,y,z, x,y,z, ...] when streamed inline
  count: number
  seq?: number
  bytes?: number
}

export interface SavedMapEvent {
  type: 'saved_map'  // localizer-refined static map, map frame, the 底图
  points: number[]
  count: number
}

export interface MapLifecycleEvent {
  type: 'map_event'
  data: {
    schema_version: 'map.event'
    event: string
    action?: string
    map_id?: string
    success?: boolean
    message?: string
    record_version?: string
    timestamp?: string
    [key: string]: unknown
  }
}

export interface SessionEvent {
  type: 'session'
  data: {
    mode: 'idle' | 'mapping' | 'navigating' | 'exploring'
    slam_profile?: string | null
    localization_backend?: string | null
    health_source?: string | null
    active_map: string | null
    map_has_pcd: boolean
    map_has_tomogram: boolean
    since: number
    pending: boolean
    error: string
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
    can_start_mapping: boolean
    can_start_navigating: boolean
    can_start_exploring: boolean
    can_end: boolean
    explorer_available: boolean
    explorer_unavailable_reason?: string | null
    explorer_required_profile?: string | null
  }
}

export interface SessionTransitionResponse {
  schema_version: number
  ok: boolean
  success: boolean
  session?: SessionEvent['data'] | null
  message?: string | null
  ts: number
  [key: string]: unknown
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

export interface CostmapEvent {
  type: 'costmap'
  grid_b64: string   // base64-encoded uint8 flat array (0=free, 100=occupied)
  rows: number       // number of rows (Y axis)
  cols: number       // number of columns (X axis)
  resolution: number // meters per cell
  origin: [number, number]  // world [x, y] of bottom-left corner in map frame
  yaw?: number       // rad, map→odom yaw applied to grid orientation (navigating mode)
}

export interface SlopeGridEvent {
  type: 'slope_grid'
  grid_b64?: string  // optional base64-encoded uint8 (0-90 deg mapped to 0-255)
  payload?: 'inline' | 'omitted'
  available?: boolean
  reason?: string
  encoding?: string
  cols: number
  rows?: number
  resolution: number
  origin: [number, number]
  yaw?: number
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
  | HeartbeatEvent
  | PingEvent
  | SnapshotEvent
  | MissionEvent
  | SafetyEvent
  | NavigationStatusEvent
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
  | SavedMapEvent
  | MapLifecycleEvent
  | SessionEvent
  | CostmapEvent
  | SlopeGridEvent
  | AgentMessageEvent
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
  savedMap: SavedMapEvent | null
  mapEvent: MapLifecycleEvent['data'] | null
  session: SessionEvent['data'] | null
  navigationStatus: NavigationStatusResponse | null
  lease: LeaseResponse | Record<string, unknown> | null
  commandAck: CommandAckEvent['data'] | null
  locations: LocationsResponse | null
  stateSnapshot: StateResponse | null
  traffic: AppTrafficResponse | null
  costmap: CostmapEvent | null
  slopeGrid: SlopeGridEvent | null
  agentMessage: AgentMessageEvent | null  // latest agent chat message (ts dedups)
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
  connected: boolean
  events: SSEEvent[]
}

export type ToastKind = 'success' | 'error' | 'info'

export interface Toast {
  id: number
  message: string
  kind: ToastKind
}

export type Tab = 'console' | 'scene' | 'map' | 'slam' | 'planner'

export type SlamProfile = 'fastlio2' | 'localizer' | 'super_lio' | 'super_lio_relocation' | 'stop'

export interface SlamOperationResponse {
  schema_version: number
  ok: boolean
  success: boolean
  profile?: string | null
  message?: string | null
  quality?: number | null
  ts: number
  [key: string]: unknown
}

export interface BagStatusResponse {
  recording: boolean
  path?: string | null
  duration_s: number
  size_bytes: number
  pid?: number | null
  exit_code?: number | null
  disk_free: number
  disk_total: number
}

export interface BagOperationResponse {
  status?: string | null
  path?: string | null
  pid?: number | null
  duration?: number | null
  prefix?: string | null
  error?: string | null
  detail?: unknown
}
