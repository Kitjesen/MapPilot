#!/usr/bin/env node

import { readFile } from 'node:fs/promises'
import { resolve } from 'node:path'

const root = resolve(import.meta.dirname, '..')

async function text(path) {
  return readFile(resolve(root, path), 'utf8')
}

function ensure(condition, failures, message) {
  if (!condition) failures.push(message)
}

function has(source, needle) {
  return source.includes(needle)
}

function functionBody(source, name) {
  const start = source.indexOf(`export async function ${name}(`)
  if (start < 0) return ''
  const next = source.indexOf('\nexport ', start + 1)
  return source.slice(start, next < 0 ? source.length : next)
}

const [
  packageJson,
  app,
  topbar,
  tabbar,
  api,
  types,
  view,
  inspectionView,
  smoke,
  readme,
] = await Promise.all([
  text('package.json'),
  text('src/App.tsx'),
  text('src/components/Topbar.tsx'),
  text('src/components/TabBar.tsx'),
  text('src/services/api.ts'),
  text('src/types/index.ts'),
  text('src/components/RuntimeDataflowView.tsx'),
  text('src/components/InspectionAcceptanceView.tsx'),
  text('scripts/gateway-smoke.mjs'),
  text('README.md'),
])

const failures = []

ensure(
  has(packageJson, '"smoke:dataflow-ui": "node scripts/dataflow-ui-contract.mjs"'),
  failures,
  'package.json must expose smoke:dataflow-ui',
)

ensure(has(types, "runtime_dataflow?: string"), failures, 'ClientLinks must type runtime_dataflow')
ensure(has(types, "runtime_dataflow_topic?: string"), failures, 'ClientLinks must type runtime_dataflow_topic')
ensure(has(types, "runtime_dataflow_subscribe?: string"), failures, 'ClientLinks must type runtime_dataflow_subscribe')
ensure(has(types, "runtime_switch_plan?: string"), failures, 'ClientLinks must type runtime_switch_plan')
ensure(has(types, "runtime_switch?: string"), failures, 'ClientLinks must type runtime_switch')
ensure(has(types, "algorithm_benchmark_latest?: string"), failures, 'ClientLinks must type algorithm_benchmark_latest')
ensure(has(types, "field_check?: string"), failures, 'ClientLinks must type field_check')
ensure(has(types, "real_runtime_evidence_latest?: string"), failures, 'ClientLinks must type real_runtime_evidence_latest')
ensure(has(types, "inspection_acceptance?: string"), failures, 'ClientLinks must type inspection_acceptance')
ensure(has(types, "export type Tab = 'console' | 'scene' | 'map' | 'slam' | 'dataflow' | 'inspection'"), failures, 'Tab union must include dataflow and inspection')
ensure(has(types, 'export interface RuntimeDataflowResponse'), failures, 'RuntimeDataflowResponse type missing')
ensure(has(types, 'export interface RuntimeDataflowTopicDetailResponse'), failures, 'RuntimeDataflowTopicDetailResponse type missing')
ensure(has(types, 'export interface RuntimeDataflowSubscribeRequest'), failures, 'RuntimeDataflowSubscribeRequest type missing')
ensure(has(types, 'export interface RuntimeDataflowSubscribeResponse'), failures, 'RuntimeDataflowSubscribeResponse type missing')
ensure(has(types, 'export interface RuntimeDataflowStageEvidence'), failures, 'RuntimeDataflowStageEvidence type missing')
ensure(has(types, 'export interface RuntimeSwitchPlanRequest'), failures, 'RuntimeSwitchPlanRequest type missing')
ensure(has(types, 'export interface RuntimeSwitchPlanResponse'), failures, 'RuntimeSwitchPlanResponse type missing')
ensure(has(types, 'export interface RuntimeSwitchRequest'), failures, 'RuntimeSwitchRequest type missing')
ensure(has(types, 'export interface RuntimeSwitchResponse'), failures, 'RuntimeSwitchResponse type missing')
ensure(has(types, 'export interface AlgorithmBenchmarkLatestResponse'), failures, 'AlgorithmBenchmarkLatestResponse type missing')
ensure(has(types, 'export interface RealRuntimeEvidenceLatestResponse'), failures, 'RealRuntimeEvidenceLatestResponse type missing')
ensure(has(types, 'export interface ProductFieldCheckResponse'), failures, 'ProductFieldCheckResponse type missing')
ensure(has(types, 'export interface InspectionAcceptanceResponse'), failures, 'InspectionAcceptanceResponse type missing')
ensure(has(types, 'export interface InspectionAcceptanceTargetResult'), failures, 'InspectionAcceptanceTargetResult type missing')
ensure(has(types, 'frontier_preview: Record<string, unknown>'), failures, 'Field/inspection responses must type frontier_preview evidence')
ensure(has(types, 'runtime_switch: Record<string, unknown>'), failures, 'Product field response must type runtime_switch evidence')
ensure(has(types, 'stage_evidence: RuntimeDataflowStageEvidence[]'), failures, 'RuntimeDataflowResponse must type stage_evidence')
ensure(has(types, 'points?: string[]'), failures, 'InspectionAcceptanceRequest points must be saved-location names only')
ensure(!has(types, 'points?: unknown[]'), failures, 'InspectionAcceptanceRequest must not expose arbitrary point payloads')

ensure(has(app, "import { RuntimeDataflowView }"), failures, 'App must import RuntimeDataflowView')
ensure(has(app, "import { InspectionAcceptanceView }"), failures, 'App must import InspectionAcceptanceView')
ensure(has(app, "activeTab === 'dataflow'"), failures, 'App must render dataflow tab panel')
ensure(has(app, "activeTab === 'inspection'"), failures, 'App must render inspection tab panel')
ensure(has(topbar, "key: 'dataflow'"), failures, 'Topbar must expose dataflow tab')
ensure(has(topbar, "key: 'inspection'"), failures, 'Topbar must expose inspection tab')
ensure(has(tabbar, "key: 'dataflow'"), failures, 'TabBar must stay in sync with dataflow tab')
ensure(has(tabbar, "key: 'inspection'"), failures, 'TabBar must stay in sync with inspection tab')

const summaryApi = functionBody(api, 'fetchRuntimeDataflow')
const detailApi = functionBody(api, 'fetchRuntimeDataflowTopic')
const subscribeApi = functionBody(api, 'subscribeRuntimeDataflow')
const evidenceApi = functionBody(api, 'fetchRealRuntimeEvidenceLatest')
const algorithmApi = functionBody(api, 'fetchAlgorithmBenchmarkLatest')
const fieldCheckApi = functionBody(api, 'runProductFieldCheck')
const switchPlanApi = functionBody(api, 'runRuntimeSwitchPlan')
const switchApi = functionBody(api, 'runRuntimeSwitch')
const inspectionApi = functionBody(api, 'runInspectionAcceptance')
ensure(has(summaryApi, "apiPath('runtime_dataflow'"), failures, 'fetchRuntimeDataflow must use bootstrap runtime_dataflow link')
ensure(has(detailApi, "apiPath('runtime_dataflow_topic'"), failures, 'fetchRuntimeDataflowTopic must use bootstrap runtime_dataflow_topic link')
ensure(has(subscribeApi, "apiPath('runtime_dataflow_subscribe'"), failures, 'subscribeRuntimeDataflow must use bootstrap runtime_dataflow_subscribe link')
ensure(has(subscribeApi, "transport: 'gateway_sse'"), failures, 'subscribeRuntimeDataflow must default to Gateway SSE')
ensure(has(evidenceApi, "apiPath('real_runtime_evidence_latest'"), failures, 'fetchRealRuntimeEvidenceLatest must use bootstrap real_runtime_evidence_latest link')
ensure(has(algorithmApi, "apiPath('algorithm_benchmark_latest'"), failures, 'fetchAlgorithmBenchmarkLatest must use bootstrap algorithm_benchmark_latest link')
ensure(has(fieldCheckApi, "apiPath('field_check'"), failures, 'runProductFieldCheck must use bootstrap field_check link')
ensure(has(switchPlanApi, "apiPath('runtime_switch_plan'"), failures, 'runRuntimeSwitchPlan must use bootstrap runtime_switch_plan link')
ensure(has(switchApi, "apiPath('runtime_switch'"), failures, 'runRuntimeSwitch must use bootstrap runtime_switch link')
ensure(has(switchApi, 'allow_restart: false'), failures, 'runRuntimeSwitch must default to plan-only execution')
ensure(has(switchApi, 'client_id: WEB_CLIENT_ID'), failures, 'runRuntimeSwitch must identify the web client')
ensure(has(inspectionApi, "apiPath('inspection_acceptance'"), failures, 'runInspectionAcceptance must use bootstrap inspection_acceptance link')
ensure(has(inspectionApi, "client_id: WEB_CLIENT_ID"), failures, 'runInspectionAcceptance must identify the web client')
ensure(has(detailApi, 'new URLSearchParams({ topic })'), failures, 'fetchRuntimeDataflowTopic must URL encode topic selector')
ensure(!/postJson|method:\s*['\"]POST['\"]/.test(summaryApi + detailApi + evidenceApi + algorithmApi), failures, 'runtime dataflow API wrappers must stay read-only GET')

ensure(has(view, 'fetchRuntimeDataflow()'), failures, 'RuntimeDataflowView must load summary through API wrapper')
ensure(has(view, 'fetchRuntimeDataflowTopic(topic)'), failures, 'RuntimeDataflowView must load detail through API wrapper')
ensure(has(view, 'subscribeRuntimeDataflow({ selector: selected'), failures, 'RuntimeDataflowView must request read-only stream subscription plans for the selected stream')
ensure(has(view, 'new EventSource(subscriptionPlan.stream_url)'), failures, 'RuntimeDataflowView must open the selected Gateway SSE stream from the subscription plan')
ensure(has(view, 'event_types'), failures, 'RuntimeDataflowView must display selected stream SSE event types')
ensure(has(view, 'Realtime Stream'), failures, 'RuntimeDataflowView must expose live selected-stream evidence')
ensure(has(view, 'isRealRuntimeBoundary'), failures, 'RuntimeDataflowView must derive field-check mode from runtime boundary')
ensure(has(view, "const fieldCheckMode = realRuntime ? 'field' : 'simulation'"), failures, 'RuntimeDataflowView must switch product check mode for real runtime')
ensure(has(view, 'runProductFieldCheck({ mode: fieldCheckMode })'), failures, 'RuntimeDataflowView must load backend product check verdict for current runtime')
ensure(!has(view, "runProductFieldCheck({ mode: 'simulation' })"), failures, 'RuntimeDataflowView must not hard-code simulation product check mode')
ensure(has(view, 'runtimeProfile(next'), failures, 'RuntimeDataflowView must derive switch-plan current profile from runtime boundary')
ensure(has(view, 'current_endpoint: currentEndpoint'), failures, 'RuntimeDataflowView must pass the current runtime endpoint into switch-plan')
ensure(has(view, 'target_endpoint: targetEndpoint'), failures, 'RuntimeDataflowView must pass the target runtime endpoint into switch-plan')
ensure(has(view, 'runRuntimeSwitchPlan({'), failures, 'RuntimeDataflowView must load runtime switch-plan through API wrapper')
ensure(!has(view, 'fetchRoutecheckLatest()'), failures, 'RuntimeDataflowView must not recompute field verdict from routecheck')
ensure(!has(view, 'fetchRealRuntimeEvidenceLatest()'), failures, 'RuntimeDataflowView must not recompute field verdict from raw real runtime evidence')
ensure(!has(view, 'fieldChecks'), failures, 'RuntimeDataflowView must not keep local field verdict checks')
ensure(!has(view, 'routecheckOk('), failures, 'RuntimeDataflowView must not define routecheckOk')
ensure(!has(view, 'realEvidenceOk('), failures, 'RuntimeDataflowView must not define realEvidenceOk')
ensure(!has(view, 'stageEvidenceOk('), failures, 'RuntimeDataflowView must not define stageEvidenceOk')
ensure(!has(view, 'commandSinkOk('), failures, 'RuntimeDataflowView must not define commandSinkOk')
ensure(!has(view, 'fetch('), failures, 'RuntimeDataflowView must not bypass centralized API layer')
ensure(!/postJson|method:\s*['\"]POST['\"]|sendGoal|sendStop|cancelNavigation|sendInstruction/.test(view), failures, 'RuntimeDataflowView must not contain state-changing command calls')
ensure(!/<input|<textarea|contentEditable/.test(view), failures, 'RuntimeDataflowView must not expose arbitrary payload entry controls')
ensure(!/\/api\/v1\/publish|\/runtime\/dataflow\/publish|\/ros\//.test(view), failures, 'RuntimeDataflowView must not expose publish/ROS browser routes')

ensure(has(view, 'Gateway + ModulePorts'), failures, 'RuntimeDataflowView must lead with Gateway + ModulePorts')
ensure(has(view, 'Runtime Dataflow'), failures, 'RuntimeDataflowView must expose runtime dataflow scope')
ensure(has(view, 'Product Check'), failures, 'RuntimeDataflowView must expose product-check status without implying simulation-only scope')
ensure(has(view, 'Route Preview'), failures, 'RuntimeDataflowView must expose route preview readiness')
ensure(has(view, 'Real S100P Evidence'), failures, 'RuntimeDataflowView must expose real S100P evidence readiness')
ensure(has(view, 'Algorithm Benchmark'), failures, 'RuntimeDataflowView must expose algorithm benchmark readiness')
ensure(has(view, 'Algorithm Claim'), failures, 'RuntimeDataflowView must expose backend algorithm claim')
ensure(has(view, 'Command Boundary'), failures, 'RuntimeDataflowView must expose command boundary readiness')
ensure(has(view, 'Frontier Preview'), failures, 'RuntimeDataflowView must expose traversable frontier preview readiness')
ensure(has(view, 'Runtime Switch'), failures, 'RuntimeDataflowView must expose runtime switch-plan readiness')
ensure(has(view, 'dry-run preflight'), failures, 'RuntimeDataflowView must label switch-plan as dry-run preflight')
ensure(has(view, 'not a ROS2 topic browser'), failures, 'RuntimeDataflowView must state it is not a ROS2 topic browser')
ensure(has(view, 'ROS2 Required'), failures, 'RuntimeDataflowView must display ROS2-required boundary')
ensure(has(view, 'Arbitrary Publish'), failures, 'RuntimeDataflowView must display arbitrary publish boundary')
ensure(has(view, 'arbitrary_publish_supported='), failures, 'RuntimeDataflowView must show detail arbitrary_publish_supported')
ensure(has(view, 'WHITELIST CMD'), failures, 'RuntimeDataflowView must show whitelist command boundary')
ensure(has(view, 'Stage Evidence'), failures, 'RuntimeDataflowView must show stage evidence')
ensure(has(view, 'stage_evidence'), failures, 'RuntimeDataflowView must surface stage_evidence gaps')
ensure(has(view, 'SSE'), failures, 'RuntimeDataflowView must show realtime/SSE health')
ensure(has(view, 'traffic'), failures, 'RuntimeDataflowView must show Gateway traffic health')

ensure(has(inspectionView, 'runInspectionAcceptance({'), failures, 'InspectionAcceptanceView must load backend acceptance result')
ensure(has(inspectionView, 'selectedLocationNames'), failures, 'InspectionAcceptanceView must support saved-location multi-select')
ensure(has(inspectionView, 'selectedTag'), failures, 'InspectionAcceptanceView must support saved-location tag/group selection')
ensure(has(inspectionView, 'availableTags'), failures, 'InspectionAcceptanceView must derive tag/group choices from saved locations')
ensure(has(inspectionView, 'toggleLocation'), failures, 'InspectionAcceptanceView must toggle saved inspection locations')
ensure(has(inspectionView, 'points: selectedLocationNames.length > 0 ? selectedLocationNames : undefined'), failures, 'InspectionAcceptanceView must send selected saved locations to backend acceptance')
ensure(has(inspectionView, 'tag: selectedLocationNames.length === 0 ? selectedTag || undefined : undefined'), failures, 'InspectionAcceptanceView must send selected saved-location tag only when explicit points are not selected')
ensure(has(inspectionView, 'location.tags.includes(selectedTag)'), failures, 'InspectionAcceptanceView must filter locations by saved tag membership')
ensure(!has(inspectionView, 'void runCheck()\n  }, [runCheck])'), failures, 'InspectionAcceptanceView must not auto-run acceptance on every location toggle')
ensure(!has(inspectionView, '.slice(0, 12).map'), failures, 'InspectionAcceptanceView must not hide saved locations behind a fixed first-12 cap')
ensure(has(inspectionView, "useState<InspectionAcceptanceMode>('simulation')"), failures, 'InspectionAcceptanceView must default to simulation acceptance mode')
ensure(has(inspectionView, "{ key: 'simulation', label: 'Simulation' }"), failures, 'InspectionAcceptanceView must expose simulation acceptance mode')
ensure(has(inspectionView, 'fetchLocations()'), failures, 'InspectionAcceptanceView must load locations through API wrapper')
ensure(!has(inspectionView, 'fetch('), failures, 'InspectionAcceptanceView must not bypass centralized API layer')
ensure(!/sendGoal|sendStop|cancelNavigation|sendInstruction|constructGoalCandidate|previewNavigationPlan/.test(inspectionView), failures, 'InspectionAcceptanceView must not contain motion command calls')
ensure(!/<input|<textarea|contentEditable/.test(inspectionView), failures, 'InspectionAcceptanceView must not expose arbitrary payload entry controls')
ensure(has(inspectionView, 'report?.summary'), failures, 'InspectionAcceptanceView must display backend summary')
ensure(has(inspectionView, 'target.status'), failures, 'InspectionAcceptanceView must display backend target status')
ensure(has(inspectionView, 'command_published'), failures, 'InspectionAcceptanceView must display command_published boundary')
ensure(has(inspectionView, 'ROS2 Required'), failures, 'InspectionAcceptanceView must show ROS2-required boundary')
ensure(has(inspectionView, '<span>Algorithm</span>'), failures, 'InspectionAcceptanceView must show backend algorithm verdict')
ensure(has(inspectionView, '<span>Frontier</span>'), failures, 'InspectionAcceptanceView must show traversable frontier preview verdict')
ensure(has(inspectionView, '<span>Runtime Switch</span>'), failures, 'InspectionAcceptanceView must show runtime switch-plan verdict')
ensure(has(inspectionView, 'motion=false'), failures, 'InspectionAcceptanceView must show runtime switch-plan is non-motion')
ensure(has(inspectionView, 'command_published=false'), failures, 'InspectionAcceptanceView must show frontier preview is read-only')

ensure(has(smoke, "'runtime_dataflow'"), failures, 'Gateway smoke must require runtime_dataflow link')
ensure(has(smoke, "'runtime_dataflow_topic'"), failures, 'Gateway smoke must require runtime_dataflow_topic link')
ensure(has(smoke, "'runtime_dataflow_subscribe'"), failures, 'Gateway smoke must require runtime_dataflow_subscribe link')
ensure(has(smoke, "'runtime_switch_plan'"), failures, 'Gateway smoke must require runtime_switch_plan link')
ensure(has(smoke, "'field_check'"), failures, 'Gateway smoke must require field_check link')
ensure(has(smoke, "'real_runtime_evidence_latest'"), failures, 'Gateway smoke must require real_runtime_evidence_latest link')
ensure(has(smoke, "'algorithm_benchmark_latest'"), failures, 'Gateway smoke must require algorithm_benchmark_latest link')
ensure(has(smoke, "'inspection_acceptance'"), failures, 'Gateway smoke must require inspection_acceptance link')
ensure(has(smoke, 'lingtu.inspection_acceptance.v1'), failures, 'Gateway smoke must verify inspection acceptance schema')
ensure(has(smoke, 'ros2_topic_required === false'), failures, 'Gateway smoke must verify no ROS2 topic requirement')
ensure(has(smoke, 'arbitrary_publish_supported === false'), failures, 'Gateway smoke must verify no arbitrary publish support')
ensure(has(smoke, 'stage_evidence'), failures, 'Gateway smoke must verify runtime stage evidence')
ensure(has(smoke, 'requiredRuntimeTopics'), failures, 'Gateway smoke must verify required runtime topics')
ensure(has(smoke, 'requiredRuntimeStages'), failures, 'Gateway smoke must verify required runtime stages')
ensure(has(smoke, 'runtimeDataflowSubscribePath'), failures, 'Gateway smoke must call runtime_dataflow_subscribe')
ensure(has(smoke, 'lingtu.runtime_dataflow_subscription.v1'), failures, 'Gateway smoke must verify runtime dataflow subscription schema')
ensure(has(smoke, "body: { selector: topic, transport: 'gateway_sse' }"), failures, 'Gateway smoke must request Gateway SSE subscriptions for required runtime topics')
ensure(has(smoke, 'readFirstSseEvent'), failures, 'Gateway smoke must open the filtered SSE stream')
ensure(has(smoke, "sseEvent.body?.type === 'runtime_dataflow_subscription'"), failures, 'Gateway smoke must verify the filtered SSE subscription first event')
ensure(has(smoke, 'lingtu.runtime_switch_plan.v1'), failures, 'Gateway smoke must verify runtime switch-plan schema')
ensure(has(smoke, 'lingtu.algorithm_benchmark_latest.v1'), failures, 'Gateway smoke must verify algorithm benchmark schema')
ensure(has(smoke, 'field_check algorithm benchmark should be read-only'), failures, 'Gateway smoke must verify field-check algorithm read-only boundary')
ensure(has(smoke, 'field_check algorithm summary_path should match latest benchmark'), failures, 'Gateway smoke must verify field-check algorithm provenance')
ensure(has(smoke, 'dynamic_obstacle_gate'), failures, 'Gateway smoke must verify switch-plan dynamic obstacle stage')

ensure(has(readme, 'The Dataflow tab uses `runtime_dataflow`'), failures, 'README must document Dataflow tab contract')
ensure(has(readme, '`runtime_dataflow_subscribe`'), failures, 'README must document runtime dataflow subscribe contract')
ensure(has(readme, '`runtime_switch_plan`'), failures, 'README must document runtime switch-plan contract')
ensure(has(readme, '`algorithm_benchmark_latest`'), failures, 'README must document algorithm benchmark contract')
ensure(has(readme, 'no arbitrary ModulePort publish'), failures, 'README must document no arbitrary ModulePort publish')
ensure(has(readme, 'no arbitrary ROS topic publish'), failures, 'README must document no arbitrary ROS topic publish')

const result = {
  ok: failures.length === 0,
  checked: {
    package_script: true,
    tab_wiring: true,
    api_links: true,
    readonly_boundary: true,
    smoke_contract: true,
    docs_contract: true,
  },
  failures,
}

console.log(JSON.stringify(result, null, 2))
if (failures.length > 0) {
  process.exitCode = 1
}
