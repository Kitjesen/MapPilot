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
  api,
  types,
  view,
  smoke,
  readme,
] = await Promise.all([
  text('package.json'),
  text('src/App.tsx'),
  text('src/components/Topbar.tsx'),
  text('src/services/api.ts'),
  text('src/types/index.ts'),
  text('src/components/RuntimeDataflowView.tsx'),
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
ensure(!has(types, "runtime_switch?: string"), failures, 'ClientLinks must not expose removed runtime_switch')
ensure(has(types, "field_check?: string"), failures, 'ClientLinks must type field_check')
ensure(has(types, "real_runtime_evidence_latest?: string"), failures, 'ClientLinks must type real_runtime_evidence_latest')
ensure(has(types, "inspection_acceptance?: string"), failures, 'ClientLinks must type inspection_acceptance')
ensure(has(types, "export type Tab = 'console' | 'scene' | 'map' | 'slam' | 'dataflow' | 'inspection'"), failures, 'Tab union must include dataflow and inspection')
ensure(has(types, 'export interface RuntimeDataflowResponse'), failures, 'RuntimeDataflowResponse type missing')
ensure(has(types, 'export interface RuntimeDataflowTopicDetailResponse'), failures, 'RuntimeDataflowTopicDetailResponse type missing')
ensure(has(types, 'export interface RuntimeDataflowSubscribeRequest'), failures, 'RuntimeDataflowSubscribeRequest type missing')
ensure(has(types, 'export interface RuntimeDataflowSubscribeResponse'), failures, 'RuntimeDataflowSubscribeResponse type missing')
ensure(has(types, 'export interface RuntimeDataflowStageEvidence'), failures, 'RuntimeDataflowStageEvidence type missing')
ensure(!has(types, 'export interface RuntimeSwitchRequest'), failures, 'RuntimeSwitchRequest must stay removed')
ensure(!has(types, 'export interface RuntimeSwitchResponse'), failures, 'RuntimeSwitchResponse must stay removed')
ensure(has(types, 'export interface RealRuntimeEvidenceLatestResponse'), failures, 'RealRuntimeEvidenceLatestResponse type missing')
ensure(has(types, 'export interface ProductFieldCheckResponse'), failures, 'ProductFieldCheckResponse type missing')
ensure(has(types, 'stage_evidence: RuntimeDataflowStageEvidence[]'), failures, 'RuntimeDataflowResponse must type stage_evidence')

ensure(has(app, "import { RuntimeDataflowView }"), failures, 'App must import RuntimeDataflowView')
ensure(has(app, "import { InspectionWorkbench }"), failures, 'App must import InspectionWorkbench')
ensure(has(app, "activeTab === 'dataflow'"), failures, 'App must render dataflow tab panel')
ensure(has(app, "activeTab === 'inspection'"), failures, 'App must render inspection tab panel')
ensure(has(topbar, "key: 'dataflow'"), failures, 'Topbar must expose dataflow tab')
ensure(has(topbar, "key: 'inspection'"), failures, 'Topbar must expose inspection tab')

const summaryApi = functionBody(api, 'fetchRuntimeDataflow')
const detailApi = functionBody(api, 'fetchRuntimeDataflowTopic')
const subscribeApi = functionBody(api, 'subscribeRuntimeDataflow')
const evidenceApi = functionBody(api, 'fetchRealRuntimeEvidenceLatest')
const fieldCheckApi = functionBody(api, 'runProductFieldCheck')
ensure(has(summaryApi, "apiPath('runtime_dataflow'"), failures, 'fetchRuntimeDataflow must use bootstrap runtime_dataflow link')
ensure(has(detailApi, "apiPath('runtime_dataflow_topic'"), failures, 'fetchRuntimeDataflowTopic must use bootstrap runtime_dataflow_topic link')
ensure(has(subscribeApi, "apiPath('runtime_dataflow_subscribe'"), failures, 'subscribeRuntimeDataflow must use bootstrap runtime_dataflow_subscribe link')
ensure(has(subscribeApi, "transport: 'gateway_sse'"), failures, 'subscribeRuntimeDataflow must default to Gateway SSE')
ensure(has(evidenceApi, "apiPath('real_runtime_evidence_latest'"), failures, 'fetchRealRuntimeEvidenceLatest must use bootstrap real_runtime_evidence_latest link')
ensure(has(fieldCheckApi, "apiPath('field_check'"), failures, 'runProductFieldCheck must use bootstrap field_check link')
ensure(!has(api, 'runRuntimeSwitch'), failures, 'web API must not expose runtime switch preview')
ensure(has(detailApi, 'new URLSearchParams({ topic })'), failures, 'fetchRuntimeDataflowTopic must URL encode topic selector')
ensure(!/postJson|method:\s*['\"]POST['\"]/.test(summaryApi + detailApi + evidenceApi), failures, 'runtime dataflow API wrappers must stay read-only GET')

ensure(has(view, 'fetchRuntimeDataflow()'), failures, 'RuntimeDataflowView must load summary through API wrapper')
ensure(has(view, 'fetchRuntimeDataflowTopic(topic)'), failures, 'RuntimeDataflowView must load detail through API wrapper')
ensure(has(view, 'subscribeRuntimeDataflow({ selector: selected'), failures, 'RuntimeDataflowView must request read-only stream subscription plans for the selected stream')
ensure(has(view, 'new EventSource(subscriptionPlan.stream_url)'), failures, 'RuntimeDataflowView must open the selected Gateway SSE stream from the subscription plan')
ensure(has(view, 'event_types'), failures, 'RuntimeDataflowView must display selected stream SSE event types')
ensure(has(view, 'Realtime Stream'), failures, 'RuntimeDataflowView must expose live selected-stream evidence')
ensure(has(view, 'isRealRuntimeBoundary'), failures, 'RuntimeDataflowView must derive field-check mode from runtime boundary')
ensure(has(view, "boundary.env === 'real'"), failures, 'RuntimeDataflowView must classify the runtime from Env only')
ensure(has(view, "const fieldCheckMode = realRuntime ? 'field' : 'simulation'"), failures, 'RuntimeDataflowView must switch product check mode for real runtime')
ensure(has(view, 'runProductFieldCheck({ mode: fieldCheckMode })'), failures, 'RuntimeDataflowView must load backend product check verdict for current runtime')
ensure(!has(view, "runProductFieldCheck({ mode: 'simulation' })"), failures, 'RuntimeDataflowView must not hard-code simulation product check mode')
ensure(!/current_endpoint|target_endpoint|\bendpoint\s*:/.test(view), failures, 'RuntimeDataflowView must not expose deployment endpoint selectors')
ensure(!has(view, 'runRuntimeSwitch('), failures, 'RuntimeDataflowView must not preview Product switching')
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
ensure(has(view, 'Readiness'), failures, 'RuntimeDataflowView must expose Product readiness')
ensure(has(view, 'Localization'), failures, 'RuntimeDataflowView must expose localization readiness')
ensure(has(view, 'Observability'), failures, 'RuntimeDataflowView must expose Gateway observability')
ensure(has(view, 'Driver Command'), failures, 'RuntimeDataflowView must expose driver evidence')
ensure(has(view, 'Field Evidence'), failures, 'RuntimeDataflowView must expose real field evidence')
ensure(!has(view, 'strictBenchmark'), failures, 'RuntimeDataflowView must not mirror standalone benchmark state')
ensure(!has(view, 'fieldFrontier'), failures, 'RuntimeDataflowView must not mirror removed frontier state')
ensure(!has(view, 'Runtime Switch'), failures, 'RuntimeDataflowView must not expose removed switch preview')
ensure(!has(view, 'dry-run preflight'), failures, 'RuntimeDataflowView must not label a removed preflight')
ensure(has(view, 'Arbitrary Publish'), failures, 'RuntimeDataflowView must display arbitrary publish boundary')
ensure(has(view, 'arbitrary_publish_supported='), failures, 'RuntimeDataflowView must show detail arbitrary_publish_supported')
ensure(has(view, 'WHITELIST CMD'), failures, 'RuntimeDataflowView must show whitelist command boundary')
ensure(has(view, 'Stage Evidence'), failures, 'RuntimeDataflowView must show stage evidence')
ensure(has(view, 'stage_evidence'), failures, 'RuntimeDataflowView must surface stage_evidence gaps')
ensure(has(view, 'SSE'), failures, 'RuntimeDataflowView must show realtime/SSE health')
ensure(has(view, 'traffic'), failures, 'RuntimeDataflowView must show Gateway traffic health')

ensure(has(smoke, "'runtime_dataflow'"), failures, 'Gateway smoke must require runtime_dataflow link')
ensure(has(smoke, "'runtime_dataflow_topic'"), failures, 'Gateway smoke must require runtime_dataflow_topic link')
ensure(has(smoke, "'runtime_dataflow_subscribe'"), failures, 'Gateway smoke must require runtime_dataflow_subscribe link')
ensure(!has(smoke, "'runtime_switch'"), failures, 'Gateway smoke must not require removed runtime_switch link')
ensure(has(smoke, "'field_check'"), failures, 'Gateway smoke must require field_check link')
ensure(has(smoke, "'real_runtime_evidence_latest'"), failures, 'Gateway smoke must require real_runtime_evidence_latest link')
ensure(has(smoke, "'inspection_acceptance'"), failures, 'Gateway smoke must require inspection_acceptance link')
ensure(has(smoke, 'lingtu.inspection_acceptance.v1'), failures, 'Gateway smoke must verify inspection acceptance schema')
ensure(has(smoke, 'arbitrary_publish_supported === false'), failures, 'Gateway smoke must verify no arbitrary publish support')
ensure(has(smoke, 'stage_evidence'), failures, 'Gateway smoke must verify runtime stage evidence')
ensure(has(smoke, 'requiredRuntimeTopics'), failures, 'Gateway smoke must verify required runtime topics')
ensure(has(smoke, 'requiredRuntimeStages'), failures, 'Gateway smoke must verify required runtime stages')
ensure(has(smoke, 'runtimeDataflowSubscribePath'), failures, 'Gateway smoke must call runtime_dataflow_subscribe')
ensure(has(smoke, 'lingtu.runtime_dataflow_subscription.v1'), failures, 'Gateway smoke must verify runtime dataflow subscription schema')
ensure(has(smoke, "body: { selector: topic, transport: 'gateway_sse' }"), failures, 'Gateway smoke must request Gateway SSE subscriptions for required runtime topics')
ensure(has(smoke, 'readFirstSseEvent'), failures, 'Gateway smoke must open the filtered SSE stream')
ensure(has(smoke, "sseEvent.body?.type === 'runtime_dataflow_subscription'"), failures, 'Gateway smoke must verify the filtered SSE subscription first event')

ensure(has(readme, 'The Dataflow tab uses `runtime_dataflow`'), failures, 'README must document Dataflow tab contract')
ensure(has(readme, '`runtime_dataflow_subscribe`'), failures, 'README must document runtime dataflow subscribe contract')
ensure(!has(readme, '`runtime_switch`'), failures, 'README must not document removed runtime switch contract')
ensure(has(readme, 'no arbitrary ModulePort publish'), failures, 'README must document no arbitrary ModulePort publish')
ensure(has(readme, 'no arbitrary runtime topic publish'), failures, 'README must document no arbitrary runtime topic publish')

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
