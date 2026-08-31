import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const appSource = readFileSync(
  new URL('../src/App.tsx', import.meta.url),
  'utf8',
)
const apiSource = readFileSync(
  new URL('../src/services/api.ts', import.meta.url),
  'utf8',
)
const workbenchSource = readFileSync(
  new URL('../src/components/InspectionWorkbench.tsx', import.meta.url),
  'utf8',
)
const typesSource = readFileSync(
  new URL('../src/types/index.ts', import.meta.url),
  'utf8',
)
const topbarSource = readFileSync(
  new URL('../src/components/Topbar.tsx', import.meta.url),
  'utf8',
)
const sseSource = readFileSync(
  new URL('../src/hooks/useSSE.ts', import.meta.url),
  'utf8',
)

test('inspection tab uses the product workbench as the run entry', () => {
  assert.match(appSource, /InspectionWorkbench/)
  assert.match(topbarSource, /key: 'inspection', en: 'Inspect'/)
})

test('dashboard emergency stop always reports command acceptance or failure', () => {
  assert.match(appSource, /const response = await api\.sendStop\(\)/)
  assert.match(appSource, /api\.formatCommandAck\(response, '紧急停止'\)/)
  assert.match(appSource, /api\.formatCommandError\(error/)
  assert.doesNotMatch(appSource, /best-effort/)
})

test('inspection API surface exposes one task-addressed lifecycle', () => {
  assert.match(apiSource, /fetchInspectionRoutes/)
  assert.match(apiSource, /fetchInspectionRoute/)
  assert.match(apiSource, /saveInspectionRoute/)
  assert.match(apiSource, /fetchInspectionTasks/)
  assert.match(apiSource, /fetchInspectionTask/)
  assert.match(apiSource, /startInspectionTask/)
  assert.match(apiSource, /pauseInspectionTask/)
  assert.match(apiSource, /resumeInspectionTask/)
  assert.match(apiSource, /cancelInspectionTask/)
  assert.match(apiSource, /inspectionTaskActionPath/)
  assert.match(apiSource, /inspection_task_pause/)
  assert.match(apiSource, /inspection_task_resume/)
  assert.match(apiSource, /inspection_task_cancel/)
  assert.match(apiSource, /fetchInspectionStatus/)
  assert.match(apiSource, /fetchInspectionEvidence/)
  assert.match(apiSource, /inspectionEvidenceArtifactUrl/)
  assert.doesNotMatch(apiSource, /startInspectionRoute/)
  assert.doesNotMatch(apiSource, /pauseInspectionRun/)
  assert.doesNotMatch(apiSource, /resumeInspectionRun/)
  assert.doesNotMatch(apiSource, /cancelInspectionRun/)
})

test('workbench builds routes from saved locations and uses task lifecycle', () => {
  assert.match(workbenchSource, /api\.fetchLocations/)
  assert.match(workbenchSource, /routePointFromLocation/)
  assert.match(workbenchSource, /movePoint/)
  assert.match(workbenchSource, /failurePolicy/)
  assert.match(workbenchSource, /maxRetries/)
  assert.match(workbenchSource, /activeTaskId/)
  assert.match(workbenchSource, /taskStatus/)
  assert.match(workbenchSource, /api\.startInspectionTask/)
  assert.match(workbenchSource, /api\.pauseInspectionTask/)
  assert.match(workbenchSource, /api\.resumeInspectionTask/)
  assert.match(workbenchSource, /api\.cancelInspectionTask/)
  assert.doesNotMatch(workbenchSource, /api\.startInspectionRoute/)
  assert.doesNotMatch(workbenchSource, /api\.pauseInspectionRun/)
  assert.doesNotMatch(workbenchSource, /api\.resumeInspectionRun/)
  assert.doesNotMatch(workbenchSource, /api\.cancelInspectionRun/)
})

test('workbench loads route detail and starts the persisted revision', () => {
  assert.match(workbenchSource, /api\.fetchInspectionRoute/)
  assert.match(workbenchSource, /savedRevision/)
  assert.match(
    workbenchSource,
    /api\.startInspectionTask\(selectedRouteId,[\s\S]*?revision: savedRevision/,
  )
  assert.doesNotMatch(workbenchSource, /action: 'inspect'/)
})

test('workbench exposes native phase evidence and keeps manual release separate from task resume', () => {
  assert.match(workbenchSource, /taskStatus\?\.reason/)
  assert.match(workbenchSource, /taskProgress\?\.evidence_id/)
  assert.match(workbenchSource, /taskProgress\?\.action/)
  assert.match(workbenchSource, /status\.evidence_worker/)
  assert.match(workbenchSource, /Capture readiness/)
  assert.match(workbenchSource, /releaseManualTakeover/)
  assert.match(workbenchSource, /Release manual control/)
  assert.match(workbenchSource, /manualTakeoverReleaseRequired/)
  assert.doesNotMatch(
    workbenchSource,
    /if \(pauseReason\.includes\('operator_takeover'\)\) \{\s*await api\.resumeNavigation\(\)\s*\}\s*return api\.resumeInspectionTask/,
  )
})

test('workbench follows native task events but reads task truth from the task API', () => {
  assert.match(typesSource, /InspectionTaskEvent/)
  assert.match(typesSource, /inspectionTaskEvent/)
  assert.match(sseSource, /case 'inspection_task_event'/)
  assert.match(sseSource, /inspectionTaskEvent/)
  assert.match(workbenchSource, /sseState\.inspectionTaskEvent/)
  assert.match(workbenchSource, /api\.fetchInspectionTask\(activeTaskId\)/)
})

test('workbench lets an operator select a retained task and inspect native event history', () => {
  assert.match(
    workbenchSource,
    /api\.fetchInspectionTasks\(\{ mapId: mapId \|\| null, includeTerminal: true, limit: 12 \}\)/,
  )
  assert.match(workbenchSource, /Recent tasks/)
  assert.match(workbenchSource, /Native task events/)
  assert.match(workbenchSource, /taskStatus\?\.timeline/)
  assert.match(workbenchSource, /selectInspectionTask/)
  assert.match(workbenchSource, /inspectionTasks\.some\(task => !task\.terminal\)/)
  assert.doesNotMatch(workbenchSource, /const currentTask = taskResult\.value\.tasks\[0\] \?\? null/)
})

test('workbench presents execution, inspection result, and acceptance as separate facts', () => {
  assert.match(typesSource, /InspectionTaskReportResponse/)
  assert.match(apiSource, /inspection_task_report/)
  assert.match(apiSource, /fetchInspectionTaskReport/)
  assert.match(workbenchSource, /api\.fetchInspectionTaskReport\(selectedTaskId\)/)
  assert.match(workbenchSource, /Inspection result/)
  assert.match(workbenchSource, /巡检结果/)
  assert.match(workbenchSource, /Acceptance/)
  assert.match(workbenchSource, /验收结论/)
  assert.match(workbenchSource, /Verified evidence/)
  assert.match(workbenchSource, /已验证证据/)
  assert.match(workbenchSource, /task_history_incomplete/)
  assert.match(workbenchSource, /taskReport\.points\.map/)
  assert.match(workbenchSource, /Point results/)
  assert.match(workbenchSource, /点位结果/)
  assert.match(workbenchSource, /native_event_identity_conflict/)
  assert.match(workbenchSource, /The terminal result was rejected/)
  assert.match(workbenchSource, /inspection_task_route_snapshot_unavailable/)
  assert.match(workbenchSource, /this historical task cannot be accepted/)
  assert.match(
    workbenchSource,
    /\.catch\(err => \{[\s\S]*?setTaskReport\(null\)[\s\S]*?setTaskReportError/,
  )
  assert.doesNotMatch(workbenchSource, /recording.*acceptance|acceptance.*recording/is)
})

test('workbench only admits locations bound to the selected map revision', () => {
  assert.match(workbenchSource, /location\.map_id === mapId/)
  assert.match(workbenchSource, /location\.map_content_epoch === mapVersion/)
  assert.match(workbenchSource, /binding_status/)
  assert.match(workbenchSource, /disabled=\{!bound/)
})

test('workbench capability-gates route APIs instead of surfacing backend 404s', () => {
  assert.match(workbenchSource, /api\.fetchAppBootstrap\(\)/)
  assert.match(workbenchSource, /inspectionAvailability/)
  assert.match(workbenchSource, /links\?\.inspection_routes/)
  assert.match(workbenchSource, /inspectionAvailability === 'unavailable'/)
  assert.match(workbenchSource, /err\.statusCode === 404 \|\| err\.statusCode === 503/)
})

test('workbench gates evidence actions on worker readiness and supported actions', () => {
  assert.match(workbenchSource, /INSPECTION_ACTION_OPTIONS/)
  assert.match(workbenchSource, /capture:overview/)
  assert.match(workbenchSource, /capture:parking/)
  assert.match(workbenchSource, /capture:bin_full/)
  assert.match(workbenchSource, /capture:plate_ocr/)
  assert.match(workbenchSource, /evidenceWorkerReady/)
  assert.match(workbenchSource, /supportedEvidenceActions/)
  assert.match(workbenchSource, /unsupportedRouteActions/)
  assert.match(workbenchSource, /availableActionOptions/)
  assert.match(workbenchSource, /startBlockedByEvidenceWorker/)
  assert.match(
    workbenchSource,
    /routeHasEvidenceActions\(points\)[\s\S]*?!evidenceWorkerReady \|\| unsupportedRouteActions\.length > 0/,
  )
  assert.match(workbenchSource, /disabled=\{inspectionAvailability !== 'available'[\s\S]*?startBlockedByEvidenceWorker/)
  assert.match(workbenchSource, /<select[\s\S]*?onChange=\{event => updatePoint\(index, \{ action: event\.target\.value \}\)\}/)
  assert.match(workbenchSource, /Unsupported action/)
  assert.match(workbenchSource, /不支持的动作/)
})

test('workbench shows recent verified evidence without exposing storage paths', () => {
  assert.match(workbenchSource, /api\.fetchInspectionEvidence/)
  assert.match(workbenchSource, /api\.inspectionEvidenceArtifactUrl/)
  assert.match(workbenchSource, /Recent evidence/)
  assert.match(workbenchSource, /最近证据/)
  assert.match(workbenchSource, /evidenceItems\.map/)
  assert.doesNotMatch(workbenchSource, /manifest_path|evidence_dir/)
})

test('workbench copy presents repeatable inspection and review instead of native internals', () => {
  assert.match(workbenchSource, /Repeatable Routes and Evidence/)
  assert.match(workbenchSource, /可复现路线与证据闭环/)
  assert.match(workbenchSource, /review and reinspection/)
  assert.match(workbenchSource, /inspectionVerdictLabel/)
  assert.match(workbenchSource, /Needs review/)
  assert.match(workbenchSource, /需复核/)
})

test('public evidence worker status type does not expose robot filesystem paths', () => {
  const workerType = typesSource
    .split('export interface InspectionEvidenceWorkerStatus {', 2)[1]
    .split('\n}', 1)[0]
  assert.doesNotMatch(workerType, /\bevidence_root\??:/)
  assert.doesNotMatch(workerType, /\bpath\??:/)
})

test('robot status is a single console card without a duplicate topbar page', () => {
  assert.doesNotMatch(topbarSource, /key: 'mode', en: 'Mode'/)
  assert.doesNotMatch(appSource, /activeTab === 'mode'/)
  assert.match(appSource, /activeTab === 'console'[\s\S]*?<RobotStatusPanel/)
  assert.doesNotMatch(appSource, /variant="page"/)
})
