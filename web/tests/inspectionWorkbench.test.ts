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

test('inspection tab uses the product workbench as the run entry', () => {
  assert.match(appSource, /InspectionWorkbench/)
  assert.doesNotMatch(appSource, /<InspectionAcceptanceView/)
  assert.match(topbarSource, /key: 'inspection', en: 'Inspect'/)
})

test('inspection API surface exposes route store and native run controls', () => {
  assert.match(apiSource, /fetchInspectionRoutes/)
  assert.match(apiSource, /fetchInspectionRoute/)
  assert.match(apiSource, /saveInspectionRoute/)
  assert.match(apiSource, /startInspectionRoute/)
  assert.match(apiSource, /pauseInspectionRun/)
  assert.match(apiSource, /resumeInspectionRun/)
  assert.match(apiSource, /cancelInspectionRun/)
  assert.match(apiSource, /fetchInspectionStatus/)
  assert.match(apiSource, /fetchInspectionEvidence/)
  assert.match(apiSource, /inspectionEvidenceArtifactUrl/)
})

test('workbench builds routes from saved locations and avoids acceptance runner', () => {
  assert.match(workbenchSource, /api\.fetchLocations/)
  assert.match(workbenchSource, /routePointFromLocation/)
  assert.match(workbenchSource, /movePoint/)
  assert.match(workbenchSource, /failurePolicy/)
  assert.match(workbenchSource, /maxRetries/)
  assert.match(workbenchSource, /api\.startInspectionRoute/)
  assert.match(workbenchSource, /api\.pauseInspectionRun/)
  assert.match(workbenchSource, /api\.resumeInspectionRun/)
  assert.match(workbenchSource, /api\.cancelInspectionRun/)
  assert.doesNotMatch(workbenchSource, /runInspectionAcceptance/)
})

test('workbench loads route detail and starts the persisted revision', () => {
  assert.match(workbenchSource, /api\.fetchInspectionRoute/)
  assert.match(workbenchSource, /savedRevision/)
  assert.match(
    workbenchSource,
    /api\.startInspectionRoute\(selectedRouteId,[\s\S]*?revision: savedRevision/,
  )
  assert.doesNotMatch(workbenchSource, /action: 'inspect'/)
})

test('workbench exposes native phase evidence and safely releases operator takeover', () => {
  assert.match(workbenchSource, /status\.reason/)
  assert.match(workbenchSource, /status\.evidence_id/)
  assert.match(workbenchSource, /status\.action/)
  assert.match(workbenchSource, /status\.evidence_worker/)
  assert.match(workbenchSource, /Evidence worker/)
  assert.match(
    workbenchSource,
    /operator_takeover[\s\S]*?api\.resumeNavigation\(\)[\s\S]*?api\.resumeInspectionRun/,
  )
})

test('workbench only admits locations bound to the selected map revision', () => {
  assert.match(workbenchSource, /location\.map_id === mapId/)
  assert.match(workbenchSource, /location\.map_version === mapVersion/)
  assert.match(workbenchSource, /binding_status/)
  assert.match(workbenchSource, /disabled=\{!bound/)
})

test('workbench capability-gates route APIs instead of surfacing backend 404s', () => {
  assert.match(workbenchSource, /api\.fetchAppBootstrap\(\)/)
  assert.match(workbenchSource, /inspectionAvailability/)
  assert.match(workbenchSource, /links\?\.inspection_routes/)
  assert.match(workbenchSource, /inspectionAvailability === 'unavailable'/)
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

test('public evidence worker status type does not expose robot filesystem paths', () => {
  const workerType = typesSource
    .split('export interface InspectionEvidenceWorkerStatus {', 2)[1]
    .split('\n}', 1)[0]
  assert.doesNotMatch(workerType, /\bevidence_root\??:/)
  assert.doesNotMatch(workerType, /\bpath\??:/)
})

test('product mode is a single console card without a duplicate topbar page', () => {
  assert.doesNotMatch(topbarSource, /key: 'mode', en: 'Mode'/)
  assert.doesNotMatch(appSource, /activeTab === 'mode'/)
  assert.match(appSource, /activeTab === 'console'[\s\S]*?<ProductModePanel/)
  assert.doesNotMatch(appSource, /variant="page"/)
})
