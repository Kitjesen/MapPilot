import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const panelSource = readFileSync(
  new URL('../src/components/ProductModePanel.tsx', import.meta.url),
  'utf8',
)
const panelStyles = readFileSync(
  new URL('../src/components/ProductModePanel.module.css', import.meta.url),
  'utf8',
)
const typesSource = readFileSync(
  new URL('../src/types/index.ts', import.meta.url),
  'utf8',
)
const apiSource = readFileSync(
  new URL('../src/services/api.ts', import.meta.url),
  'utf8',
)
const readinessSource = readFileSync(
  new URL('../src/services/mapReadiness.ts', import.meta.url),
  'utf8',
)
const sceneSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const mapSource = readFileSync(
  new URL('../src/components/MapView.tsx', import.meta.url),
  'utf8',
)
const slamSource = readFileSync(
  new URL('../src/components/SlamPanel.tsx', import.meta.url),
  'utf8',
)
const dataflowSource = readFileSync(
  new URL('../src/components/RuntimeDataflowView.tsx', import.meta.url),
  'utf8',
)
const gatewaySmokeSource = readFileSync(
  new URL('../scripts/gateway-smoke.mjs', import.meta.url),
  'utf8',
)
const legacyProductModeType = ['ProductMode', 'Profile'].join('')
const legacyWireFields = ['current', 'target'].map(prefix => `${prefix}_profile`).join('|')
const legacyUiNames = [
  ...['current', 'target', 'selected'].map(prefix => `${prefix}Profile`),
  ['restartProduct', 'Profile'].join(''),
  ['Product', 'Profile'].join(''),
].join('|')
const retiredExploreProduct = ['tare', 'explore'].join('_')

test('field Product type and picker expose the same seven operator Products', () => {
  const expectedProducts = [
    'teleop',
    'teleop_avoid',
    'map',
    'explore',
    'nav',
    'tracking',
    'inspection',
  ].sort()
  const fieldProductType = typesSource.match(
    /export type ProductName =([\s\S]*?)export type VisualServoMode/,
  )?.[1] ?? ''
  const fieldProducts = [...fieldProductType.matchAll(/'([^']+)'/g)]
    .map(match => match[1])
    .sort()
  const pickerSource = panelSource.match(
    /const PRODUCT_MODES: ProductModeOption\[\] = \[([\s\S]*?)\n\]/,
  )?.[1] ?? ''
  const pickerProducts = [...pickerSource.matchAll(/product: '([^']+)'/g)]
    .map(match => match[1])
    .sort()

  assert.deepEqual(fieldProducts, expectedProducts)
  assert.deepEqual(pickerProducts, expectedProducts)
  assert.doesNotMatch([typesSource, panelSource].join('\n'), new RegExp(retiredExploreProduct))
  assert.doesNotMatch(typesSource, /FieldProductName/)
  assert.doesNotMatch(typesSource, new RegExp(legacyProductModeType))
})

test('Explore offers an optional saved map without exposing an implementation variant', () => {
  assert.match(panelSource, /selectedProduct\s*===\s*'explore'/)
  assert.match(panelSource, /map_name:\s*usesSavedMap\s*\?\s*selectedMap\s*:\s*null/)
  assert.match(panelSource, /mapName:\s*usesSavedMap\s*\?\s*selectedMap\s*:\s*null/)
  assert.match(panelSource, /selectedSavedMapUnavailable\s*=\s*usesSavedMap\s*&&\s*!selectedMapReady/)
  assert.match(apiSource, /targetProduct\s*===\s*'explore'\s*&&\s*mapName\s*!==\s*null/)
  assert.match(apiSource, /map_name:\s*usesSavedMap\s*\?\s*mapName\s*:\s*null/)
  assert.doesNotMatch(panelSource, /explore[_ -]?variant|variant.*explore/i)
  assert.doesNotMatch(apiSource, /explore[_ -]?variant|variant.*explore/i)
})

test('field Product switch uses Product terminology from UI variables through JSON wires', () => {
  const switchTypes = typesSource.match(
    /export interface RuntimeSwitchPlanRequest([\s\S]*?)export interface AppBootstrapResponse/,
  )?.[1] ?? ''
  const switchSources = [
    switchTypes,
    apiSource,
    panelSource,
    readinessSource,
    sceneSource,
    mapSource,
    slamSource,
    dataflowSource,
    gatewaySmokeSource,
  ].join('\n')

  assert.match(switchTypes, /current_product/)
  assert.match(switchTypes, /target_product/)
  assert.match(apiSource, /current_product:/)
  assert.match(apiSource, /target_product:/)
  assert.match(apiSource, /target_product: targetProduct/)
  assert.doesNotMatch(dataflowSource, /boundary\.profile/)
  assert.doesNotMatch(switchSources, new RegExp(`${legacyWireFields}|${legacyProductModeType}`))
  assert.doesNotMatch(
    [apiSource, panelSource, readinessSource, sceneSource, mapSource, slamSource, dataflowSource].join('\n'),
    new RegExp(legacyUiNames),
  )
})

test('Product switch requests cannot select an Env or deployment endpoint', () => {
  const planRequestType = typesSource.match(
    /export interface RuntimeSwitchPlanRequest\s*\{([\s\S]*?)\n\}/,
  )?.[1] ?? ''
  const productSessionSwitch = apiSource.match(
    /export interface ProductSessionSwitchOptions\s*\{([\s\S]*?)\n\}/,
  )?.[1] ?? ''
  const panelSwitchRequest = panelSource.match(
    /const buildSwitchRequest = useCallback\([\s\S]*?const runPreflight/,
  )?.[0] ?? ''
  const dataflowSwitchPlan = dataflowSource.match(
    /const loadSummary = useCallback\([\s\S]*?const loadDetail/,
  )?.[0] ?? ''
  const forbiddenSelector = /\b(?:current_endpoint|target_endpoint|endpoint|current_env|target_env|env)\??\s*:/

  for (const source of [
    planRequestType,
    productSessionSwitch,
    panelSwitchRequest,
    dataflowSwitchPlan,
  ]) {
    assert.doesNotMatch(source, forbiddenSelector)
  }
})

test('session identity exposes Env and Product without a Product Profile alias', () => {
  const sessionType = typesSource.match(
    /export interface SessionEvent\s*\{([\s\S]*?)export interface SessionTransitionResponse/,
  )?.[1] ?? ''
  const sessionConsumers = [
    panelSource,
    readinessSource,
    sceneSource,
    mapSource,
    slamSource,
    dataflowSource,
  ].join('\n')

  assert.match(sessionType, /env:\s*EnvName/)
  assert.match(sessionType, /product\?:\s*ProductName\s*\|\s*null/)
  assert.doesNotMatch(sessionType, /product_profile|explorer_required_profile/)
  assert.doesNotMatch(sessionConsumers, /product_profile/)
})

test('runtime dataflow selects real versus sim behavior only from Env', () => {
  const runtimeClassifier = dataflowSource.match(
    /function isRealRuntimeBoundary[\s\S]*?\n\}/,
  )?.[0] ?? ''

  assert.match(runtimeClassifier, /boundary\.env\s*===\s*'real'/)
  assert.doesNotMatch(
    runtimeClassifier,
    /simulation_only|runtime_contract|data_source|endpoint/,
  )
})

test('runtime responses expose resolved Env, Product, and RunPlan identity', () => {
  const dataflowType = typesSource.match(
    /export interface RuntimeDataflowResponse\s*\{([\s\S]*?)export interface RuntimeDataflowTopicDetailResponse/,
  )?.[1] ?? ''
  const planResponseType = typesSource.match(
    /export interface RuntimeSwitchPlanResponse\s*\{([\s\S]*?)export type EnvName/,
  )?.[1] ?? ''
  const switchBoundary = dataflowSource.match(
    /const switchBoundary\s*=\s*[^\n]+/,
  )?.[0] ?? ''

  assert.match(dataflowType, /runtime_boundary:\s*RuntimeIdentity/)
  assert.match(planResponseType, /run_plan\?:\s*Record<string, unknown>\s*\|\s*null/)
  assert.match(planResponseType, /operator_command\?:\s*string\s*\|\s*null/)
  assert.match(switchBoundary, /switchFrom\.product/)
  assert.match(switchBoundary, /switchTo\.product/)
  assert.doesNotMatch(switchBoundary, /endpoint/)
})

test('unknown Product identity stays unknown until Gateway or the user selects one', () => {
  const planRequestType = typesSource.match(
    /export interface RuntimeSwitchPlanRequest\s*\{([\s\S]*?)\n\}/,
  )?.[1] ?? ''
  const planApi = apiSource.match(
    /export async function runRuntimeSwitchPlan\([\s\S]*?\n\}/,
  )?.[0] ?? ''
  const sceneProductSession = sceneSource.match(
    /function productSessionFromProduct\([\s\S]*?\n\}/,
  )?.[0] ?? ''

  assert.match(planRequestType, /target_product:\s*ProductName/)
  assert.doesNotMatch(planRequestType, /target_product\?/)
  assert.doesNotMatch(planApi, /request:\s*RuntimeSwitchPlanRequest\s*=|target_product:\s*'explore'/)
  assert.doesNotMatch(apiSource, /PRODUCT_BY_SESSION_MODE/)
  assert.doesNotMatch(apiSource, /targetProduct\s*\?\?|restartSlam\(options:\s*RestartSlamOptions\s*=/)
  assert.match(apiSource, /targetProduct:\s*ProductName/)
  assert.doesNotMatch(panelSource, /sessionFallbackProduct|useState<ProductName>\('teleop'\)|\?\?\s*PRODUCT_MODES\[0\]/)
  assert.match(panelSource, /useState<ProductName \| null>\(null\)/)
  assert.doesNotMatch(readinessSource, /session\.product\s*\?\?\s*'teleop'/)
  assert.doesNotMatch(dataflowSource, /currentProduct\s*\?\?\s*'explore'/)
  assert.match(dataflowSource, /runtime_product_unknown/)
  assert.doesNotMatch(sceneProductSession, /\bmode\b|return\s+'teleop'/)
  assert.match(sceneProductSession, /return\s+'unknown'/)
})

test('Product switching is a read-only preflight plus ProductControl command handoff', () => {
  const switchTypes = typesSource.match(
    /export interface RuntimeSwitchPlanRequest([\s\S]*?)export interface AppBootstrapResponse/,
  )?.[1] ?? ''
  const forbiddenTypeField = /^\s*(?:strategy|allow_restart|lifecycle|product_mode_switch)\??\s*:/m

  assert.doesNotMatch(switchTypes, forbiddenTypeField)
  assert.doesNotMatch(apiSource, /strategy:\s*'(?:auto|cold)'|allow_restart:/)
  assert.doesNotMatch(
    panelSource,
    /STRATEGIES|strategyLabel|lifecycleFrom|allowRestart|Allow restart|label=\{text\(locale, 'Strategy'|\bpolicy:/,
  )
  assert.doesNotMatch(gatewaySmokeSource, /allow_restart/)
  assert.doesNotMatch(dataflowSource, /product_mode_switch|\.lifecycle/)
  assert.match(apiSource, /prepareProductSwitch/)
  assert.match(apiSource, /copyProductSwitchCommand/)
  assert.match(apiSource, /plan\.operator_command/)
  assert.match(apiSource, /!plan\.read_only\s*\|\|\s*!plan\.dry_run\s*\|\|\s*plan\.motion/)
  assert.doesNotMatch(apiSource, /execute:\s*(?:true|false)/)
  assert.doesNotMatch(apiSource, /\/api\/v1\/runtime\/switch['"`]/)
})

test('compact mode control uses the themed listbox instead of a native select', () => {
  assert.match(panelSource, /function ModePicker/)
  assert.match(panelSource, /aria-haspopup="listbox"/)
  assert.match(panelSource, /role="listbox"/)
  assert.match(panelSource, /createPortal/)
  assert.doesNotMatch(panelSource, /const modeSelect = \(\s*<label[\s\S]*?<select/)
})

test('mode picker exposes descriptions and selected state without lifecycle policy', () => {
  assert.match(panelSource, /summaryForOption\(selected, locale\)/)
  assert.match(panelSource, /aria-selected=\{isSelected\}/)
  assert.doesNotMatch(panelSource, /hotBadge|coldBadge/)
  assert.match(panelStyles, /\.modeMenuOptionSelected/)
  assert.match(panelStyles, /\.modeMenuCheckVisible/)
})

test('mode picker supports keyboard navigation and escapes card clipping', () => {
  assert.match(panelSource, /event\.key === 'ArrowDown'/)
  assert.match(panelSource, /event\.key === 'Escape'/)
  assert.match(panelStyles, /\.modeMenu\s*\{[\s\S]*?position: fixed/)
  assert.match(panelStyles, /z-index: 5000/)
})

test('map control uses the themed floating picker', () => {
  assert.match(panelSource, /function SimplePicker/)
  assert.match(panelSource, /label=\{text\(locale, 'Map'/)
  assert.match(panelStyles, /\.simplePickerTrigger/)
  assert.match(panelStyles, /\.simplePickerOption/)
})

test('product mode renders only the console card surface', () => {
  assert.match(panelSource, /styles\.compactCard/)
  assert.doesNotMatch(panelSource, /variant\?: 'page' \| 'card'/)
  assert.doesNotMatch(panelSource, /panel-mode/)
  assert.doesNotMatch(panelSource, /sendVisualServo/)
  assert.doesNotMatch(panelStyles, /\.page\s*\{/)
  assert.doesNotMatch(panelStyles, /\.modeGrid\s*\{/)
})
