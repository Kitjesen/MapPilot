import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import {
  mapIsActivationReady,
  navigationRuntimeReady,
  navigationSessionReady,
  productReady,
  resolveNavigationTargetMapName,
  waitForProductReady,
} from '../src/services/mapReadiness.ts'

const originalFetch = globalThis.fetch

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

const source = readFileSync(
  new URL('../src/components/MapView.tsx', import.meta.url),
  'utf8',
)
const apiSource = readFileSync(
  new URL('../src/services/api.ts', import.meta.url),
  'utf8',
)
const sceneSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
test('map page navigates only when the current Product is ready', () => {
  assert.match(source, /onNavigate/)
  assert.match(source, /ensureNavigationSession\(name\)/)
  assert.doesNotMatch(source, /onActivate|激活地图/)
  assert.doesNotMatch(apiSource, /activateMap|\/api\/v1\/map\/activate/)
})

test('map-point goal is gated behind navigation-session readiness', () => {
  const readinessCheck = source.indexOf('ensureNavigationSession')
  const goalDispatch = source.indexOf('api.navigateClick')

  assert.notEqual(readinessCheck, -1)
  assert.notEqual(goalDispatch, -1)
  assert.ok(readinessCheck < goalDispatch)
})

test('map page trusts the artifact gate instead of OctoMap presence alone', () => {
  assert.equal(mapIsActivationReady({ has_pcd: true, has_octomap: true, activation_ready: true }), true)
  assert.equal(mapIsActivationReady({ has_pcd: true, has_octomap: false, activation_ready: true }), false)
  assert.equal(mapIsActivationReady({ has_pcd: true, has_octomap: true, activation_ready: false }), false)
  assert.match(source, /mapIsActivationReady/)
})

test('navigation session must match the map and have live localization', () => {
  const base = {
    mode: 'navigating' as const,
    env: 'real' as const,
    product: 'nav' as const,
    active_map: 'demo',
    saved_active_map: 'demo',
    map_has_pcd: true,
    map_has_octomap: true,
    since: 1,
    icp_quality: 0.1,
    localizer_ready: true,
    pose_fresh: true,
    explorer_available: false,
  }

  assert.equal(navigationSessionReady(base, 'demo'), true)
  assert.equal(navigationSessionReady({ ...base, active_map: 'other' }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, active_map: null, saved_active_map: 'demo' }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, localizer_ready: false }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, map_has_octomap: false }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, product: null }, 'demo'), false)

  const navigation = {
    can_accept_goal: true,
    readiness: { can_accept_goal: true, blockers: [] },
  }
  assert.equal(navigationRuntimeReady(base, navigation as never, 'demo'), true)
  assert.equal(
    navigationRuntimeReady(
      base,
      { ...navigation, can_accept_goal: false } as never,
      'demo',
    ),
    false,
  )
})

test('scene map load is an explicit read-only preview', () => {
  assert.match(sceneSource, /fetchSavedMapPointCloud\(name\)/)
  assert.match(sceneSource, /不改变机器人当前任务或激活地图/)
  assert.doesNotMatch(sceneSource, /waitForMapNavigationReady/)
  assert.match(sceneSource, /mapSwitchBusy/)
  assert.doesNotMatch(sceneSource, /relocalize\(name,\s*0,\s*0,\s*0\)/)
})

test('navigation target map helper prefers an explicit selection', () => {
  assert.equal(resolveNavigationTargetMapName('active-map', 'selected-map'), 'selected-map')
  assert.equal(resolveNavigationTargetMapName(null, 'selected-map'), 'selected-map')
  assert.equal(resolveNavigationTargetMapName('active-map', ''), 'active-map')
  assert.equal(resolveNavigationTargetMapName(null, ''), null)
  assert.doesNotMatch(sceneSource, /activeMapName \?\? savedActiveMapName/)
})

test('map-backed Explore renders its active saved map while live Explore stays unbound', () => {
  const helper = sceneSource.match(
    /function shouldShowSavedMapForProduct[\s\S]*?\n\}/,
  )?.[0] ?? ''

  assert.match(helper, /product\s*===\s*'explore'/)
  assert.match(sceneSource, /activeMapName\s*&&\s*shouldShowSavedMapForProduct\(currentProduct\)/)
  assert.match(sceneSource, /showSavedMapInScene\s*=\s*Boolean\(activeMapName\s*&&\s*shouldShowSavedMapForProduct\(currentProduct\)\)/)
})

test('Product readiness waits for the requested Product, map, and session state', () => {
  const navigationSession = {
    mode: 'navigating' as const,
    env: 'real' as const,
    product: 'nav' as const,
    active_map: 'demo',
    saved_active_map: 'old-map',
    map_has_pcd: true,
    map_has_octomap: true,
    since: 1,
    icp_quality: 0.1,
    localizer_ready: true,
    pose_fresh: true,
    explorer_available: false,
  }

  assert.equal(productReady(navigationSession, 'nav', 'demo'), true)
  assert.equal(productReady({ ...navigationSession, active_map: 'other' }, 'nav', 'demo'), false)
  assert.equal(productReady({ ...navigationSession, product: 'map' }, 'nav', 'demo'), false)
  assert.equal(productReady({
    ...navigationSession,
    mode: 'mapping',
    product: 'map',
    active_map: null,
    localizer_ready: false,
  }, 'map'), true)
})

test('Explore readiness follows whether the session uses a saved map', () => {
  const liveExplore = {
    mode: 'exploring' as const,
    env: 'real' as const,
    product: 'explore' as const,
    active_map: null,
    saved_active_map: 'old-map',
    map_has_pcd: false,
    map_has_octomap: false,
    since: 1,
    icp_quality: 0,
    localizer_ready: false,
    pose_fresh: true,
    explorer_available: true,
  }

  assert.equal(productReady(liveExplore, 'explore', null), true)
  assert.equal(productReady({ ...liveExplore, active_map: 'demo' }, 'explore', null), false)

  const savedMapExplore = {
    ...liveExplore,
    active_map: 'demo',
    map_has_pcd: true,
    map_has_octomap: true,
    localizer_ready: true,
    pose_fresh: true,
  }
  assert.equal(productReady(savedMapExplore, 'explore', 'demo'), true)
  assert.equal(productReady({ ...savedMapExplore, active_map: 'other' }, 'explore', 'demo'), false)
  assert.equal(productReady({ ...savedMapExplore, localizer_ready: false }, 'explore', 'demo'), false)
  assert.equal(productReady({ ...savedMapExplore, pose_fresh: false }, 'explore', 'demo'), false)
})

test('Product readiness polling survives the old runtime and returns the requested runtime', async () => {
  const ready = {
    mode: 'mapping' as const,
    env: 'real' as const,
    product: 'map' as const,
    active_map: null,
    saved_active_map: 'old-map',
    map_has_pcd: false,
    map_has_octomap: false,
    since: 2,
    icp_quality: 0,
    localizer_ready: false,
    pose_fresh: true,
    explorer_available: false,
  }
  let attempts = 0

  const result = await waitForProductReady('map', null, {
    fetchSession: async () => {
      attempts += 1
      return attempts === 1
        ? { ...ready, mode: 'navigating', product: 'nav' as const }
        : ready
    },
    intervalMs: 0,
    sleep: async () => undefined,
  })

  assert.equal(attempts, 2)
  assert.equal(result.product, 'map')
})

test('saved-map Products wait for native navigation readiness', async () => {
  const session = {
    mode: 'navigating' as const,
    env: 'real' as const,
    product: 'nav' as const,
    active_map: 'demo',
    saved_active_map: 'old-map',
    map_has_pcd: true,
    map_has_octomap: true,
    since: 1,
    icp_quality: 0.1,
    localizer_ready: true,
    pose_fresh: true,
    explorer_available: false,
  }
  let navigationAttempts = 0

  const result = await waitForProductReady('nav', 'demo', {
    fetchSession: async () => session,
    fetchNavigation: async () => {
      navigationAttempts += 1
      const ready = navigationAttempts > 1
      return {
        can_accept_goal: ready,
        readiness: {
          can_accept_goal: ready,
          blockers: ready ? [] : ['native_input_gate_not_ready'],
        },
      } as never
    },
    intervalMs: 0,
    sleep: async () => undefined,
  })

  assert.equal(navigationAttempts, 2)
  assert.equal(result.active_map, 'demo')
})

test('browser has no ProductControl command-copy entry point', () => {
  assert.doesNotMatch(sceneSource, /copyProductSwitchCommand|ProductControl 命令/)
  assert.doesNotMatch(apiSource, /prepareProductSwitch|copyProductSwitchCommand|runtime_switch/)
})

test('scene goal feedback reflects navigation readiness, not a browser switch state', () => {
  assert.doesNotMatch(sceneSource, /productSwitchInProgress|产品模式正在切换/)
  assert.match(sceneSource, /navigation_session_inactive/)
})
