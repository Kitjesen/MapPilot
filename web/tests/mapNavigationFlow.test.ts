import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import {
  mapIsNavigationReady,
  navigationRuntimeReady,
  navigationSessionReady,
  productProfileSessionReady,
  resolveNavigationTargetMapName,
  waitForProductProfileReady,
} from '../src/services/mapReadiness.ts'

const source = readFileSync(
  new URL('../src/components/MapView.tsx', import.meta.url),
  'utf8',
)
const sceneSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const productModeSource = readFileSync(
  new URL('../src/components/ProductModePanel.tsx', import.meta.url),
  'utf8',
)

test('map page exposes an explicit full product navigation action', () => {
  assert.match(source, /onNavigate/)
  assert.match(source, /switchProductSession\('navigating'/)
})

test('map-point goal is gated behind navigation-session readiness', () => {
  const readinessCheck = source.indexOf('ensureNavigationSession')
  const goalDispatch = source.indexOf('api.navigateClick')

  assert.notEqual(readinessCheck, -1)
  assert.notEqual(goalDispatch, -1)
  assert.ok(readinessCheck < goalDispatch)
})

test('map page trusts the artifact gate instead of OctoMap presence alone', () => {
  assert.equal(mapIsNavigationReady({ has_pcd: true, navigation_ready: true }), true)
  assert.equal(mapIsNavigationReady({ has_pcd: true, navigation_ready: false }), false)
  assert.match(source, /mapIsNavigationReady/)
})

test('navigation session must match the map and have live localization', () => {
  const base = {
    mode: 'navigating' as const,
    product_profile: 'nav',
    product_session: 'navigation',
    active_map: 'demo',
    saved_active_map: 'demo',
    map_has_pcd: true,
    map_has_tomogram: true,
    map_has_octomap: true,
    since: 1,
    pending: false,
    error: '',
    icp_quality: 0.1,
    localizer_ready: true,
    pose_fresh: true,
    can_start_mapping: false,
    can_start_navigating: false,
    can_start_exploring: false,
    can_end: true,
    explorer_available: false,
  }

  assert.equal(navigationSessionReady(base, 'demo'), true)
  assert.equal(navigationSessionReady({ ...base, active_map: 'other' }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, active_map: null, saved_active_map: 'demo' }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, localizer_ready: false }, 'demo'), false)
  assert.equal(navigationSessionReady({ ...base, pending: true }, 'demo'), false)

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

test('scene map load uses the full navigation switch and never guesses origin', () => {
  assert.match(sceneSource, /switchProductSession\('navigating'/)
  assert.match(sceneSource, /await waitForMapNavigationReady\(name\)/)
  assert.match(sceneSource, /mapSwitchBusy/)
  assert.doesNotMatch(sceneSource, /relocalize\(name,\s*0,\s*0,\s*0\)/)
})

test('scene cruise only uses the active map or an explicit current selection', () => {
  assert.equal(resolveNavigationTargetMapName('active-map', 'selected-map'), 'selected-map')
  assert.equal(resolveNavigationTargetMapName(null, 'selected-map'), 'selected-map')
  assert.equal(resolveNavigationTargetMapName('active-map', ''), 'active-map')
  assert.equal(resolveNavigationTargetMapName(null, ''), null)
  assert.match(sceneSource, /resolveNavigationTargetMapName\(activeMapName, relocMap\)/)
  assert.doesNotMatch(sceneSource, /activeMapName \?\? savedActiveMapName/)
})

test('product profile readiness waits for the requested profile, map, and session state', () => {
  const navigationSession = {
    mode: 'navigating' as const,
    product_profile: 'nav',
    product_session: 'navigation',
    active_map: 'demo',
    saved_active_map: 'old-map',
    map_has_pcd: true,
    map_has_tomogram: true,
    map_has_octomap: true,
    since: 1,
    pending: false,
    error: '',
    icp_quality: 0.1,
    localizer_ready: true,
    pose_fresh: true,
    can_start_mapping: false,
    can_start_navigating: false,
    can_start_exploring: false,
    can_end: true,
    explorer_available: false,
  }

  assert.equal(productProfileSessionReady(navigationSession, 'nav', 'demo'), true)
  assert.equal(productProfileSessionReady({ ...navigationSession, active_map: 'other' }, 'nav', 'demo'), false)
  assert.equal(productProfileSessionReady({ ...navigationSession, pending: true }, 'nav', 'demo'), false)
  assert.equal(productProfileSessionReady({ ...navigationSession, product_profile: 'map' }, 'nav', 'demo'), false)
  assert.equal(productProfileSessionReady({
    ...navigationSession,
    mode: 'mapping',
    product_profile: 'map',
    product_session: 'mapping',
    active_map: null,
    localizer_ready: false,
  }, 'map'), true)
})

test('profile readiness polling survives the old runtime and returns the requested runtime', async () => {
  const ready = {
    mode: 'mapping' as const,
    product_profile: 'map',
    product_session: 'mapping',
    active_map: null,
    saved_active_map: 'old-map',
    map_has_pcd: false,
    map_has_tomogram: false,
    map_has_octomap: false,
    since: 2,
    pending: false,
    error: '',
    icp_quality: 0,
    localizer_ready: false,
    pose_fresh: true,
    can_start_mapping: false,
    can_start_navigating: false,
    can_start_exploring: false,
    can_end: true,
    explorer_available: false,
  }
  let attempts = 0

  const result = await waitForProductProfileReady('map', null, {
    fetchSession: async () => {
      attempts += 1
      return attempts === 1
        ? { ...ready, mode: 'navigating', product_profile: 'nav', product_session: 'navigation' }
        : ready
    },
    intervalMs: 0,
    sleep: async () => undefined,
  })

  assert.equal(attempts, 2)
  assert.equal(result.product_profile, 'map')
})

test('saved-map profiles wait for native navigation readiness', async () => {
  const session = {
    mode: 'navigating' as const,
    product_profile: 'nav',
    product_session: 'navigation',
    active_map: 'demo',
    saved_active_map: 'old-map',
    map_has_pcd: true,
    map_has_tomogram: false,
    map_has_octomap: true,
    since: 1,
    pending: false,
    error: '',
    icp_quality: 0.1,
    localizer_ready: true,
    pose_fresh: true,
    can_start_mapping: false,
    can_start_navigating: false,
    can_start_exploring: false,
    can_end: true,
    explorer_available: false,
  }
  let navigationAttempts = 0

  const result = await waitForProductProfileReady('nav', 'demo', {
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

test('profile polling reports a target runtime error immediately', async () => {
  let attempts = 0
  await assert.rejects(
    waitForProductProfileReady('map', null, {
      fetchSession: async () => {
        attempts += 1
        return {
          mode: 'idle',
          product_profile: 'map',
          product_session: 'idle',
          active_map: null,
          saved_active_map: null,
          map_has_pcd: false,
          map_has_tomogram: false,
          map_has_octomap: false,
          since: 1,
          pending: false,
          error: 'slam_start_failed',
          icp_quality: 1,
          localizer_ready: false,
          can_start_mapping: false,
          can_start_navigating: false,
          can_start_exploring: false,
          can_end: false,
          explorer_available: false,
        }
      },
      intervalMs: 0,
      sleep: async () => undefined,
    }),
    /slam_start_failed/,
  )
  assert.equal(attempts, 1)
})

test('all profile switch entry points wait for the target runtime before reporting success', () => {
  assert.match(sceneSource, /await waitForProductProfileReady\('map'\)/)
  assert.match(sceneSource, /await waitForMapNavigationReady\(navigationTargetMapName\)/)
  assert.match(sceneSource, /mapIsNavigationReady\(navigationTargetMap\)/)
  assert.match(productModeSource, /await waitForProductProfileReady\(/)
  assert.match(productModeSource, /fetchNavigation: api\.fetchNavigationStatus/)
  assert.doesNotMatch(productModeSource, /switchAcceptedAtReconnects/)
})

test('scene goal feedback distinguishes an in-progress product switch', () => {
  assert.match(sceneSource, /productSwitchInProgress/)
  assert.match(sceneSource, /产品模式正在切换/)
})
