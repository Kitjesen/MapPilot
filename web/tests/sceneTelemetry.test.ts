import { Quaternion, Vector3 } from 'three'
import assert from 'node:assert/strict'
import test from 'node:test'
import { currentNativeLocalPath, freshSource, freshScenePose, projectScenePath, projectScenePose, scanCanStandAlone, scenePoseEpoch, sceneTrailStorageKey, sceneRelocalizationSeed, activeMapRelocalizationTarget } from '../src/services/sceneTelemetry.ts'
import { PoseInterpolation } from '../src/components/scene3d/robot/poseInterpolation.ts'

const odom = { type: 'odometry' as const, frame_id: 'odom', x: 2, y: 1, z: 0.5, yaw: 0, vx: 0.5 }
const near = (actual: number, expected: number) => assert.ok(Math.abs(actual - expected) < 1e-9)

test('slow simulation poses use source time while real poses keep wall-clock freshness', () => {
  const localization = { ts: 100, diag_received_ts: 500, raw: { source_data_age_s: 0 } }
  assert.equal(freshScenePose(100, 500.2, 'sim', localization), true)
  assert.equal(freshScenePose(100, 500.2, 'real', localization), false)
  assert.equal(freshScenePose(500, 500.2, 'real', localization), true)
  assert.equal(freshScenePose(95, 500.2, 'sim', localization), false, 'a newly delivered stale pose remains stale')
})

test('simulation poses expire when source progress or Gateway delivery stops', () => {
  const localization = { ts: 100, diag_received_ts: 500, raw: { source_data_age_s: 0 } }
  assert.equal(freshScenePose(100, 503.1, 'sim', localization), false, 'no diagnostic delivery')
  const repeated = { ...localization, diag_received_ts: 510, raw: { source_data_age_s: 10 } }
  assert.equal(freshScenePose(100, 510.1, 'sim', repeated), false, 'repeated diagnostics do not renew source progress')
  assert.equal(freshScenePose(100, 500, 'sim', { ...localization, raw: {} }), false)
})

test('SLAM restart and map jump reset pose interpolation and isolate history inside one Product session', () => {
  const first = scenePoseEpoch('product-1', { runtime_instance_id: 'slam-1', map_frame_jump_sequence: 0 })
  const restarted = scenePoseEpoch('product-1', { runtime_instance_id: 'slam-2', map_frame_jump_sequence: 0 })
  const jumped = scenePoseEpoch('product-1', { runtime_instance_id: 'slam-2', map_frame_jump_sequence: 1 })
  assert.equal(new Set([first, restarted, jumped]).size, 3)
  const oldTrailKey = sceneTrailStorageKey('map-1', first)!
  const history = new Map([[oldTrailKey, [[0, 0, 0.4], [1, 0, 0.5]]]])
  assert.equal(history.get(sceneTrailStorageKey('map-1', restarted)!), undefined)
  assert.equal(history.get(sceneTrailStorageKey('map-1', jumped)!), undefined)
  assert.notEqual(sceneTrailStorageKey('map-2', first), oldTrailKey)
  const display = new PoseInterpolation()
  display.update({ x: 0, y: 0, z: 0.4, yaw: 0, stampS: 100, epoch: first }, 0)
  display.update({ x: 0.2, y: 0, z: 0.5, yaw: 0, stampS: 100, epoch: restarted }, 10)
  near(display.sample(10)!.x, 0.2)
  near(display.sample(10)!.z, 0.5)
  display.update({ x: 0.3, y: 0, z: 0.6, yaw: 0, stampS: 100.1, epoch: jumped }, 20)
  near(display.sample(20)!.x, 0.3)
  near(display.sample(20)!.z, 0.6)
})

test('unreported coordinate identities remain unknown and cannot load old map-only history', () => {
  assert.equal(scenePoseEpoch(undefined, {}), null)
  assert.equal(scenePoseEpoch(null, { runtime_instance_id: '', map_frame_jump_sequence: -1 }), null)
  assert.equal(sceneTrailStorageKey('map-1', null), null)
  assert.notEqual(sceneTrailStorageKey('map-1', scenePoseEpoch('product-1', null)), 'lingtu.trail.map-1')
})

test('native map pose and path are displayed without another transform', () => {
  const pose = projectScenePose({ ...odom, frame_id: 'map', x: 9, y: 0, z: 3.5, yaw: Math.PI / 2 })
  assert.ok(pose)
  near(pose.x, 9); near(pose.y, 0); near(pose.z!, 3.5); near(pose.yaw, Math.PI / 2)
  assert.equal(pose.vx, 0.5, 'body-frame forward speed does not rotate with the map')
  assert.equal(projectScenePose(pose), pose, 'already-map pose must not be transformed twice')
  const path = projectScenePath([{ x: 9, y: 0, z: 3.5 }], 'map')
  near(path[0].x, 9); near(path[0].z!, 3.5)
})

test('non-map telemetry is never silently relabeled', () => {
  assert.equal(projectScenePose(odom), null)
  assert.equal(projectScenePose({ ...odom, frame_id: 'unknown' }), null)
  assert.equal(projectScenePose({ ...odom, frame_id: 'body' }), null)
  assert.deepEqual(projectScenePath([{ x: 1, y: 2 }], 'livox_frame'), [])
})

test('clicked relocalization keeps map XY and uses the projected map heading, never a default or raw odom heading', () => {
  const seed = sceneRelocalizationSeed(20, 30, projectScenePose({ ...odom, frame_id: 'map', x: 9, y: 0, z: 3.5, yaw: Math.PI / 2 }))
  assert.ok(seed)
  assert.equal(seed.x, 20)
  assert.equal(seed.y, 30)
  near(seed.z, 3.5)
  near(seed.yaw, Math.PI / 2)
  assert.equal(sceneRelocalizationSeed(20, 30, odom), null)
  assert.equal(sceneRelocalizationSeed(20, 30, null), null)
  assert.equal(sceneRelocalizationSeed(20, 30, { ...odom, frame_id: 'map', z: undefined }), null)
})

test('automatic matching follows active map A to B and does not use the unchanged manual selection', () => {
  const manualSelection = 'map-A'
  assert.equal(activeMapRelocalizationTarget({ active_map: 'map-A' }), manualSelection)
  const nextTarget = activeMapRelocalizationTarget({ active_map: 'map-B' })
  assert.equal(nextTarget, 'map-B')
  assert.notEqual(nextTarget, manualSelection)
  assert.equal(activeMapRelocalizationTarget({ active_map: null }), null)
  assert.equal(activeMapRelocalizationTarget({}), null)
})

test('local trajectory disappears on stop, empty path, expired native timestamp or disconnect', () => {
  const native = { stamp_s: 100, planning_frame_id: 'map', local_path: [[2, 1, 0.5], [3, 1, 0.5]],
    last_local: { tracking: { active: true } } }
  assert.equal(currentNativeLocalPath(native, true, 101).length, 2)
  assert.deepEqual(currentNativeLocalPath({ ...native, last_local: { tracking: { active: false } } }, true, 101), [])
  assert.deepEqual(currentNativeLocalPath({ ...native, local_path: [] }, true, 101), [])
  assert.deepEqual(currentNativeLocalPath(native, true, 110), [])
  assert.deepEqual(currentNativeLocalPath(native, false, 101), [])
  assert.equal(freshSource(undefined, 101), false)
})

test('registered scan is visible without an accumulated cloud only in the published map epoch', () => {
  const scan = { count: 4000, frameId: 'map', epoch: 2 }
  assert.equal(scanCanStandAlone(scan, { count: 0, frameId: null, epoch: null }), true)
  assert.equal(scanCanStandAlone(scan, { count: 0, frameId: 'map', epoch: 2 }), true)
  assert.equal(scanCanStandAlone(scan, { count: 0, frameId: 'map', epoch: 1 }), false)
  assert.equal(scanCanStandAlone({ ...scan, frameId: 'body' }, { count: 0, frameId: null, epoch: null }), false)
})


test('native map orientation is displayed unchanged, including tilt', () => {
  const body = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), 0.4)
  const pose = { ...odom, frame_id: 'map', orientation: body.toArray() }
  assert.equal(projectScenePose(pose), pose)
  near(new Quaternion().fromArray(projectScenePose(pose)!.orientation!).angleTo(body), 0)
  assert.deepEqual(projectScenePath([{ x: 2, y: 1, z: 0.5 }], 'odom'), [])
})
