import { Quaternion, Vector3 } from 'three'
import assert from 'node:assert/strict'
import test from 'node:test'
import { currentNativeLocalPath, freshSource, projectScenePath, projectScenePose, scanCanStandAlone, scenePoseEpoch, sceneTrailStorageKey, sceneRelocalizationSeed, activeMapRelocalizationTarget } from '../src/services/sceneTelemetry.ts'
import { PoseInterpolation } from '../src/components/scene3d/robot/poseInterpolation.ts'

const tf = { valid: true, frame_id: 'map', child_frame_id: 'odom', tx: 10, ty: -2, tz: 3,
  qx: 0, qy: 0, qz: Math.SQRT1_2, qw: Math.SQRT1_2, ts: 100 }
const odom = { type: 'odometry' as const, frame_id: 'odom', x: 2, y: 1, z: 0.5, yaw: 0, vx: 0.5 }
const near = (actual: number, expected: number) => assert.ok(Math.abs(actual - expected) < 1e-9)

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

test('published nonidentity map-from-odom transform projects position, height and heading once', () => {
  const pose = projectScenePose(odom, tf)
  assert.ok(pose)
  near(pose.x, 9); near(pose.y, 0); near(pose.z!, 3.5); near(pose.yaw, Math.PI / 2)
  assert.equal(pose.vx, 0.5, 'body-frame forward speed does not rotate with the map')
  assert.equal(projectScenePose(pose, tf), pose, 'already-map pose must not be transformed twice')
  const path = projectScenePath([{ x: 2, y: 1, z: 0.5 }], 'odom', tf)
  near(path[0].x, 9); near(path[0].z!, 3.5)
})

test('unknown, invalid, or reversed frame transforms never silently relabel odometry', () => {
  assert.equal(projectScenePose(odom, null), null)
  assert.equal(projectScenePose(odom, { ...tf, valid: false }), null)
  assert.equal(projectScenePose(odom, { ...tf, frame_id: 'odom', child_frame_id: 'map' }), null)
  assert.deepEqual(projectScenePath([{ x: 1, y: 2 }], 'livox_frame', tf), [])
})

test('clicked relocalization keeps map XY and uses the projected map heading, never a default or raw odom heading', () => {
  const seed = sceneRelocalizationSeed(20, 30, projectScenePose(odom, tf))
  assert.ok(seed)
  assert.equal(seed.x, 20)
  assert.equal(seed.y, 30)
  near(seed.yaw, Math.PI / 2)
  assert.equal(sceneRelocalizationSeed(20, 30, odom), null)
  assert.equal(sceneRelocalizationSeed(20, 30, null), null)
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
  const native = { stamp_s: 100, planning_frame_id: 'odom', local_path: [[2, 1, 0.5], [3, 1, 0.5]],
    last_local: { tracking: { active: true } } }
  assert.equal(currentNativeLocalPath(native, true, 101, tf).length, 2)
  assert.deepEqual(currentNativeLocalPath({ ...native, last_local: { tracking: { active: false } } }, true, 101, tf), [])
  assert.deepEqual(currentNativeLocalPath({ ...native, local_path: [] }, true, 101, tf), [])
  assert.deepEqual(currentNativeLocalPath(native, true, 110, tf), [])
  assert.deepEqual(currentNativeLocalPath(native, false, 101, tf), [])
  assert.equal(freshSource(undefined, 101), false)
})

test('registered scan is visible without an accumulated cloud only in the published map epoch', () => {
  const scan = { count: 4000, frameId: 'map', epoch: 2 }
  assert.equal(scanCanStandAlone(scan, { count: 0, frameId: null, epoch: null }), true)
  assert.equal(scanCanStandAlone(scan, { count: 0, frameId: 'map', epoch: 2 }), true)
  assert.equal(scanCanStandAlone(scan, { count: 0, frameId: 'map', epoch: 1 }), false)
  assert.equal(scanCanStandAlone({ ...scan, frameId: 'body' }, { count: 0, frameId: null, epoch: null }), false)
})


test('map transform composes full body tilt once, not only heading', () => {
  const body = new Quaternion().setFromAxisAngle(new Vector3(1, 0, 0), 0.4)
  const mapped = projectScenePose({ ...odom, orientation: body.toArray() }, tf)!
  const expected = new Quaternion(tf.qx, tf.qy, tf.qz, tf.qw).multiply(body)
  near(new Quaternion().fromArray(mapped.orientation!).angleTo(expected), 0)
  assert.equal(projectScenePose(mapped, tf), mapped)
})
