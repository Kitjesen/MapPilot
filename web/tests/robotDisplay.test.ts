import assert from 'node:assert/strict'
import test from 'node:test'
import * as THREE from 'three'
import { lingtuToThree, lingtuQuaternionToThree } from '../src/services/coordinateFrame.ts'
import { createGo2Model, GO2_STANDING_JOINTS } from '../src/components/scene3d/robot/go2Model.ts'
import { GO2_URDF } from '../src/components/scene3d/robot/go2UrdfSpec.ts'
import { PoseInterpolation } from '../src/components/scene3d/robot/poseInterpolation.ts'
import { createPathDisplay, PATH_COLORS } from '../src/components/scene3d/layers/pathDisplay.ts'

// Three.js instance matrices are stored as float32.
const near = (a: number, b: number) => assert.ok(Math.abs(a - b) < 1e-7, `${a} != ${b}`)
const pose = (x: number, stampS: number, yaw = 0, epoch: string = 'run-a') => ({ x, y: 0, z: 0, yaw, stampS, epoch })

test('display interpolates measured XYZ and stops exactly at the latest sample', () => {
  const display = new PoseInterpolation()
  display.update(pose(0, 1), 0)
  display.update({ ...pose(0.1, 1.1), y: 0.2, z: 0.1 }, 100)
  const middle = display.sample(150)!
  near(middle.x, 0.05); near(middle.y, 0.1); near(middle.z, 0.05)
  assert.deepEqual(display.sample(5000), { ...pose(0.1, 1.1), y: 0.2, z: 0.1 })
  display.update({ ...pose(0.1, 1.2), y: 0.2, z: 0.1 }, 5100)
  near(display.sample(5200)!.x, 0.1)
})

test('heading takes the short arc across +/- pi instead of spinning around', () => {
  const display = new PoseInterpolation()
  display.update(pose(0, 1, Math.PI - 0.04), 0)
  display.update(pose(0, 1.1, -Math.PI + 0.04), 100)
  near(display.sample(150)!.yaw, Math.PI)
  near(display.sample(200)!.yaw, -Math.PI + 0.04)
})

test('invalid or lost tracking hides immediately and fresh recovery never sweeps from the old pose', () => {
  const display = new PoseInterpolation()
  display.update(pose(1, 10), 0)
  display.update(pose(1.1, 10.1), 100)
  display.update(null, 110)
  assert.equal(display.sample(110), null)
  display.update(pose(1.2, 10.2), 120)
  near(display.sample(120)!.x, 1.2)
  display.update({ ...pose(1.2, 10.3), z: NaN }, 140)
  assert.equal(display.sample(140), null)
})

test('epoch changes, clock rollback, source gaps and localization jumps snap to measured poses', () => {
  for (const next of [pose(0.2, 1.1, 0, 'run-b'), pose(0.2, 0.5), pose(0.2, 1.4), pose(2, 1.1)]) {
    const display = new PoseInterpolation()
    display.update(pose(0, 1), 0)
    display.update(next, 100)
    assert.deepEqual(display.sample(100), next)
  }
})

test('missing timestamps do not invent an interpolation clock; duplicate samples do not restart animation', () => {
  const display = new PoseInterpolation()
  display.update(pose(0, 1), 0)
  const next = pose(0.1, 1.1)
  display.update(next, 100)
  display.update(next, 150)
  near(display.sample(200)!.x, 0.1)
  const unstamped = { x: 0.3, y: 0, z: 0, yaw: 0 }
  display.update(unstamped, 220)
  assert.deepEqual(display.sample(220), unstamped)
})

test('official Go2 preserves the full URDF tree and anchors standing feet below the body origin', async () => {
  const requested: string[] = []
  const model = createGo2Model(async file => {
    requested.push(file)
    const source = new THREE.Group()
    source.add(new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.01, 0.01)))
    return source
  })
  await model.ready
  assert.equal(model.name, 'go2-official-urdf-model')
  assert.equal(model.userData.loadState, 'ready')
  assert.equal(model.userData.jointTelemetry, false)
  assert.equal(model.userData.jointPose, 'nominal-standing')
  assert.deepEqual(model.position.toArray(), [0, 0, 0])
  assert.deepEqual(model.scale.toArray(), [1, 1, 1])
  assert.equal(requested.length, 7, 'shared official meshes load only once per model')
  for (const link of GO2_URDF.links) assert.ok(model.getObjectByName(`go2:link:${link.name}`))
  for (const joint of GO2_URDF.joints) {
    const frame = model.getObjectByName(`go2:joint:${joint.name}`)!
    assert.equal(frame.parent!.name, `go2:link:${joint.parent}`)
    assert.equal(frame.children[0].name, `go2:link:${joint.child}`)
    assert.deepEqual(frame.position.toArray(), joint.origin.xyz)
    assert.deepEqual(frame.userData.axis, joint.axis)
  }
  model.updateMatrixWorld(true)
  for (const leg of ['FL', 'FR', 'RL', 'RR']) {
    const foot = model.getObjectByName(`go2:link:${leg}_foot`)!.getWorldPosition(new THREE.Vector3())
    near(foot.x, leg.startsWith('F') ? 0.1934 : -0.1934)
    near(foot.y, -0.426 * Math.cos(GO2_STANDING_JOINTS.thigh))
    assert.equal(Math.sign(foot.z), leg.endsWith('L') ? -1 : 1)
  }
  const radar = model.getObjectByName('go2:joint:radar_joint')!
  const radarForward = new THREE.Vector3(1, 0, 0).applyQuaternion(radar.quaternion)
  near(radarForward.x, Math.cos(2.8782))
  near(radarForward.z, -Math.sin(2.8782))
  const leftCalf = model.getObjectByName('go2:visual:FL_calf')!.children[0].children[0] as THREE.Mesh
  const rearCalf = model.getObjectByName('go2:visual:RL_calf')!.children[0].children[0] as THREE.Mesh
  assert.equal(leftCalf.geometry, rearCalf.geometry, 'clones share immutable mesh geometry')
})

test('Go2 mesh loads finishing after scene disposal cannot reattach or leak geometry', async () => {
  let finish!: (group: THREE.Group) => void
  const pending = new Promise<THREE.Group>(resolve => { finish = resolve })
  const model = createGo2Model(() => pending)
  model.cancelLoading()
  const geometry = new THREE.BoxGeometry()
  let disposed = 0
  geometry.addEventListener('dispose', () => { disposed++ })
  const source = new THREE.Group()
  source.add(new THREE.Mesh(geometry))
  finish(source)
  await model.ready
  let visuals = 0
  model.traverse(node => { if (node.name.startsWith('go2:visual:')) visuals++ })
  assert.equal(visuals, 0)
  assert.ok(disposed > 0)
})

test('a missing official asset reports failure and never displays a partial Go2', async t => {
  t.mock.method(console, 'warn', () => {})
  const model = createGo2Model(async file => {
    if (file === 'dae/base.dae') throw new Error('asset unavailable')
    return new THREE.Group()
  })
  await model.ready
  assert.equal(model.userData.loadState, 'error')
  assert.equal(model.getObjectByName('go2:urdf-z-up')!.visible, false)
})

test('displayed path segments and heights match input; no extra spline cuts a corner', () => {
  const group = createPathDisplay([{ x: 0, y: 0, z: 0.4 }, { x: 1, y: 0, z: 0.4 }, { x: 1, y: 1, z: 0.6 }], 'local')!
  const segments = group.children[0] as THREE.InstancedMesh
  assert.equal(segments.count, 2)
  assert.equal((segments.material as THREE.MeshBasicMaterial).color.getHex(), PATH_COLORS.local)
  const matrix = new THREE.Matrix4()
  segments.getMatrixAt(0, matrix)
  const start = new THREE.Vector3(0, -0.5, 0).applyMatrix4(matrix)
  const end = new THREE.Vector3(0, 0.5, 0).applyMatrix4(matrix)
  near(start.x, 0); near(start.y, 0.425); near(start.z, 0)
  near(end.x, 1); near(end.y, 0.425); near(end.z, 0)
  assert.equal(createPathDisplay([], 'global'), null)
  assert.equal(createPathDisplay([{ x: 0, y: 0 }, { x: NaN, y: 0 }, { x: 2, y: 0 }], 'global'), null)
})


test('body roll and pitch rotate the model in the same basis as map points', () => {
  for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1]]) {
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(...axis), 0.4)
    const point = new THREE.Vector3(0.3, 0.2, -0.3)
    const expected = lingtuToThree(point.clone().applyQuaternion(q).toArray())
    const actual = new THREE.Vector3(...lingtuToThree(point.toArray()))
      .applyQuaternion(new THREE.Quaternion(...lingtuQuaternionToThree(q.toArray())))
    actual.toArray().forEach((value, i) => near(value, expected[i]))
  }
})

test('measured body tilt interpolates on the shortest quaternion arc and resets with epoch', () => {
  const display = new PoseInterpolation()
  const a = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), 0.1)
  const b = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), 0.3)
  display.update({ ...pose(0, 1), orientation: a.toArray() }, 0)
  display.update({ ...pose(0, 1.1), orientation: b.toArray().map(v => -v) as [number, number, number, number] }, 100)
  const middle = new THREE.Quaternion().fromArray(display.sample(150)!.orientation!)
  near(middle.angleTo(a), 0.1)
  const reset = { ...pose(0, 1.2, 0, 'run-b'), orientation: b.toArray() }
  display.update(reset, 160)
  assert.deepEqual(display.sample(160), reset)
})
