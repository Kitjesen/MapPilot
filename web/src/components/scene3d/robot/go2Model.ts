import * as THREE from 'three'
import {
  GO2_JOINT_NAMES,
  GO2_JOINT_SOURCE_MAX_AGE_S,
  go2JointSampleStatus,
  type Go2JointName,
  type Go2JointSample,
} from '../../../services/robotJointState.ts'
import { GO2_URDF } from './go2UrdfSpec.ts'

export const GO2_ASSET_ROOT = '/assets/robots/go2/'
// Display pose only. Joint telemetry is not currently provided by the scene.
export const GO2_STANDING_JOINTS = { hip: 0, thigh: 0.8, calf: -1.6 } as const
export const GO2_JOINT_INTERPOLATION_MIN_MS = 16
export const GO2_JOINT_INTERPOLATION_MAX_MS = 80
export const GO2_JOINT_INTERPOLATION_DEFAULT_MS = 33
export const GO2_JOINT_STALE_AFTER_MS = GO2_JOINT_SOURCE_MAX_AGE_S * 1000
type MeshLoader = (path: string) => Promise<THREE.Object3D>
export type Go2JointStateStatus = 'fallback' | 'live' | 'stale'
export type Go2Model = THREE.Group & {
  ready: Promise<void>
  cancelLoading: () => void
  ingestJointState: (sample: Go2JointSample, nowMs: number) => boolean
  updateJointPose: (nowMs: number) => boolean
  jointStateStatus: (nowMs: number) => Go2JointStateStatus
}

interface JointFrame {
  frame: THREE.Group
  origin: THREE.Quaternion
  axis: THREE.Vector3
}

function standingAngle(name: Go2JointName): number {
  const kind = name.split('_')[1] as keyof typeof GO2_STANDING_JOINTS
  return GO2_STANDING_JOINTS[kind]
}

function disposeMeshTree(root: THREE.Object3D): void {
  const geometries = new Set<THREE.BufferGeometry>()
  const materials = new Set<THREE.Material>()
  root.traverse(node => {
    if (!(node instanceof THREE.Mesh)) return
    geometries.add(node.geometry)
    for (const material of Array.isArray(node.material) ? node.material : [node.material]) materials.add(material)
  })
  geometries.forEach(geometry => geometry.dispose())
  materials.forEach(material => material.dispose())
}

/** Return the original URDF Z-up mesh, undoing only ColladaLoader's Y-up wrapper. */
async function loadCollada(path: string): Promise<THREE.Object3D> {
  const { ColladaLoader } = await import('three/examples/jsm/loaders/ColladaLoader.js')
  const collada = await new ColladaLoader().loadAsync(`${GO2_ASSET_ROOT}${path}`)
  if (!collada) throw new Error(`Go2 mesh could not be parsed: ${path}`)
  const { scene } = collada
  scene.rotation.set(0, 0, 0)
  const converted = new Map<THREE.Material, THREE.MeshStandardMaterial>()
  scene.traverse(node => {
    if (!(node instanceof THREE.Mesh)) return
    const neutral = (original: THREE.Material): THREE.MeshStandardMaterial => {
      const existing = converted.get(original)
      if (existing) return existing
      const color = (original as THREE.MeshPhongMaterial).color ?? new THREE.Color(0x808080)
      const gray = color.r * 0.2126 + color.g * 0.7152 + color.b * 0.0722
      const material = new THREE.MeshStandardMaterial({ color: new THREE.Color(gray, gray, gray),
        roughness: 0.64, metalness: 0.16, opacity: original.opacity, transparent: original.transparent })
      material.name = original.name
      converted.set(original, material)
      original.dispose()
      return material
    }
    node.material = Array.isArray(node.material) ? node.material.map(neutral) : neutral(node.material)
  })
  return scene
}

/** Build the complete official Go2 URDF tree, anchored at the published body/base origin. */
export function createGo2Model(loadMesh: MeshLoader = loadCollada): Go2Model {
  const display = new THREE.Group() as Go2Model
  display.name = 'go2-official-urdf-model'
  display.userData.anchor = 'body'
  display.userData.jointTelemetry = false
  display.userData.jointPose = 'nominal-standing'
  display.userData.assetSource = 'unitreerobotics/unitree_ros@7d6075f7f58588b189b940130e3edab3c839b2df'
  display.userData.loadState = 'loading'
  const urdfRoot = new THREE.Group()
  urdfRoot.name = 'go2:urdf-z-up'
  urdfRoot.rotation.x = -Math.PI / 2
  urdfRoot.visible = false
  display.add(urdfRoot)

  const links = new Map<string, THREE.Group>()
  const measuredJoints = new Map<Go2JointName, JointFrame>()
  for (const link of GO2_URDF.links) {
    const group = new THREE.Group()
    group.name = `go2:link:${link.name}`
    links.set(link.name, group)
  }
  urdfRoot.add(links.get('base')!)
  for (const joint of GO2_URDF.joints) {
    const frame = new THREE.Group()
    frame.name = `go2:joint:${joint.name}`
    frame.position.fromArray(joint.origin.xyz)
    frame.quaternion.setFromEuler(new THREE.Euler(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], 'ZYX'))
    const origin = frame.quaternion.clone()
    let angle = 0
    if (joint.type === 'revolute') {
      const name = joint.name as Go2JointName
      angle = standingAngle(name)
      const axis = new THREE.Vector3(...joint.axis).normalize()
      frame.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(axis, angle))
      measuredJoints.set(name, { frame, origin, axis })
    }
    frame.userData.jointAngle = angle
    frame.userData.axis = [...joint.axis]
    links.get(joint.parent)!.add(frame)
    frame.add(links.get(joint.child)!)
  }

  let lastStampS: number | null = null
  let receivedAtMs: number | null = null
  let sourceAgeS = 0
  let transitionStartMs = 0
  let transitionDurationMs = GO2_JOINT_INTERPOLATION_DEFAULT_MS
  let hasMeasuredPose = false
  const current = Object.fromEntries(GO2_JOINT_NAMES.map(name => [name, standingAngle(name)])) as Record<Go2JointName, number>
  const start = { ...current }
  const target = { ...current }

  const applyAngle = (name: Go2JointName, angle: number) => {
    const joint = measuredJoints.get(name)!
    joint.frame.quaternion.copy(joint.origin)
      .multiply(new THREE.Quaternion().setFromAxisAngle(joint.axis, angle))
    joint.frame.userData.jointAngle = angle
  }

  display.jointStateStatus = nowMs => {
    if (receivedAtMs === null) return 'fallback'
    return go2JointSampleStatus({
      stamp: lastStampS!, sourceAgeS, receivedAtMs, positions: target,
    }, nowMs)
  }

  display.updateJointPose = nowMs => {
    if (!Number.isFinite(nowMs) || receivedAtMs === null
      || display.jointStateStatus(nowMs) === 'stale') return false
    const progress = Math.max(0, Math.min(1, (nowMs - transitionStartMs) / transitionDurationMs))
    for (const name of GO2_JOINT_NAMES) {
      current[name] = start[name] + (target[name] - start[name]) * progress
      applyAngle(name, current[name])
    }
    return true
  }

  display.ingestJointState = (sample, nowMs) => {
    if (!Number.isFinite(nowMs) || !Number.isFinite(sample.stamp)
      || !Number.isFinite(sample.receivedAtMs)
      || go2JointSampleStatus(sample, nowMs) === 'stale'
      || (lastStampS !== null && sample.stamp <= lastStampS)) return false
    for (const name of GO2_JOINT_NAMES) {
      if (!Number.isFinite(sample.positions[name])) return false
    }
    if (hasMeasuredPose) display.updateJointPose(nowMs)
    const previousReceipt = receivedAtMs
    for (const name of GO2_JOINT_NAMES) {
      start[name] = current[name]
      target[name] = sample.positions[name]
    }
    lastStampS = sample.stamp
    receivedAtMs = sample.receivedAtMs
    sourceAgeS = sample.sourceAgeS
    transitionStartMs = nowMs
    transitionDurationMs = previousReceipt === null
      ? GO2_JOINT_INTERPOLATION_DEFAULT_MS
      : Math.max(GO2_JOINT_INTERPOLATION_MIN_MS, Math.min(
          GO2_JOINT_INTERPOLATION_MAX_MS, sample.receivedAtMs - previousReceipt,
        ))
    if (!hasMeasuredPose) {
      for (const name of GO2_JOINT_NAMES) {
        current[name] = target[name]
        start[name] = target[name]
        applyAngle(name, current[name])
      }
      hasMeasuredPose = true
    }
    display.userData.jointTelemetry = true
    display.userData.jointPose = 'measured'
    return true
  }

  let cancelled = false
  const sources = new Map<string, Promise<THREE.Object3D>>()
  const getMesh = (path: string): Promise<THREE.Object3D> => {
    let source = sources.get(path)
    if (!source) {
      source = loadMesh(path).then(mesh => {
        if (cancelled) disposeMeshTree(mesh)
        return mesh
      })
      sources.set(path, source)
    }
    return source
  }
  const jobs = GO2_URDF.links.flatMap(link => link.visuals.map(async visual => {
    const source = await getMesh(visual.mesh)
    if (cancelled) return
    const frame = new THREE.Group()
    frame.name = `go2:visual:${link.name}`
    frame.position.fromArray(visual.origin.xyz)
    frame.quaternion.setFromEuler(new THREE.Euler(visual.origin.rpy[0], visual.origin.rpy[1], visual.origin.rpy[2], 'ZYX'))
    frame.scale.fromArray(visual.scale)
    frame.add(source.clone(true))
    links.get(link.name)!.add(frame)
  }))
  display.ready = Promise.allSettled(jobs).then(results => {
    if (cancelled) return
    const failures = results.filter(result => result.status === 'rejected')
    display.userData.loadState = failures.length ? 'error' : 'ready'
    urdfRoot.visible = failures.length === 0
    if (failures.length) console.warn('Go2 official model assets could not all be loaded', failures)
  })
  // Scene3D owns attached geometry disposal; this also releases pending loads on unmount.
  display.cancelLoading = () => {
    cancelled = true
  }
  return display
}
