import * as THREE from 'three'

import type { MapSceneEvent, MapSceneLayer } from '../../../types'

const MAX_AGE_S = 5
const FUTURE_TOLERANCE_S = 1

export const LOOP_CLOSURE_LAYER_COLORS = {
  accepted: 0x35f26d,
  rejected: 0xffa629,
  node: 0x8bf5ff,
} as const

export interface LoopClosureConstraint {
  fromIndex: number
  toIndex: number
  from: [number, number, number]
  to: [number, number, number]
  state: 'accepted' | 'rejected'
  reason?: string
  rmseM?: number
}

interface LoopClosureMapSceneLayer extends MapSceneLayer {
  id: 'localization.loop_constraints'
  type: 'loop_constraints'
  frame_id: string
  stamp_s: number
  producer_boot_id: string
  reset_epoch: number
  observation_sequence: number
  generation: number
  online: true
  identity_verified: true
  constraint_semantics: 'loop_closure_validation_v1'
  constraints: unknown[]
}

export type LoopClosureLayerState =
  | {
      status: 'ready'
      layer: LoopClosureMapSceneLayer
      accepted: LoopClosureConstraint[]
      rejected: LoopClosureConstraint[]
      message: string
    }
  | { status: 'unavailable' | 'stale' | 'error'; message: string }

function finiteNumber(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) ? value : null
}

function positiveInteger(value: unknown): number | null {
  return typeof value === 'number' && Number.isInteger(value) && value > 0 ? value : null
}

function point(value: unknown): [number, number, number] | null {
  if (!Array.isArray(value) || value.length < 2) return null
  const x = finiteNumber(value[0])
  const y = finiteNumber(value[1])
  const z = value.length > 2 ? finiteNumber(value[2]) : 0
  return x === null || y === null || z === null ? null : [x, y, z]
}

function loopLayer(scene: MapSceneEvent): MapSceneLayer | undefined {
  return scene.layers.find(layer => (
    layer.id === 'localization.loop_constraints'
    || layer.topic === '/localization/loop_constraints'
  ))
}

function parseConstraint(value: unknown): LoopClosureConstraint | null {
  if (!value || typeof value !== 'object' || Array.isArray(value)) return null
  const raw = value as Record<string, unknown>
  const fromIndex = finiteNumber(raw.from_index)
  const toIndex = finiteNumber(raw.to_index)
  const from = point(raw.from)
  const to = point(raw.to)
  if (
    fromIndex === null
    || toIndex === null
    || !Number.isInteger(fromIndex)
    || !Number.isInteger(toIndex)
    || fromIndex < 0
    || toIndex < 0
    || !from
    || !to
  ) return null

  if (raw.state !== 'accepted' && raw.state !== 'rejected') return null
  if (typeof raw.geometrically_verified !== 'boolean') return null
  if (raw.state === 'accepted' && raw.geometrically_verified !== true) return null
  if (raw.reason !== undefined && typeof raw.reason !== 'string') return null
  const rmse = finiteNumber(raw.rmse_m)
  if (raw.rmse_m !== undefined && (rmse === null || rmse < 0)) return null
  return {
    fromIndex,
    toIndex,
    from,
    to,
    state: raw.state,
    reason: raw.reason,
    rmseM: rmse === null ? undefined : rmse,
  }
}

export function resolveLoopClosureLayer(
  scene: MapSceneEvent | null | undefined,
  options: { nowS: number; allowedFrameIds: Array<string | null | undefined>; maxAgeS?: number },
): LoopClosureLayerState {
  if (!scene) return { status: 'unavailable', message: '等待在线回环验证数据' }
  const rawLayer = loopLayer(scene)
  if (!rawLayer) return { status: 'unavailable', message: '当前定位后端未发布在线回环验证层' }

  const layer = rawLayer as Partial<LoopClosureMapSceneLayer>
  const frameId = String(layer.frame_id ?? '').trim()
  const allowedFrames = new Set(
    options.allowedFrameIds.filter((value): value is string => typeof value === 'string' && value.trim() !== ''),
  )
  if (
    layer.id !== 'localization.loop_constraints'
    || layer.type !== 'loop_constraints'
    || scene.frame_id !== 'map'
    || frameId !== 'map'
    || !allowedFrames.has('map')
  ) {
    return { status: 'error', message: `回环验证 frame 不匹配: ${frameId || 'missing'}` }
  }
  if (
    layer.online !== true
    || layer.identity_verified !== true
    || layer.constraint_semantics !== 'loop_closure_validation_v1'
  ) {
    return { status: 'error', message: '回环验证来源或语义无效' }
  }

  const sceneIdentity = scene.metadata ?? {}
  const producerBootId = String(layer.producer_boot_id ?? '').trim()
  const sceneProducerBootId = String(sceneIdentity.producer_boot_id ?? '').trim()
  const resetEpoch = positiveInteger(layer.reset_epoch)
  const sceneResetEpoch = positiveInteger(sceneIdentity.reset_epoch)
  const observationSequence = positiveInteger(layer.observation_sequence)
  const sceneObservationSequence = positiveInteger(sceneIdentity.observation_sequence)
  const generation = positiveInteger(layer.generation)
  const sceneGeneration = positiveInteger(sceneIdentity.generation)
  if (
    !producerBootId
    || !sceneProducerBootId
    || producerBootId !== sceneProducerBootId
    || resetEpoch === null
    || resetEpoch !== sceneResetEpoch
    || observationSequence === null
    || observationSequence !== sceneObservationSequence
    || generation === null
    || generation !== sceneGeneration
  ) {
    return { status: 'error', message: '回环验证未绑定当前 SLAM 坐标时代' }
  }

  const nowS = finiteNumber(options.nowS)
  const stampS = finiteNumber(layer.stamp_s)
  if (nowS === null || stampS === null || stampS <= 0 || stampS > nowS + FUTURE_TOLERANCE_S) {
    return { status: 'error', message: '回环验证时间戳无效' }
  }
  const maxAgeS = finiteNumber(options.maxAgeS)
  const freshnessLimit = maxAgeS !== null && maxAgeS > 0 ? maxAgeS : MAX_AGE_S
  if (nowS - stampS > freshnessLimit) {
    return { status: 'stale', message: `在线回环验证已过期 ${Math.floor(nowS - stampS)}s` }
  }
  if (!Array.isArray(layer.constraints)) {
    return { status: 'error', message: '回环约束数据格式无效' }
  }

  const parsedConstraints = layer.constraints.map(parseConstraint)
  if (parsedConstraints.some(item => item === null)) {
    return { status: 'error', message: '回环约束包含无效记录' }
  }
  const constraints = parsedConstraints as LoopClosureConstraint[]
  const accepted = constraints.filter(item => item.state === 'accepted')
  const rejected = constraints.filter(item => item.state === 'rejected')
  return {
    status: 'ready',
    layer: layer as LoopClosureMapSceneLayer,
    accepted,
    rejected,
    message: `在线验证 · 已接受 ${accepted.length} · 已拒绝 ${rejected.length}`,
  }
}

function edgeSegments(
  constraints: LoopClosureConstraint[],
  color: number,
  name: string,
  opacity: number,
): THREE.LineSegments | null {
  if (constraints.length === 0) return null
  const points: THREE.Vector3[] = []
  for (const constraint of constraints) {
    points.push(
      new THREE.Vector3(constraint.from[0], constraint.from[2] + 0.12, -constraint.from[1]),
      new THREE.Vector3(constraint.to[0], constraint.to[2] + 0.12, -constraint.to[1]),
    )
  }
  const line = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthTest: false }),
  )
  line.name = name
  line.renderOrder = 46
  return line
}

export function createLoopClosureLayer(state: LoopClosureLayerState): THREE.Group | null {
  if (state.status !== 'ready') return null
  const group = new THREE.Group()
  group.name = 'online-loop-closure-validation'
  const accepted = edgeSegments(
    state.accepted,
    LOOP_CLOSURE_LAYER_COLORS.accepted,
    'accepted-loop-constraints',
    0.95,
  )
  const rejected = edgeSegments(
    state.rejected,
    LOOP_CLOSURE_LAYER_COLORS.rejected,
    'rejected-loop-candidates',
    0.45,
  )
  if (accepted) group.add(accepted)
  if (rejected) group.add(rejected)
  group.userData.acceptedCount = state.accepted.length
  group.userData.rejectedCount = state.rejected.length
  return group.children.length > 0 ? group : null
}

export function disposeLoopClosureLayer(
  scene: THREE.Scene,
  group: THREE.Group | null | undefined,
): void {
  if (!group) return
  scene.remove(group)
  group.traverse(object => {
    const line = object as THREE.LineSegments
    line.geometry?.dispose()
    const material = line.material
    if (Array.isArray(material)) material.forEach(item => item.dispose())
    else material?.dispose()
  })
}
