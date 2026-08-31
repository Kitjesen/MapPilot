import * as THREE from 'three'

import type { NativeTraversabilityEvent } from '../../../types'
import { createFlatGridGroup, type GroupedMesh } from './layerUtils.ts'

const MAX_CELLS = 1_000_000
const MAX_AGE_S = 2
const FUTURE_TOLERANCE_S = 1

export type NativeTraversabilityLayerState =
  | { status: 'ready'; event: NativeTraversabilityEvent; values: Uint8Array; message: string }
  | { status: 'unavailable' | 'stale' | 'error'; message: string }

function decodeGrid(value: string, count: number): Uint8Array | null {
  try {
    const binary = atob(value)
    if (binary.length !== count) return null
    return Uint8Array.from(binary, char => char.charCodeAt(0))
  } catch {
    return null
  }
}

function finite(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

export function resolveNativeTraversabilityLayer(
  event: NativeTraversabilityEvent | null | undefined,
  options: { nowS: number; allowedFrameIds: Array<string | null | undefined>; maxAgeS?: number },
): NativeTraversabilityLayerState {
  if (!event) return { status: 'unavailable', message: '等待 native 控制可通行性栅格' }
  const count = event.rows * event.cols
  if (!Number.isInteger(event.rows) || !Number.isInteger(event.cols)
    || event.rows <= 0 || event.cols <= 0 || count > MAX_CELLS) {
    return { status: 'error', message: '控制可通行性栅格尺寸无效' }
  }
  if (event.identity_verified !== true
    || event.source !== 'native_nav_client'
    || event.control_authority !== true
    || event.value_semantics !== 'control_risk_0_100') {
    return { status: 'error', message: '控制可通行性身份或语义无效' }
  }
  const allowed = new Set(options.allowedFrameIds.filter((frame): frame is string => typeof frame === 'string' && frame.trim() !== ''))
  if (event.frame_id !== 'map' || !allowed.has(event.frame_id)) {
    return { status: 'error', message: `控制可通行性 frame 不匹配: ${event.frame_id ?? 'missing'}` }
  }
  if (!finite(event.stamp_s) || event.stamp_s <= 0 || !finite(options.nowS)
    || event.stamp_s > options.nowS + FUTURE_TOLERANCE_S) {
    return { status: 'error', message: '控制可通行性时间戳无效' }
  }
  const maxAge = finite(options.maxAgeS) && options.maxAgeS > 0 ? options.maxAgeS : MAX_AGE_S
  if (options.nowS - event.stamp_s > maxAge) {
    return { status: 'stale', message: `控制可通行性已过期 ${Math.floor(options.nowS - event.stamp_s)}s` }
  }
  if (event.reset_epoch <= 0 || event.sequence <= 0 || !finite(event.resolution) || event.resolution <= 0
    || !Array.isArray(event.origin) || event.origin.length < 3 || !event.origin.slice(0, 3).every(finite)
    || !finite(event.yaw) || Math.abs(event.yaw) > 1e-6) {
    return { status: 'error', message: '控制可通行性变换或序列无效' }
  }
  const values = decodeGrid(event.grid_b64, count)
  if (!values || values.some(value => value > 100)) {
    return { status: 'error', message: '控制可通行性数据解码失败' }
  }
  return {
    status: 'ready',
    event,
    values,
    message: `${event.frame_id} · R${event.reset_epoch} S${event.sequence} · native 控制风险 0–100`,
  }
}

function cellColor(value: number): [number, number, number, number] {
  if (value === 0) return [0, 0, 0, 0]
  const t = Math.min(1, Math.max(0, value / 100))
  return [Math.round(40 + 210 * t), Math.round(210 - 170 * t), Math.round(110 - 90 * t), Math.round(80 + 170 * t)]
}

export function createNativeTraversabilityLayer(state: NativeTraversabilityLayerState): GroupedMesh | null {
  if (state.status !== 'ready') return null
  const { event, values } = state
  const canvas = document.createElement('canvas')
  canvas.width = event.cols
  canvas.height = event.rows
  const context = canvas.getContext('2d')
  if (!context) return null
  const image = context.createImageData(event.cols, event.rows)
  values.forEach((value, index) => {
    const [r, g, b, a] = cellColor(value)
    const offset = index * 4
    image.data[offset] = r
    image.data[offset + 1] = g
    image.data[offset + 2] = b
    image.data[offset + 3] = a
  })
  context.putImageData(image, 0, 0)
  const texture = new THREE.CanvasTexture(canvas)
  texture.flipY = false
  texture.minFilter = THREE.LinearFilter
  texture.magFilter = THREE.NearestFilter
  const sizeX = event.cols * event.resolution
  const sizeY = event.rows * event.resolution
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(sizeX, sizeY),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false }),
  ) as GroupedMesh
  mesh.name = 'native-traversability-risk'
  mesh.renderOrder = 6
  // The grid origin is a LingTu cell-boundary origin.  Keep its Z component
  // instead of silently flattening a non-zero native origin into the floor.
  createFlatGridGroup(
    mesh,
    [event.origin[0], event.origin[1]],
    sizeX,
    sizeY,
    event.yaw,
    event.origin[2] + 0.025,
  ).renderOrder = 6
  return mesh
}
