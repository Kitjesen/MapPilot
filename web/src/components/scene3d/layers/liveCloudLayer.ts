import * as THREE from 'three'

import type { BinaryCloud } from '../../../hooks/useBinaryCloud'

const MIN_CONTROL_POINT_SIZE = 0.02
const MAX_CONTROL_POINT_SIZE = 0.4
const MIN_WORLD_POINT_SIZE = 0.004
const MAX_WORLD_POINT_SIZE = 0.05
export const DEFAULT_POINT_SIZE = 0.32

export interface LiveCloudLayerOptions {
  color?: number
  opacity?: number
  pointSizeScale?: number
  renderOrder?: number
  vertexColors?: boolean
}

export function pointSizeToWorld(pointSize: number): number {
  const safePointSize = Number.isFinite(pointSize) ? pointSize : DEFAULT_POINT_SIZE
  const clamped = Math.max(MIN_CONTROL_POINT_SIZE, Math.min(MAX_CONTROL_POINT_SIZE, safePointSize))
  const normalized = (clamped - MIN_CONTROL_POINT_SIZE) / (MAX_CONTROL_POINT_SIZE - MIN_CONTROL_POINT_SIZE)
  return MIN_WORLD_POINT_SIZE + normalized * (MAX_WORLD_POINT_SIZE - MIN_WORLD_POINT_SIZE)
}

export function createLiveCloudLayer(
  cloud: BinaryCloud,
  pointSize: number,
  options: LiveCloudLayerOptions = {},
): THREE.Points | null {
  const count = Math.min(cloud.count, Math.floor(cloud.positions.length / 3))
  if (count <= 0) return null

  const positionData = cloud.positions.length === count * 3
    ? cloud.positions
    : cloud.positions.subarray(0, count * 3)
  const colorData = cloud.colors.length >= count * 3
    ? cloud.colors.subarray(0, count * 3)
    : null
  const useVertexColors = options.vertexColors ?? Boolean(colorData)

  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(positionData, 3))
  if (colorData && useVertexColors) geometry.setAttribute('color', new THREE.BufferAttribute(colorData, 3))

  const material = new THREE.PointsMaterial({
    size: pointSizeToWorld(pointSize) * (options.pointSizeScale ?? 1),
    sizeAttenuation: true,
    vertexColors: useVertexColors,
    color: options.color ?? 0x54f4ff,
    transparent: true,
    opacity: options.opacity ?? 0.96,
    depthWrite: false,
  })

  const points = new THREE.Points(geometry, material)
  points.frustumCulled = false
  points.renderOrder = options.renderOrder ?? 8
  return points
}

function updateBufferAttribute(
  geometry: THREE.BufferGeometry,
  name: string,
  data: Float32Array,
): void {
  const required = data.length
  const current = geometry.getAttribute(name) as THREE.BufferAttribute | undefined
  if (current && current.array instanceof Float32Array && current.array.length >= required) {
    current.array.set(data, 0)
    current.needsUpdate = true
    return
  }
  const vertices = Math.ceil(required / 3)
  let capacityVertices = 4096
  while (capacityVertices < vertices) capacityVertices = Math.ceil(capacityVertices * 1.5)
  const next = new Float32Array(capacityVertices * 3)
  next.set(data, 0)
  geometry.setAttribute(name, new THREE.BufferAttribute(next, 3))
}

export function upsertLiveCloudLayer(
  scene: THREE.Scene,
  current: THREE.Points | null,
  cloud: BinaryCloud,
  pointSize: number,
  options: LiveCloudLayerOptions = {},
): THREE.Points | null {
  const count = Math.min(cloud.count, Math.floor(cloud.positions.length / 3))
  if (count <= 0) {
    if (current) current.visible = false
    return current
  }

  const positionData = cloud.positions.length === count * 3
    ? cloud.positions
    : cloud.positions.subarray(0, count * 3)
  const colorData = cloud.colors.length >= count * 3
    ? cloud.colors.subarray(0, count * 3)
    : null
  const useVertexColors = options.vertexColors ?? Boolean(colorData)

  if (!current) {
    const created = createLiveCloudLayer(cloud, pointSize, options)
    if (created) scene.add(created)
    return created
  }

  current.visible = true
  current.renderOrder = options.renderOrder ?? current.renderOrder
  updateBufferAttribute(current.geometry, 'position', positionData)
  if (colorData && useVertexColors) updateBufferAttribute(current.geometry, 'color', colorData)
  else current.geometry.deleteAttribute('color')
  current.geometry.setDrawRange(0, count)

  const material = current.material
  const size = pointSizeToWorld(pointSize) * (options.pointSizeScale ?? 1)
  if (!Array.isArray(material)) {
    const pointsMaterial = material as THREE.PointsMaterial
    if (pointsMaterial.size !== size) pointsMaterial.size = size
    const opacity = options.opacity ?? pointsMaterial.opacity
    if (pointsMaterial.opacity !== opacity) pointsMaterial.opacity = opacity
    if (options.color !== undefined && pointsMaterial.color.getHex() !== options.color) {
      pointsMaterial.color.set(options.color)
    }
    if (pointsMaterial.vertexColors !== useVertexColors) {
      pointsMaterial.vertexColors = useVertexColors
      pointsMaterial.needsUpdate = true
    }
  }
  return current
}

export function disposeLiveCloudLayer(scene: THREE.Scene, points: THREE.Points | null) {
  if (!points) return
  scene.remove(points)
  points.geometry.dispose()
  const material = points.material
  if (Array.isArray(material)) material.forEach(m => m.dispose())
  else material.dispose()
}
