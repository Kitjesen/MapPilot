import * as THREE from 'three'

import type { ElevationMapSceneLayer, MapSceneEvent, MapSceneLayer } from '../../../types'
import { createHeightGridGroup, type GroupedMesh } from './layerUtils.ts'
import { lingtuToThree } from '../../../services/coordinateFrame.ts'

export const ELEVATION_MAX_CELLS = 131_072
export const ELEVATION_DEFAULT_MAX_AGE_S = 5
const FUTURE_STAMP_TOLERANCE_S = 1

export interface ElevationLayerOptions {
  nowS: number
  savedMapFrameId?: string | null
  maxAgeS?: number
}

export type ElevationLayerState =
  | {
      status: 'ready'
      layer: ElevationMapSceneLayer
      heights: Float32Array
      minZ: number
      maxZ: number
      validCount: number
      message: string
    }
  | {
      status: 'unavailable' | 'stale' | 'error'
      message: string
    }

function isElevationLayer(layer: MapSceneLayer): boolean {
  return layer.id === 'maps.elevation'
    || layer.topic === '/maps/elevation'
}

function finiteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

function nonNegativeInteger(value: unknown): value is number {
  return finiteNumber(value) && Number.isInteger(value) && value >= 0
}

function positiveInteger(value: unknown): value is number {
  return nonNegativeInteger(value) && value > 0
}

function validOrigin(value: unknown): value is [number, number, number] {
  return Array.isArray(value)
    && value.length >= 3
    && finiteNumber(value[0])
    && finiteNumber(value[1])
    && finiteNumber(value[2])
}

function error(message: string): ElevationLayerState {
  return { status: 'error', message }
}

let decodedGridCache: { gridB64: string; cellCount: number; values: Float32Array } | null = null

function decodeFloat32LE(gridB64: string, cellCount: number): Float32Array | null {
  if (decodedGridCache?.gridB64 === gridB64 && decodedGridCache.cellCount === cellCount) {
    return decodedGridCache.values
  }
  try {
    const binary = atob(gridB64)
    if (binary.length !== cellCount * 4) return null
    const view = new DataView(new ArrayBuffer(binary.length))
    for (let index = 0; index < binary.length; index++) {
      view.setUint8(index, binary.charCodeAt(index))
    }
    const values = new Float32Array(cellCount)
    for (let index = 0; index < cellCount; index++) {
      values[index] = view.getFloat32(index * 4, true)
    }
    decodedGridCache = { gridB64, cellCount, values }
    return values
  } catch {
    return null
  }
}

export function resolveElevationLayer(
  mapScene: MapSceneEvent | null | undefined,
  options: ElevationLayerOptions,
): ElevationLayerState {
  const candidate = mapScene?.layers.find(isElevationLayer)
  if (!candidate || !mapScene) return { status: 'unavailable', message: '等待最低观测高程数据' }
  const layer = candidate as ElevationMapSceneLayer

  const layerProducerBootId = String(layer.producer_boot_id ?? '').trim()
  const sceneProducerValue = mapScene.metadata?.producer_boot_id
  const sceneProducerBootId = sceneProducerValue == null ? '' : String(sceneProducerValue).trim()
  if (!layerProducerBootId || (sceneProducerValue != null
    && (!sceneProducerBootId || layerProducerBootId !== sceneProducerBootId))) {
    return error('最低观测高程 producer identity 无效')
  }
  const sceneResetEpoch = mapScene.metadata?.reset_epoch
  if (!nonNegativeInteger(layer.reset_epoch)
    || (sceneResetEpoch !== undefined
      && (!nonNegativeInteger(sceneResetEpoch) || layer.reset_epoch !== sceneResetEpoch))) {
    return error('最低观测高程 reset epoch 无效')
  }

  const allowedFrames = new Set(
    [mapScene.frame_id, options.savedMapFrameId]
      .filter((value): value is string => typeof value === 'string' && value.trim().length > 0)
      .map(value => value.trim()),
  )
  if (typeof layer.frame_id !== 'string' || !layer.frame_id.trim()) return error('最低观测高程 frame 缺失')
  if (allowedFrames.size === 0 || !allowedFrames.has(layer.frame_id.trim())) {
    return error(`最低观测高程 frame 不匹配: ${layer.frame_id}`)
  }

  const sceneGenerationValue = mapScene.metadata?.generation
  const latestGeneration = sceneGenerationValue ?? layer.retained_for_generation ?? layer.generation
  const payloadGeneration = layer.payload_generation ?? layer.generation
  if (!nonNegativeInteger(latestGeneration) || !nonNegativeInteger(payloadGeneration)
    || (sceneGenerationValue !== undefined && !nonNegativeInteger(sceneGenerationValue))) {
    return error('最低观测高程 generation 无效')
  }
  const generationMatches = layer.payload_retained === true
    ? payloadGeneration <= latestGeneration
    : payloadGeneration === latestGeneration
  if (!generationMatches) return error('最低观测高程 generation 与场景不一致')
  const sceneLive = mapScene.metadata?.live
  if ((sceneLive !== undefined && sceneLive !== true)
    || (sceneLive !== true && layer.live !== true)) return error('最低观测高程场景不是 live 状态')

  if (!finiteNumber(mapScene.ts) || mapScene.ts <= 0 || !finiteNumber(layer.stamp_s) || layer.stamp_s <= 0) {
    return error('最低观测高程 stamp 无效')
  }
  if (layer.stamp_s > mapScene.ts + FUTURE_STAMP_TOLERANCE_S || layer.stamp_s > options.nowS + FUTURE_STAMP_TOLERANCE_S) {
    return error('最低观测高程 stamp 位于未来')
  }
  const maxAgeS = finiteNumber(options.maxAgeS) && options.maxAgeS > 0
    ? options.maxAgeS
    : ELEVATION_DEFAULT_MAX_AGE_S
  if (options.nowS - layer.stamp_s > maxAgeS) {
    return { status: 'stale', message: `最低观测高程已过期 ${Math.floor(options.nowS - layer.stamp_s)}s` }
  }

  if (!positiveInteger(layer.rows) || !positiveInteger(layer.cols) || layer.rows < 2 || layer.cols < 2) {
    return error('最低观测高程网格尺寸无效')
  }
  const cellCount = layer.rows * layer.cols
  if (!Number.isSafeInteger(cellCount) || cellCount > ELEVATION_MAX_CELLS) {
    return error(`最低观测高程网格超过 ${ELEVATION_MAX_CELLS.toLocaleString()} 单元上限`)
  }
  if (!finiteNumber(layer.resolution) || layer.resolution <= 0 || !validOrigin(layer.origin) || !finiteNumber(layer.yaw)) {
    return error('最低观测高程网格变换无效')
  }
  if (layer.encoding !== 'float32_le') return error(`不支持的最低观测高程编码: ${String(layer.encoding ?? 'missing')}`)
  if (!positiveInteger(layer.downsample_factor)) return error('最低观测高程降采样倍率无效')
  const valueSemantics = layer.value_semantics ?? layer.semantic
  if (valueSemantics !== 'min_observed_z_not_ground') return error('最低观测高程值语义无效')

  if (layer.valid_count === 0) {
    return { status: 'unavailable', message: '最低观测高程无有效观测' }
  }
  if (typeof layer.grid_b64 !== 'string' || !layer.grid_b64) {
    return { status: 'unavailable', message: '等待最低观测高程数据' }
  }
  const heights = decodeFloat32LE(layer.grid_b64, cellCount)
  if (!heights) return error('最低观测高程数据解码失败')
  let actualValidCount = 0
  let actualMin = Number.POSITIVE_INFINITY
  let actualMax = Number.NEGATIVE_INFINITY
  for (const height of heights) {
    if (!Number.isFinite(height)) continue
    actualValidCount += 1
    actualMin = Math.min(actualMin, height)
    actualMax = Math.max(actualMax, height)
  }
  if (!nonNegativeInteger(layer.valid_count) || layer.valid_count !== actualValidCount) {
    return error('最低观测高程有效单元计数不一致')
  }
  if (actualValidCount === 0) return { status: 'unavailable', message: '最低观测高程无有效观测' }
  if (!finiteNumber(layer.min_z) || !finiteNumber(layer.max_z) || layer.min_z > layer.max_z) {
    return error('最低观测高程范围无效')
  }
  const tolerance = Math.max(1e-4, Math.abs(actualMin) * 1e-5, Math.abs(actualMax) * 1e-5)
  if (Math.abs(actualMin - layer.min_z) > tolerance || Math.abs(actualMax - layer.max_z) > tolerance) {
    return error('最低观测高程范围与 payload 不一致')
  }
  return {
    status: 'ready',
    layer,
    heights,
    minZ: layer.min_z,
    maxZ: layer.max_z,
    validCount: layer.valid_count,
    message: `最低观测高程 ${layer.min_z.toFixed(2)}–${layer.max_z.toFixed(2)} m${layer.payload_retained === true ? ' · 保留上一帧' : ''}`,
  }
}

function setElevationColor(target: THREE.Color, height: number, minZ: number, maxZ: number): void {
  const t = maxZ > minZ ? THREE.MathUtils.clamp((height - minZ) / (maxZ - minZ), 0, 1) : 0.5
  target.setHSL(0.62 - t * 0.48, 0.72, 0.46 + t * 0.12)
}

export function createElevationLayer(state: ElevationLayerState): GroupedMesh | null {
  if (state.status !== 'ready') return null
  const { layer, heights, minZ, maxZ } = state
  const positions = new Float32Array(layer.rows * layer.cols * 3)
  const colors = new Float32Array(layer.rows * layer.cols * 3)
  const width = (layer.cols - 1) * layer.resolution
  const depth = (layer.rows - 1) * layer.resolution
  const color = new THREE.Color()

  for (let row = 0; row < layer.rows; row++) {
    for (let col = 0; col < layer.cols; col++) {
      const index = row * layer.cols + col
      const offset = index * 3
      const height = heights[index]
      const [tx, ty, tz] = lingtuToThree([
        col * layer.resolution - width / 2,
        row * layer.resolution - depth / 2,
        Number.isFinite(height) ? height : 0,
      ])
      positions[offset] = tx
      positions[offset + 1] = ty
      positions[offset + 2] = tz
      if (Number.isFinite(height)) {
        setElevationColor(color, height, minZ, maxZ)
        colors[offset] = color.r
        colors[offset + 1] = color.g
        colors[offset + 2] = color.b
      }
    }
  }

  const indices: number[] = []
  for (let row = 0; row < layer.rows - 1; row++) {
    for (let col = 0; col < layer.cols - 1; col++) {
      const a = row * layer.cols + col
      const b = a + 1
      const c = a + layer.cols
      const d = c + 1
      const aValid = Number.isFinite(heights[a])
      const bValid = Number.isFinite(heights[b])
      const cValid = Number.isFinite(heights[c])
      const dValid = Number.isFinite(heights[d])
      if (aValid && bValid && cValid) indices.push(a, c, b)
      if (bValid && cValid && dValid) indices.push(b, c, d)
    }
  }
  if (indices.length === 0) return null

  const geometry = new THREE.BufferGeometry()
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))
  geometry.setIndex(indices)
  geometry.computeVertexNormals()
  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshStandardMaterial({
      vertexColors: true,
      side: THREE.DoubleSide,
      roughness: 0.82,
      metalness: 0,
    }),
  ) as GroupedMesh
  mesh.name = 'elevation-surface'
  mesh.renderOrder = 4
  createHeightGridGroup(
    mesh,
    layer.origin,
    layer.cols * layer.resolution,
    layer.rows * layer.resolution,
    layer.yaw,
  )
  return mesh
}
