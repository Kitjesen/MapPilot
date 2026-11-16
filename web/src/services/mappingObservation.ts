import type { MapSceneEvent, OccupancyMapSceneLayer } from '../types/index.ts'
import { mapResetEpoch } from './mapSceneIdentity.ts'

export const MAPPING_OBSERVATION_MAX_CELLS = 131_072
export const MAPPING_OBSERVATION_DEFAULT_MAX_AGE_S = 5
export type MappingObservationCell = -1 | 0 | 100

export const MAPPING_OBSERVATION_LABELS: Record<MappingObservationCell, string> = {
  [-1]: '未确认 · 缺少地面依据',
  0: '支撑候选 · 已观测低表面',
  100: '障碍回波 · 高于附近地面',
}

export const MAPPING_OBSERVATION_COLORS: Record<MappingObservationCell, readonly [number, number, number, number]> = {
  [-1]: [83, 88, 94, 100],
  0: [83, 133, 129, 125],
  100: [201, 111, 99, 230],
}

export interface MappingObservationOptions {
  nowS: number
  savedMapFrameId?: string | null
  maxAgeS?: number
}

export interface ReadyMappingObservation {
  status: 'ready'
  layer: OccupancyMapSceneLayer
  cells: Int8Array
  unknownCount: number
  freeCount: number
  occupiedCount: number
  message: string
}

export type MappingObservationState = ReadyMappingObservation | {
  status: 'unavailable' | 'stale' | 'error'
  message: string
}

function finite(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

function integer(value: unknown): value is number {
  return finite(value) && Number.isSafeInteger(value) && value >= 0
}

function error(message: string): MappingObservationState {
  return { status: 'error', message }
}

interface DecodedGrid {
  gridB64: string
  cells: Int8Array
  unknownCount: number
  freeCount: number
  occupiedCount: number
}

let decodedGridCache: DecodedGrid | null = null

function decodeGrid(gridB64: string, count: number): DecodedGrid | null {
  if (decodedGridCache?.gridB64 === gridB64 && decodedGridCache.cells.length === count) {
    return decodedGridCache
  }
  try {
    const binary = atob(gridB64)
    if (binary.length !== count) return null
    const cells = new Int8Array(count)
    let unknownCount = 0
    let freeCount = 0
    let occupiedCount = 0
    for (let index = 0; index < count; index++) {
      const value = binary.charCodeAt(index)
      if (value === 255) {
        cells[index] = -1
        unknownCount++
      } else if (value === 0) {
        freeCount++
      } else if (value === 100) {
        cells[index] = 100
        occupiedCount++
      } else {
        return null
      }
    }
    decodedGridCache = { gridB64, cells, unknownCount, freeCount, occupiedCount }
    return decodedGridCache
  } catch {
    return null
  }
}

export function resolveMappingObservation(
  mapScene: MapSceneEvent | null | undefined,
  options: MappingObservationOptions,
): MappingObservationState {
  const candidate = mapScene?.layers.find(layer => layer.id === 'maps.occupancy')
  if (!candidate || !mapScene) return { status: 'unavailable', message: '等待建图观测数据' }
  const layer = candidate as OccupancyMapSceneLayer
  if (layer.payload === 'omitted' && layer.retain_previous === false
    && (typeof layer.grid_b64 !== 'string' || !layer.grid_b64)) {
    return { status: 'unavailable', message: '观测图暂不可用，等待新数据' }
  }
  const producer = typeof layer.producer_boot_id === 'string' ? layer.producer_boot_id.trim() : ''
  const sceneProducer = mapScene.metadata?.producer_boot_id
  if (!producer || (sceneProducer !== undefined && sceneProducer !== producer)) {
    return error('观测图来源不一致，等待新数据')
  }
  const resetEpoch = mapResetEpoch(layer.reset_epoch)
  const sceneResetEpoch = mapScene.metadata?.reset_epoch
  if (resetEpoch === null || (sceneResetEpoch !== undefined
    && mapResetEpoch(sceneResetEpoch) !== resetEpoch)) {
    return error('观测图已切换，等待当前数据')
  }
  for (const key of ['generation', 'observation_sequence'] as const) {
    const sceneValue = mapScene.metadata?.[key]
    if (!integer(layer[key]) || (sceneValue !== undefined && sceneValue !== layer[key])) {
      return error('观测图已切换，等待当前数据')
    }
  }
  if (typeof layer.frame_id !== 'string' || !layer.frame_id.trim()
    || layer.frame_id !== mapScene.frame_id
    || (options.savedMapFrameId && layer.frame_id !== options.savedMapFrameId)) {
    return error('观测图坐标尚未对齐')
  }
  if (layer.live !== true || (mapScene.metadata?.live !== undefined && mapScene.metadata.live !== true)) {
    return { status: 'unavailable', message: '建图观测当前未更新' }
  }
  if (!finite(options.nowS) || !finite(mapScene.ts) || mapScene.ts <= 0
    || !finite(layer.stamp_s) || layer.stamp_s <= 0) return error('观测图时间无效，等待新数据')
  if (layer.stamp_s > options.nowS + 1 || layer.stamp_s > mapScene.ts + 1) {
    return error('观测图时间尚未同步')
  }
  const maxAgeS = finite(options.maxAgeS) && options.maxAgeS > 0
    ? options.maxAgeS : MAPPING_OBSERVATION_DEFAULT_MAX_AGE_S
  if (options.nowS - layer.stamp_s > maxAgeS) {
    return { status: 'stale', message: `建图观测已过期 ${Math.floor(options.nowS - layer.stamp_s)}s` }
  }
  const count = layer.rows * layer.cols
  if (!integer(layer.rows) || layer.rows === 0 || !integer(layer.cols) || layer.cols === 0
    || !integer(count) || count > MAPPING_OBSERVATION_MAX_CELLS) return error('观测图范围无效，等待新数据')
  if (!finite(layer.resolution) || layer.resolution <= 0 || !finite(layer.yaw)
    || !Array.isArray(layer.origin) || layer.origin.length !== 3 || !layer.origin.every(finite)) {
    return error('观测图坐标尚未对齐')
  }
  if (layer.encoding !== 'int8' || layer.value_semantics !== 'height_band_occupancy_not_traversability'
    || layer.scope !== 'rolling_window' || !integer(layer.downsample_factor) || layer.downsample_factor === 0) {
    return error('观测图数据格式暂不支持')
  }
  if (typeof layer.grid_b64 !== 'string' || !layer.grid_b64) {
    return { status: 'unavailable', message: '等待建图观测网格' }
  }
  const decoded = decodeGrid(layer.grid_b64, count)
  if (!decoded) return error('观测图数据无效，等待新数据')
  if (layer.unknown_count !== decoded.unknownCount || layer.free_count !== decoded.freeCount
    || layer.occupied_count !== decoded.occupiedCount) return error('观测图数据不完整，等待新数据')
  return {
    status: 'ready',
    layer: layer.reset_epoch === resetEpoch ? layer : { ...layer, reset_epoch: resetEpoch },
    cells: decoded.cells,
    unknownCount: decoded.unknownCount,
    freeCount: decoded.freeCount,
    occupiedCount: decoded.occupiedCount,
    message: `实时占据观测 · ${(layer.cols * layer.resolution).toFixed(1)} × ${(layer.rows * layer.resolution).toFixed(1)} m`,
  }
}

export function mappingObservationPointAt(
  state: MappingObservationState,
  worldX: number,
  worldY: number,
): { row: number; col: number; value: MappingObservationCell; label: string } | null {
  if (state.status !== 'ready' || !finite(worldX) || !finite(worldY)) return null
  const { layer } = state
  const dx = worldX - layer.origin[0]
  const dy = worldY - layer.origin[1]
  const cos = Math.cos(layer.yaw)
  const sin = Math.sin(layer.yaw)
  const col = Math.floor((cos * dx + sin * dy) / layer.resolution)
  const row = Math.floor((-sin * dx + cos * dy) / layer.resolution)
  if (col < 0 || col >= layer.cols || row < 0 || row >= layer.rows) return null
  const value = state.cells[row * layer.cols + col] as MappingObservationCell
  return { row, col, value, label: MAPPING_OBSERVATION_LABELS[value] }
}
