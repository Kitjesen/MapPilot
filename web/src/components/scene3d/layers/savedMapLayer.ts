import * as THREE from 'three'
import type { MapSceneEvent, MapSceneLayer } from '../../../types'
import { DEFAULT_POINT_SIZE, pointSizeToWorld } from './liveCloudLayer.ts'
import { lingtuToThree } from '../../../services/coordinateFrame.ts'
import { cloudHeightRange, writeCloudHeightColor } from '../../../services/cloudHeightColor.ts'

export const SAVED_MAP_Z_FLOOR = Number.NEGATIVE_INFINITY
export const SAVED_MAP_Z_CEIL = Number.POSITIVE_INFINITY

function parseHexColor(value: unknown): [number, number, number] | null {
  if (typeof value !== 'string') return null
  const raw = value.startsWith('#') ? value.slice(1) : value
  if (!/^[0-9a-fA-F]{6}$/.test(raw)) return null
  return [
    Number.parseInt(raw.slice(0, 2), 16) / 255,
    Number.parseInt(raw.slice(2, 4), 16) / 255,
    Number.parseInt(raw.slice(4, 6), 16) / 255,
  ]
}

function fallbackLabelColor(label: number): [number, number, number] {
  const hue = ((label * 47) % 360) / 360
  const x = (1 - Math.abs((hue * 6) % 2 - 1)) * 0.75
  if (hue < 1 / 6) return [0.75, x, 0.18]
  if (hue < 2 / 6) return [x, 0.75, 0.18]
  if (hue < 3 / 6) return [0.18, 0.75, x]
  if (hue < 4 / 6) return [0.18, x, 0.75]
  if (hue < 5 / 6) return [x, 0.18, 0.75]
  return [0.75, 0.18, x]
}

function semanticPointLayer(mapScene: MapSceneEvent | null | undefined, pointCount: number): MapSceneLayer | null {
  const layers = mapScene?.layers ?? []
  for (const layer of layers) {
    const layerType = String(layer.type ?? layer.layer_type ?? '')
    if (layerType !== 'pointcloud') continue
    if (!Array.isArray(layer.labels) || layer.labels.length !== pointCount) continue
    return layer
  }
  return null
}

export function createSavedMapLayer(
  savedMapFlat: number[] | undefined,
  zFloor: number,
  zCeil: number,
  mapScene?: MapSceneEvent | null,
  pointSize = DEFAULT_POINT_SIZE,
  colorMode: 'height' | 'semantic' = 'height',
): THREE.Points | null {
  if (!savedMapFlat || savedMapFlat.length < 3) return null

  const totalTriples = Math.floor(savedMapFlat.length / 3)
  const semantic = colorMode === 'semantic' ? semanticPointLayer(mapScene, totalTriples) : null
  const positions = new Float32Array(totalTriples * 3)
  const colors = new Float32Array(totalTriples * 3)
  const heightRange = cloudHeightRange(savedMapFlat)
  let length = 0

  for (let i = 0; i + 2 < savedMapFlat.length; i += 3) {
    const wx = savedMapFlat[i]
    const wy = savedMapFlat[i + 1]
    const wz = savedMapFlat[i + 2]
    if (!Number.isFinite(wx) || !Number.isFinite(wy) || !Number.isFinite(wz)) continue
    if (wz < zFloor || wz > zCeil) continue
    positions.set(lingtuToThree([wx, wy, wz]), length)
    if (semantic?.labels) {
      const label = semantic.labels[Math.floor(i / 3)] ?? 0
      const paletteColor = parseHexColor(semantic.palette?.[String(label)]?.color)
      colors.set(paletteColor ?? fallbackLabelColor(label), length)
    } else if (heightRange) {
      writeCloudHeightColor(wz, colors, length, heightRange.min, heightRange.max)
    }
    length += 3
  }
  if (length === 0) return null
  const vertexColors = Boolean(semantic || heightRange)

  const geo = new THREE.BufferGeometry()
  geo.setAttribute('position', new THREE.BufferAttribute(positions.subarray(0, length), 3))
  if (vertexColors) geo.setAttribute('color', new THREE.BufferAttribute(colors.subarray(0, length), 3))
  const mat = new THREE.PointsMaterial({
    size: pointSizeToWorld(pointSize) * 2,
    color: vertexColors ? 0xffffff : 0x7896af,
    vertexColors,
    sizeAttenuation: true,
    opacity: 0.95,
    transparent: true,
    depthWrite: false,
    toneMapped: false,
  })
  return new THREE.Points(geo, mat)
}

export function updateSavedMapPointSize(points: THREE.Points | null, pointSize: number): void {
  if (!points || Array.isArray(points.material)) return
  const material = points.material as THREE.PointsMaterial
  material.size = pointSizeToWorld(pointSize) * 2
}
