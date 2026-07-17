import * as THREE from 'three'
import type { MapSceneEvent, MapSceneLayer } from '../../../types'
import { DEFAULT_POINT_SIZE, pointSizeToWorld } from './liveCloudLayer.ts'

export const SAVED_MAP_Z_FLOOR = -50
export const SAVED_MAP_Z_CEIL = 50

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

const HEIGHT_STOPS: Array<[number, number, number]> = [
  [0.95, 0.18, 0.24],
  [1.00, 0.55, 0.06],
  [0.66, 0.96, 0.10],
  [0.08, 0.88, 0.84],
  [0.66, 0.20, 0.96],
]

function heightColor(z: number, zMin: number, zMax: number): [number, number, number] {
  const normalized = Math.max(0, Math.min(1, (z - zMin) / Math.max(1e-6, zMax - zMin)))
  const scaled = normalized * (HEIGHT_STOPS.length - 1)
  const index = Math.min(HEIGHT_STOPS.length - 2, Math.floor(scaled))
  const t = scaled - index
  const start = HEIGHT_STOPS[index]
  const end = HEIGHT_STOPS[index + 1]
  return [
    start[0] + (end[0] - start[0]) * t,
    start[1] + (end[1] - start[1]) * t,
    start[2] + (end[2] - start[2]) * t,
  ]
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
): THREE.Points | null {
  if (!savedMapFlat || savedMapFlat.length < 3) return null

  const maxSavedPoints = 80_000
  const totalTriples = Math.floor(savedMapFlat.length / 3)
  const stride = totalTriples > maxSavedPoints ? Math.ceil(totalTriples / maxSavedPoints) : 1
  const semantic = semanticPointLayer(mapScene, totalTriples)
  const positions: number[] = []
  const colors: number[] = []
  const heights: number[] = []
  let zMin = Number.POSITIVE_INFINITY
  let zMax = Number.NEGATIVE_INFINITY
  const step = stride * 3

  for (let i = 0; i + 2 < savedMapFlat.length; i += step) {
    const wx = savedMapFlat[i]
    const wy = savedMapFlat[i + 1]
    const wz = savedMapFlat[i + 2]
    if (wz < zFloor || wz > zCeil) continue
    positions.push(wx, wz, -wy)
    heights.push(wz)
    zMin = Math.min(zMin, wz)
    zMax = Math.max(zMax, wz)
    if (semantic?.labels) {
      const label = semantic.labels[Math.floor(i / 3)] ?? 0
      const paletteColor = parseHexColor(semantic.palette?.[String(label)]?.color)
      colors.push(...(paletteColor ?? fallbackLabelColor(label)))
    }
  }
  if (positions.length === 0) return null
  if (!semantic && heights.length > 0) {
    for (const height of heights) colors.push(...heightColor(height, zMin, zMax))
  }

  const geo = new THREE.BufferGeometry()
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
  if (colors.length === positions.length) {
    geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))
  }
  const mat = new THREE.PointsMaterial({
    size: pointSizeToWorld(pointSize),
    color: 0x26364f,
    vertexColors: colors.length === positions.length,
    sizeAttenuation: true,
    opacity: colors.length === positions.length ? 0.82 : 0.55,
    transparent: true,
  })
  return new THREE.Points(geo, mat)
}

export function updateSavedMapPointSize(points: THREE.Points | null, pointSize: number): void {
  if (!points || Array.isArray(points.material)) return
  const material = points.material as THREE.PointsMaterial
  material.size = pointSizeToWorld(pointSize)
}
