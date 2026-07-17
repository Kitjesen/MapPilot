import * as THREE from 'three'
import type { CostmapEvent } from '../../../types'
import { createFlatGridGroup, type GroupedMesh } from './layerUtils'

const COST_STOPS: Array<[number, number, number, number, number]> = [
  [0, 46, 196, 182, 0],
  [20, 46, 196, 182, 44],
  [40, 255, 214, 90, 74],
  [60, 255, 133, 38, 108],
  [80, 231, 72, 72, 138],
  [100, 183, 28, 28, 162],
]

function costColor(value: number): [number, number, number, number] {
  for (let i = 0; i < COST_STOPS.length - 1; i++) {
    const [a, ar, ag, ab, aa] = COST_STOPS[i]
    const [b, br, bg, bb, ba] = COST_STOPS[i + 1]
    if (value <= b) {
      const t = (value - a) / Math.max(1e-6, b - a)
      return [
        Math.round(ar + (br - ar) * t),
        Math.round(ag + (bg - ag) * t),
        Math.round(ab + (bb - ab) * t),
        Math.round(aa + (ba - aa) * t),
      ]
    }
  }
  const last = COST_STOPS[COST_STOPS.length - 1]
  return [last[1], last[2], last[3], last[4]]
}

export function createCostmapLayer(costmap: CostmapEvent): GroupedMesh | null {
  const { grid_b64, cols, resolution, origin } = costmap
  const bytes = Uint8Array.from(atob(grid_b64), c => c.charCodeAt(0))
  const rows = costmap.rows ?? Math.round(bytes.length / cols)
  if (rows <= 0 || cols <= 0) return null

  const canvas = document.createElement('canvas')
  canvas.width = cols
  canvas.height = rows
  const ctx = canvas.getContext('2d')
  if (!ctx) return null
  const img = ctx.createImageData(cols, rows)

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const value = bytes[r * cols + c]
      const offset = (r * cols + c) * 4
      if (value === 0) {
        img.data[offset] = 0
        img.data[offset + 1] = 0
        img.data[offset + 2] = 0
        img.data[offset + 3] = 0
        continue
      }
      const [red, green, blue, alpha] = costColor(value)
      img.data[offset] = red
      img.data[offset + 1] = green
      img.data[offset + 2] = blue
      img.data[offset + 3] = alpha
    }
  }
  ctx.putImageData(img, 0, 0)

  const tex = new THREE.CanvasTexture(canvas)
  tex.flipY = false
  tex.minFilter = THREE.LinearFilter
  tex.magFilter = THREE.NearestFilter

  const sizeX = cols * resolution
  const sizeY = rows * resolution
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(sizeX, sizeY),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }),
  ) as GroupedMesh
  mesh.renderOrder = 5
  const group = createFlatGridGroup(mesh, origin, sizeX, sizeY, costmap.yaw ?? 0, 0.01)
  group.renderOrder = 5
  return mesh
}
