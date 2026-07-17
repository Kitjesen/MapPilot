import * as THREE from 'three'
import type { SlopeGridEvent } from '../../../types'
import { createFlatGridGroup, type GroupedMesh } from './layerUtils'

function slopeColor(raw: number): [number, number, number, number] {
  const deg = raw * (90.0 / 255.0)
  if (deg < 3) return [0, 0, 0, 0]
  if (deg < 15) {
    const t = (deg - 3) / 12
    return [Math.round(40 * t), Math.round(180 + 40 * t), Math.round(60 * t), Math.round(30 + 50 * t)]
  }
  if (deg < 25) {
    const t = (deg - 15) / 10
    return [Math.round(200 + 55 * t), Math.round(200 - 60 * t), 30, Math.round(80 + 40 * t)]
  }
  return [240, 50, 50, 140]
}

export function createSlopeLayer(slopeGrid: SlopeGridEvent): GroupedMesh | null {
  if (!slopeGrid.grid_b64) return null
  const { grid_b64, cols, resolution, origin } = slopeGrid
  const bytes = Uint8Array.from(atob(grid_b64), c => c.charCodeAt(0))
  const rows = slopeGrid.rows ?? Math.round(bytes.length / cols)
  if (rows <= 0 || cols <= 0) return null

  const canvas = document.createElement('canvas')
  canvas.width = cols
  canvas.height = rows
  const ctx = canvas.getContext('2d')
  if (!ctx) return null
  const img = ctx.createImageData(cols, rows)

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const [red, green, blue, alpha] = slopeColor(bytes[r * cols + c])
      const offset = (r * cols + c) * 4
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
  createFlatGridGroup(mesh, origin, sizeX, sizeY, slopeGrid.yaw ?? 0, 0.02)
  return mesh
}
