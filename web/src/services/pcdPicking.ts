import { lingtuToThree } from './coordinateFrame.ts'

export interface PointCloudPick { x: number; y: number; z: number }

export const PCD_POINT_SIZE_CSS = 2.2

export function projectPcdPoint(m: Float32Array, p: PointCloudPick, width: number, height: number) {
  const [x, y, z] = lingtuToThree([p.x, p.y, p.z])
  const cx = m[0]*x + m[4]*y + m[8]*z + m[12]
  const cy = m[1]*x + m[5]*y + m[9]*z + m[13]
  const cz = m[2]*x + m[6]*y + m[10]*z + m[14]
  const cw = m[3]*x + m[7]*y + m[11]*z + m[15]
  if (!Number.isFinite(cw) || cw <= 0) return null
  const nx = cx / cw, ny = cy / cw, nz = cz / cw
  if (!Number.isFinite(nx) || !Number.isFinite(ny) || !Number.isFinite(nz)
    || nx < -1 || nx > 1 || ny < -1 || ny > 1 || nz < -1 || nz > 1) return null
  return { x: (nx * 0.5 + 0.5) * width, y: (0.5 - ny * 0.5) * height, depth: nz }
}

export function pickVisiblePcdPoint(
  points: Float32Array,
  mvp: Float32Array,
  width: number,
  height: number,
  clickX: number,
  clickY: number,
  pickRadius: number,
  pointSize: number,
): PointCloudPick | null {
  // Only keep depth for pixel centers in the click neighborhood, not the full canvas.
  const left = Math.max(0, Math.ceil(clickX - pickRadius - 0.5))
  const right = Math.min(width - 1, Math.floor(clickX + pickRadius - 0.5))
  const top = Math.max(0, Math.ceil(clickY - pickRadius - 0.5))
  const bottom = Math.min(height - 1, Math.floor(clickY + pickRadius - 0.5))
  if (left > right || top > bottom) return null
  const columns = right - left + 1
  const count = columns * (bottom - top + 1)
  const depths = new Float64Array(count).fill(1)
  const hits = new Int32Array(count).fill(-1)
  const radius = pointSize / 2
  const radiusSquared = radius * radius

  for (let i = 0; i < points.length; i += 3) {
    const projected = projectPcdPoint(mvp, { x: points[i], y: points[i + 1], z: points[i + 2] }, width, height)
    if (!projected) continue
    const x0 = Math.max(left, Math.ceil(projected.x - radius - 0.5))
    const x1 = Math.min(right, Math.floor(projected.x + radius - 0.5))
    const y0 = Math.max(top, Math.ceil(projected.y - radius - 0.5))
    const y1 = Math.min(bottom, Math.floor(projected.y + radius - 0.5))
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const dx = x + 0.5 - projected.x, dy = y + 0.5 - projected.y
        // Match the fragment shader's circular gl_PointCoord cutout and GL_LESS.
        if (dx * dx + dy * dy > radiusSquared) continue
        const cell = (y - top) * columns + x - left
        if (projected.depth < depths[cell]) {
          depths[cell] = projected.depth
          hits[cell] = i
        }
      }
    }
  }

  let bestIndex = -1
  let bestDistance = pickRadius * pickRadius
  let bestDepth = Infinity
  for (let y = top; y <= bottom; y++) {
    for (let x = left; x <= right; x++) {
      const cell = (y - top) * columns + x - left
      if (hits[cell] < 0) continue
      const dx = x + 0.5 - clickX, dy = y + 0.5 - clickY
      const distance = dx * dx + dy * dy
      if (distance < bestDistance || (distance === bestDistance && depths[cell] < bestDepth)) {
        bestIndex = hits[cell]
        bestDistance = distance
        bestDepth = depths[cell]
      }
    }
  }
  return bestIndex < 0 ? null : { x: points[bestIndex], y: points[bestIndex + 1], z: points[bestIndex + 2] }
}
