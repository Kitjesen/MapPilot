// A fixed map-height scale keeps a stationary surface the same color as scans arrive.
export const CLOUD_HEIGHT_MIN = -1
export const CLOUD_HEIGHT_MAX = 2.5
export const HEIGHT_COLORS = [
  [0.02, 0.16, 1.0],
  [0.0, 0.85, 1.0],
  [0.12, 0.95, 0.22],
  [1.0, 0.85, 0.02],
  [1.0, 0.16, 0.02],
] as const

export const CLOUD_HEIGHT_GRADIENT = `linear-gradient(to right, ${HEIGHT_COLORS.map(
  color => `rgb(${color.map(channel => Math.round(channel * 255)).join(' ')})`,
).join(', ')})`

export function cloudHeightRange(points: ArrayLike<number>): { min: number; max: number } | null {
  let min = Infinity, max = -Infinity
  for (let i = 0; i + 2 < points.length; i += 3) {
    if (!Number.isFinite(points[i]) || !Number.isFinite(points[i + 1]) || !Number.isFinite(points[i + 2])) continue
    min = Math.min(min, points[i + 2])
    max = Math.max(max, points[i + 2])
  }
  return Number.isFinite(min) ? { min, max } : null
}

/** Display scale only: isolated extreme heights must not flatten the main scene's colors. */
export function cloudColorRange(points: ArrayLike<number>, stride = 3): { min: number; max: number } | null {
  const heights: number[] = []
  const step = Math.max(1, Math.ceil(points.length / stride / 65536)) * stride
  for (let i = 2; i < points.length; i += step) {
    if (Number.isFinite(points[i])) heights.push(points[i])
  }
  if (!heights.length) return null
  heights.sort((a, b) => a - b)
  return { min: heights[Math.floor((heights.length - 1) * .02)], max: heights[Math.ceil((heights.length - 1) * .98)] }
}

export function writeCloudHeightColor(
  z: number, out: Float32Array, offset: number,
  min = CLOUD_HEIGHT_MIN, max = CLOUD_HEIGHT_MAX,
): void {
  const t = max > min ? Math.max(0, Math.min(1, (z - min) / (max - min))) : 0.5
  const scaled = t * (HEIGHT_COLORS.length - 1)
  const index = Math.min(HEIGHT_COLORS.length - 2, Math.floor(scaled))
  const fraction = scaled - index
  for (let channel = 0; channel < 3; channel++) {
    out[offset + channel] = HEIGHT_COLORS[index][channel]
      + (HEIGHT_COLORS[index + 1][channel] - HEIGHT_COLORS[index][channel]) * fraction
  }
}
