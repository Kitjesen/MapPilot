// A fixed map-height scale keeps a stationary surface the same color as scans arrive.
export const CLOUD_HEIGHT_MIN = -1
export const CLOUD_HEIGHT_MAX = 2.5
const HEIGHT_COLORS = [
  [0.02, 0.16, 1.0],
  [0.0, 0.85, 1.0],
  [0.12, 0.95, 0.22],
  [1.0, 0.85, 0.02],
  [1.0, 0.16, 0.02],
] as const

export function writeCloudHeightColor(z: number, out: Float32Array, offset: number): void {
  const t = Math.max(0, Math.min(1, (z - CLOUD_HEIGHT_MIN) / (CLOUD_HEIGHT_MAX - CLOUD_HEIGHT_MIN)))
  const scaled = t * (HEIGHT_COLORS.length - 1)
  const index = Math.min(HEIGHT_COLORS.length - 2, Math.floor(scaled))
  const fraction = scaled - index
  for (let channel = 0; channel < 3; channel++) {
    out[offset + channel] = HEIGHT_COLORS[index][channel]
      + (HEIGHT_COLORS[index + 1][channel] - HEIGHT_COLORS[index][channel]) * fraction
  }
}
