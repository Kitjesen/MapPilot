/** Single Web-side LingTu (Z-up) to Three.js (Y-up) conversion. */
export type LingtuPoint = readonly [number, number, number]
export type ThreePoint = readonly [number, number, number]

function negate(value: number): number {
  return value === 0 ? 0 : -value
}

/** LingTu: X forward, Y left, Z up -> Three.js: X, Y up, Z. */
export function lingtuToThree([x, y, z]: LingtuPoint): [number, number, number] {
  return [x, z, negate(y)]
}

export function threeToLingtu([x, y, z]: ThreePoint): [number, number, number] {
  return [x, negate(z), y]
}

/** Positive LingTu yaw maps to positive Three.js Y rotation. */
export function lingtuYawToThree(yaw: number): number {
  return yaw
}

export function lingtuXYToThree(x: number, y: number, height = 0): [number, number, number] {
  return lingtuToThree([x, y, height])
}
