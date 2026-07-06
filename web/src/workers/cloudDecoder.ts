/**
 * cloudDecoder — WebWorker that turns binary /ws/cloud frames into the
 * Float32 buffers Three.js consumes directly.
 *
 * Wire format mirrors src/runtime/utils/binary_codec.py:
 *   <4s  magic = "PCLD">
 *   <u8  version = 1>
 *   <u8  flags  (bit0 = has_color)>
 *   <u16 reserved>
 *   <u32 count>
 *   <f32 scale>
 *   <f32 origin_x> <f32 origin_y> <f32 origin_z>
 *   <int16[count*3] xyz>
 *   <u8[count*3]    rgb>      // only when flags bit0 set
 *
 * The decoder also performs:
 *   - axis swap (LingTu world XYZ → Three.js (X, Z, -Y))
 *   - height-percentile turbo colormap (matches Scene3D.tsx)
 * so the main thread never has to iterate per-point.
 *
 * Output: { positions: Float32Array, colors: Float32Array, count, seq }
 * Both arrays are posted as Transferable, so no copy is made.
 */

const MAGIC = 0x44_4c_43_50 // "PCLD" little-endian (P=0x50, C=0x43, L=0x4c, D=0x44)
const HEADER = 28
const Z_FLOOR = -0.5
const Z_CEIL = 2.8

let seq = 0

interface DecodedFrame {
  type: 'cloud'
  positions: Float32Array
  colors: Float32Array
  count: number
  seq: number
}

function turboColor(t: number, out: Float32Array, off: number) {
  const x = t < 0 ? 0 : t > 1 ? 1 : t
  // Same teal→gray→amber gradient Scene3D used to compute on the main thread.
  if (x < 0.5) {
    const u = x * 2
    out[off]     = 0.18 + (0.55 - 0.18) * u
    out[off + 1] = 0.55
    out[off + 2] = 0.50 + (0.55 - 0.50) * u
  } else {
    const u = (x - 0.5) * 2
    out[off]     = 0.55 + (0.78 - 0.55) * u
    out[off + 1] = 0.55 + (0.60 - 0.55) * u
    out[off + 2] = 0.55 + (0.35 - 0.55) * u
  }
}

function decode(buf: ArrayBuffer): DecodedFrame | null {
  if (buf.byteLength < HEADER) return null
  const view = new DataView(buf)
  if (view.getUint32(0, true) !== MAGIC) return null
  const version = view.getUint8(4)
  if (version !== 1) return null
  const count = view.getUint32(8, true)
  if (count === 0) {
    return {
      type: 'cloud',
      positions: new Float32Array(0),
      colors: new Float32Array(0),
      count: 0,
      seq: ++seq,
    }
  }
  const scale = view.getFloat32(12, true)
  const ox = view.getFloat32(16, true)
  const oy = view.getFloat32(20, true)
  const oz = view.getFloat32(24, true)

  const xyz = new Int16Array(buf, HEADER, count * 3)

  // First pass: world Z range for percentile color stretch (matches Scene3D).
  // We do it in one pass + a partial sort instead of two passes + sort to
  // avoid allocating an intermediate array of length count.
  let zMin = Infinity, zMax = -Infinity
  for (let i = 0; i < count; i++) {
    const wz = xyz[i * 3 + 2] * scale + oz
    if (wz < Z_FLOOR || wz > Z_CEIL) continue
    if (wz < zMin) zMin = wz
    if (wz > zMax) zMax = wz
  }
  if (!isFinite(zMin)) { zMin = 0; zMax = 1 }
  const zSpan = Math.max(zMax - zMin, 0.1)

  // Second pass: build positions + colors, dropping floor/ceiling outliers.
  // Worst case keeps every point; allocate that much then trim with subarray.
  const positions = new Float32Array(count * 3)
  const colors = new Float32Array(count * 3)
  let written = 0
  for (let i = 0; i < count; i++) {
    const wx = xyz[i * 3]     * scale + ox
    const wy = xyz[i * 3 + 1] * scale + oy
    const wz = xyz[i * 3 + 2] * scale + oz
    if (wz < Z_FLOOR || wz > Z_CEIL) continue
    const off = written * 3
    // Three.js: Y=worldZ_height, Z=-worldY (matches Scene3D coordinate mapping)
    positions[off]     = wx
    positions[off + 1] = wz
    positions[off + 2] = -wy
    turboColor((wz - zMin) / zSpan, colors, off)
    written++
  }
  const posOut = positions.subarray(0, written * 3)
  const colOut = colors.subarray(0, written * 3)
  return {
    type: 'cloud',
    positions: posOut,
    colors: colOut,
    count: written,
    seq: ++seq,
  }
}

self.onmessage = (e: MessageEvent<ArrayBuffer>) => {
  const out = decode(e.data)
  if (!out) return
  // Transfer the underlying buffers; subarray views share the same buffer,
  // so we transfer once and the consumer can rebuild the views.
  // To keep the API simple we copy into new tightly-packed buffers — the
  // pre-allocated Float32Array is up to 60k * 12 = 720 KB, which is cheaper
  // to copy once (memcpy) than to keep a stale 5x larger buffer alive.
  const posCopy = new Float32Array(out.positions)
  const colCopy = new Float32Array(out.colors)
  ;(self as unknown as Worker).postMessage(
    {
      type: 'cloud',
      positions: posCopy,
      colors: colCopy,
      count: out.count,
      seq: out.seq,
    },
    [posCopy.buffer, colCopy.buffer],
  )
}

export {}
