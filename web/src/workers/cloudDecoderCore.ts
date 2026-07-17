const MAGIC = 0x44_4c_43_50 // "PCLD" as a little-endian uint32
const FLAG_HAS_COLOR = 0x01
const KNOWN_FLAGS = FLAG_HAS_COLOR
const V1_HEADER_SIZE = 28
const V2_HEADER_BASE_SIZE = 48
const MAX_POINT_COUNT = 1_000_000
const Z_FLOOR = -20
const Z_CEIL = 20
const COLOR_Z_MIN = -1.0
const COLOR_Z_SPAN = 3.5

export type CloudStreamKind = 'cloud' | 'map' | 'scan' | 'reset'
export type CloudEndpointKind = 'cloud' | 'scan'

export interface CloudFrameIdentity {
  frameId: string | null
  epoch: number | null
}

export interface CloudFrameMetadata extends CloudFrameIdentity {
  protocolVersion: 1 | 2 | null
  stampS: number | null
  sequence: number | null
  streamKind: CloudStreamKind | null
}

export interface DecodedCloudFrame extends CloudFrameMetadata {
  positions: Float32Array
  colors: Float32Array
  count: number
}

export interface CloudSequenceCursor {
  epoch: number | null
  sequence: number | null
}

export interface CloudFrameContract extends CloudFrameMetadata {
  count: number
}

const STREAM_KINDS: Readonly<Record<number, CloudStreamKind>> = {
  0: 'cloud',
  1: 'map',
  2: 'scan',
  3: 'reset',
}

function nullableString(value: unknown): string | null {
  if (typeof value !== 'string') return null
  const normalized = value.trim()
  return normalized.length > 0 ? normalized : null
}

function nullableProtocolVersion(value: unknown): 1 | 2 | null {
  return value === 1 || value === 2 ? value : null
}

function nullableUint32(value: unknown): number | null {
  return typeof value === 'number'
    && Number.isInteger(value)
    && value >= 0
    && value <= 0xffff_ffff
    ? value
    : null
}

function nullableFiniteTimestamp(value: unknown): number | null {
  return typeof value === 'number' && Number.isFinite(value) && value >= 0
    ? value
    : null
}

function nullableStreamKind(value: unknown): CloudStreamKind | null {
  return value === 'cloud' || value === 'map' || value === 'scan' || value === 'reset'
    ? value
    : null
}

export function parseHttpCloudMetadata(
  payload: Record<string, unknown>,
): CloudFrameMetadata {
  return {
    protocolVersion: nullableProtocolVersion(payload.protocol_version),
    frameId: nullableString(payload.frame_id),
    epoch: nullableUint32(payload.epoch),
    stampS: nullableFiniteTimestamp(payload.stamp_s ?? payload.ts),
    sequence: nullableUint32(payload.sequence ?? payload.seq),
    streamKind: nullableStreamKind(payload.stream_kind),
  }
}

export class PointCloudDecodeError extends Error {
  constructor(message: string) {
    super(message)
    this.name = 'PointCloudDecodeError'
  }
}

function decodeError(message: string): never {
  throw new PointCloudDecodeError(message)
}

function turboColor(t: number, out: Float32Array, off: number): void {
  const x = t < 0 ? 0 : t > 1 ? 1 : t
  if (x < 0.5) {
    const u = x * 2
    out[off] = 0.18 + (0.55 - 0.18) * u
    out[off + 1] = 0.55
    out[off + 2] = 0.50 + (0.55 - 0.50) * u
  } else {
    const u = (x - 0.5) * 2
    out[off] = 0.55 + (0.78 - 0.55) * u
    out[off + 1] = 0.55 + (0.60 - 0.55) * u
    out[off + 2] = 0.55 + (0.35 - 0.55) * u
  }
}

function containsControlCharacter(value: string): boolean {
  for (let index = 0; index < value.length; index++) {
    const code = value.charCodeAt(index)
    if (code < 0x20 || code === 0x7f) return true
  }
  return false
}

function decodeFrameId(buf: ArrayBuffer, offset: number, length: number): string | null {
  if (length === 0) return null
  let frameId: string
  try {
    frameId = new TextDecoder('utf-8', { fatal: true }).decode(
      new Uint8Array(buf, offset, length),
    )
  } catch {
    decodeError('PCLD v2 frame_id is not valid UTF-8')
  }
  if (
    frameId.length === 0
    || frameId.trim() !== frameId
    || containsControlCharacter(frameId)
  ) {
    decodeError('PCLD v2 frame_id is invalid')
  }
  return frameId
}

function int16PayloadView(
  buf: ArrayBuffer,
  offset: number,
  elementCount: number,
): Int16Array {
  const byteLength = elementCount * Int16Array.BYTES_PER_ELEMENT
  if (offset % Int16Array.BYTES_PER_ELEMENT === 0) {
    return new Int16Array(buf, offset, elementCount)
  }
  // frame_id is variable length, so an odd-length identifier leaves the v2
  // int16 payload unaligned. Copy only that compact wire slice in this case.
  return new Int16Array(buf.slice(offset, offset + byteLength))
}

export function cloudFramesShareCoordinateEpoch(
  accumulated: CloudFrameIdentity,
  scan: CloudFrameIdentity,
): boolean {
  return accumulated.frameId !== null
    && scan.frameId !== null
    && accumulated.epoch !== null
    && scan.epoch !== null
    && accumulated.frameId === scan.frameId
    && accumulated.epoch === scan.epoch
}

export function cloudFrameSharesSavedMapFrame(
  live: CloudFrameIdentity,
  savedMapFrameId: string | null | undefined,
  savedMapEpoch: number | null | undefined,
): boolean {
  const normalizeFrameId = (value: string | null | undefined): string | null => {
    if (typeof value !== 'string') return null
    const normalized = value.trim().replace(/^\/+/, '')
    return normalized.length > 0 ? normalized : null
  }
  const liveFrameId = normalizeFrameId(live.frameId)
  const savedFrameId = normalizeFrameId(savedMapFrameId)
  if (liveFrameId === null || savedFrameId === null || liveFrameId !== savedFrameId) {
    return false
  }
  if (live.epoch === null || savedMapEpoch == null) {
    return live.epoch === null && savedMapEpoch == null
  }
  return live.epoch === savedMapEpoch
}

export function savedMapNeedsSceneRebind(
  live: CloudFrameIdentity,
  saved: CloudFrameIdentity,
): boolean {
  return live.frameId !== null
    && saved.frameId !== null
    && live.frameId === saved.frameId
    && live.epoch !== null
    && saved.epoch !== null
    && live.epoch !== saved.epoch
}

export function validateCloudFrameContract(
  frame: CloudFrameContract,
  endpoint: CloudEndpointKind,
  cursor: CloudSequenceCursor,
): CloudSequenceCursor {
  if (
    frame.protocolVersion === 2
    && (
      frame.frameId === null
      || frame.epoch === null
      || frame.stampS === null
      || frame.sequence === null
      || frame.streamKind === null
    )
  ) {
    decodeError('PCLD v2 frame is missing required alignment metadata')
  }

  // PCLD v1 has no stream metadata. Keep legacy accumulated-cloud support;
  // frame/epoch overlay gating still prevents an unproven v1 scan alignment.
  if (frame.streamKind === null) return cursor

  const expectedKind: CloudStreamKind = endpoint === 'cloud' ? 'map' : 'scan'
  if (frame.streamKind !== expectedKind && frame.streamKind !== 'reset') {
    decodeError(
      `unexpected PCLD stream kind for /ws/${endpoint}: ${frame.streamKind}`,
    )
  }
  if (frame.streamKind === 'reset' && frame.count !== 0) {
    decodeError('PCLD reset frame must be empty')
  }

  if (frame.epoch === null || frame.sequence === null) return cursor
  if (cursor.epoch === null) {
    return { epoch: frame.epoch, sequence: frame.sequence }
  }
  if (frame.epoch < cursor.epoch) {
    decodeError(`PCLD epoch regressed: ${frame.epoch} < ${cursor.epoch}`)
  }
  if (
    frame.epoch === cursor.epoch
    && cursor.sequence !== null
    && frame.sequence <= cursor.sequence
  ) {
    decodeError(
      `PCLD sequence did not increase: ${frame.sequence} <= ${cursor.sequence}`,
    )
  }
  return { epoch: frame.epoch, sequence: frame.sequence }
}

export function decodePointCloudFrame(buf: ArrayBuffer): DecodedCloudFrame {
  if (buf.byteLength < 5) decodeError('PCLD buffer is too small')
  const view = new DataView(buf)
  if (view.getUint32(0, true) !== MAGIC) decodeError('PCLD magic mismatch')

  const version = view.getUint8(4)
  if (version !== 1 && version !== 2) {
    decodeError(`unsupported PCLD version: ${version}`)
  }
  const minimumHeaderSize = version === 1 ? V1_HEADER_SIZE : V2_HEADER_BASE_SIZE
  if (buf.byteLength < minimumHeaderSize) {
    decodeError(`PCLD v${version} header is truncated`)
  }

  const flags = view.getUint8(5)
  if ((flags & ~KNOWN_FLAGS) !== 0) decodeError('PCLD flags contain unknown bits')
  const headerField = view.getUint16(6, true)
  if (version === 1 && headerField !== 0) {
    decodeError('PCLD v1 reserved header field is non-zero')
  }

  const count = view.getUint32(8, true)
  if (count > MAX_POINT_COUNT) {
    decodeError(`PCLD point count exceeds browser limit: ${count}`)
  }
  const scale = view.getFloat32(12, true)
  const ox = view.getFloat32(16, true)
  const oy = view.getFloat32(20, true)
  const oz = view.getFloat32(24, true)
  if (!Number.isFinite(scale) || scale <= 0) decodeError('PCLD scale must be finite and positive')
  if (![ox, oy, oz].every(Number.isFinite)) decodeError('PCLD origin must be finite')

  let payloadOffset = V1_HEADER_SIZE
  let metadata: CloudFrameMetadata = {
    protocolVersion: 1,
    frameId: null,
    epoch: null,
    stampS: null,
    sequence: null,
    streamKind: null,
  }

  if (version === 2) {
    const headerSize = headerField
    if (headerSize < V2_HEADER_BASE_SIZE || headerSize > buf.byteLength) {
      decodeError(`invalid PCLD v2 header size: ${headerSize}`)
    }
    const epoch = view.getUint32(28, true)
    const sequence = view.getUint32(32, true)
    const stampS = view.getFloat64(36, true)
    const streamKindCode = view.getUint8(44)
    const frameIdLength = view.getUint8(45)
    if (view.getUint16(46, true) !== 0) {
      decodeError('PCLD v2 reserved header field is non-zero')
    }
    if (!Number.isFinite(stampS)) {
      decodeError('PCLD v2 stamp must be finite')
    }
    const streamKind = STREAM_KINDS[streamKindCode]
    if (streamKind === undefined) {
      decodeError(`unknown PCLD v2 stream kind: ${streamKindCode}`)
    }
    const frameIdEnd = V2_HEADER_BASE_SIZE + frameIdLength
    if (frameIdEnd > headerSize) decodeError('PCLD v2 frame_id is truncated')
    const padding = new Uint8Array(buf, frameIdEnd, headerSize - frameIdEnd)
    if (padding.some(byte => byte !== 0)) decodeError('PCLD v2 header padding is non-zero')
    payloadOffset = headerSize
    metadata = {
      protocolVersion: 2,
      frameId: decodeFrameId(buf, V2_HEADER_BASE_SIZE, frameIdLength),
      epoch,
      stampS,
      sequence,
      streamKind,
    }
  }

  if (metadata.streamKind === 'reset' && count !== 0) {
    decodeError('PCLD reset frame must be empty')
  }

  const positionBytes = count * 3 * Int16Array.BYTES_PER_ELEMENT
  const colorBytes = (flags & FLAG_HAS_COLOR) !== 0 ? count * 3 : 0
  const expectedByteLength = payloadOffset + positionBytes + colorBytes
  if (buf.byteLength !== expectedByteLength) {
    decodeError(
      `PCLD length mismatch: expected ${expectedByteLength}, received ${buf.byteLength}`,
    )
  }

  if (count === 0) {
    return {
      ...metadata,
      positions: new Float32Array(0),
      colors: new Float32Array(0),
      count: 0,
    }
  }

  const xyz = int16PayloadView(buf, payloadOffset, count * 3)
  const rgb = colorBytes > 0
    ? new Uint8Array(buf, payloadOffset + positionBytes, count * 3)
    : null
  const positions = new Float32Array(count * 3)
  const colors = new Float32Array(count * 3)
  let written = 0

  for (let i = 0; i < count; i++) {
    const wx = xyz[i * 3] * scale + ox
    const wy = xyz[i * 3 + 1] * scale + oy
    const wz = xyz[i * 3 + 2] * scale + oz
    if (!Number.isFinite(wx) || !Number.isFinite(wy) || !Number.isFinite(wz)) {
      decodeError('PCLD point expands to a non-finite coordinate')
    }
    if (wz < Z_FLOOR || wz > Z_CEIL) continue

    const off = written * 3
    positions[off] = wx
    positions[off + 1] = wz
    positions[off + 2] = -wy
    if (rgb) {
      colors[off] = rgb[i * 3] / 255
      colors[off + 1] = rgb[i * 3 + 1] / 255
      colors[off + 2] = rgb[i * 3 + 2] / 255
    } else {
      turboColor((wz - COLOR_Z_MIN) / COLOR_Z_SPAN, colors, off)
    }
    written++
  }

  return {
    ...metadata,
    positions: positions.subarray(0, written * 3),
    colors: colors.subarray(0, written * 3),
    count: written,
  }
}
