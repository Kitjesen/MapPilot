import assert from 'node:assert/strict'
import test from 'node:test'

import {
  cloudFramesShareCoordinateEpoch,
  cloudFrameSharesSavedMapFrame,
  decodePointCloudFrame,
  parseHttpCloudMetadata,
  PointCloudDecodeError,
  savedMapNeedsSceneRebind,
  validateCloudFrameContract,
} from '../src/workers/cloudDecoderCore.ts'

const V1_HEADER_SIZE = 28
const V2_HEADER_BASE_SIZE = 48

function setMagic(view: DataView): void {
  view.setUint8(0, 0x50)
  view.setUint8(1, 0x43)
  view.setUint8(2, 0x4c)
  view.setUint8(3, 0x44)
}

function setCommonHeader(
  view: DataView,
  version: 1 | 2,
  flags: number,
  count: number,
): void {
  setMagic(view)
  view.setUint8(4, version)
  view.setUint8(5, flags)
  view.setUint16(6, 0, true)
  view.setUint32(8, count, true)
  view.setFloat32(12, 0.5, true)
  view.setFloat32(16, 1, true)
  view.setFloat32(20, 2, true)
  view.setFloat32(24, 3, true)
}

function v1Frame(): ArrayBuffer {
  const count = 1
  const buf = new ArrayBuffer(V1_HEADER_SIZE + count * 3 * 2)
  const view = new DataView(buf)
  setCommonHeader(view, 1, 0, count)
  view.setInt16(V1_HEADER_SIZE, 2, true)
  view.setInt16(V1_HEADER_SIZE + 2, 4, true)
  view.setInt16(V1_HEADER_SIZE + 4, -2, true)
  return buf
}

function v2Frame(): ArrayBuffer {
  const frameBytes = new TextEncoder().encode('map')
  const paddingBytes = frameBytes.length % 2
  const count = 2
  const positionBytes = count * 3 * 2
  const colorBytes = count * 3
  const payloadOffset = V2_HEADER_BASE_SIZE + frameBytes.length + paddingBytes
  const buf = new ArrayBuffer(payloadOffset + positionBytes + colorBytes)
  const view = new DataView(buf)
  setCommonHeader(view, 2, 0x01, count)
  view.setUint16(6, payloadOffset, true)
  view.setUint32(28, 7, true)
  view.setUint32(32, 42, true)
  view.setFloat64(36, 12.5, true)
  view.setUint8(44, 2)
  view.setUint8(45, frameBytes.length)
  view.setUint16(46, 0, true)
  new Uint8Array(buf, V2_HEADER_BASE_SIZE, frameBytes.length).set(frameBytes)

  const quantized = [0, 0, 0, 2, 4, -2]
  quantized.forEach((value, index) => {
    view.setInt16(payloadOffset + index * 2, value, true)
  })
  new Uint8Array(buf, payloadOffset + positionBytes, colorBytes).set([
    255, 128, 0,
    0, 64, 255,
  ])
  return buf
}

test('decodes legacy PCLD v1 and exposes absent alignment metadata', () => {
  const decoded = decodePointCloudFrame(v1Frame())

  assert.equal(decoded.protocolVersion, 1)
  assert.equal(decoded.frameId, null)
  assert.equal(decoded.epoch, null)
  assert.equal(decoded.stampS, null)
  assert.equal(decoded.sequence, null)
  assert.equal(decoded.streamKind, null)
  assert.equal(decoded.count, 1)
  assert.deepEqual(Array.from(decoded.positions), [2, 2, -4])
})

test('decodes PCLD v2 metadata, colors, and odd-length frame_id padding', () => {
  const decoded = decodePointCloudFrame(v2Frame())

  assert.equal(decoded.protocolVersion, 2)
  assert.equal(decoded.frameId, 'map')
  assert.equal(decoded.epoch, 7)
  assert.equal(decoded.stampS, 12.5)
  assert.equal(decoded.sequence, 42)
  assert.equal(decoded.streamKind, 'scan')
  assert.equal(decoded.count, 2)
  assert.deepEqual(Array.from(decoded.positions), [1, 3, -2, 2, 2, -4])
  assert.ok(Math.abs(decoded.colors[0] - 1) < 1e-6)
  assert.ok(Math.abs(decoded.colors[1] - (128 / 255)) < 1e-6)
  assert.ok(Math.abs(decoded.colors[5] - 1) < 1e-6)
})

test('rejects truncated and trailing PCLD payload bytes', () => {
  const valid = v2Frame()
  assert.throws(
    () => decodePointCloudFrame(valid.slice(0, valid.byteLength - 1)),
    (error: unknown) => error instanceof PointCloudDecodeError
      && /length mismatch/.test(error.message),
  )

  const trailing = new Uint8Array(valid.byteLength + 1)
  trailing.set(new Uint8Array(valid))
  assert.throws(
    () => decodePointCloudFrame(trailing.buffer),
    (error: unknown) => error instanceof PointCloudDecodeError
      && /length mismatch/.test(error.message),
  )
})

test('rejects unknown v2 stream kinds before allocating point buffers', () => {
  const buf = v2Frame()
  new DataView(buf).setUint8(44, 99)

  assert.throws(
    () => decodePointCloudFrame(buf),
    (error: unknown) => error instanceof PointCloudDecodeError
      && /stream kind/.test(error.message),
  )
})

test('rejects a non-empty v2 reset before display filtering', () => {
  const buf = v2Frame()
  new DataView(buf).setUint8(44, 3)

  assert.throws(
    () => decodePointCloudFrame(buf),
    (error: unknown) => error instanceof PointCloudDecodeError
      && /reset frame must be empty/.test(error.message),
  )
})

test('scan overlay compatibility requires both frame_id and epoch equality', () => {
  assert.equal(
    cloudFramesShareCoordinateEpoch(
      { frameId: 'map', epoch: 7 },
      { frameId: 'map', epoch: 7 },
    ),
    true,
  )
  assert.equal(
    cloudFramesShareCoordinateEpoch(
      { frameId: 'map', epoch: 7 },
      { frameId: 'odom', epoch: 7 },
    ),
    false,
  )
  assert.equal(
    cloudFramesShareCoordinateEpoch(
      { frameId: 'map', epoch: 7 },
      { frameId: 'map', epoch: 8 },
    ),
    false,
  )
  assert.equal(
    cloudFramesShareCoordinateEpoch(
      { frameId: null, epoch: null },
      { frameId: null, epoch: null },
    ),
    false,
  )
})

test('saved map overlay requires an exact known live frame match', () => {
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'map', epoch: 7 }, 'map', 7),
    true,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'odom', epoch: 7 }, 'map', 7),
    false,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'map', epoch: 8 }, 'map', 7),
    false,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: null, epoch: null }, 'map', 7),
    false,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'map', epoch: 7 }, null, null),
    false,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'map', epoch: null }, 'map', null),
    true,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: '/map', epoch: null }, 'map', null),
    true,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'odom', epoch: null }, 'map', null),
    false,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'map', epoch: null }, 'map', 7),
    false,
  )
  assert.equal(
    cloudFrameSharesSavedMapFrame({ frameId: 'map', epoch: 7 }, 'map', null),
    false,
  )
})

test('saved map identity is rebound after a same-frame scene epoch reset', () => {
  assert.equal(
    savedMapNeedsSceneRebind(
      { frameId: 'map', epoch: 8 },
      { frameId: 'map', epoch: 7 },
    ),
    true,
  )
  assert.equal(
    savedMapNeedsSceneRebind(
      { frameId: 'odom', epoch: 8 },
      { frameId: 'map', epoch: 7 },
    ),
    false,
  )
})

const baseV2Metadata = {
  protocolVersion: 2 as const,
  frameId: 'map',
  epoch: 7,
  stampS: 12.5,
  sequence: 1,
}

test('endpoint contracts accept only the expected v2 stream kind or empty reset', () => {
  const initial = { epoch: null, sequence: null }
  assert.deepEqual(
    validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'map', count: 10 },
      'cloud',
      initial,
    ),
    { epoch: 7, sequence: 1 },
  )
  assert.deepEqual(
    validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'scan', count: 10 },
      'scan',
      initial,
    ),
    { epoch: 7, sequence: 1 },
  )
  assert.deepEqual(
    validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'reset', count: 0 },
      'cloud',
      initial,
    ),
    { epoch: 7, sequence: 1 },
  )

  assert.throws(
    () => validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'scan', count: 10 },
      'cloud',
      initial,
    ),
    /unexpected PCLD stream kind/,
  )
  assert.throws(
    () => validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'map', count: 10 },
      'scan',
      initial,
    ),
    /unexpected PCLD stream kind/,
  )
  assert.throws(
    () => validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'reset', count: 1 },
      'cloud',
      initial,
    ),
    /reset frame must be empty/,
  )
})

test('frame ordering rejects epoch rollback and non-increasing same-epoch sequence', () => {
  const first = validateCloudFrameContract(
    { ...baseV2Metadata, streamKind: 'map', count: 10 },
    'cloud',
    { epoch: null, sequence: null },
  )
  const second = validateCloudFrameContract(
    { ...baseV2Metadata, streamKind: 'map', sequence: 2, count: 10 },
    'cloud',
    first,
  )
  const nextEpoch = validateCloudFrameContract(
    { ...baseV2Metadata, streamKind: 'reset', epoch: 8, sequence: 1, count: 0 },
    'cloud',
    second,
  )
  assert.deepEqual(nextEpoch, { epoch: 8, sequence: 1 })

  assert.throws(
    () => validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'map', epoch: 7, sequence: 99, count: 10 },
      'cloud',
      nextEpoch,
    ),
    /epoch regressed/,
  )
  assert.throws(
    () => validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'map', epoch: 8, sequence: 1, count: 10 },
      'cloud',
      nextEpoch,
    ),
    /sequence did not increase/,
  )
  assert.throws(
    () => validateCloudFrameContract(
      { ...baseV2Metadata, streamKind: 'map', epoch: 8, sequence: 0, count: 10 },
      'cloud',
      nextEpoch,
    ),
    /sequence did not increase/,
  )
})

test('legacy v1 frames remain accepted without ordering metadata', () => {
  const cursor = { epoch: 7, sequence: 9 }
  assert.deepEqual(
    validateCloudFrameContract(
      {
        protocolVersion: 1,
        frameId: null,
        epoch: null,
        stampS: null,
        sequence: null,
        streamKind: null,
        count: 10,
      },
      'cloud',
      cursor,
    ),
    cursor,
  )
})

test('HTTP fallback metadata preserves the PCLD v2 alignment contract', () => {
  const metadata = parseHttpCloudMetadata({
    protocol_version: 2,
    frame_id: 'map',
    epoch: 9,
    stamp_s: 123.5,
    sequence: 77,
    stream_kind: 'map',
  })
  assert.deepEqual(metadata, {
    protocolVersion: 2,
    frameId: 'map',
    epoch: 9,
    stampS: 123.5,
    sequence: 77,
    streamKind: 'map',
  })
  assert.deepEqual(
    validateCloudFrameContract(
      { ...metadata, count: 5 },
      'cloud',
      { epoch: null, sequence: null },
    ),
    { epoch: 9, sequence: 77 },
  )

  const incomplete = parseHttpCloudMetadata({
    protocol_version: 2,
    frame_id: 'map',
    stream_kind: 'map',
  })
  assert.throws(
    () => validateCloudFrameContract(
      { ...incomplete, count: 5 },
      'cloud',
      { epoch: null, sequence: null },
    ),
    /missing required alignment metadata/,
  )
})
