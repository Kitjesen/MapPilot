/**
 * Web Worker entry point for strict PCLD v1/v2 decoding.
 *
 * Geometry conversion and validation live in cloudDecoderCore so the wire
 * contract can be exercised without a browser Worker runtime.
 */
import {
  decodePointCloudFrame,
  PointCloudDecodeError,
} from './cloudDecoderCore.ts'

let presentationSequence = 0

interface DecodeRequest {
  buffer: ArrayBuffer
  connectionGeneration: number
  httpRequestGeneration?: number
  globalMapping?: Record<string, unknown>
}

self.onmessage = (event: MessageEvent<DecodeRequest>) => {
  const { buffer, connectionGeneration, httpRequestGeneration, globalMapping } = event.data
  try {
    const decoded = decodePointCloudFrame(buffer)
    const positions = new Float32Array(decoded.positions)
    const colors = new Float32Array(decoded.colors)
    ;(self as unknown as Worker).postMessage(
      {
        type: 'cloud',
        positions,
        colors,
        count: decoded.count,
        seq: ++presentationSequence,
        protocolVersion: decoded.protocolVersion,
        frameId: decoded.frameId,
        epoch: decoded.epoch,
        stampS: decoded.stampS,
        sequence: decoded.sequence,
        streamKind: decoded.streamKind,
        connectionGeneration,
        httpRequestGeneration,
        globalMapping,
      },
      [positions.buffer, colors.buffer],
    )
  } catch (error) {
    const message = error instanceof PointCloudDecodeError || error instanceof Error
      ? error.message
      : 'unknown PCLD decode failure'
    ;(self as unknown as Worker).postMessage({
      type: 'error',
      error: message,
      connectionGeneration,
      httpRequestGeneration,
    })
  }
}

export {}
