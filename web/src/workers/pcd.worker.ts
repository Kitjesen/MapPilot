/// <reference lib="webworker" />
import { parsePcd, pcdPointCount, type PcdCloud } from '../services/pcdPreview.ts'
import { Box3, BufferAttribute } from 'three'
import { cloudColorRange } from '../services/cloudHeightColor.ts'

self.onmessage = (event: MessageEvent<ArrayBuffer>) => {
  try {
    const points = parsePcd(event.data)
    if (!points?.length) throw new Error('地图点云为空或格式不支持')
    const bounds = new Box3().setFromBufferAttribute(new BufferAttribute(points, 3))
    const cloud: PcdCloud = {
      positions: points,
      bounds: { min: bounds.min.toArray(), max: bounds.max.toArray() },
      colorRange: cloudColorRange(points)!,
      sourceCount: pcdPointCount(event.data),
    }
    self.postMessage({ cloud }, { transfer: [cloud.positions.buffer] })
  } catch (error) {
    self.postMessage({ error: error instanceof Error ? error.message : String(error) })
  }
}
