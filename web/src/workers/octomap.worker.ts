/// <reference lib="webworker" />
import { parseOctomap } from '../services/octomap.ts'

self.onmessage = (event: MessageEvent<ArrayBuffer>) => {
  try {
    const map = parseOctomap(event.data)
    self.postMessage({ map }, { transfer: [map.cells.buffer] })
  } catch (error) {
    self.postMessage({ error: error instanceof Error ? error.message : String(error) })
  }
}
