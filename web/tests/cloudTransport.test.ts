import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import * as decoder from '../src/workers/cloudDecoderCore.ts'
import * as heightColor from '../src/services/cloudHeightColor.ts'
import type { BinaryCloud } from '../src/hooks/useBinaryCloud.ts'

const code = ts.transpileModule(readFileSync(new URL('../src/hooks/useBinaryCloud.ts', import.meta.url), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022, esModuleInterop: true },
}).outputText

function mount(path: string | null = '/ws/cloud', fallback: string | null = '/api/v1/map/points', payload?: Record<string, unknown>) {
  let state: BinaryCloud
  let cleanup: () => void = () => {}
  let now = 10000
  let nextTimer = 1
  let fetches = 0
  const timers = new Map<number, { at: number; fn: () => void }>()
  const sockets: FakeSocket[] = []
  const workers: FakeWorker[] = []
  const listeners = new Map<string, () => void>()
  class FakeSocket {
    static OPEN = 1
    static CLOSING = 2
    readyState = 0
    onopen: (() => void) | null = null
    onclose: (() => void) | null = null
    onmessage: ((event: { data: unknown }) => void) | null = null
    onerror: (() => void) | null = null
    constructor() { sockets.push(this) }
    open() { this.readyState = 1; this.onopen?.() }
    close() { this.readyState = 3; this.onclose?.() }
  }
  class FakeWorker {
    onmessage: ((event: { data: unknown }) => void) | null = null
    onerror: ((event: unknown) => void) | null = null
    constructor() { workers.push(this) }
    requests: Array<{ connectionGeneration: number }> = []
    postMessage(message: { connectionGeneration: number }) { this.requests.push(message) }
    terminate() {}
    frame(sequence: number, connectionGeneration = 1) {
      this.onmessage?.({ data: { type: 'cloud', protocolVersion: 2, frameId: 'map', epoch: 1,
        sequence, seq: sequence, stampS: now / 1000, streamKind: path?.includes('scan') ? 'scan' : 'map',
        count: 1, positions: new Float32Array([1, 0, 0]), colors: new Float32Array(3), connectionGeneration } })
    }
  }
  const exports: Partial<typeof import('../src/hooks/useBinaryCloud.ts')> = {}
  runInNewContext(code, {
    exports,
    require: (name: string) => {
      if (name === 'react') return {
        useState: (value: BinaryCloud) => { state = value; return [state, (update: BinaryCloud | ((previous: BinaryCloud) => BinaryCloud)) => { state = typeof update === 'function' ? update(state) : update }] },
        useRef: (current: unknown) => ({ current }),
        useEffect: (effect: () => () => void) => { cleanup = effect() },
      }
      if (name.includes('?worker')) return FakeWorker
      if (name.includes('cloudDecoderCore')) return decoder
      if (name.includes('cloudHeightColor')) return heightColor
      throw new Error(name)
    },
    window: { location: { protocol: 'http:', host: 'robot' },
      addEventListener(name: string, fn: () => void) { listeners.set(name, fn) },
      removeEventListener(name: string) { listeners.delete(name) } },
    WebSocket: FakeSocket, Date: { now: () => now },
    setTimeout: (fn: () => void, delay: number) => { const id = nextTimer++; timers.set(id, { at: now + delay, fn }); return id },
    clearTimeout: (id: number) => timers.delete(id),
    fetch: () => { fetches++; return payload ? Promise.resolve({ ok: true, json: async () => payload }) : new Promise(() => {}) },
    Float32Array, ArrayBuffer, Error, console,
  })
  assert.ok(exports.useBinaryCloud)
  exports.useBinaryCloud(path, fallback, 4)
  const advance = (ms: number) => {
    const end = now + ms
    while (true) {
      const due = [...timers].filter(([, timer]) => timer.at <= end).sort((a, b) => a[1].at - b[1].at)[0]
      if (!due) break
      now = due[1].at; timers.delete(due[0]); due[1].fn()
    }
    now = end
  }
  return { socket: sockets[0], sockets, worker: workers[0], advance,
    reset: () => listeners.get('lingtu:cloud-reset')?.(),
    cleanup: () => cleanup(), state: () => state, fetches: () => fetches }
}

test('reset discards in-flight decoded frames and accepts the new stream', () => {
  const h = mount(); h.socket.open()
  h.socket.onmessage?.({ data: new ArrayBuffer(1) })
  const oldGeneration = h.worker.requests[0].connectionGeneration
  h.reset()
  h.worker.frame(1, oldGeneration)
  assert.equal(h.state().count, 0, 'a pre-reset frame must not repaint the cleared map')
  const socket = h.sockets.at(-1)!
  socket.open()
  h.advance(300)
  socket.onmessage?.({ data: new ArrayBuffer(1) })
  h.worker.frame(1, h.worker.requests.at(-1)!.connectionGeneration)
  assert.equal(h.state().count, 1)
  assert.equal(h.state().error, null)
  h.cleanup()
})

test('map stream silence after a valid frame activates HTTP recovery', () => {
  const h = mount(); h.socket.open(); h.worker.frame(1)
  h.advance(2600)
  assert.equal(h.fetches(), 1)
  assert.equal(h.state().transport, 'http')
  assert.equal(h.state().connected, false)
  h.worker.frame(2)
  assert.equal(h.state().transport, 'ws')
  assert.equal(h.state().connected, true)
  h.cleanup()
})

test('scan silence clears the old scan and a resumed frame restores connected state', () => {
  const h = mount('/ws/scan', null); h.socket.open(); h.worker.frame(1)
  h.advance(2600)
  assert.equal(h.state().count, 0)
  assert.equal(h.state().connected, false)
  h.worker.frame(2)
  assert.equal(h.state().count, 1)
  assert.equal(h.state().connected, true)
  assert.equal(h.state().error, null)
  h.cleanup()
})

test('fresh frames renew the deadline; disposal cancels recovery', () => {
  const h = mount(); h.socket.open(); h.worker.frame(1)
  h.advance(2000); h.worker.frame(2); h.advance(2000)
  assert.equal(h.fetches(), 0)
  h.cleanup(); h.advance(3000)
  assert.equal(h.fetches(), 0)
})

test('whole-map HTTP preview opens without a local-cloud websocket', () => {
  const h = mount(null, '/api/v1/map/global/points')
  assert.equal(h.socket, undefined)
  assert.equal(h.fetches(), 1)
  assert.equal(h.state().transport, 'http')
  h.cleanup()
})

test('disabled preview opens neither HTTP nor websocket', () => {
  const h = mount(null, null)
  assert.equal(h.socket, undefined)
  assert.equal(h.fetches(), 0)
  h.cleanup()
})

test('reset during an HTTP request starts only one replacement polling chain', async () => {
  const h = mount(null, '/api/v1/map/global/points', {
    points: [], count: 0, frame_id: 'map', epoch: 1,
    sequence: 1, stamp_s: 12, stream_kind: 'map',
  })
  h.reset()
  await new Promise(resolve => setImmediate(resolve))
  assert.equal(h.fetches(), 2)
  h.advance(1000)
  await new Promise(resolve => setImmediate(resolve))
  assert.equal(h.fetches(), 3)
  h.cleanup()
})

test('whole-map preview preserves native correction quality counts', async () => {
  const h = mount(null, '/api/v1/map/global/points', {
    points: [[1, 2, 3]], count: 1, layout: 'xyz_rows', frame_id: 'map',
    epoch: 1, sequence: 3, stamp_s: 12, stream_kind: 'map',
    global_mapping: { state: 'accumulating', loops: 7, optimizations: 3, rejected_keyframes: 43, dropped_frames: 0 },
  })
  await new Promise(resolve => setImmediate(resolve))
  assert.equal(h.state().error, null)
  assert.equal(h.state().count, 1)
  assert.equal(h.state().mappingSummary.registrationRejections, 43)
  assert.equal(h.state().mappingSummary.optimizations, 3)
  h.cleanup()
})
