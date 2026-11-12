import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import * as truth from '../src/services/authoritativeTruth.ts'
import * as mapScene from '../src/services/mapSceneState.ts'
import * as telemetry from '../src/services/jointTelemetryStream.ts'
import * as navigation from '../src/services/navigationStatus.ts'
import { mapLocalizationState } from '../src/services/mapLabel.ts'
import type { StateResponse } from '../src/types/index.ts'

const code = ts.transpileModule(readFileSync(new URL('../src/hooks/useSSE.ts', import.meta.url), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
}).outputText

function snapshot(map: string, state = 'tracking'): StateResponse {
  return { schema_version: 4, ts: 100, session: { active_map: map },
    localization: { state, pose_fresh: true, odometry: { x: 1, y: 2 } } }
}

function navigationSnapshot(ts: number, permission: 'HELD' | 'CLEAR'): StateResponse {
  return { ...snapshot('site'), ts, navigation: {
    schema_version: 3, ts, task: { state: 'IDLE', task_id: null, reason: '' },
    goal_admission: { state: 'BLOCKED', reason: '' },
    control: { authority: 'NONE', resume_required: false, reason: '' },
    motion: { permission, observation: 'UNKNOWN', stop_confirmation: 'UNKNOWN', reason: '' },
  } }
}

test('late HTTP snapshots cannot rewind newer SSE status or its freshness clock', async () => {
  let resolve!: (value: StateResponse) => void
  const h = mount(() => new Promise(done => { resolve = done }))
  try {
    h.socket.onopen?.()
    h.socket.snapshot(navigationSnapshot(100, 'HELD'))
    const request = h.refresh()
    await Promise.resolve()
    h.socket.snapshot(navigationSnapshot(102, 'CLEAR'))
    resolve(navigationSnapshot(101, 'HELD'))
    await request
    assert.equal(h.state().navigationStatus?.motion.permission, 'CLEAR')
    assert.equal(h.state().stateSnapshot?.ts, 102)
    h.socket.snapshot(navigationSnapshot(101, 'HELD'))
    assert.equal(h.state().stateSnapshot?.ts, 102)
  } finally { h.cleanup() }
})

test('navigation events stay ordered across snapshot refreshes and still expire', async () => {
  const h = mount(async () => navigationSnapshot(103, 'HELD'))
  try {
    h.socket.onopen?.()
    h.socket.snapshot(navigationSnapshot(100, 'HELD'))
    const emit = (ts: number, permission: 'HELD' | 'CLEAR') => h.socket.onmessage?.({ data: JSON.stringify({ type: 'navigation_status', data: navigationSnapshot(ts, permission).navigation }) })
    emit(105, 'CLEAR')
    await h.refresh()
    assert.equal(h.state().navigationStatus?.ts, 105)
    emit(104, 'HELD')
    assert.equal(h.state().navigationStatus?.motion.permission, 'CLEAR')
    emit(107, 'HELD')
    assert.equal(h.state().navigationStatus?.motion.permission, 'HELD')
    h.advance(15000)
    assert.equal(navigation.liveNavigationStatus(h.state(), 1015), null)
  } finally { h.cleanup() }
})

function mount(fetchState: () => Promise<StateResponse>, fetchPath = async () => null, enabled = true) {
  let now = 1000_000
  let cursor = 0
  let timerId = 0
  const timers = new Map<number, () => void>()
  const intervals = new Map<number, () => void>()
  const slots: Array<{ value?: unknown; deps?: unknown[]; cleanup?: () => void }> = []
  let effects: Array<() => void> = []
  const calls: string[] = []
  const sockets: FakeSource[] = []
  class FakeSource {
    onopen: (() => void) | null = null
    onmessage: ((event: { data: string }) => void) | null = null
    onerror: (() => void) | null = null
    constructor() { sockets.push(this) }
    closed = false
    close() { this.closed = true }
    snapshot(data: StateResponse) { this.onmessage?.({ data: JSON.stringify({ type: 'snapshot', data }) }) }
  }
  const exports: Partial<typeof import('../src/hooks/useSSE.ts')> = {}
  runInNewContext(code, {
    exports,
    require: (name: string) => {
      if (name === 'react') return {
        useState(initial: unknown) {
          const slot = cursor++
          slots[slot] ??= { value: typeof initial === 'function' ? initial() : initial }
          return [slots[slot].value, (update: unknown) => {
            slots[slot].value = typeof update === 'function' ? update(slots[slot].value) : update
          }]
        },
        useRef(value: unknown) {
          const slot = cursor++
          slots[slot] ??= { value: { current: value } }
          return slots[slot].value
        },
        useCallback(callback: unknown, deps: unknown[]) {
          const slot = cursor++
          const previous = slots[slot]
          if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) {
            slots[slot] = { value: callback, deps }
          }
          return slots[slot].value
        },
        useEffect(effect: () => void | (() => void), deps: unknown[]) {
          const slot = cursor++
          const previous = slots[slot]
          if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) {
            effects.push(() => {
              previous?.cleanup?.()
              slots[slot] = { deps, cleanup: effect() || undefined }
            })
          }
        },
      }
      if (name.endsWith('/api')) return {
        fetchState: () => { calls.push('state'); return fetchState() }, fetchPath,
        fetchSceneGraph: async () => null,
        fetchLocations: async () => null,
        fetchAppBootstrap: async () => { calls.push('bootstrap'); return null },
        fetchAppCapabilities: async () => { calls.push('capabilities'); return null },
      }
      if (name.endsWith('/authoritativeTruth')) return truth
      if (name.endsWith('/navigationStatus')) return navigation
      if (name.endsWith('/mapSceneState.ts')) return mapScene
      if (name.endsWith('/jointTelemetryStream.ts')) return telemetry
      throw new Error(name)
    },
    EventSource: FakeSource,
    Date: { now: () => now }, performance: { now: () => now }, Error,
    setTimeout: (callback: () => void) => { timers.set(++timerId, callback); return timerId },
    clearTimeout: (id: number) => timers.delete(id),
    setInterval: (callback: () => void) => { intervals.set(++timerId, callback); return timerId },
    clearInterval: (id: number) => intervals.delete(id),
  })
  assert.ok(exports.useSSE)
  const runHook = exports.useSSE
  const render = (nextEnabled = enabled) => {
    enabled = nextEnabled
    cursor = 0
    effects = []
    const result = runHook(undefined, enabled)
    effects.forEach(effect => effect())
    return result
  }
  render()
  return {
    render, calls, intervals, sockets,
    refresh: () => render().refresh(),
    state: () => render(),
    get socket() { return sockets[sockets.length - 1] },
    advance: (ms: number) => { now += ms },
    runTimers: () => {
      const pending = [...timers.values()]
      timers.clear()
      pending.forEach(callback => callback())
    },
    cleanup: () => slots.forEach(slot => slot.cleanup?.()),
  }
}

test('manual refresh replaces the displayed source and later SSE snapshots take over', async () => {
  let requests = 0
  const h = mount(async () => { requests++; return snapshot('refreshed') })
  try {
    h.socket.onopen?.()
    h.socket.snapshot(snapshot('initial', 'initializing'))
    assert.equal(h.state().session?.active_map, 'initial')
    assert.equal(mapLocalizationState(h.state(), 1000), 'initializing')
    await h.refresh()
    assert.equal(requests, 1)
    assert.equal(h.state().session?.active_map, 'refreshed')
    assert.equal(mapLocalizationState(h.state(), 1000), 'tracking')
    h.advance(8000)
    assert.equal(mapLocalizationState(h.state(), 1008), 'stale')
    h.socket.snapshot(snapshot('stream-resumed', 'relocalizing'))
    assert.equal(h.state().session?.active_map, 'stream-resumed')
    assert.equal(mapLocalizationState(h.state(), 1008), 'relocalizing')
    h.socket.onerror?.()
    assert.equal(mapLocalizationState(h.state(), 1008), 'disconnected')
  } finally { h.cleanup() }
})

test('standby leaves runtime requests idle and reconnects on start', async () => {
  const h = mount(async () => snapshot('office'), undefined, false)
  try {
    await h.refresh()
    assert.equal(h.sockets.length, 0)
    assert.equal(h.intervals.size, 0)
    assert.deepEqual(h.calls, [])
    h.render(true)
    await new Promise(resolve => setImmediate(resolve))
    assert.equal(h.sockets.length, 1)
    assert.equal(h.intervals.size, 1)
    h.socket.onopen?.()
    h.socket.snapshot(snapshot('office'))
    assert.equal(h.state().connected, true)
    h.render(false)
    assert.equal(h.socket.closed, true)
    assert.equal(h.intervals.size, 0)
    assert.equal(h.state().connected, false)
    assert.equal(h.state().session, null)
    const requests = h.calls.length
    h.runTimers()
    await h.refresh()
    assert.equal(h.calls.length, requests)
    h.render(true)
    assert.equal(h.sockets.length, 2)
    assert.equal(h.state().connected, false)
    assert.equal(h.state().lastTruthAt, null)
  } finally { h.cleanup() }
})

test('unexpected disconnect still reconnects', () => {
  const h = mount(async () => snapshot('office'))
  try {
    h.socket.onopen?.()
    h.socket.onerror?.()
    assert.equal(h.state().connected, false)
    h.runTimers()
    assert.equal(h.sockets.length, 2)
  } finally { h.cleanup() }
})

test('a stopped run cannot overwrite the restarted run with a late response', async () => {
  let release: (value: StateResponse) => void = () => {}
  const response = new Promise<StateResponse>(resolve => { release = resolve })
  const h = mount(() => response)
  try {
    const pending = h.refresh()
    await new Promise(resolve => setImmediate(resolve))
    h.render(false)
    h.render(true)
    h.socket.onopen?.()
    h.socket.snapshot(snapshot('new-run'))
    release(snapshot('stopped-run'))
    await pending
    assert.equal(h.state().session?.active_map, 'new-run')
  } finally { h.cleanup() }
})

test('manual refresh waits for auxiliary work and still fetches authoritative state', async () => {
  let requests = 0
  let releasePath: (value: null) => void = () => {}
  const pathPending = new Promise<null>(resolve => { releasePath = resolve })
  const h = mount(async () => { requests++; return snapshot('refreshed') }, () => pathPending)
  try {
    h.socket.onopen?.()
    h.socket.snapshot(snapshot('initial'))
    h.runTimers()
    const refreshing = h.refresh()
    assert.equal(requests, 0)
    releasePath(null)
    await refreshing
    assert.equal(requests, 1)
    assert.equal(h.state().session?.active_map, 'refreshed')
  } finally { h.cleanup() }
})

test('refresh failure keeps prior evidence and reports the failure to the refresh control', async () => {
  const h = mount(async () => { throw new Error('gateway unavailable') })
  try {
    h.socket.onopen?.()
    h.socket.snapshot(snapshot('initial'))
    await assert.rejects(h.refresh(), /gateway unavailable/)
    assert.equal(h.state().session?.active_map, 'initial')
    assert.equal(h.state().truthError, 'gateway unavailable')
    h.advance(8000)
    assert.equal(mapLocalizationState(h.state(), 1008), 'stale')
    h.socket.snapshot(snapshot('stream-recovered'))
    assert.equal(h.state().truthError, null)
    assert.equal(mapLocalizationState(h.state(), 1008), 'tracking')
  } finally { h.cleanup() }
})
