import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import {
  ProductControlError, switchIsTerminal,
  type ProductControlSnapshot, type ProductSwitchOperation,
} from '../src/services/productControl.ts'

const code = ts.transpileModule(readFileSync(new URL('../src/hooks/useProductControl.ts', import.meta.url), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
}).outputText
const pendingKey = 'lingtu.product-control.pending'
const settle = () => new Promise<void>(resolve => setImmediate(resolve))

function snapshot(product: string, session = 'current'): ProductControlSnapshot {
  return { available: true, robot: 'unitree/go2', env: 'real',
    current: { product, status: 'active', product_session_id: session }, operation: null }
}

function operation(state: ProductSwitchOperation['state']): ProductSwitchOperation {
  return { request_id: 'pending', state, message: state, result: null,
    request: { request_id: 'pending', product: 'nav', map_name: 'office', expected_product_session_id: 'old' } }
}

function mount(
  storage: Map<string, string>,
  fetchState: () => Promise<ProductControlSnapshot>,
  fetchOperation: () => Promise<ProductSwitchOperation> = async () => { throw new Error('unexpected operation query') },
) {
  const calls: string[] = []
  const changed: Array<[string, boolean]> = []
  const effects: Array<() => void | (() => void)> = []
  const timers = new Map<number, () => void>()
  let timerId = 0
  let stateSlot = 0
  let refSlot = 0
  const states: unknown[] = []
  const refs: Array<{ current: unknown }> = []
  const exports: Partial<typeof import('../src/hooks/useProductControl.ts')> = {}
  runInNewContext(code, {
    exports,
    localStorage: {
      getItem: (key: string) => storage.get(key) ?? null,
      setItem: (key: string, value: string) => storage.set(key, value),
      removeItem: (key: string) => storage.delete(key),
    },
    setTimeout: (callback: () => void) => { timers.set(++timerId, callback); return timerId },
    clearTimeout: (id: number) => timers.delete(id),
    require: (name: string) => {
      if (name === 'react') return {
        useState: (initial: unknown) => {
          const slot = stateSlot++
          if (slot === states.length) states.push(typeof initial === 'function' ? initial() : initial)
          return [states[slot], (update: unknown) => {
            states[slot] = typeof update === 'function' ? update(states[slot]) : update
          }]
        },
        useRef: (current: unknown) => {
          const slot = refSlot++
          if (slot === refs.length) refs.push({ current })
          return refs[slot]
        },
        useCallback: (callback: unknown) => callback,
        useEffect: (effect: () => void | (() => void)) => effects.push(effect),
      }
      if (name.endsWith('/observationMode.ts')) return { isObservationMode: () => false }
      if (name.endsWith('/api.ts')) return { makeRequestId: () => { throw new Error('reconnect created a new request') } }
      if (name.endsWith('/productControl.ts')) return {
        ProductControlError, switchIsTerminal,
        fetchProductControl: () => { calls.push('status'); return fetchState() },
        fetchProductOperation: (id: string) => { calls.push(`operation:${id}`); return fetchOperation() },
        submitProductSwitch: () => { throw new Error('reconnect resubmitted a switch') },
      }
      throw new Error(`Unexpected import ${name}`)
    },
  })
  assert.ok(exports.useProductControl)
  const runHook = exports.useProductControl
  const read = () => {
    stateSlot = 0
    refSlot = 0
    return runHook((product, restoring) => changed.push([product, restoring]))
  }
  read()
  const cleanups = effects.map(effect => effect())
  return {
    calls, changed, read,
    poll: async () => {
      const pending = [...timers.values()]
      timers.clear()
      pending.forEach(callback => callback())
      await settle()
    },
    close: () => cleanups.forEach(cleanup => cleanup?.()),
  }
}

for (const product of ['map', 'nav', 'inspection']) {
  test(`opening the App restores the current ${product} workspace once`, async () => {
    const h = mount(new Map(), async () => snapshot(product))
    try {
      await settle()
      await h.poll()
      assert.deepEqual(h.changed, [[product, true]])
      assert.deepEqual(h.calls, ['status', 'status'])
    } finally { h.close() }
  })
}

test('an old successful receipt is reconciled with the current robot Product', async () => {
  const storage = new Map([[pendingKey, 'pending']])
  const h = mount(storage, async () => snapshot('map'), async () => operation('succeeded'))
  try {
    await settle()
    assert.deepEqual(h.changed, [['map', true]])
    assert.deepEqual(h.calls, ['operation:pending', 'status'])
    assert.equal(storage.has(pendingKey), false)
  } finally { h.close() }
})

test('a missing receipt reads current state without guessing or resubmitting', async () => {
  const storage = new Map([[pendingKey, 'pending']])
  const h = mount(storage, async () => snapshot('nav'), async () => { throw new ProductControlError('missing', 404) })
  try {
    await settle()
    assert.deepEqual(h.changed, [['nav', true]])
    assert.deepEqual(h.calls, ['operation:pending', 'status'])
    assert.equal(storage.has(pendingKey), false)
    assert.equal(h.read().message, '上次请求结果不可用，已读取机器人当前状态')
    h.read().dismiss()
    assert.equal(h.read().message, '')
  } finally { h.close() }
})

test('closing and reopening resumes the same accepted operation query', async () => {
  const storage = new Map([[pendingKey, 'pending']])
  const first = mount(storage, async () => snapshot('nav'), async () => operation('running'))
  await settle()
  first.close()
  assert.deepEqual(first.changed, [])
  assert.equal(storage.get(pendingKey), 'pending')
  const reopened = mount(storage, async () => snapshot('nav'), async () => operation('succeeded'))
  try {
    await settle()
    assert.deepEqual(reopened.calls, ['operation:pending', 'status'])
    assert.deepEqual(reopened.changed, [['nav', true]])
  } finally { reopened.close() }
})

test('reconnection observes a changed session but leaves repeated polls alone', async () => {
  let current = snapshot('map', 'first')
  const h = mount(new Map(), async () => current)
  try {
    await settle()
    current = snapshot('nav', 'second')
    await h.poll()
    await h.poll()
    assert.deepEqual(h.changed, [['map', true], ['nav', false]])
  } finally { h.close() }
})

test('stopped runtime keeps lifecycle control available without replaying motion', async () => {
  const stopped = snapshot('map')
  stopped.current = { product: null, status: 'stopped' }
  stopped.operation = { ...operation('succeeded'), request_id: 'stop', request: {
    request_id: 'stop', action: 'stop', expected_product_session_id: 'previous',
  } }
  const h = mount(new Map(), async () => stopped)
  try {
    await settle()
    assert.equal(h.read().current?.status, 'stopped')
    assert.equal(h.read().available, true)
    assert.equal(h.read().allowed, true)
    assert.deepEqual(h.changed, [])
    assert.deepEqual(h.calls, ['status'])
  } finally { h.close() }
})

test('initial restoration preserves an explicitly selected workspace', () => {
  const source = readFileSync(new URL('../src/App.tsx', import.meta.url), 'utf8')
  const ast = ts.createSourceFile('App.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
  let callback: ts.Expression | undefined
  function find(node: ts.Node) {
    if (ts.isVariableDeclaration(node) && node.name.getText(ast) === 'handleProductChanged') {
      assert.ok(node.initializer && ts.isCallExpression(node.initializer))
      callback = node.initializer.arguments[0]
    }
    ts.forEachChild(node, find)
  }
  find(ast)
  assert.ok(callback)
  const code = ts.transpileModule(`(${callback.getText(ast)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022 },
  }).outputText
  let tab = 'scene'
  let teleopClosed = 0
  const changed = runInNewContext(code, {
    setTeleopOpen: () => { teleopClosed++ },
    setActiveTab: (update: (current: string) => string) => { tab = update(tab) },
  }) as (product: string, restoring: boolean) => void
  for (const chosen of ['map', 'mapping']) {
    tab = chosen
    changed('nav', true)
    assert.equal(tab, chosen)
  }
  tab = 'scene'
  changed('map', true)
  assert.equal(tab, 'mapping')
  assert.equal(teleopClosed, 0)
  changed('nav', false)
  assert.equal(tab, 'scene')
  assert.equal(teleopClosed, 0)
  changed('inspection', true)
  assert.equal(tab, 'scene')
})
