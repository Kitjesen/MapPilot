import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as jsx from 'react/jsx-runtime'
import ts from 'typescript'
import * as api from '../src/services/api.ts'
import * as presentation from '../src/services/mapSavePresentation.ts'

test('save data polls independently of details and only actions retry or cancel', async () => {
  const originalFetch = globalThis.fetch
  const requests: string[] = []
  const continued: api.SaveMapResult[] = []
  const completed: api.SaveMapResult[] = []
  const timers: (() => void)[] = []
  const operations = [
    { operation_id: 'failed-1', map_id: 'warehouse', state: 'FAILED', phase: 'BUILD_ARTIFACTS', reason_code: 'artifact_failed' },
    { operation_id: 'running-1', map_id: 'hall', state: 'RUNNING', phase: 'PROCESS_SOURCE' },
    { operation_id: 'captured-1', map_id: 'store', state: 'CAPTURED', phase: 'CAPTURE', source_saved: true },
  ]
  globalThis.fetch = async (input, init) => {
    const url = String(input)
    requests.push(`${init?.method ?? 'GET'} ${url}`)
    return new Response(JSON.stringify(url.endsWith('/retry') ? {
      ok: true, accepted: true, operation_id: 'retry-2', operation: { state: 'QUEUED' }, ts: 30,
    } : url.endsWith('/process') ? {
      ok: true, accepted: true, operation_id: 'captured-1', operation: { state: 'QUEUED', source_saved: true }, ts: 30,
    } : url.endsWith('/cancel') ? {
      ok: true, accepted: true, operation: { state: 'RUNNING', cancel_requested: true }, ts: 30,
    } : { operations, ts: 30 }), { status: 200 })
  }
  try {
    const slots: unknown[] = []
    const effects: (() => (() => void) | void)[] = []
    let cursor = 0
    const hooks = {
      useState: (initial: unknown) => {
        const index = cursor++
        if (!(index in slots)) slots[index] = initial
        return [slots[index], (value: unknown) => { slots[index] = value }]
      },
      useEffect: (effect: () => (() => void) | void) => { effects.push(effect) },
      useCallback: (callback: unknown) => callback,
      useRef: (initial: unknown) => {
        const index = cursor++
        if (!(index in slots)) slots[index] = { current: initial }
        return slots[index]
      },
    }
    const module = { exports: {} as { MapSaveHistory: (props: object) => unknown; useMapSaves: (props: object) => { history: api.MapSaveOperationList } } }
    for (const path of ['../src/hooks/useMapSaves.ts', '../src/components/MapSaveHistory.tsx']) {
    const source = readFileSync(new URL(path, import.meta.url), 'utf8')
    const compiled = ts.transpileModule(source, {
      compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022, jsx: ts.JsxEmit.ReactJSX, esModuleInterop: true },
    }).outputText
    runInNewContext(compiled, {
      module, exports: module.exports, AbortController, Date,
      setTimeout: (callback: () => void) => { timers.push(callback); return timers.length }, clearTimeout: () => {},
      require: (name: string) => {
        if (name === 'react') return hooks
        if (name === 'react/jsx-runtime') return jsx
        if (name === 'lucide-react') return { RefreshCw: () => null }
        if (name.endsWith('api.ts')) return api
        if (name.endsWith('mapSavePresentation.ts')) return presentation
        if (name.endsWith('.css')) return { __esModule: true, default: {} }
        throw new Error(`Unexpected import ${name}`)
      },
    })
    }
    type Element = { type: string; props: {
      children?: unknown; onClick?: () => void; disabled?: boolean; href?: string
      onChange?: (event: { target: { value: string } }) => void
    } }
    const elements = (node: unknown): Element[] => {
      if (Array.isArray(node)) return node.flatMap(elements)
      if (typeof node !== 'object' || node === null || !('props' in node)) return []
      const element = node as Element
      return [element, ...elements(element.props.children)]
    }
    let readOnly = false
    const render = () => {
      cursor = 0
      const saves = module.exports.useMapSaves({ refreshKey: 'session-1',
        onContinue: (result: api.SaveMapResult) => continued.push(result),
        onComplete: (result: api.SaveMapResult) => completed.push(result) })
      return module.exports.MapSaveHistory({ saves, operations: saves.history.operations, readOnly })
    }
    render()
    const cleanup = effects[0]()
    await new Promise(setImmediate)
    assert.deepEqual(requests, ['GET /api/v1/maps/operations'])
    let tree = elements(render())
    assert.ok(tree.some(element => element.type === 'p' && [element.props.children].flat().includes('保存点云')))
    assert.equal(tree.some(element => element.type === 'p' && element.props.children === '等待地图快照'), false,
      'stored CAPTURED source still appeared to be waiting for a snapshot')
    effects.at(-1)!()
    assert.equal(completed.length, 0)
    assert.equal(tree.find(element => element.type === 'a')?.props.href, '/api/v1/maps/operations/captured-1/source/pcd')
    assert.equal(tree.some(element => element.props.children === '按状态筛选'), false)
    assert.equal(tree.some(element => element.type === 'button' && element.props.children === '全部'), false)
    const retry = tree.find(element => element.type === 'button' && element.props.children === '重试保存')!
    assert.ok(retry)
    retry.props.onClick!()
    await new Promise(setImmediate)
    assert.equal(continued[0].operation_id, 'retry-2')
    assert.equal(continued[0].name, 'warehouse')
    assert.deepEqual(requests.slice(1), ['POST /api/v1/maps/operations/failed-1/retry', 'GET /api/v1/maps/operations'])
    tree = elements(render())
    const cancel = tree.find(element => element.type === 'button' && element.props.children === '取消保存')!
    cancel.props.onClick!()
    await new Promise(setImmediate)
    assert.deepEqual(requests.slice(-2), ['POST /api/v1/maps/operations/running-1/cancel', 'GET /api/v1/maps/operations'])
    tree = elements(render())
    tree.find(element => element.type === 'button' && element.props.children === '开始处理')!.props.onClick!()
    await new Promise(setImmediate)
    assert.deepEqual(requests.slice(-2), ['POST /api/v1/maps/operations/captured-1/process', 'GET /api/v1/maps/operations'])
    assert.equal(continued.at(-1)?.operation_id, 'captured-1')
    tree = elements(render())
    const stopPolling = effects.at(-2)!()
    assert.equal(timers.length, 1)
    operations[1].state = 'SUCCEEDED'
    timers[0]()
    await new Promise(setImmediate)
    render()
    effects.at(-1)!()
    assert.deepEqual(completed.map(result => result.operation_id), ['running-1'])
    stopPolling?.()
    const timerCount = timers.length
    effects.at(-2)!()
    assert.equal(timers.length, timerCount, 'CAPTURED records started background polling')
    readOnly = true
    tree = elements(render())
    assert.equal(tree.find(element => element.type === 'button' && element.props.children === '重试保存')!.props.disabled, true)
    assert.equal(tree.find(element => element.type === 'button' && element.props.children === '开始处理')!.props.disabled, true)
    cleanup?.()
  } finally {
    globalThis.fetch = originalFetch
  }
})
