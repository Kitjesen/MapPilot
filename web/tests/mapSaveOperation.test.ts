import assert from 'node:assert/strict'
import { getEventListeners } from 'node:events'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

import {
  waitForMapSaveOperation,
  saveMap,
  type SaveMapResult,
} from '../src/services/api.ts'
import { formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, mapSaveElapsedMs, mapSaveProgressValue, formatMapSaveElapsed, type MapSaveStatus } from '../src/services/mapSavePresentation.ts'

test('save timing uses robot timestamps and freezes at completion, independent of browser clock', () => {
  const result: SaveMapResult = { ...admission(), ts: 222, operation: { created_at_ns: 100e9, progress: 0.3 } }
  assert.equal(mapSaveElapsedMs(result, 900000), 122000)
  result.operation!.completed_at_ns = 220e9
  assert.equal(mapSaveElapsedMs(result, 900000), 120000)
  assert.equal(mapSaveElapsedMs(admission(), 5000), 5000)
  assert.equal(formatMapSaveElapsed(122000), '2 分 2 秒')
  assert.equal(mapSaveProgressValue(admission()), undefined)
  assert.equal(mapSaveProgressValue(result), 0.3)
})

const originalFetch = globalThis.fetch
const testGlobals = globalThis as typeof globalThis & {
  window?: { location: { origin: string } }
}
const originalWindow = testGlobals.window

testGlobals.window = { location: { origin: 'http://robot.test' } }

test.afterEach(() => {
  globalThis.fetch = originalFetch
})

test.after(() => {
  if (originalWindow === undefined) delete testGlobals.window
  else testGlobals.window = originalWindow
})

function admission(): SaveMapResult {
  return {
    schema_version: 1,
    ok: true,
    success: null,
    accepted: true,
    operation_id: 'save-17',
    name: 'warehouse',
    ts: 1,
  }
}

test('accepted SaveMap polls until the durable operation succeeds', async () => {
  const states = ['RUNNING', 'SUCCEEDED']
  const urls: string[] = []
  globalThis.fetch = async (input: string | URL | Request) => {
    urls.push(String(input))
    const state = states.shift()
    return new Response(JSON.stringify({
      schema_version: 1,
      ok: true,
      success: true,
      operation_id: 'save-17',
      operation: { operation_id: 'save-17', state },
      ts: 2,
    }), { status: 200 })
  }

  const result = await waitForMapSaveOperation(admission(), {
    timeoutMs: 1_000,
    pollIntervalMs: 0,
  })

  assert.equal(result.state, 'saved')
  assert.equal(result.result.success, true)
  assert.equal(result.result.operation?.state, 'SUCCEEDED')
  assert.equal(result.result.name, 'warehouse')
  assert.deepEqual(urls, [
    '/api/v1/maps/operations/save-17',
    '/api/v1/maps/operations/save-17',
  ])
})

test('failed SaveMap operation never becomes a saved UI result', async () => {
  globalThis.fetch = async () => new Response(JSON.stringify({
    schema_version: 1,
    ok: true,
    success: true,
    operation_id: 'save-17',
    operation: {
      operation_id: 'save-17',
      state: 'FAILED',
      reason_code: 'artifact_failed',
      message: 'Map artifact generation failed.',
    },
    ts: 2,
  }), { status: 200 })

  await assert.rejects(
    waitForMapSaveOperation(admission(), { timeoutMs: 1_000, pollIntervalMs: 0 }),
    /artifact generation failed/,
  )
})

test('accepted SaveMap without operation identity fails closed', async () => {
  const value = admission()
  delete value.operation_id

  await assert.rejects(
    waitForMapSaveOperation(value),
    /without an operation_id/,
  )
})

test('a slow save remains pending and continuing polls the same operation without another POST', async () => {
  const requests: Array<{ url: string; method: string }> = []
  let polls = 0
  const progress: string[] = []
  globalThis.fetch = async (input, init) => {
    requests.push({ url: String(input), method: init?.method ?? 'GET' })
    if (init?.method === 'POST') return Response.json(admission())
    polls += 1
    if (polls === 1) await new Promise(resolve => setTimeout(resolve, 5))
    return Response.json({
      ...admission(), success: true,
      operation: { state: polls === 1 ? 'RUNNING' : 'SUCCEEDED', phase: 'OPTIMIZE_SOURCE', progress: 0.2 },
    })
  }

  const accepted = await saveMap('warehouse')
  const waiting = await waitForMapSaveOperation(accepted, {
    timeoutMs: 1, pollIntervalMs: 0,
    onProgress: result => progress.push(formatMapSaveProgress(result)),
  })
  assert.equal(waiting.state, 'pending')
  if (waiting.state !== 'pending') throw new Error('Expected pending save')
  assert.equal(waiting.result.success, null)
  assert.equal(waiting.result.operation_id, 'save-17')
  assert.equal(pendingMapSaveStatus(waiting.result, waiting.reason).state, 'pending')
  assert.match(progress.at(-1)!, /优化建图轨迹.*20%/)

  const finished = await waitForMapSaveOperation(waiting.result, { pollIntervalMs: 0 })
  assert.equal(finished.state, 'saved')
  assert.deepEqual(requests, [
    { url: '/api/v1/map/save', method: 'POST' },
    { url: '/api/v1/maps/operations/save-17', method: 'GET' },
    { url: '/api/v1/maps/operations/save-17', method: 'GET' },
  ])
})

test('a status connection failure preserves the accepted operation for another query', async () => {
  globalThis.fetch = async () => { throw new TypeError('Network disconnected') }
  const waiting = await waitForMapSaveOperation(admission())
  assert.equal(waiting.state, 'pending')
  if (waiting.state !== 'pending') throw new Error('Expected pending save')
  assert.equal(waiting.reason, 'status_unavailable')
  assert.equal(waiting.result.operation_id, 'save-17')
  assert.doesNotMatch(pendingMapSaveStatus(waiting.result, waiting.reason).detail, /保存失败/)
})

test('cancelled saves remain terminal even when the status request itself succeeded', async () => {
  globalThis.fetch = async () => Response.json({
    ...admission(), success: true, operation: { state: 'CANCELLED', message: 'Save was cancelled.' },
  })
  await assert.rejects(waitForMapSaveOperation(admission()), /cancelled/)
})

test('poll delays remove abort listeners after completion and abort never reports a saved map', async () => {
  const controller = new AbortController()
  let calls = 0
  globalThis.fetch = async () => Response.json({
    ...admission(), operation: { state: ++calls < 4 ? 'RUNNING' : 'SUCCEEDED' },
  })
  await waitForMapSaveOperation(admission(), { pollIntervalMs: 1, signal: controller.signal })
  assert.equal(getEventListeners(controller.signal, 'abort').length, 0)
  controller.abort()
  await assert.rejects(waitForMapSaveOperation(admission(), { signal: controller.signal }), { name: 'AbortError' })
})

test('successful save with skipped PGO does not claim optimization or navigation success', () => {
  const status = savedMapStatus({
    ...admission(), success: true,
    operation: { state: 'SUCCEEDED', processing: {
      optimization: { success: true, performed: false, reason_code: 'sequential_chain_incomplete' },
      cleanup: { success: true },
    } },
  })
  assert.equal(status.state, 'saved')
  assert.match(status.summary!, /优化未完成（连续轨迹约束不完整）/)
  assert.doesNotMatch(status.summary!, /优化已完成/)
  assert.match(status.summary!, /动态点清理完成/)
  assert.match(status.detail, /保存成功不等于可走/)
  assert.equal(status.location, '机载地图 / warehouse')
  const unconfirmed = savedMapStatus({ ...admission(), success: true })
  assert.match(unconfirmed.summary!, /优化未完成.*动态点清理未确认/)
  const completed = savedMapStatus({ ...admission(), success: true, operation: { processing: {
    optimization: { success: true, performed: true }, cleanup: { success: true },
  } } })
  assert.match(completed.summary!, /优化已完成；动态点清理完成/)
})

test('both save callbacks continue the pending operation without resubmitting or requiring mapping mode', async () => {
  for (const component of ['MapView', 'SceneView']) {
    const source = readFileSync(new URL(`../src/components/${component}.tsx`, import.meta.url), 'utf8')
    const callbackName = component === 'MapView' ? 'confirmSave' : 'confirmSaveMap'
    const ast = ts.createSourceFile(`${component}.tsx`, source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
    let callback: ts.VariableDeclaration | undefined
    const findCallback = (node: ts.Node) => {
      if (ts.isVariableDeclaration(node) && ts.isIdentifier(node.name) && node.name.text === callbackName) callback = node
      ts.forEachChild(node, findCallback)
    }
    findCallback(ast)
    assert.ok(callback)
    const saved: MapSaveStatus[] = []
    const queried: SaveMapResult[] = []
    let posts = 0
    const noOp = () => {}
    const context = {
      api: {
        saveMap: async () => { posts += 1; return admission() },
        waitForMapSaveOperation: async (value: SaveMapResult) => {
          queried.push(value)
          return queried.length === 1
            ? { state: 'pending', reason: 'timeout', result: admission() }
            : { state: 'saved', result: { ...admission(), success: true } }
        },
      },
      setSaveOpen: noOp, setSaveModalOpen: noOp, showToast: noOp,
      openWorkspaceTool: noOp,
      mapSaveBlockedReason: () => '', saveBlockedReason: '', session: {},
      setSaveStatus: (value: MapSaveStatus) => saved.push(value),
      formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus,
      mapSaveElapsedMs, mapSaveProgressValue,
      saveStartedAt: { current: 0 }, setSaveTiming: noOp, setSaveClock: noOp, setSaveProgress: noOp,
      savePreviewSession: { current: 'mapping-session' }, hasAutoSelected: { current: false },
      setSelectedMap: noOp, setLibraryOpen: noOp, setGoalPickingMap: noOp, loadMaps: noOp,
    }
    const confirm = runInNewContext(ts.transpileModule(`const ${callback.getText(ast)}; ${callbackName};`, {
      compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
    }).outputText, context) as (name: string, operation?: SaveMapResult) => Promise<void>
    await confirm('warehouse')
    const pending = saved.at(-1)!
    assert.equal(pending.state, 'pending', component)
    if (pending.state !== 'pending') throw new Error('Expected pending UI state')
    context.mapSaveBlockedReason = () => '请先启动建图模式'
    context.saveBlockedReason = '请先启动建图模式'
    await confirm(pending.name, pending.operation)
    assert.equal(posts, 1, component)
    assert.deepEqual(queried.map(value => value.operation_id), ['save-17', 'save-17'])
    assert.equal(saved.at(-1)?.state, 'saved', component)
  }
})
