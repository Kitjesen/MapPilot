import assert from 'node:assert/strict'
import { getEventListeners } from 'node:events'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

import {
  waitForMapSaveOperation,
  saveMap,
  fetchMapSaveOperations,
  retryMapSaveOperation,
  cancelMapSaveOperation,
  processMapSaveOperation,
  reoptimizeMapSaveOperation,
  mapSaveSourceUrl,
  MapSaveError,
  type MapSaveOperationStatus,
  type SaveMapResult,
} from '../src/services/api.ts'
import { formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, archivedMapStatus, failedMapSaveStatus, mapSavePhaseLabel, mapSaveStateLabel, mapSaveReason, mapSaveElapsedMs, mapSaveProgressValue, formatMapSaveElapsed, mappingCapacityNotice, mappingFailureNotice, type MapSaveStatus } from '../src/services/mapSavePresentation.ts'

test('online failure notice distinguishes retained geometry from an empty preview', () => {
  const notice = (product: string, state: string, points: number) =>
    mappingFailureNotice(product, { global_mapping: { state, points } })
  const failed = 'online_mapping_failed: synthetic backend error'
  assert.match(notice('map', failed, 120)!, /在线建图处理失败；保留上次生成的地图预览/)
  assert.match(notice('map', failed, 0)!, /在线建图处理失败；还没有可用的地图预览/)
  assert.doesNotMatch(notice('map', failed, 120)!, /未收敛|synthetic backend error/)
  assert.equal(notice('map', 'accumulating', 120), null)
  assert.equal(notice('map', 'loop_corrected', 120), null)
  assert.equal(notice('nav', failed, 120), null)
  assert.equal(mappingFailureNotice('map', null), null)
})

test('capacity notice follows the native mapping diagnostic, independently of the selected cloud layer', () => {
  const diagnostics = { global_mapping: { state: 'mapping_keyframe_capacity_reached' } }
  const notice = mappingCapacityNotice('map', diagnostics)
  assert.ok(notice)
  assert.match(notice, /定位仍会处理新数据/)
  assert.match(notice, /已停止记录新区域/)
  assert.match(notice, /保存仅包含满额前的记录/)
  assert.equal(mappingCapacityNotice('nav', diagnostics), null)
  assert.equal(mappingCapacityNotice(null, diagnostics), null)
  assert.equal(mappingCapacityNotice('map', { global_mapping: { state: 'ready' } }), null)
  assert.equal(mappingCapacityNotice('map', {}), null)
  assert.equal(mappingCapacityNotice('map', null), null)
})

test('both save dialogs explain the capacity boundary and keep normal replacement guidance', () => {
  const notice = mappingCapacityNotice('map', {
    global_mapping: { state: 'mapping_keyframe_capacity_reached' },
  })
  for (const component of ['MapView', 'SceneView']) {
    const source = readFileSync(new URL(`../src/components/${component}.tsx`, import.meta.url), 'utf8')
    const ast = ts.createSourceFile(`${component}.tsx`, source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
    let message: ts.Expression | undefined
    const visit = (node: ts.Node) => {
      if ((ts.isJsxSelfClosingElement(node) || ts.isJsxOpeningElement(node)) && node.tagName.getText(ast) === 'PromptModal') {
        const attributes = node.attributes.properties
        const title = attributes.find(attribute => ts.isJsxAttribute(attribute)
          && attribute.name.getText(ast) === 'title')
        if (title && ts.isJsxAttribute(title) && title.initializer
          && ts.isStringLiteral(title.initializer) && title.initializer.text === '保存地图') {
          const attribute = attributes.find(attribute => ts.isJsxAttribute(attribute)
            && attribute.name.getText(ast) === 'message')
          if (attribute && ts.isJsxAttribute(attribute) && attribute.initializer
            && ts.isJsxExpression(attribute.initializer)) message = attribute.initializer.expression
        }
      }
      ts.forEachChild(node, visit)
    }
    visit(ast)
    assert.ok(message, `${component} must use the current capacity notice in its save dialog`)
    const expression = ts.transpileModule(`(${message.getText(ast)})`, {
      compilerOptions: { target: ts.ScriptTarget.ES2022 },
    }).outputText
    const withCapacity = runInNewContext(expression, { capacityNotice: notice }) as string
    const normal = runInNewContext(expression, { capacityNotice: null }) as string
    assert.ok(withCapacity.includes(notice!), component)
    assert.match(normal, /同名会替换原地图/)
    assert.doesNotMatch(normal, /容量已满/)
    assert.match(source, /(?:capacityNotice|mappingNotice) && <div className=\{styles\.(sceneAlert|saveNotice)\} role="status">/)
  }
  const scene = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
  const styles = readFileSync(new URL('../src/components/SceneView.module.css', import.meta.url), 'utf8')
  assert.match(scene, /<span className=\{styles\.capacityNotice\}>\{mappingNotice\}<\/span>/)
  assert.match(styles, /\.capacityNotice\s*\{[^}]*white-space:\s*normal;[^}]*overflow-wrap:\s*anywhere;/)
})

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
    let mapLoads = 0
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
      mapSaveBlockedReason: () => '', saveBlockedReason: '', session: {}, autoProcess: true,
      setSaveStatus: (value: MapSaveStatus) => saved.push(value),
      formatMapSaveProgress, pendingMapSaveStatus, savedMapStatus, archivedMapStatus, failedMapSaveStatus,
      mapSaveElapsedMs, mapSaveProgressValue,
      saveStartedAt: { current: 0 }, setSaveTiming: noOp, setSaveClock: noOp, setSaveProgress: noOp,
      savePreviewSession: { current: 'mapping-session' }, hasAutoSelected: { current: false },
      setSelectedMap: noOp, setLibraryOpen: noOp, setPreviewRefresh: noOp, setGoalPickingMap: noOp,
      loadMaps: () => { mapLoads += 1 },
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
    context.api.waitForMapSaveOperation = async () => {
      throw new MapSaveError('Map save failed.', {
        ...admission(), operation: { state: 'FAILED', phase: 'PROCESS_SOURCE', reason_code: 'dynamic_filter_failed' },
      })
    }
    await confirm('warehouse', admission())
    const failed = saved.at(-1)!
    assert.equal(failed.state, 'failed', component)
    if (failed.state !== 'failed') throw new Error('Expected failed UI state')
    assert.equal(failed.operation?.operation_id, 'save-17', component)
    assert.match(failed.summary!, /清理动态点.*dynamic_filter_failed/)
    assert.equal(posts, 1, component)
    const archive = { ...admission(), operation: { state: 'CAPTURED', source_saved: true } }
    context.api.waitForMapSaveOperation = async () => ({ state: 'source_saved', result: archive })
    const loadedBeforeCapture = mapLoads
    await confirm('warehouse', admission())
    assert.equal(saved.at(-1)?.state, 'archived', component)
    assert.match(saved.at(-1)!.detail, /PCD 已保存/)
    assert.equal(mapLoads, loadedBeforeCapture, `${component} refreshed formal maps before processing completed`)
  }
})

test('new saves default to background processing and manual mode is sent explicitly', async () => {
  const bodies: unknown[] = []
  globalThis.fetch = async (_input, init) => {
    bodies.push(JSON.parse(String(init?.body)))
    return new Response(JSON.stringify(admission()), { status: 202 })
  }
  await saveMap('warehouse')
  await saveMap('warehouse', false)
  assert.deepEqual(bodies, [{ name: 'warehouse', auto_process: true }, { name: 'warehouse', auto_process: false }])
})

test('source waiting finishes when PCD is stored while default ready waiting continues through CAPTURED', async () => {
  let reads = 0
  globalThis.fetch = async () => {
    reads += 1
    return new Response(JSON.stringify({ ...admission(), operation: {
      state: reads === 1 ? 'WAITING_SNAPSHOT' : 'CAPTURED', source_saved: reads > 1,
    } }), { status: 200 })
  }
  const source = await waitForMapSaveOperation(admission(), { until: 'source', pollIntervalMs: 0 })
  assert.equal(source.state, 'source_saved')
  assert.equal(source.result.operation_id, 'save-17')
  assert.equal(reads, 2)
  reads = 0
  globalThis.fetch = async () => new Response(JSON.stringify({ ...admission(), operation: {
    state: ++reads === 1 ? 'CAPTURED' : 'SUCCEEDED', source_saved: true,
  } }), { status: 200 })
  assert.equal((await waitForMapSaveOperation(admission(), { pollIntervalMs: 0 })).state, 'saved')
  assert.equal(reads, 2)
})

test('source_saved and failed processing remain two separate facts', async () => {
  const result = { ...admission(), operation: {
    state: 'FAILED', phase: 'PROCESS_SOURCE', source_saved: true, reason_code: 'dynamic_filter_failed',
  } }
  const source = await waitForMapSaveOperation(result, { until: 'source' })
  assert.equal(source.state, 'source_saved')
  if (source.state !== 'source_saved') throw new Error('Expected stored source')
  const status = archivedMapStatus(source.result)
  assert.equal(status.state, 'failed')
  assert.match(status.detail, /PCD 已保存，处理失败/)
  await assert.rejects(waitForMapSaveOperation(result), MapSaveError)
})

test('manual processing uses the same operation and capture download has a task identity', async () => {
  const requests: string[] = []
  globalThis.fetch = async (input, init) => {
    requests.push(`${init?.method ?? 'GET'} ${String(input)}`)
    return new Response(JSON.stringify({ ...admission(), operation: { state: 'QUEUED', source_saved: true } }), { status: 202 })
  }
  const result = await processMapSaveOperation('save-17')
  assert.equal(result.operation_id, 'save-17')
  assert.deepEqual(requests, ['POST /api/v1/maps/operations/save-17/process'])
  assert.equal(mapSaveSourceUrl('save-17'), '/api/v1/maps/operations/save-17/source/pcd')
})

test('reoptimization submits an explicit request identity and follows the new task', async () => {
  const requests: Array<{ url: string; method: string; body?: unknown }> = []
  globalThis.fetch = async (input, init) => {
    requests.push({ url: String(input), method: init?.method ?? 'GET',
      ...(init?.body ? { body: JSON.parse(String(init.body)) } : {}) })
    return Response.json({ ...admission(), operation_id: 'optimize-18', operation: {
      operation_id: 'optimize-18', source_operation_id: 'save/17', source_saved: true,
      state: init?.method === 'POST' ? 'QUEUED' : 'SUCCEEDED',
    } }, { status: init?.method === 'POST' ? 202 : 200 })
  }
  const started = await reoptimizeMapSaveOperation('save/17', 'optimize-request-1')
  assert.equal(started.operation_id, 'optimize-18')
  assert.match(formatMapSaveProgress(started), /排队等待重新优化/)
  const completed = await waitForMapSaveOperation(started, { pollIntervalMs: 0 })
  assert.equal(completed.state, 'saved')
  assert.deepEqual(requests, [
    { url: '/api/v1/maps/operations/save%2F17/reoptimize', method: 'POST', body: { request_id: 'optimize-request-1' } },
    { url: '/api/v1/maps/operations/optimize-18', method: 'GET' },
  ])
})

test('reoptimization keeps the native failure reason', async () => {
  globalThis.fetch = async () => Response.json({ ok: false, success: false,
    reason_code: 'patch_bundle_incomplete', message: 'Original scans are missing.' }, { status: 409 })
  await assert.rejects(reoptimizeMapSaveOperation('save-17'), error => {
    assert.ok(error instanceof MapSaveError)
    assert.equal(error.result.reason_code, 'patch_bundle_incomplete')
    assert.equal(error.message, 'Original scans are missing.')
    assert.match(mapSaveReason(error.result), /缺少完整的分帧扫描或位姿/)
    return true
  })
})

interface HistoryElement { type: string; props: Record<string, unknown> & { children?: unknown } }

function historyHarness(operations: MapSaveOperationStatus[], reoptimize: typeof reoptimizeMapSaveOperation) {
  const state: unknown[] = [{ operations, ts: 1 }]
  const refs: Array<{ current: unknown }> = []
  let stateIndex = 0
  let refIndex = 0
  let effects: Array<() => void> = []
  const continued: SaveMapResult[] = []
  const completed: SaveMapResult[] = []
  let history = operations
  let snapshot: { error: string; feedback: { error?: boolean } | null }
  const jsx = (type: string, props: HistoryElement['props']) => ({ type, props })
  const exports: Record<string, unknown> = {}
  for (const path of ['../src/hooks/useMapSaves.ts', '../src/components/MapSaveHistory.tsx']) {
  const code = ts.transpileModule(readFileSync(new URL(path, import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
  runInNewContext(code, { exports, AbortController, setTimeout, clearTimeout, require: (name: string) => {
    if (name === 'react/jsx-runtime') return { jsx, jsxs: jsx }
    if (name === 'react') return {
      useState: (initial: unknown) => {
        const index = stateIndex++
        if (!(index in state)) state[index] = initial
        return [state[index], (value: unknown) => { state[index] = value }]
      },
      useRef: (initial: unknown) => refs[refIndex++] ?? (refs[refIndex - 1] = { current: initial }),
      useEffect: (effect: () => void) => { effects.push(effect) },
      useCallback: (callback: unknown) => callback,
    }
    if (name.endsWith('/api.ts')) return { MapSaveError, makeRequestId: () => 'stable-optimize-request',
      reoptimizeMapSaveOperation: reoptimize,
      fetchMapSaveOperations: async () => ({ operations: history, ts: 2 }), mapSaveSourceUrl }
    if (name.endsWith('/mapSavePresentation.ts')) return { mapSaveStateLabel, mapSavePhaseLabel, mapSaveReason }
    return { default: {} }
  } })
  }
  return {
    continued, completed,
    get snapshot() { return snapshot },
    setHistory: (value: MapSaveOperationStatus[]) => { history = value; state[0] = { operations: value, ts: 3 } },
    render: (readOnly = false) => {
      stateIndex = 0; refIndex = 0; effects = []
      const component = exports.MapSaveHistory as (props: Record<string, unknown>) => HistoryElement
      const renderSaveHook = exports.useMapSaves as (props: Record<string, unknown>) => { history: { operations: MapSaveOperationStatus[] }; error: string; feedback: { error?: boolean } | null }
      const saves = renderSaveHook({ refreshKey: '',
        onContinue: (value: SaveMapResult) => continued.push(value),
        onComplete: (value: SaveMapResult) => completed.push(value) })
      snapshot = saves
      const root = component({ operations: saves.history.operations, saves, readOnly })
      const elements: HistoryElement[] = []
      const visit = (value: unknown) => {
        if (Array.isArray(value)) { value.forEach(visit); return }
        if (!value || typeof value !== 'object' || !('props' in value)) return
        const element = value as HistoryElement
        elements.push(element); visit(element.props.children)
      }
      visit(root)
      effects[2]()
      return elements
    },
  }
}

test('history starts a new optimization, reuses an uncertain request and refreshes on completion', async () => {
  const source = { operation_id: 'save-17', map_id: 'warehouse', state: 'SUCCEEDED', source_saved: true }
  const optimized = { operation_id: 'optimize-18', map_id: 'warehouse', source_operation_id: 'save-17', state: 'QUEUED', source_saved: true }
  const requests: string[] = []
  let calls = 0
  const h = historyHarness([source], async (_id, requestId) => {
    requests.push(requestId!)
    if (++calls === 1) throw new TypeError('Network disconnected')
    h.setHistory([optimized, source])
    return { ...admission(), operation_id: 'optimize-18', operation: optimized }
  })
  const button = (readOnly = false) => h.render(readOnly).find(element => element.type === 'button' && element.props.children === '重新优化')!
  assert.equal(button().props.disabled, false)
  assert.ok(h.render().some(element => element.type === 'p'
    && element.props.children === '按这次存档重新生成，完成后替换同名地图。'))
  assert.equal(button(true).props.disabled, true)
  ;(button().props.onClick as () => void)()
  await new Promise(resolve => setImmediate(resolve))
  assert.equal(h.continued.length, 0)
  h.render()
  assert.equal(h.snapshot.error, '', 'one rejected action must not mark every map status unavailable')
  assert.equal(h.snapshot.feedback?.error, true)
  ;(button().props.onClick as () => void)()
  await new Promise(resolve => setImmediate(resolve))
  assert.deepEqual(requests, ['stable-optimize-request', 'stable-optimize-request'])
  assert.equal(h.continued[0].operation_id, 'optimize-18')
  assert.equal(button().props.disabled, true)
  h.setHistory([{ ...optimized, state: 'SUCCEEDED' }, source])
  h.render()
  assert.equal(h.completed.length, 1)
  assert.equal(h.completed[0].operation_id, 'optimize-18')
})

test('history only offers reoptimization for successful tasks with saved source', () => {
  const h = historyHarness([
    { operation_id: 'done-without-source', state: 'SUCCEEDED', source_saved: false },
    { operation_id: 'captured', state: 'CAPTURED', source_saved: true },
    { operation_id: 'failed', state: 'FAILED', source_saved: true },
  ], reoptimizeMapSaveOperation)
  const elements = h.render()
  assert.equal(elements.filter(element => element.type === 'button' && element.props.children === '重新优化').length, 0)
  assert.equal(elements.filter(element => element.type === 'p'
    && element.props.children === '按这次存档重新生成，完成后替换同名地图。').length, 0)
})

test('save history lists durable states and retries through the operation endpoint to a real terminal result', async () => {
  const requests: { url: string; method: string }[] = []
  const operations = [
    { operation_id: 'failed-1', map_id: 'warehouse', state: 'FAILED', phase: 'BUILD_ARTIFACTS', reason_code: 'artifact_failed', created_at_ns: 10e9 },
    { operation_id: 'running-1', map_id: 'hall', state: 'RUNNING', phase: 'PROCESS_SOURCE' },
    { operation_id: 'cancelled-1', map_id: 'yard', state: 'CANCELLED', phase: 'CAPTURE' },
    { operation_id: 'done-1', map_id: 'room', state: 'SUCCEEDED', phase: 'DONE' },
  ]
  globalThis.fetch = async (input, init) => {
    const url = String(input)
    requests.push({ url, method: init?.method ?? 'GET' })
    let data: object
    if (url === '/api/v1/maps/operations') data = { operations, ts: 20 }
    else if (url.endsWith('/failed-1/retry')) data = {
      ...admission(), operation_id: 'retry-2', operation: { operation_id: 'retry-2', map_id: 'warehouse', state: 'QUEUED' },
    }
    else if (url.endsWith('/running-1/cancel')) data = {
      ...admission(), operation_id: 'running-1', operation: { state: 'RUNNING', cancel_requested: true },
    }
    else if (url.endsWith('/retry-2')) data = {
      ...admission(), operation_id: 'retry-2', operation: { state: 'SUCCEEDED', phase: 'DONE' },
    }
    else throw new Error(`Unexpected request ${url}`)
    return new Response(JSON.stringify(data), { status: 200 })
  }
  const history = await fetchMapSaveOperations()
  assert.deepEqual(history.operations.map(operation => mapSaveStateLabel(operation.state)), ['失败', '进行中', '已取消', '成功'])
  assert.equal(history.ts, 20)
  assert.match(mapSaveReason(history.operations[0]), /导航地图生成失败.*artifact_failed/)
  assert.equal(mapSavePhaseLabel(history.operations[0].phase), '生成导航地图')
  assert.deepEqual(requests, [{ url: '/api/v1/maps/operations', method: 'GET' }])
  const retried = await retryMapSaveOperation('failed-1')
  const terminal = await waitForMapSaveOperation(retried, { pollIntervalMs: 0 })
  assert.equal(terminal.state, 'saved')
  assert.equal(terminal.result.operation_id, 'retry-2')
  const cancelled = await cancelMapSaveOperation('running-1')
  assert.equal(cancelled.operation?.cancel_requested, true)
  assert.equal(cancelled.operation?.state, 'RUNNING')
  assert.deepEqual(requests.slice(1), [
    { url: '/api/v1/maps/operations/failed-1/retry', method: 'POST' },
    { url: '/api/v1/maps/operations/retry-2', method: 'GET' },
    { url: '/api/v1/maps/operations/running-1/cancel', method: 'POST' },
  ])
})

test('failed operation errors retain the queried task and actual failure stage', async () => {
  const result: SaveMapResult = {
    ...admission(), operation: { operation_id: 'save-17', state: 'FAILED', phase: 'BUILD_ARTIFACTS', reason_code: 'artifact_failed' },
  }
  globalThis.fetch = async () => new Response(JSON.stringify(result), { status: 200 })
  await assert.rejects(waitForMapSaveOperation(admission()), error => {
    assert.ok(error instanceof MapSaveError)
    const status = failedMapSaveStatus('warehouse', error)
    assert.equal(status.state, 'failed')
    if (status.state !== 'failed') return false
    assert.equal(status.operation?.operation_id, 'save-17')
    assert.equal(status.operation?.operation?.state, 'FAILED')
    assert.match(status.summary!, /生成导航地图.*artifact_failed/)
    return true
  })
  assert.equal(failedMapSaveStatus('warehouse', new Error('connection unavailable')).state, 'failed')
})

test('a rejected retry reports its server failure instead of submitting a new save', async () => {
  const urls: string[] = []
  globalThis.fetch = async input => {
    urls.push(String(input))
    return new Response(JSON.stringify({ ok: false, success: false, message: 'Map save requires mapping Product.' }), { status: 409 })
  }
  await assert.rejects(retryMapSaveOperation('failed-1'), /requires mapping Product/)
  assert.deepEqual(urls, ['/api/v1/maps/operations/failed-1/retry'])
})

test('expired snapshot retry keeps the operation reason for the history recovery message', async () => {
  globalThis.fetch = async () => new Response(JSON.stringify({
    ok: false, success: false, operation_id: 'failed-1', reason_code: 'snapshot_session_expired',
    operation: { operation_id: 'failed-1', state: 'FAILED', phase: 'CAPTURE', reason_code: 'snapshot_session_expired' },
    message: 'Map save failed.',
  }), { status: 409 })
  await assert.rejects(retryMapSaveOperation('failed-1'), error => {
    assert.ok(error instanceof MapSaveError)
    assert.equal(error.result.operation_id, 'failed-1')
    assert.match(mapSaveReason(error.result.operation!), /原建图会话已结束，请重新保存地图。.*snapshot_session_expired/)
    return true
  })
})
