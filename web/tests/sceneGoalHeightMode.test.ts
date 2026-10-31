import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import { sceneGoalHeight, navigationSupportHeightFromRunPlan } from '../src/services/navigationPreview.ts'
import { appendRouteDraftPoint, EMPTY_ROUTE_DRAFT } from '../src/services/routeDraft.ts'

const source = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
const file = ts.createSourceFile('SceneView.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)

function findNode(predicate: (node: ts.Node) => boolean): ts.Node {
  let found: ts.Node | undefined
  const visit = (node: ts.Node) => {
    if (!found && predicate(node)) found = node
    ts.forEachChild(node, visit)
  }
  visit(file)
  assert.ok(found)
  return found
}

function callback(name: string): ts.Expression {
  const declaration = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === name) as ts.VariableDeclaration
  assert.ok(declaration.initializer && ts.isCallExpression(declaration.initializer))
  return declaration.initializer.arguments[0]
}

function evaluate(expression: ts.Expression, context: Record<string, unknown>) {
  const code = ts.transpileModule(`(${expression.getText(file)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  return runInNewContext(code, context)
}

function hasAttribute(node: ts.Node, name: string, value: string): node is ts.JsxOpeningElement | ts.JsxSelfClosingElement {
  return (ts.isJsxOpeningElement(node) || ts.isJsxSelfClosingElement(node))
    && node.attributes.properties.some(attribute => ts.isJsxAttribute(attribute)
      && attribute.name.getText(file) === name && attribute.initializer?.getText(file) === value)
}

test('one plane-default picking mode reaches the shared single and route scene', () => {
  const state = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === '[goalPickMode, setGoalPickMode]') as ts.VariableDeclaration
  assert.ok(state.initializer && ts.isCallExpression(state.initializer))
  assert.equal(state.initializer.arguments[0].getText(file), "'plane'")
  const scene = findNode(node => ts.isJsxSelfClosingElement(node) && node.tagName.getText(file) === 'Scene3D') as ts.JsxSelfClosingElement
  assert.match(scene.getText(file), /goalPickMode=\{goalPickMode\}/)
  assert.match(scene.getText(file), /navigationSelectionMode === 'route' \? handleRouteDraftPoint : selectPendingGoal/)
})

test('the existing dock offers touch-accessible height choices independently of route selection', () => {
  const dock = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === 'sceneDockControls') as ts.VariableDeclaration
  assert.ok(dock.initializer)
  assert.match(dock.initializer.getText(file), /aria-label="选点方式"/)
  assert.match(dock.initializer.getText(file), /aria-pressed=\{goalPickMode === 'plane'\}/)
  assert.match(dock.initializer.getText(file), /aria-pressed=\{goalPickMode === 'surface'\}/)
  assert.match(dock.initializer.getText(file), /setGoalPickMode\('plane'\)/)
  assert.match(dock.initializer.getText(file), /setGoalPickMode\('surface'\)/)
  assert.doesNotMatch(dock.initializer.getText(file), /setNavigationSelectionMode/)
  assert.match(dock.initializer.getText(file), /goalPickMode === 'plane' \? '同层选点' : '三维表面'/)
})

test('single and route callbacks preserve the same selected three-dimensional position', () => {
  for (const height of [0.42, 1.8, 0]) {
    let selected: { x: number; y: number; z?: number } | undefined
    const select = evaluate(callback('selectPendingGoal'), {
      invalidatePendingGoalPreview() {},
      normalizeGoalHeight: (value: number | null) => value,
      sceneGoalHeight,
      navigationSupportHeight: .435,
      mapOdom: { z: 0.42 },
      setPendingGoal: (point: typeof selected) => { selected = point },
      setGoalPreviewError() {},
      setFollowRobot() {},
      scene3DRef: { current: { focusPoint() {} } },
    }) as (x: number, y: number, z: number) => void
    select(1, 2, height)
    let route = EMPTY_ROUTE_DRAFT
    const add = evaluate(callback('handleRouteDraftPoint'), {
      routeDraftLocked: false,
      mapOdom: { z: .42 },
      sceneGoalHeight,
      navigationSupportHeight: .435,
      routeDraftIdentity: { mapId: 'hall', contentEpoch: 1 },
      appendRouteDraftPoint,
      setRouteDraft: (update: (draft: typeof route) => typeof route) => { route = update(route) },
      setSavedDraftRoute() {},
      setRouteStartIntent() {},
      setRouteTaskError() {},
      setFollowRobot() {},
      showToast: () => assert.fail('a finite selected height should not produce a warning'),
    }) as (x: number, y: number, z: number) => void
    add(1, 2, height)
    assert.deepEqual(JSON.parse(JSON.stringify(selected)), { x: 1, y: 2, z: height })
    assert.deepEqual(route.points.map(point => ({ x: point.x, y: point.y, z: point.z })), [{ x: 1, y: 2, z: height }])
  }
})

test('single and route surface clicks convert to body height once and keep manual edits explicit', () => {
  for (const support of [.435, .32, null]) {
    let selected: { x: number; y: number; z?: number } | undefined
    let warning: string | null = null
    const select = evaluate(callback('selectPendingGoal'), {
      invalidatePendingGoalPreview() {}, normalizeGoalHeight: (value: number | null) => value,
      sceneGoalHeight, mapOdom: { z: .42 }, navigationSupportHeight: support,
      setPendingGoal: (point: typeof selected) => { selected = point },
      setGoalPreviewError: (message: string | null) => { warning = message },
      setFollowRobot() {}, scene3DRef: { current: { focusPoint() {} } },
    }) as (x: number, y: number, z: number, mode: string) => void
    select(1, 2, 3.6, 'surface')
    let route = EMPTY_ROUTE_DRAFT
    const add = evaluate(callback('handleRouteDraftPoint'), {
      routeDraftLocked: false, routeDraftIdentity: { mapId: 'hall', contentEpoch: 1 },
      sceneGoalHeight, mapOdom: { z: .42 }, navigationSupportHeight: support,
      appendRouteDraftPoint,
      setRouteDraft: (update: (draft: typeof route) => typeof route) => { route = update(route) },
      setSavedDraftRoute() {}, setRouteStartIntent() {}, setRouteTaskError() {}, setFollowRobot() {},
      showToast: (message: string) => { warning = message },
    }) as (x: number, y: number, z: number, mode: string) => void
    add(1, 2, 3.6, 'surface')
    if (support === null) {
      assert.equal(selected?.z, undefined)
      assert.equal(route.points.length, 0)
      assert.match(warning!, /支撑高度/)
    } else {
      assert.equal(selected?.z, 3.6 + support)
      assert.equal(route.points[0].z, selected?.z)
      assert.equal(warning, null)
    }
  }
})

test('surface height configuration is bound to the active native Product session', async () => {
  const effect = findNode(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('api.fetchRuntimeDataflow()')) as ts.CallExpression
  for (const responseSession of ['current', 'previous']) {
    let saved: { session: string; supportHeight: number | null } | null = null
    const run = evaluate(effect.arguments[0], {
      sseState: { connected: true }, nativeSession: 'current',
      api: { fetchRuntimeDataflow: async () => ({
        runtime_boundary: { product_session_id: responseSession },
        run_plan: { host: { config: { octoplanner3d_support_height_m: .32 } } },
      }) },
      safetyEnvelopeFromRunPlan: () => null, navigationSupportHeightFromRunPlan,
      setEnvelopeConfig: (value: typeof saved) => { saved = value },
    }) as () => () => void
    run()
    await Promise.resolve()
    if (responseSession === 'current') {
      assert.equal(saved?.supportHeight, .32)
      assert.equal(saved?.session, 'current')
    } else assert.equal(saved, null)
  }
  let resolve: (value: unknown) => void = () => {}
  let writes = 0
  const run = evaluate(effect.arguments[0], {
    sseState: { connected: true }, nativeSession: 'previous',
    api: { fetchRuntimeDataflow: () => new Promise(value => { resolve = value }) },
    safetyEnvelopeFromRunPlan: () => null, navigationSupportHeightFromRunPlan,
    setEnvelopeConfig: () => { writes++ },
  }) as () => () => void
  const cleanup = run()
  cleanup()
  resolve({ runtime_boundary: { product_session_id: 'previous' }, run_plan: {} })
  await Promise.resolve()
  assert.equal(writes, 0, 'a response from the old session must be discarded after rebind')
})

test('route selection cannot replace a missing picked height with the robot height', () => {
  let warnings = 0
  const add = evaluate(callback('handleRouteDraftPoint'), {
    routeDraftLocked: false, routeDraftIdentity: { mapId: 'hall', contentEpoch: 1 },
    sceneGoalHeight, mapOdom: { z: .42 }, navigationSupportHeight: .435,
    appendRouteDraftPoint,
    setRouteDraft: () => assert.fail('a missing picked height cannot add a route point'),
    showToast: () => { warnings++ },
  }) as (x: number, y: number, z?: number) => void
  add(1, 2)
  add(1, 2, NaN)
  assert.equal(warnings, 2)
})

test('clearing target height invalidates preview without inventing zero; explicit zero remains valid', () => {
  const input = findNode(node => hasAttribute(node, 'aria-label', '"目标高度（米）"')) as ts.JsxSelfClosingElement
  const change = input.attributes.properties.find(attribute => ts.isJsxAttribute(attribute) && attribute.name.getText(file) === 'onChange') as ts.JsxAttribute
  assert.ok(change.initializer && ts.isJsxExpression(change.initializer) && change.initializer.expression)
  for (const value of ['', '  ', '0', '1.25']) {
    let selected: { x: number; y: number; z?: number } = { x: 1, y: 2, z: 0.42 }
    let error: string | null = null
    let invalidations = 0
    const update = evaluate(change.initializer.expression, {
      pendingGoal: selected,
      invalidatePendingGoalPreview: () => { invalidations++; error = null },
      setPendingGoal: (point: typeof selected) => { selected = point },
      setGoalPreviewError: (message: string) => { error = message },
    }) as (event: { target: { value: string } }) => void
    update({ target: { value } })
    assert.equal(invalidations, 1)
    assert.equal(selected.z, value.trim() ? Number(value) : undefined)
    assert.equal(error, value.trim() ? null : '请输入目标高度')
    assert.equal(selected.x, 1)
    assert.equal(selected.y, 2)
  }
})

test('a missing height does not schedule an automatic preview request', () => {
  const effect = findNode(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('void handlePendingGoal(pendingGoal.x, pendingGoal.y, pendingGoal.z)')) as ts.CallExpression
  for (const height of [undefined, 0, 1.25]) {
    let scheduled = 0
    const run = evaluate(effect.arguments[0], {
      pendingGoal: { x: 1, y: 2, z: height },
      previewCurrent: false,
      goalSendPending: false,
      isMappingSession: false,
      observe: false,
      navigationSelectionMode: 'single',
      window: {
        setTimeout: () => { scheduled++; return 1 },
        clearTimeout() {},
      },
    }) as () => unknown
    run()
    assert.equal(scheduled, height === undefined ? 0 : 1)
  }
})

test('the compact target card shows height even while settings are closed', () => {
  const panel = findNode(node => hasAttribute(node, 'aria-label', '"导航目标"'))
  assert.ok(panel.parent && ts.isJsxElement(panel.parent))
  const metrics = panel.parent.children.find(child => ts.isJsxElement(child)
    && child.openingElement.attributes.properties.some(attribute => ts.isJsxAttribute(attribute)
      && attribute.name.getText(file) === 'className'
      && attribute.initializer?.getText(file) === '{styles.goalMetrics}')) as ts.JsxElement
  assert.ok(metrics)
  assert.match(metrics.getText(file), /机身高度/)
  assert.match(metrics.getText(file), /typeof pendingGoal\.z === 'number' \? pendingGoal\.z\.toFixed\(2\) : '—'/)
  assert.doesNotMatch(metrics.getText(file), /<details/)
})

test('changing the arrival radius invalidates the old preview without dispatching motion', () => {
  const input = findNode(node => hasAttribute(node, 'value', '{goalAcceptanceRadius}')) as ts.JsxOpeningElement
  const change = input.attributes.properties.find(attribute => ts.isJsxAttribute(attribute) && attribute.name.getText(file) === 'onChange') as ts.JsxAttribute
  assert.ok(change.initializer && ts.isJsxExpression(change.initializer) && change.initializer.expression)
  const calls: Array<string | number> = []
  const update = evaluate(change.initializer.expression, {
    invalidatePendingGoalPreview: () => calls.push('invalidate'),
    setGoalAcceptanceRadius: (radius: number) => calls.push(radius),
  }) as (event: { target: { value: string } }) => void
  update({ target: { value: '0.8' } })
  assert.deepEqual(calls, ['invalidate', 0.8])
})

test('preview, explicit submission and the displayed circle share the requested arrival radius', async () => {
  const scene = findNode(node => ts.isJsxSelfClosingElement(node) && node.tagName.getText(file) === 'Scene3D')
  assert.match(scene.getText(file), /pendingGoalRadius=\{isMappingSession \? 0\.15 : goalAcceptanceRadius\}/)
  const pendingGoal = { x: 2, y: 3, z: 1.25 }
  const requests: Array<Record<string, unknown>> = []
  const api = {
    constructGoalCandidate: async (request: Record<string, unknown>) => {
      requests.push(request)
      return { ok: true, target: pendingGoal, preview: { feasible: true } }
    },
    sendGoal: async (x: number, y: number, options: Record<string, unknown>) => {
      requests.push({ x, y, ...options })
      return { ok: true }
    },
    formatCommandAck: () => 'accepted',
  }
  const context = {
    api, pendingGoal, goalAcceptanceRadius: 0.8, goalMaxSpeed: 0.4, goalYaw: Math.PI / 2,
    goalPreviewRequest: { current: 0 }, normalizeGoalHeight: (z: number) => z,
    previewDisabledReason: '', goalSendPending: false, goalPreviewPending: false,
    previewCurrent: true, canSendGoal: true,
    setPendingGoalPreview() {}, setGoalPreviewError() {}, setGoalPreviewPending() {},
    setPendingGoal() {}, setGoalSendPending() {}, showToast() {},
  }
  const preview = evaluate(callback('handlePendingGoal'), context) as (x: number, y: number, z: number) => Promise<void>
  await preview(pendingGoal.x, pendingGoal.y, pendingGoal.z)
  assert.equal(requests.length, 1, 'preview must not submit a motion goal')
  const submit = evaluate(callback('handleConfirmGoal'), context) as () => Promise<void>
  await submit()
  assert.equal(requests.length, 2)
  for (const request of requests) {
    assert.equal(request.acceptance_radius_m, 0.8)
    assert.equal(request.z, 1.25)
    assert.equal(request.yaw, Math.PI / 2, 'preview and dispatch must use the displayed target heading')
  }
})

test('reconnection invalidates preview while map or product changes also clear the target', () => {
  const target = { x: 2, y: 3, z: 1.25 }
  let pending: typeof target | null = target
  let preview: object | null = { feasible: true }
  const request = { current: 4 }
  const effects: ts.CallExpression[] = []
  const visit = (node: ts.Node) => {
    if (ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
      && node.arguments[1]?.getText(file).includes('activeMapName')) effects.push(node)
    ts.forEachChild(node, visit)
  }
  visit(file)
  const connectionEffect = effects.find(effect => effect.arguments[0].getText(file).includes('setSavedMapCloud(undefined)'))
  const identityEffect = effects.find(effect => effect.arguments[0].getText(file).includes('setPendingGoal(null)'))
  assert.ok(connectionEffect && identityEffect)
  assert.match(connectionEffect.arguments[1].getText(file), /sseState.connected/)
  assert.doesNotMatch(identityEffect.arguments[1].getText(file), /sseState.connected/)
  assert.match(identityEffect.arguments[1].getText(file), /session\?\.product_session_id/)
  const context = {
    setPendingGoal: (value: typeof pending) => { pending = value },
    setPendingGoalPreview: (value: null) => { preview = value },
    goalPreviewRequest: request,
    setSavedMapCloud() {}, setSavedMapLoadError() {}, setGoalPreviewPending() {}, setGoalPreviewError() {},
  }
  ;(evaluate(connectionEffect.arguments[0], context) as () => void)()
  assert.equal(pending, target)
  assert.equal(preview, null)
  assert.equal(request.current, 5, 'an in-flight preview from before disconnect must be ignored')
  ;(evaluate(identityEffect.arguments[0], context) as () => void)()
  assert.equal(pending, null)
})
