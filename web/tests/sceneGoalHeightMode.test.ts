import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import { sceneGoalHeight, navigationSupportHeightFromRunPlan, navigationPlanEndpoint } from '../src/services/navigationPreview.ts'
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
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.React, jsxFactory: 'jsx', jsxFragmentFactory: 'Fragment' },
  }).outputText
  if (!('isMappingWorkspace' in context)) context.isMappingWorkspace = false
  return runInNewContext(code, context)
}

function hasAttribute(node: ts.Node, name: string, value: string): node is ts.JsxOpeningElement | ts.JsxSelfClosingElement {
  return (ts.isJsxOpeningElement(node) || ts.isJsxSelfClosingElement(node))
    && node.attributes.properties.some(attribute => ts.isJsxAttribute(attribute)
      && attribute.name.getText(file) === name && attribute.initializer?.getText(file) === value)
}

test('fixed cloud slicing works without localization and remains fixed when the pose changes', () => {
  const context: Record<string, unknown> = {
    activeMap: '903', poseEpoch: null, cloudHeightModePreference: 'fixed',
    cloudHeightBase: null, fixedCloudHeight: null, poseAvailable: false, robotZ: 0,
    cloudHeightBelowBase: 1, cloudHeightAboveBase: 1, mapColorHeightRange: { min: 8, max: 12 },
  }
  const render = () => {
    for (const name of ['cloudHeightKey', 'cloudHeightMode', 'lastCloudHeight', 'followedCloudHeight',
      'currentFixedHeight', 'fixedHeightBase', 'displayHeight', 'cloudMinZ', 'cloudMaxZ']) {
      const declaration = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === name) as ts.VariableDeclaration
      context[name] = evaluate(declaration.initializer!, context)
    }
  }
  const effect = findNode(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('setFixedCloudHeight')) as ts.CallExpression
  context.setFixedCloudHeight = (update: (current: unknown) => unknown) => {
    context.fixedCloudHeight = update(context.fixedCloudHeight)
  }
  render()
  assert.equal(context.cloudMinZ, 9)
  assert.equal(context.cloudMaxZ, 11)
  evaluate(effect.arguments[0], context)()
  context.poseAvailable = true
  context.robotZ = 8.4
  render()
  evaluate(effect.arguments[0], context)()
  assert.equal(context.displayHeight, 10, 'new localization must not move a fixed slice')
  context.activeMap = 'second-map'
  context.poseAvailable = false
  context.mapColorHeightRange = { min: -2, max: 0 }
  render()
  assert.equal(context.displayHeight, -1, 'another map must not reuse the old slice')
  context.cloudHeightModePreference = 'follow'
  render()
  assert.equal(context.cloudMinZ, null, 'map heights are not a substitute for a robot pose')
  context.cloudHeightModePreference = 'all'
  render()
  assert.equal(context.cloudMaxZ, null)
})

test('preview explains missing, stale and non-map poses without weakening its pose gate', () => {
  const declaration = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === 'previewDisabledReason') as ts.VariableDeclaration
  for (const [connected, odom, telemetryFresh, poseAvailable, expected] of [
    [false, null, false, false, '连接断开，等待恢复'],
    [true, null, false, false, '等待机器人位姿'],
    [true, { ts: 1 }, false, false, '定位数据已过期，等待更新'],
    [true, { frame_id: 'odom' }, true, false, '定位坐标无效，等待地图坐标位姿'],
    [true, { frame_id: 'map' }, true, true, ''],
  ]) {
    assert.equal(evaluate(declaration.initializer!, { sseState: { connected }, odom, telemetryFresh, poseAvailable }), expected)
  }
})

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
  assert.match(dock.initializer.getText(file), /aria-label="目标选择"/)
  assert.match(dock.initializer.getText(file), /aria-pressed=\{goalPickMode === 'plane'\}/)
  assert.match(dock.initializer.getText(file), /aria-pressed=\{goalPickMode === 'surface'\}/)
  assert.match(dock.initializer.getText(file), /setGoalPickMode\('plane'\)/)
  assert.match(dock.initializer.getText(file), /setGoalPickMode\('surface'\)/)
  assert.doesNotMatch(dock.initializer.getText(file), /setNavigationSelectionMode/)
  assert.match(dock.initializer.getText(file), /aria-label="目标选择模式"/)
})

test('single and route callbacks preserve the same selected three-dimensional position', () => {
  for (const height of [0.42, 1.8, 0]) {
    let selected: { x: number; y: number; z?: number } | undefined
    let heading: number | undefined = 1, editing = true
    const select = evaluate(callback('selectPendingGoal'), {
      invalidatePendingGoalPreview() {},
      normalizeGoalHeight: (value: number | null) => value,
      sceneGoalHeight,
      navigationSupportHeight: .435,
      mapOdom: { z: 0.42 },
      setPendingGoal: (point: typeof selected) => { selected = point },
      setGoalPreviewError() {}, setGoalHeadingEditing(value: boolean) { editing = value }, setGoalYaw(value: number | undefined) { heading = value },
      setFollowRobot() {},
      scene3DRef: { current: { focusPoint() { assert.fail('selecting a gesture target must not move its camera') } } },
    }) as (x: number, y: number, z: number) => typeof selected
    const returned = select(1, 2, height)
    assert.equal(returned, selected, 'pointer heading must use the same normalized target stored by React')
    assert.equal(heading, undefined, 'a new point has no implied arrival direction')
    assert.equal(editing, false, 'a normal click must not enter heading editing')
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
      setGoalPreviewError: (message: string | null) => { warning = message }, setGoalHeadingEditing() {}, setGoalYaw() {},
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

test('missing height or unfinished heading does not schedule an automatic preview request', () => {
  const effect = findNode(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('void handlePendingGoal(pendingGoal.x, pendingGoal.y, pendingGoal.z)')) as ts.CallExpression
  for (const goalHeadingEditing of [false, true]) for (const goalYaw of [undefined, 0, Math.PI / 2]) for (const height of [undefined, 0, 1.25]) {
    let scheduled = 0
    const run = evaluate(effect.arguments[0], {
      pendingGoal: { x: 1, y: 2, z: height },
      previewCurrent: false,
      goalSendPending: false,
      goalHeadingEditing,
      goalYaw,
      isMappingSession: false,
      observe: false,
      navigationSelectionMode: 'single',
      window: {
        setTimeout: () => { scheduled++; return 1 },
        clearTimeout() {},
      },
    }) as () => unknown
    run()
    assert.equal(scheduled, height === undefined || goalHeadingEditing ? 0 : 1)
  }
})

test('the compact target card shows distance without a body-height metric', () => {
  const panel = findNode(node => hasAttribute(node, 'aria-label', '"导航目标"'))
  assert.ok(panel.parent && ts.isJsxElement(panel.parent))
  const metrics = findNode(node => hasAttribute(node, 'className', '{styles.goalMetrics}')).parent as ts.JsxElement
  assert.ok(metrics)
  assert.doesNotMatch(metrics.getText(file), /机身高度|pendingGoal\.z/)
  assert.match(metrics.getText(file), /selectedGoalDistance/)
  assert.doesNotMatch(metrics.getText(file), /<details/)
})

test('optional heading can be set graphically, typed or cleared without an extra finish action', () => {
  const input = findNode(node => hasAttribute(node, 'aria-label', '"到达朝向（度）"')) as ts.JsxSelfClosingElement
  const button = findNode(node => hasAttribute(node, 'aria-pressed', '{goalHeadingEditing}')) as ts.JsxOpeningElement
  let editing = false, yaw: number | undefined, invalidations = 0
  const context = {
    goalYaw: 0,
    setGoalHeadingEditing(value: boolean | ((previous: boolean) => boolean)) { editing = typeof value === 'function' ? value(editing) : value },
    handleGoalHeadingChange(value: number | undefined) { yaw = value },
    invalidatePendingGoalPreview() { invalidations++ },
  }
  const eventHandler = (node: typeof input | typeof button, name: string) => {
    const attr = node.attributes.properties.find(item => ts.isJsxAttribute(item) && item.name.getText(file) === name) as ts.JsxAttribute
    assert.ok(attr.initializer && ts.isJsxExpression(attr.initializer) && attr.initializer.expression)
    return evaluate(attr.initializer.expression, context)
  }
  eventHandler(button, 'onClick')()
  assert.equal(editing, true)
  eventHandler(input, 'onBlur')({ currentTarget: { value: '90', valueAsNumber: 90 } })
  assert.equal(editing, false)
  assert.equal(yaw, Math.PI / 2)
  assert.equal(invalidations, 1)
  eventHandler(input, 'onBlur')({ currentTarget: { value: '', valueAsNumber: NaN } })
  assert.equal(yaw, undefined)
  assert.equal(editing, false)
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

test('preview and submission preserve arrival tolerance independently of the point marker', async () => {
  const scene = findNode(node => ts.isJsxSelfClosingElement(node) && node.tagName.getText(file) === 'Scene3D')
  assert.doesNotMatch(scene.getText(file), /pendingGoalRadius/)
  const pendingGoal = { x: 2, y: 3, z: 1.25 }
  const requests: Array<Record<string, unknown>> = []
  const api = {
    constructGoalCandidate: async (request: Record<string, unknown>) => {
      requests.push(request)
      return { ok: true, target: pendingGoal, preview: { ok: true, feasible: true, frame_id: 'map', goal: pendingGoal, path: [pendingGoal] } }
    },
    sendGoal: async (x: number, y: number, options: Record<string, unknown>) => {
      requests.push({ x, y, ...options })
      return { ok: true }
    },
    formatCommandAck: () => 'accepted',
  }
  const context = {
    api, pendingGoal, navigationPlanEndpoint, goalHeadingEditing: false, plannedGoal: pendingGoal, goalAcceptanceRadius: 0.8, goalMaxSpeed: 0.4 as number | null, goalYaw: Math.PI / 2,
    goalPreviewRequest: { current: 0 }, normalizeGoalHeight: (z: number) => z,
    previewDisabledReason: '', goalSendPending: false, goalPreviewPending: false,
    previewCurrent: true, canSendGoal: true,
    setPendingGoalPreview() {}, setGoalPreviewError() {}, setGoalHeadingEditing() {}, setGoalPreviewPending() {},
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
    assert.equal(request.max_speed_mps, 0.4)
  }
  context.goalMaxSpeed = null
  await preview(pendingGoal.x, pendingGoal.y, pendingGoal.z)
  await submit()
  for (const request of requests.slice(2)) {
    assert.equal('max_speed_mps' in JSON.parse(JSON.stringify(request)), false,
      'default speed must leave the limit to the active navigation configuration')
  }
  context.goalMaxSpeed = 1.0
  await preview(pendingGoal.x, pendingGoal.y, pendingGoal.z)
  await submit()
  for (const request of requests.slice(4)) assert.equal(request.max_speed_mps, 1.0)
})

test('goal speed starts and resets to the active navigation configuration', () => {
  const state = findNode(node => ts.isVariableDeclaration(node)
    && node.name.getText(file) === '[goalMaxSpeed, setGoalMaxSpeed]') as ts.VariableDeclaration
  assert.ok(state.initializer && ts.isCallExpression(state.initializer))
  assert.equal(state.initializer.arguments[0].getText(file), 'null')
  assert.match(source, /setGoalMaxSpeed\(null\); setGoalAcceptanceRadius\(0\.45\)/)
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
    setSavedMapCloud() {}, setSavedMapLoadError() {}, setGoalPreviewPending() {}, setGoalPreviewError() {}, setGoalHeadingEditing() {},
  }
  ;(evaluate(connectionEffect.arguments[0], context) as () => void)()
  assert.equal(pending, target)
  assert.equal(preview, null)
  assert.equal(request.current, 5, 'an in-flight preview from before disconnect must be ignored')
  ;(evaluate(identityEffect.arguments[0], context) as () => void)()
  assert.equal(pending, null)
})


test('heading changes and clearing invalidate only the preview and preserve selected coordinates', () => {
  let invalidated = 0, heading: number | undefined = 0
  const target = { x: 1, y: 2, z: .435 }
  const change = evaluate(callback('handleGoalHeadingChange'), {
    goalSendPending: false, pendingGoal: target,
    invalidatePendingGoalPreview: () => { invalidated++ }, setGoalYaw: (value: number | undefined) => { heading = value },
    api: { sendGoal: () => assert.fail('heading must not dispatch movement') },
  }) as (yaw: number | undefined) => void
  change(Math.PI / 3)
  assert.equal(heading, Math.PI / 3); assert.equal(invalidated, 1)
  assert.deepEqual(target, { x: 1, y: 2, z: .435 })
  change(NaN); assert.equal(invalidated, 1)
  change(undefined)
  assert.equal(heading, undefined)
  assert.equal(invalidated, 2)
})

test('a candidate response does not replace the user-selected target', async () => {
  const selected = { x: 2, y: 3, z: .435 }
  let preview: unknown = null
  const run = evaluate(callback('handlePendingGoal'), {
    goalPreviewRequest: { current: 0 }, normalizeGoalHeight: (z: number) => z,
    navigationPlanEndpoint, previewDisabledReason: '', goalYaw: 0, goalHeadingEditing: false, goalAcceptanceRadius: .45, goalMaxSpeed: null,
    setPendingGoalPreview: (value: unknown) => { preview = value }, setGoalPreviewError() {}, setGoalPreviewPending() {},
    setPendingGoal: () => assert.fail('server normalization must not overwrite the selected point'),
    api: { constructGoalCandidate: async () => ({ ok: true, target: { ...selected, x: 2.1 },
      preview: { ok: true, feasible: true, frame_id: 'map', goal: selected, path: [{ ...selected, x: 2.1 }] } }) },
  }) as (x: number, y: number, z: number) => Promise<void>
  await run(selected.x, selected.y, selected.z)
  assert.ok(preview)
})

test('a late preview cannot restore an old heading and invalid endpoints cannot be confirmed', async () => {
  const target = { x: 2, y: 3, z: .435 }
  let resolve: (result: unknown) => void = () => {}
  let preview: unknown = null, error: string | null = null, sent = 0
  const context = {
    pendingGoal: target, goalPreviewRequest: { current: 0 }, normalizeGoalHeight: (z: number) => z,
    navigationPlanEndpoint, previewDisabledReason: '', goalYaw: 0, goalAcceptanceRadius: .45, goalMaxSpeed: null,
    setPendingGoalPreview: (value: unknown) => { preview = value },
    setGoalPreviewError: (value: string | null) => { error = value }, setGoalPreviewPending() {},
    goalSendPending: false, canSendGoal: true, goalHeadingEditing: false, goalPreviewPending: false,
    previewCurrent: true, plannedGoal: null,
    api: {
      constructGoalCandidate: () => new Promise(result => { resolve = result }),
      sendGoal: () => { sent++; return { ok: true } },
    },
  }
  const request = evaluate(callback('handlePendingGoal'), context)
  const pending = request(target.x, target.y, target.z)
  evaluate(callback('invalidatePendingGoalPreview'), context)()
  resolve({ ok: true, preview: { ok: true, feasible: true, frame_id: 'map', goal: target, path: [target] } })
  await pending
  assert.equal(preview, null)
  const invalid = request(target.x, target.y, target.z)
  resolve({ ok: true, preview: { ok: true, feasible: true, frame_id: 'map', goal: target, path: [] } })
  await invalid
  assert.match(error!, /preview_result_invalid/)
  await evaluate(callback('handleConfirmGoal'), context)()
  assert.equal(sent, 0)
})

test('position-only goals preview and dispatch without inventing a heading', async () => {
  const pendingGoal = { x: 1, y: 2, z: .435 }
  const requests: Array<Record<string, unknown>> = []
  const context = {
    pendingGoal, goalSendPending: false, goalYaw: undefined,
    goalHeadingEditing: false, goalPreviewPending: false, previewCurrent: true, plannedGoal: pendingGoal,
    goalPreviewRequest: { current: 0 }, previewDisabledReason: '', goalAcceptanceRadius: .45, goalMaxSpeed: null,
    normalizeGoalHeight: (z: number) => z, navigationPlanEndpoint, canSendGoal: true,
    setPendingGoalPreview() {}, setGoalPreviewError() {}, setGoalPreviewPending() {}, setGoalSendPending() {},
    setPendingGoal() {}, setGoalHeadingEditing() {}, showToast() {},
    api: {
      constructGoalCandidate: async (request: Record<string, unknown>) => {
        requests.push(request)
        return { ok: true, preview: { ok: true, feasible: true, frame_id: 'map', goal: pendingGoal, path: [pendingGoal] } }
      },
      sendGoal: async (x: number, y: number, options: Record<string, unknown>) => {
        requests.push({ x, y, ...options }); return { ok: true }
      },
      formatCommandAck: () => 'accepted',
    },
  }
  const preview = evaluate(callback('handlePendingGoal'), context)
  const submit = evaluate(callback('handleConfirmGoal'), context)
  await preview(1, 2, .435)
  assert.equal(requests.length, 1, 'clicking only previews; movement needs the start action')
  await submit()
  assert.equal(requests.length, 2)
  for (const request of requests) {
    const wire = JSON.parse(JSON.stringify(request))
    assert.equal('yaw' in wire, false)
    assert.equal(wire.x, 1); assert.equal(wire.y, 2); assert.equal(wire.z, .435)
    assert.equal(wire.acceptance_radius_m, .45)
  }
  context.goalHeadingEditing = true
  await preview(1, 2, .435)
  await submit()
  assert.equal(requests.length, 2, 'an explicitly started heading edit must finish or be cleared')
})

test('heading gestures require explicit editing and never capture ordinary point selection', () => {
  const scene = findNode(node => ts.isJsxSelfClosingElement(node) && node.tagName.getText(file) === 'Scene3D') as ts.JsxSelfClosingElement
  const prop = (name: string) => {
    const attribute = scene.attributes.properties.find(item => ts.isJsxAttribute(item) && item.name.getText(file) === name) as ts.JsxAttribute
    assert.ok(attribute.initializer && ts.isJsxExpression(attribute.initializer) && attribute.initializer.expression)
    return attribute.initializer.expression
  }
  for (const navigationSelectionMode of ['browse', 'single', 'route']) for (const observe of [false, true]) for (const goalSendPending of [false, true]) for (const goalHeadingEditing of [false, true]) {
    const context = {
      navigationSelectionMode, observe, goalSendPending, isMappingSession: false, stairEditing: false, mapEditing: false,
      routeDraftLocked: false, goalHeadingEditing, selectPendingGoal() {}, handleRouteDraftPoint() {}, handleGoalHeadingChange() {},
    }
    const target = evaluate(prop('onPendingGoal'), context)
    const heading = evaluate(prop('onGoalHeadingChange'), context)
    const canSelect = !observe && !goalSendPending && navigationSelectionMode !== 'browse'
    assert.equal(typeof target === 'function', canSelect)
    assert.equal(typeof heading === 'function', canSelect && navigationSelectionMode === 'single' && goalHeadingEditing)
  }
})

test('target coordinates stay in collapsed details and heading starts blank', () => {
  const input = findNode(node => hasAttribute(node, 'aria-label', '"到达朝向（度）"'))
  assert.match(input.getText(file), /defaultValue=\{goalYaw === undefined \? ''/)
  assert.match(input.getText(file), /placeholder="不限"/)
  let headingDetails: ts.Node | undefined = input.parent
  while (headingDetails && !(ts.isJsxElement(headingDetails) && headingDetails.openingElement.tagName.getText(file) === 'details')) headingDetails = headingDetails.parent
  assert.ok(headingDetails, 'optional heading belongs in collapsed settings')
  const endpoints = findNode(node => hasAttribute(node, 'aria-label', '"目标位置对照"'))
  let container: ts.Node | undefined = endpoints.parent
  while (container && !(ts.isJsxElement(container) && container.openingElement.tagName.getText(file) === 'details')) container = container.parent
  assert.ok(container, 'coordinates and optional settings stay inside collapsed details')
  const details = container as ts.JsxElement
  assert.equal(details.openingElement.tagName.getText(file), 'details')
  assert.equal(details.openingElement.attributes.properties.some(attribute => ts.isJsxAttribute(attribute) && attribute.name.getText(file) === 'open'), false)
  assert.match(details.getText(file), /点击目标/)
  assert.match(details.getText(file), /规划终点/)
  assert.match(details.getText(file), /端点偏移/)
})


test('business workspaces expose distinct actions without starting a Product during render', () => {
  const dock = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === 'sceneDockControls') as ts.VariableDeclaration
  assert.ok(dock.initializer)
  type Element = { type: string; props: Record<string, unknown> | null; children: unknown[] }
  const elements = (value: unknown): Element[] => {
    if (!value || typeof value !== 'object' || !('children' in value)) return []
    const element = value as Element
    return [element, ...element.children.flatMap(elements)]
  }
  for (const isMappingWorkspace of [false, true]) for (const isMappingSession of [false, true]) {
    let starts = 0, saves = 0
    const opened: Array<string | null> = []
    const rendered = evaluate(dock.initializer, {
      jsx: (type: string, props: Element['props'], ...children: unknown[]) => ({ type, props, children }),
      Fragment: 'fragment', styles: {}, MapPinned: 'map-icon', LocateFixed: 'location-icon', Target: 'target-icon',
      Maximize2: 'view-icon', Route: 'route-icon', Save: 'save-icon',
      isMappingWorkspace, isMappingSession, observe: false, productSwitchAllowed: true, productSwitchReason: '',
      stairEditing: false, mapEditing: false, workspaceTool: 'maps', drawerOpen: false,
      routeDraftLocked: false, routeDraft: { points: [] }, navigationSelectionMode: 'single', goalPickMode: 'plane',
      saveStatus: null, saveBlockedReason: '',
      onStartMapping: () => { starts++ }, handleSaveMap: () => { saves++ },
      onOpenSavedMap: (name: string | null) => opened.push(name),
      openWorkspaceTool() {}, selectNavigationMode() {}, setGoalPickMode() {},
    })
    assert.equal(starts, 0)
    assert.equal(saves, 0)
    assert.deepEqual(opened, [])
    const nodes = elements(rendered)
    const start = nodes.find(node => node.type === 'button' && elements(node).some(child => child.children.includes('开始建图')))
    const save = nodes.find(node => node.type === 'button' && elements(node).some(child => child.children.includes('保存地图')))
    const target = nodes.find(node => node.type === 'summary' && node.props?.['aria-label'] === '目标选择')
    assert.equal(Boolean(start), isMappingWorkspace && !isMappingSession)
    assert.equal(Boolean(save), isMappingWorkspace && isMappingSession)
    assert.equal(Boolean(target), !isMappingWorkspace && !isMappingSession)
    if (start) {
      ;(start.props?.onClick as () => void)()
      assert.equal(starts, 1, 'only the explicit start action requests mapping')
    }
    const map = nodes.find(node => node.type === 'button' && elements(node).some(child => child.children.includes(isMappingWorkspace ? '已存地图' : '选地图')))
    assert.ok(map)
    ;(map.props?.onClick as () => void)()
    assert.deepEqual(opened, [null])
  }
})

test('mapping workspace cannot select navigation targets or plan navigation previews', () => {
  const scene = findNode(node => ts.isJsxSelfClosingElement(node) && node.tagName.getText(file) === 'Scene3D') as ts.JsxSelfClosingElement
  const prop = (name: string) => {
    const attribute = scene.attributes.properties.find(item => ts.isJsxAttribute(item) && item.name.getText(file) === name) as ts.JsxAttribute
    assert.ok(attribute.initializer && ts.isJsxExpression(attribute.initializer) && attribute.initializer.expression)
    return attribute.initializer.expression
  }
  for (const isMappingWorkspace of [false, true]) for (const isMappingSession of [false, true]) {
    const context = {
      isMappingWorkspace, isMappingSession, observe: false, canProbeMapping: false,
      stairEditing: false, mapEditing: false, goalSendPending: false, routeDraftLocked: false,
      navigationSelectionMode: 'single', goalHeadingEditing: true, workspaceTool: 'localization', drawerOpen: true,
      selectPendingGoal() {}, handleRouteDraftPoint() {}, handleGoalHeadingChange() {}, handleSceneRelocalize() {},
    }
    for (const name of ['onPendingGoal', 'onGoalHeadingChange', 'onRelocalize']) {
      assert.equal(typeof evaluate(prop(name), context) === 'function', !isMappingWorkspace && !isMappingSession, name)
    }
  }
  const effect = findNode(node => ts.isCallExpression(node) && node.expression.getText(file) === 'useEffect'
    && node.arguments[0]?.getText(file).includes('void handlePendingGoal(pendingGoal.x, pendingGoal.y, pendingGoal.z)')) as ts.CallExpression
  const run = evaluate(effect.arguments[0], {
    isMappingWorkspace: true, isMappingSession: false, observe: false,
    pendingGoal: { x: 1, y: 2, z: .435 }, previewCurrent: false, goalSendPending: false,
    goalHeadingEditing: false, goalYaw: 0, navigationSelectionMode: 'single',
    window: { setTimeout: () => assert.fail('mapping workspace cannot schedule navigation preview') },
    handlePendingGoal: () => assert.fail('mapping workspace cannot preview a goal'),
  }) as () => void
  run()
  const tool = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === 'workspaceTool') as ts.VariableDeclaration
  assert.ok(tool.initializer)
  assert.equal(evaluate(tool.initializer, { isMappingWorkspace: true, selectedWorkspaceTool: 'localization' }), 'maps',
    'switching from navigation keeps its open localization drawer out of mapping')
  assert.equal(evaluate(tool.initializer, { isMappingWorkspace: false, selectedWorkspaceTool: 'localization' }), 'localization')
  const mappingSession = findNode(node => ts.isVariableDeclaration(node) && node.name.getText(file) === 'isMappingSession') as ts.VariableDeclaration
  assert.ok(mappingSession.initializer)
  for (const currentProduct of ['map', 'nav', undefined]) {
    assert.equal(evaluate(mappingSession.initializer, { currentProduct, workspace: 'mapping' }), currentProduct === 'map')
  }
})
