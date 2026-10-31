import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import { liveNavigationStatus } from '../src/services/navigationStatus.ts'
import { gamepadCommand, supportsWebTeleop, teleopInputBlocked } from '../src/services/teleopControl.ts'
import type { NavigationStatusResponse, ProductName } from '../src/types/index.ts'

const sceneViewSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const panelSource = readFileSync(
  new URL('../src/components/TeleopPanel.tsx', import.meta.url),
  'utf8',
)

class FocusElement {
  tagName: string
  parentElement: FocusElement | null
  attributes: Record<string, string>
  isContentEditable: boolean

  constructor(tag: string, parent: FocusElement | null = null, attributes: Record<string, string> = {}) {
    this.tagName = tag.toUpperCase()
    this.parentElement = parent
    this.attributes = attributes
    this.isContentEditable = attributes.contenteditable === 'true' || parent?.isContentEditable === true
  }

  contains(other: FocusElement): boolean {
    for (let node: FocusElement | null = other; node; node = node.parentElement) {
      if (node === this) return true
    }
    return false
  }

  closest(selectors: string): FocusElement | null {
    const matches = selectors.split(',').some(selector => {
      const match = selector.trim().match(/^([a-z]+)?(?:\[([a-z-]+)(?:="([^"]*)")?\])?$/)
      assert.ok(match, `Supported selector: ${selector}`)
      return (!match[1] || this.tagName.toLowerCase() === match[1])
        && (!match[2] || (match[3] === undefined
          ? match[2] in this.attributes : this.attributes[match[2]] === match[3]))
    })
    return matches ? this : this.parentElement?.closest(selectors) ?? null
  }
}

// Execute the actual input callbacks rather than a copied input implementation.
const inputNames = new Set([
  'commandFromKeys', 'sendCurrentInput', 'sendMotion',
  'blocksTeleopKeyboard', 'teleopKey', 'clearInputIntent', 'sendHold', 'setManualEscape',
  'quiesceInput', 'quiesceForInteraction', 'onKeyDown', 'onKeyUp',
  'onShortcutToggle',
  'stopForWindowLoss', 'onVisibility',
  'pollGamepad', 'onDisconnect',
])
const panelAst = ts.createSourceFile('TeleopPanel.tsx', panelSource, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
const inputDeclarations: string[] = []
function collectInputCallbacks(node: ts.Node) {
  if (ts.isFunctionDeclaration(node) && node.name && inputNames.has(node.name.text)) {
    inputDeclarations.push(node.getText(panelAst))
  } else if (ts.isVariableDeclaration(node) && ts.isIdentifier(node.name) && inputNames.has(node.name.text)) {
    const initializer = node.initializer
    assert.ok(initializer)
    const callback = ts.isCallExpression(initializer) ? initializer.arguments[0] : initializer
    inputDeclarations.push(`const ${node.name.text} = ${callback.getText(panelAst)};`)
  }
  ts.forEachChild(node, collectInputCallbacks)
}
collectInputCallbacks(panelAst)
assert.equal(inputDeclarations.length, inputNames.size)
let inputGateEffect: ts.ArrowFunction | undefined
function findInputGateEffect(node: ts.Node) {
  if (ts.isCallExpression(node) && node.expression.getText(panelAst) === 'useEffect'
    && node.arguments[1]?.getText(panelAst) === '[quiesceInput, inputBlocked]') {
    assert.ok(ts.isArrowFunction(node.arguments[0]))
    inputGateEffect = node.arguments[0]
  }
  ts.forEachChild(node, findInputGateEffect)
}
findInputGateEffect(panelAst)
assert.ok(inputGateEffect)
inputDeclarations.push(`const applyInputGate = ${inputGateEffect.getText(panelAst)};`)
const inputCallbacks = ts.transpileModule(
  inputDeclarations.join('\n') + '\n({ commandFromKeys, blocksTeleopKeyboard, clearInputIntent, quiesceForInteraction, onKeyDown, onKeyUp, onShortcutToggle, sendCurrentInput, applyInputGate, stopForWindowLoss, onVisibility, pollGamepad, onDisconnect });',
  { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS } },
).outputText

function keyEvent(key: string, target: FocusElement, repeat = false) {
  return {
    key, target, repeat, code: key === ' ' ? 'Space' : /^[a-z]$/i.test(key) ? `Key${key.toUpperCase()}` : key,
    defaultPrevented: false,
    preventDefault() { this.defaultPrevented = true },
  }
}

function componentValue(name: string, context: Record<string, unknown>): unknown {
  let declaration: ts.VariableDeclaration | undefined
  function find(node: ts.Node) {
    if (ts.isVariableDeclaration(node) && ts.isIdentifier(node.name) && node.name.text === name) declaration = node
    ts.forEachChild(node, find)
  }
  find(panelAst)
  assert.ok(declaration)
  return runInNewContext(ts.transpileModule(`const ${declaration.getText(panelAst)}; ${name};`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, context)
}

test('teleop speed choices include the backend maximum without offering faster commands', () => {
  for (const [maximum, expected] of [
    [1.0, [0.1, 0.2, 0.3, 0.5, 1.0]],
    [0.6, [0.1, 0.2, 0.3, 0.5, 0.6]],
    [0.5, [0.1, 0.2, 0.3, 0.5]],
    [0.2, [0.1, 0.2]],
    [0.15, [0.1, 0.15]],
  ] as const) {
    const backendLimits = { linearMps: maximum, yawRadS: 1 }
    const options = componentValue('speedOptions', { backendLimits }) as number[]
    assert.deepEqual(Array.from(options), expected)
    for (const speedLimit of options) {
      const limits = componentValue('teleopLimits', {
        backendLimits, speedLimit,
        useMemo: (factory: () => unknown) => factory(),
      }) as { linearMps: number }
      assert.equal(limits.linearMps, speedLimit)
      assert.ok(limits.linearMps <= maximum)
    }
    const clamped = componentValue('teleopLimits', {
      backendLimits, speedLimit: maximum + 0.2,
      useMemo: (factory: () => unknown) => factory(),
    }) as { linearMps: number }
    assert.equal(clamped.linearMps, maximum)
  }
})

test('a selected 1 m/s limit applies equally to forward, reverse and lateral keyboard input', () => {
  const h = inputHarness()
  h.context.teleopLimits = componentValue('teleopLimits', {
    backendLimits: { linearMps: 1, yawRadS: 1 }, speedLimit: 1,
    useMemo: (factory: () => unknown) => factory(),
  }) as { linearMps: number; yawRadS: number }
  for (const [key, vxMps, vyMps] of [
    ['w', 1, 0], ['s', -1, 0], ['a', 0, 1], ['d', 0, -1],
  ] as const) {
    h.onKeyDown(keyEvent(key, h.panel))
    assert.equal(h.state.moves.at(-1)?.vxMps, vxMps)
    assert.equal(h.state.moves.at(-1)?.vyMps, vyMps)
    h.onKeyUp(keyEvent(key, h.panel))
    assert.equal(h.inputActiveRef.current, false)
  }
  assert.equal(h.state.holds, 4)
})

test('diagonal keyboard input preserves 1 m/s on each axis and releasing keys clears each intent', () => {
  const h = inputHarness()
  h.context.teleopLimits = { linearMps: 1, yawRadS: 1 }
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('a', h.panel))
  assert.equal(h.state.moves.at(-1)?.vxMps, 1)
  assert.equal(h.state.moves.at(-1)?.vyMps, 1)
  assert.equal(h.state.holds, 0)

  h.onKeyUp(keyEvent('w', h.panel))
  assert.equal(h.state.moves.at(-1)?.vxMps, 0)
  assert.equal(h.state.moves.at(-1)?.vyMps, 1)
  assert.equal(h.state.holds, 0, 'the remaining lateral input must keep moving')

  h.onKeyUp(keyEvent('a', h.panel))
  assert.equal(h.state.holds, 1)
  assert.equal(h.inputActiveRef.current, false)
  const stopped = h.commandFromKeys(h.keysRef.current, h.context.teleopLimits)
  assert.equal(stopped.vxMps, 0)
  assert.equal(stopped.vyMps, 0)
  const movementCount = h.state.moves.length
  h.sendCurrentInput()
  assert.equal(h.state.moves.length, movementCount, 'the refresh timer must not replay diagonal input')
})

test('a physical key release clears motion even when the input method changes its key value', () => {
  const h = inputHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyUp({ ...keyEvent('Process', h.panel), code: 'KeyW' })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
})

test('fresh keys and released axes send the latest velocity without waiting for the refresh timer', () => {
  const h = inputHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.state.moves.length, 1)
  assert.equal(h.state.moves[0].vxMps, 0.5)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.state.moves.length, 1, 'OS key-repeat must not add another publication clock')
  h.onKeyDown(keyEvent('a', h.panel))
  assert.equal(h.state.moves.at(-1)?.vyMps, 0.5)
  h.onKeyUp(keyEvent('a', h.panel))
  assert.equal(h.state.moves.at(-1)?.vxMps, 0.5)
  assert.equal(h.state.moves.at(-1)?.vyMps, 0)
  h.onKeyDown(keyEvent('Shift', h.panel))
  assert.equal(h.state.moves.at(-1)?.vxMps, 0.2)
  h.onKeyUp(keyEvent('Shift', h.panel))
  assert.equal(h.state.moves.at(-1)?.vxMps, 0.5)
  h.onKeyUp(keyEvent('w', h.panel))
  assert.equal(h.state.holds, 1)
  assert.equal(h.inputActiveRef.current, false)
})

function inputHarness(connected = true, connectionReady = true, statusAgeS = 0, resume = false,
  authority: NavigationStatusResponse['control']['authority'] = 'NONE', product: ProductName = 'teleop_avoid') {
  const body = new FocusElement('body')
  const panel = new FocusElement('div', body)
  const state = { holds: 0, connects: 0, moves: [] as Record<string, unknown>[],
    focused: true, pads: [] as Array<Gamepad | null>, gamepads: [] as Array<{ index: number; id: string; mapping: string }>, gamepadStatus: '' }
  const keysRef = { current: new Set<string>() }
  const blockedKeysRef = { current: new Set<string>() }
  const document = { activeElement: body, hidden: false, hasFocus: () => state.focused }
  const navigation = liveNavigationStatus({
    connected, stateSnapshot: null, stateSnapshotReceivedAt: null,
    navigationStatus: { ts: 100, control: { resume_required: resume, authority, reason: resume ? 'resume_required' : '' },
      motion: { permission: 'CLEAR', reason: 'motion_clear' } } as NavigationStatusResponse,
  }, 100 + statusAgeS)
  const controlKnown = componentValue('controlKnown', { navigation })
  const context = {
    Element: FocusElement, HTMLElement: FocusElement,
    document,
    panelRef: { current: Object.assign(panel, { focus: () => { document.activeElement = panel } }) }, keysRef, blockedKeysRef,
    inputActiveRef: { current: false }, manualModeRef: { current: false },
    gamepadArmedRef: { current: false }, inputMode: 'keyboard' as 'keyboard' | 'gamepad', selectedGamepad: null as number | null,
    navigator: { getGamepads: () => state.pads }, gamepadCommand,
    setGamepads: (update: (value: typeof state.gamepads) => typeof state.gamepads) => { state.gamepads = update(state.gamepads) },
    setGamepadStatus: (value: string) => { state.gamepadStatus = value },
    clientRef: { current: { hold: () => { state.holds += 1 }, move: (command: Record<string, unknown>) => state.moves.push(command) } },
    precisionMode: false, PRECISION_SCALE: 0.4, teleopLimits: { linearMps: 0.5, yawRadS: 1.0 },
    product, enabled: componentValue('enabled', {
      controlKnown, product, teleopPath: '/ws/teleop',
    }), connectionReady, resumeRequired: componentValue('resumeRequired', { navigation }),
    inputBlocked: teleopInputBlocked(product, navigation),
    connectClient: () => { state.connects += 1 },
    onExit: () => {},
    setPrecisionMode: (value: boolean) => { context.precisionMode = value },
  }
  const handlers = runInNewContext(inputCallbacks, context) as {
    commandFromKeys: (keys: Set<string>, limits: { linearMps: number; yawRadS: number }) => { vxMps: number; vyMps: number; yawRps: number }
    blocksTeleopKeyboard: (target: FocusElement | null, panel: FocusElement) => boolean
    clearInputIntent: () => void
    quiesceForInteraction: (event: { target: FocusElement }) => void
    onKeyDown: (event: ReturnType<typeof keyEvent>) => void
    onKeyUp: (event: ReturnType<typeof keyEvent>) => void
    onShortcutToggle: (event: { currentTarget: FocusElement & { open: boolean } }) => void
    sendCurrentInput: () => void
    applyInputGate: () => void
    stopForWindowLoss: () => void
    onVisibility: () => void
    pollGamepad: () => void
    onDisconnect: (event: { gamepad: Gamepad }) => void
  }
  const updateNavigation = (next: NavigationStatusResponse | null) => {
    context.inputBlocked = teleopInputBlocked(product, next)
    handlers.applyInputGate()
  }
  return { ...handlers, ...context, body, panel, state, updateNavigation, context }
}
test('teleop panel is gated by bootstrap teleop_ws and one supported-product policy', () => {
  assert.match(panelSource, /api\.fetchAppBootstrap\(\)/)
  assert.match(panelSource, /BOOTSTRAP_RETRY_MS/)
  assert.match(panelSource, /teleopPathFromBootstrap/)
  assert.match(panelSource, /supportsWebTeleop\(product\)/)
  assert.equal(sceneViewSource.match(/!observe && supportsWebTeleop\(currentProduct\)/g)?.length, 2)
  assert.match(panelSource, /正在准备遥控/)
  assert.match(panelSource, /teleopLimitsFromBootstrap/)
  assert.match(panelSource, /linear_mps/)
  assert.match(panelSource, /yaw_rad_s/)
  assert.match(sceneViewSource, /<TeleopPanel[\s\S]*sseState=\{sseState\}/)
})

test('mapping and navigation admit the actual panel without admitting undeclared Products', () => {
  const declarations = panelAst.statements.filter(node =>
    ts.isFunctionDeclaration(node) && node.name?.text === 'currentProduct',
  )
  assert.equal(declarations.length, 1)
  const code = ts.transpileModule(
    declarations.map(node => node.getText(panelAst)).join('\n') + '\ncurrentProduct;',
    { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS } },
  ).outputText
  const resolve = runInNewContext(code, { supportsWebTeleop }) as (state: { session?: { product: string } }) => string | null
  for (const product of ['map', 'teleop', 'teleop_avoid', 'nav']) {
    assert.equal(resolve({ session: { product } }), product)
  }
  for (const product of ['explore', 'tracking', 'inspection']) {
    assert.equal(resolve({ session: { product } }), null)
  }
  assert.equal(resolve({}), null)
  assert.match(sceneViewSource, /supportsWebTeleop\(currentProduct\) && \([\s\S]*?setTeleopMode/)
  assert.match(panelSource, /建图直接遥控/)
  assert.match(panelSource, /局部规划器继续避障/)
  assert.match(panelSource, /退出接管后，在任务卡点击“继续导航”/)
})

function navTakeoverStatus(): NavigationStatusResponse {
  return {
    schema_version: 3,
    ts: 100,
    task: { state: 'PAUSED', task_id: 'navigation-task', reason: 'operator_takeover' },
    goal_admission: { state: 'BLOCKED', reason: 'operator_takeover' },
    control: { authority: 'OPERATOR', resume_required: true, reason: 'operator_takeover' },
    motion: { permission: 'HELD', observation: 'MOVING', stop_confirmation: 'NOT_REQUESTED', reason: 'operator_takeover' },
  }
}

test('the first navigation takeover status keeps deliberate input active until key release', () => {
  const h = inputHarness(true, true, 0, false, 'AUTONOMY', 'nav')
  h.applyInputGate()
  assert.equal(h.state.moves.length, 0, 'opening the panel cannot take over or move')
  assert.equal(h.state.holds, 0)
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.state.moves.length, 1)
  h.updateNavigation(navTakeoverStatus())
  assert.equal(h.state.holds, 0, 'autonomy resume confirmation must not stop this manual input')
  assert.equal(h.keysRef.current.has('w'), true)
  h.sendCurrentInput()
  assert.equal(h.state.moves.length, 2)
  h.onKeyUp(keyEvent('w', h.panel))
  assert.equal(h.state.holds, 1)
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyDown(keyEvent('q', h.panel))
  assert.equal(h.state.moves.length, 3, 'a new deliberate manual input can follow a release')
  assert.doesNotMatch(panelSource, /resumeNavigationTask|submitProductSwitch|TaskResume/)
})

test('navigation takeover stops on blur and hidden-page events without replaying held keys', () => {
  const h = inputHarness(true, true, 0, false, 'AUTONOMY', 'nav')
  h.onKeyDown(keyEvent('w', h.panel))
  h.updateNavigation(navTakeoverStatus())
  h.stopForWindowLoss()
  assert.equal(h.state.holds, 1)
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.state.moves.length, 1)
  h.onKeyUp(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.state.moves.length, 2)
  h.document.hidden = true
  h.onVisibility()
  assert.equal(h.state.holds, 2)
  assert.equal(h.keysRef.current.size, 0)
})

test('independent safety holds quiesce navigation takeover and clearing them needs a fresh key', () => {
  for (const [permission, reason] of [
    ['ESTOPPED', 'estop_latched'],
    ['UNKNOWN', 'native_endpoint_status_unavailable'],
    ['HELD', 'native_control_loop_unhealthy'],
    ['HELD', 'driver_not_ready'],
  ] as const) {
    const h = inputHarness(true, true, 0, false, 'AUTONOMY', 'nav')
    h.onKeyDown(keyEvent('w', h.panel))
    const status = navTakeoverStatus()
    status.motion.permission = permission
    status.motion.reason = reason
    h.updateNavigation(status)
    assert.equal(h.state.holds, 1, reason)
    assert.equal(h.keysRef.current.size, 0, reason)
    h.sendCurrentInput()
    h.onKeyDown(keyEvent('q', h.panel))
    assert.equal(h.state.moves.length, 1, reason)
    h.updateNavigation(navTakeoverStatus())
    h.onKeyDown(keyEvent('w', h.panel, true))
    assert.equal(h.state.moves.length, 1, 'clearing a safety hold cannot replay an existing key')
    h.onKeyUp(keyEvent('w', h.panel))
    h.onKeyDown(keyEvent('w', h.panel))
    assert.equal(h.state.moves.length, 2, 'a fresh key can use the restored manual control')
  }
})

test('missing navigation truth or a different resume hold never receives the takeover exception', () => {
  const h = inputHarness(true, true, 0, false, 'AUTONOMY', 'nav')
  h.onKeyDown(keyEvent('w', h.panel))
  h.updateNavigation(null)
  assert.equal(h.state.holds, 1)
  assert.equal(h.keysRef.current.size, 0)
  const status = navTakeoverStatus()
  status.control.authority = 'AUTONOMY'
  assert.equal(teleopInputBlocked('nav', status), true)
  status.control.authority = 'OPERATOR'
  status.control.reason = 'resume_required'
  assert.equal(teleopInputBlocked('nav', status), true)
  status.control.reason = 'operator_takeover'
  assert.equal(teleopInputBlocked('nav', status), false)
  assert.equal(teleopInputBlocked('teleop_avoid', status), true, 'standalone Products retain their existing resume gate')
})

function pad(axes = [0, 0, 0, 0], deadman = false, index = 0): Gamepad {
  return { connected: true, mapping: 'standard', axes, index, id: `test-controller-${index}`,
    buttons: Array.from({ length: 17 }, (_, button) => ({ pressed: button === 4 && deadman, touched: false, value: button === 4 && deadman ? 1 : 0 })),
  } as Gamepad
}

function gamepadHarness() {
  const h = inputHarness(true, true, 0, false, 'AUTONOMY', 'nav')
  h.context.inputMode = 'gamepad'
  h.context.selectedGamepad = 0
  h.state.pads = [pad()]
  return h
}

test('standard gamepad axes follow robot conventions, deadzone and configured speed limits', () => {
  const limits = { linearMps: 0.5, yawRadS: 1 }
  const command = gamepadCommand(pad([-1, -1, 1, 0], true), limits)
  assert.ok(command)
  assert.ok(command.vxMps > 0 && command.vyMps > 0 && command.yawRps < 0)
  assert.ok(Math.hypot(command.vxMps, command.vyMps) <= limits.linearMps + 1e-12)
  assert.equal(command.deadman, true)
  assert.equal(gamepadCommand(pad([0.14, -0.14, 0.14, 1]), limits)?.neutral, true)
  assert.equal(gamepadCommand({ ...pad(), mapping: '' }, limits), null, 'unmapped layouts are never guessed')
  assert.equal(gamepadCommand(null, limits), null)
})

test('handheld input requires explicit selection, neutral sticks and a released deadman before use', () => {
  const h = gamepadHarness()
  h.context.selectedGamepad = null
  h.state.pads = [pad([0, -1, 0, 0], true)]
  h.pollGamepad()
  assert.equal(h.state.moves.length, 0)
  assert.equal(h.state.gamepadStatus, '请选择手柄')
  h.context.selectedGamepad = 0
  h.pollGamepad()
  assert.equal(h.state.moves.length, 0, 'a stick already held on entry cannot move the robot')
  h.state.pads = [pad()]
  h.pollGamepad()
  assert.equal(h.state.moves.length, 0)
  h.state.pads = [pad([0, -1, 0, 0], true)]
  h.pollGamepad()
  assert.equal(h.state.moves.length, 1)
  assert.equal(h.state.moves[0].vxMps, 0.5)
  assert.equal(h.state.moves[0].manualMode, false, 'gamepad cannot bypass the assisted local planner')
  h.updateNavigation(navTakeoverStatus())
  h.pollGamepad()
  assert.equal(h.state.moves.length, 2, 'the autonomous resume marker does not self-Hold the gamepad')
  assert.equal(h.state.holds, 0)
})

test('neutral sticks and deadman release stop the hand controller through the same hold path', () => {
  const h = gamepadHarness()
  h.pollGamepad()
  h.state.pads = [pad([0, -1, 0, 0], true)]
  h.pollGamepad()
  h.state.pads = [pad([0, 0, 0, 0], true)]
  h.pollGamepad()
  assert.equal(h.state.holds, 1)
  assert.equal(h.inputActiveRef.current, false)
  h.state.pads = [pad([0, -1, 0, 0], true)]
  h.pollGamepad()
  h.state.pads = [pad([0, -1, 0, 0], false)]
  h.pollGamepad()
  assert.equal(h.state.holds, 2)
  h.state.pads = [pad([0, -1, 0, 0], true)]
  h.pollGamepad()
  assert.equal(h.state.moves.length, 2, 'repressing the deadman on an old held stick must require neutral first')
})

test('blur, hidden pages, disconnect and independent safety holds require neutral hand input again', () => {
  for (const stop of ['blur', 'hidden', 'disconnect', 'safety']) {
    const h = gamepadHarness()
    h.pollGamepad()
    h.state.pads = [pad([0, -1, 0, 0], true)]
    h.pollGamepad()
    if (stop === 'blur') h.stopForWindowLoss()
    if (stop === 'hidden') { h.document.hidden = true; h.onVisibility() }
    if (stop === 'disconnect') h.onDisconnect({ gamepad: h.state.pads[0]! })
    if (stop === 'safety') {
      const status = navTakeoverStatus()
      status.motion.reason = 'native_control_loop_unhealthy'
      h.updateNavigation(status)
    }
    assert.equal(h.state.holds, 1, stop)
    h.document.hidden = false
    h.updateNavigation(navTakeoverStatus())
    h.pollGamepad()
    assert.equal(h.state.moves.length, 1, `${stop} must not replay a held stick`)
    h.state.pads = [pad()]
    h.pollGamepad()
    h.state.pads = [pad([0, -1, 0, 0], true)]
    h.pollGamepad()
    assert.equal(h.state.moves.length, 2, `${stop} permits a new deliberate hand input`)
  }
})

test('a disconnected, unsupported or blocked gamepad API cannot keep old hand input alive', () => {
  for (const failure of ['missing', 'unsupported', 'security', 'api']) {
    const h = gamepadHarness()
    h.pollGamepad()
    h.state.pads = [pad([0, -1, 0, 0], true)]
    h.pollGamepad()
    if (failure === 'missing') h.state.pads = [null]
    if (failure === 'unsupported') h.state.pads = [{ ...pad([0, -1, 0, 0], true), mapping: '' }]
    if (failure === 'security') h.context.navigator.getGamepads = () => { throw new Error('Gamepad access denied') }
    if (failure === 'api') Object.assign(h.context.navigator, { getGamepads: undefined })
    h.pollGamepad()
    assert.equal(h.state.holds, 1, failure)
    assert.equal(h.state.moves.length, 1, failure)
    assert.equal(h.gamepadArmedRef.current, false, failure)
  }
})

test('only the explicitly chosen input adapter and controller can publish motion', () => {
  const h = gamepadHarness()
  h.state.pads = [pad(), pad([0, -1, 0, 0], true, 1)]
  h.pollGamepad()
  h.onKeyDown(keyEvent('w', h.panel))
  h.sendCurrentInput()
  assert.equal(h.state.moves.length, 0, 'keyboard and unselected controller are inactive')
  h.context.selectedGamepad = 1
  h.clearInputIntent()
  h.pollGamepad()
  assert.equal(h.state.moves.length, 0, 'selecting a controller with pre-held input cannot start movement')
  h.state.pads[1] = pad([0, 0, 0, 0], false, 1)
  h.pollGamepad()
  h.state.pads[1] = pad([0, -1, 0, 0], true, 1)
  h.pollGamepad()
  assert.equal(h.state.moves.length, 1)
  assert.match(panelSource, /if \(inputMode === 'gamepad'\)[\s\S]*setInterval\(pollGamepad, SEND_INTERVAL_MS\)/)
  assert.match(panelSource, /aria-label="输入方式"/)
  assert.match(panelSource, /aria-label="控制手柄"/)
})

test('keyboard mode uses hold-to-move and keeps native control rejection handling', () => {
  assert.match(panelSource, /<kbd>W<\/kbd><kbd>S<\/kbd>/)
  assert.match(panelSource, /按住慢行（40%）/)
  assert.match(panelSource, /<kbd>Space<\/kbd><\/dt><dd>停止/)
  assert.match(panelSource, /const SEND_INTERVAL_MS = 20/)
  assert.match(panelSource, /const PRECISION_SCALE = 0\.4/)
  assert.match(panelSource, /sendHold/)
  assert.match(panelSource, /visibilitychange/)
  assert.match(panelSource, /manualMode: manualModeRef\.current/)
  assert.match(panelSource, /按住 M/)
  assert.match(panelSource, /clearInputIntent\(\)[\s\S]*client\.hold\('rejected_input'\)/)
  assert.match(panelSource, /resume_required/)
  assert.match(panelSource, /api\.resumeNavigation\(\)/)
  assert.match(panelSource, /恢复控制/)
  assert.doesNotMatch(panelSource, /LEASE_RENEW_INTERVAL_MS|heartbeat/)
  assert.doesNotMatch(panelSource, /keyboardDeadman/)
})

test('teleop panel clears latched input on focus loss and yields focus to other controls', () => {
  assert.match(panelSource, /function blocksTeleopKeyboard/)
  assert.match(panelSource, /keysRef\.current\.clear\(\)/)
  assert.match(panelSource, /blockedKeysRef\.current\.add\(key\)/)
  assert.match(panelSource, /blockedKeysRef\.current\.has\(key\)/)
  assert.match(panelSource, /blockedKeysRef\.current\.delete\(key\)/)
  assert.match(panelSource, /setPrecisionMode\(false\)/)
  assert.match(panelSource, /document\.addEventListener\('focusin', quiesceForInteraction\)/)
  assert.match(panelSource, /document\.addEventListener\('pointerdown', quiesceForInteraction, true\)/)
  assert.match(panelSource, /document\.removeEventListener\('focusin', quiesceForInteraction\)/)
  assert.match(panelSource, /document\.removeEventListener\('pointerdown', quiesceForInteraction, true\)/)
  assert.match(panelSource, /event\.code === 'Space'/)
  assert.match(panelSource, /keysRef\.current\.size === 0/)
})

test('menu, dialog and ordinary controls retain Space and letters; teleop buttons retain motion keys', () => {
  const h = inputHarness()
  const menu = new FocusElement('div', h.body, { role: 'menu' })
  const dialog = new FocusElement('aside', h.body, { role: 'dialog' })
  const details = new FocusElement('details', h.body, { open: '' })
  const edit = new FocusElement('div', h.panel, { contenteditable: 'true' })
  const blockedTargets = [
    new FocusElement('button', h.body), new FocusElement('summary', h.body),
    new FocusElement('summary', h.panel),
    new FocusElement('span', new FocusElement('details', h.panel, { open: '' })),
    new FocusElement('a', h.body, { href: '#' }), new FocusElement('input', h.panel),
    new FocusElement('span', edit), new FocusElement('span', menu),
    new FocusElement('div', dialog), new FocusElement('span', details),
  ]
  for (const target of blockedTargets) {
    assert.equal(h.blocksTeleopKeyboard(target, h.panel), true)
    for (const key of [' ', 'w', 'm', 'Shift']) {
      const event = keyEvent(key, target)
      h.onKeyDown(event)
      h.onKeyUp(event)
      assert.equal(event.defaultPrevented, false, `${target.tagName} retains ${key}`)
      assert.equal(h.keysRef.current.size, 0)
      assert.equal(h.manualModeRef.current, false)
    }
  }
  assert.equal(h.state.connects, 0)
  assert.equal(h.state.holds, 0)
  assert.equal(h.blocksTeleopKeyboard(new FocusElement('canvas', h.body), h.panel), false)
  const ownButton = new FocusElement('button', h.panel)
  const down = keyEvent('w', ownButton)
  h.onKeyDown(down)
  assert.equal(down.defaultPrevented, true)
  assert.equal(h.keysRef.current.has('w'), true)
  const stop = keyEvent(' ', ownButton)
  h.onKeyDown(stop)
  assert.equal(stop.defaultPrevented, true)
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
})

test('opening another control quiesces held input; return cannot replay it before release', () => {
  const h = inputHarness()
  const ownButton = new FocusElement('button', h.panel)
  const menuTrigger = new FocusElement('summary', h.body)
  h.onKeyDown(keyEvent('w', ownButton))
  h.onKeyDown(keyEvent('m', ownButton))
  assert.equal(h.manualModeRef.current, true)
  h.quiesceForInteraction({ target: menuTrigger })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.blockedKeysRef.current.has('w'), true)
  assert.equal(h.manualModeRef.current, false)
  assert.equal(h.state.holds, 1)
  h.onKeyDown(keyEvent('w', ownButton, true))
  h.onKeyDown(keyEvent('m', ownButton, true))
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.manualModeRef.current, false)
  h.onKeyUp(keyEvent('w', menuTrigger))
  h.onKeyUp(keyEvent('m', menuTrigger))
  h.onKeyDown(keyEvent('w', ownButton))
  assert.equal(h.keysRef.current.has('w'), true)
  const release = keyEvent('w', menuTrigger)
  h.onKeyUp(release)
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(release.defaultPrevented, false)
  assert.equal(h.state.holds, 2)
})

test('a key first pressed inside a menu cannot become motion when the menu closes', () => {
  const h = inputHarness()
  const menuButton = new FocusElement('button', h.body)
  h.onKeyDown(keyEvent('w', menuButton))
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyUp(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.has('w'), true)
})

test('entering keyboard mode with a key already held cannot start movement', () => {
  const h = inputHarness()
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyUp(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.has('w'), true)
})

test('Escape clears active motion before exiting keyboard mode', () => {
  const h = inputHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('Escape', h.panel))
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
})

test('browser shortcuts and IME composition cannot become motion commands', () => {
  for (const modifier of ['ctrlKey', 'metaKey', 'altKey', 'isComposing']) {
    const h = inputHarness()
    const event = { ...keyEvent('w', h.panel), [modifier]: true }
    h.onKeyDown(event)
    assert.equal(h.keysRef.current.size, 0, modifier)
    assert.equal(event.defaultPrevented, false, modifier)
    h.onKeyDown(keyEvent('w', h.panel, true))
    assert.equal(h.keysRef.current.size, 0, `${modifier} release cannot resume a held key`)
  }
})

test('pressing a browser modifier while driving immediately clears the held intent', () => {
  const h = inputHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyDown({ ...keyEvent('Control', h.panel), ctrlKey: true })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
})

test('a direction pressed before connection only connects and cannot be replayed on open', () => {
  const h = inputHarness(true, false)
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.state.connects, 1)
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.state.connects, 1)
  assert.equal(h.keysRef.current.size, 0)
})

test('disconnected telemetry blocks motion despite cached teleop configuration and an open socket', () => {
  const h = inputHarness(false)
  for (const key of ['w', 'a', 's', 'd', 'q', 'e', 'm']) h.onKeyDown(keyEvent(key, h.panel))
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.manualModeRef.current, false)
  assert.equal(h.state.connects, 0)
})

test('stale control truth blocks keys even with an open socket and either cached resume value', () => {
  for (const resume of [false, true]) {
    const h = inputHarness(true, true, 8, resume)
    for (const key of ['w', 'a', 'q', 'm']) h.onKeyDown(keyEvent(key, h.panel))
    assert.equal(h.enabled, false)
    assert.equal(h.keysRef.current.size, 0)
    assert.equal(h.manualModeRef.current, false)
    assert.equal(h.state.moves.length, 0)
  }
  const fresh = inputHarness(true, true, 0, false)
  fresh.onKeyDown(keyEvent('w', fresh.panel, true))
  assert.equal(fresh.state.moves.length, 0, 'fresh status must not replay a held key')
  fresh.onKeyDown(keyEvent('w', fresh.panel))
  assert.equal(fresh.state.moves.length, 1)
  const latched = inputHarness(true, true, 0, true)
  latched.onKeyDown(keyEvent('w', latched.panel))
  assert.equal(latched.state.moves.length, 0)
  const unknown = inputHarness(true, true, 0, false, 'UNKNOWN')
  unknown.onKeyDown(keyEvent('w', unknown.panel))
  assert.equal(unknown.state.moves.length, 0, 'fresh UNKNOWN control cannot accept keys')
})

test('teleop status cannot advertise keyboard control while telemetry is disconnected', () => {
  const context = {
    sseState: { connected: false }, enabled: false, connectionReady: true,
    resumeRequired: false, bootstrapError: null, precisionMode: false,
    sessionActive: false, openState: 'open',
    controlKnown: true, inputBlocked: false, product: 'teleop_avoid', inputMode: 'keyboard', gamepadStatus: '',
  }
  assert.equal(componentValue('controlStatus', context), '连接已断开')
  assert.equal(componentValue('controlStatus', { ...context, resumeRequired: true }), '连接已断开')
  assert.equal(componentValue('controlStatus', {
    ...context, sseState: { connected: true }, enabled: true, connectionReady: false,
  }), '按键开始')
  assert.equal(componentValue('controlStatus', {
    ...context, sseState: { connected: true }, controlKnown: false,
  }), '控制状态未知')
})

test('opening shortcut help clears held motion and requires release before another move', () => {
  const h = inputHarness()
  const summary = new FocusElement('summary', h.panel)
  h.onKeyDown(keyEvent('w', h.panel))
  h.quiesceForInteraction({ target: summary })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyUp(keyEvent('w', summary))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.has('w'), true)
})

test('closing shortcut help restores keyboard control without replaying a held key', () => {
  const h = inputHarness()
  const menu = Object.assign(new FocusElement('details', h.panel, { open: '' }), { open: true })
  const summary = new FocusElement('summary', menu)
  h.document.activeElement = summary
  h.onKeyDown(keyEvent('w', summary))
  h.onShortcutToggle({ currentTarget: menu })
  assert.equal(h.document.activeElement, summary, 'opening help must retain its keyboard focus')

  menu.open = false
  delete menu.attributes.open
  h.onShortcutToggle({ currentTarget: menu })
  assert.equal(h.document.activeElement, h.panel)
  h.onKeyDown(keyEvent('w', h.document.activeElement, true))
  assert.equal(h.keysRef.current.size, 0, 'closing help must not replay a key held inside it')
  h.onKeyUp(keyEvent('w', h.document.activeElement))
  h.onKeyDown(keyEvent('w', h.document.activeElement))
  assert.equal(h.keysRef.current.has('w'), true, 'a fresh press after closing help must work')
})

test('closing shortcut help does not take focus from another toolbar control', () => {
  const h = inputHarness()
  const menu = Object.assign(new FocusElement('details', h.panel), { open: false })
  const otherControl = new FocusElement('summary', h.body)
  h.document.activeElement = otherControl
  h.onShortcutToggle({ currentTarget: menu })
  assert.equal(h.document.activeElement, otherControl)
  h.onKeyDown(keyEvent('w', h.document.activeElement))
  assert.equal(h.keysRef.current.size, 0)
})

test('clearing input on connection failure requires a physical release before movement can restart', () => {
  const h = inputHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.clearInputIntent()
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyDown(keyEvent('w', h.panel, true))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.size, 0, 'reconnection alone does not release the blocked physical key')
  h.onKeyUp({ ...keyEvent('Process', h.panel), code: 'KeyW' })
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.has('w'), true)
})

test('opening a ready teleop panel prepares the connection before any direction key', () => {
  let effect: ts.Expression | undefined
  function find(node: ts.Node) {
    if (ts.isCallExpression(node) && node.expression.getText(panelAst) === 'useEffect'
      && node.arguments[1]?.getText(panelAst) === '[closeClient, connectClient, enabled]') effect = node.arguments[0]
    ts.forEachChild(node, find)
  }
  find(panelAst)
  assert.ok(effect)
  for (const enabled of [false, true]) {
    const calls: string[] = []
    let scheduled: (() => void) | undefined
    const run = runInNewContext(ts.transpileModule(`(${effect.getText(panelAst)})`, {
      compilerOptions: { target: ts.ScriptTarget.ES2022 },
    }).outputText, {
      enabled,
      connectClient: () => calls.push('connect'), closeClient: () => calls.push('close'),
      window: { setTimeout: (fn: () => void) => { scheduled = fn; return 1 }, clearTimeout: () => {} },
    }) as () => () => void
    const cleanup = run()
    assert.ok(scheduled)
    scheduled()
    assert.deepEqual(calls, [enabled ? 'connect' : 'close'])
    cleanup()
  }
})
