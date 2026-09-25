import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

import {
  TeleopWsClient,
  resolveTeleopWsUrl,
  type TeleopAck,
} from '../src/services/teleopWsClient.ts'

class FakeSocket {
  readyState = 0
  sent: string[] = []
  onopen: ((event: Event) => void) | null = null
  onmessage: ((event: MessageEvent) => void) | null = null
  onclose: ((event: CloseEvent) => void) | null = null
  onerror: ((event: Event) => void) | null = null

  send(data: string): void {
    this.sent.push(data)
  }

  close(): void {
    this.readyState = 3
    this.onclose?.({ type: 'close' } as CloseEvent)
  }

  open(): void {
    this.readyState = 1
    this.onopen?.({ type: 'open' } as Event)
  }

  receive(data: Record<string, unknown>): void {
    this.onmessage?.({ data: JSON.stringify(data) } as MessageEvent)
  }
}

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

// Compile the actual component callbacks; no copy of their keyboard logic is tested.
const keyboardNames = new Set([
  'blocksTeleopKeyboard', 'teleopKey', 'clearInputIntent', 'sendHold', 'setManualEscape',
  'quiesceInput', 'quiesceForInteraction', 'onKeyDown', 'onKeyUp',
  'onShortcutToggle',
])
const keyboardAst = ts.createSourceFile('TeleopPanel.tsx', panelSource, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
const keyboardDeclarations: string[] = []
function collectKeyboardCallbacks(node: ts.Node) {
  if (ts.isFunctionDeclaration(node) && node.name && keyboardNames.has(node.name.text)) {
    keyboardDeclarations.push(node.getText(keyboardAst))
  } else if (ts.isVariableDeclaration(node) && ts.isIdentifier(node.name) && keyboardNames.has(node.name.text)) {
    const initializer = node.initializer
    assert.ok(initializer)
    const callback = ts.isCallExpression(initializer) ? initializer.arguments[0] : initializer
    keyboardDeclarations.push(`const ${node.name.text} = ${callback.getText(keyboardAst)};`)
  }
  ts.forEachChild(node, collectKeyboardCallbacks)
}
collectKeyboardCallbacks(keyboardAst)
assert.equal(keyboardDeclarations.length, keyboardNames.size)
const keyboardCode = ts.transpileModule(
  keyboardDeclarations.join('\n') + '\n({ blocksTeleopKeyboard, clearInputIntent, quiesceForInteraction, onKeyDown, onKeyUp, onShortcutToggle });',
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
  find(keyboardAst)
  assert.ok(declaration)
  return runInNewContext(ts.transpileModule(`const ${declaration.getText(keyboardAst)}; ${name};`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, context)
}

test('a physical key release clears motion even when the input method changes its key value', () => {
  const h = keyboardHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyUp({ ...keyEvent('Process', h.panel), code: 'KeyW' })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
})

test('delayed gateway receipts cannot create a queue of old motion commands ahead of hold', () => {
  const socket = new FakeSocket()
  const client = new TeleopWsClient({ url: '/ws/teleop', socketFactory: () => socket })
  client.connect()
  socket.open()
  try {
    for (let i = 0; i < 150; i++) client.move({ vxMps: 0.5, deadman: true })
    client.hold()
    const samples = socket.sent.map(value => JSON.parse(value))
    assert.ok(samples.filter(value => value.deadman === true).length <= 1)
    assert.equal(samples.at(-1).deadman, false)
  } finally {
    client.disconnect()
  }
})

function keyboardHarness(connected = true, connectionReady = true) {
  const body = new FocusElement('body')
  const panel = new FocusElement('div', body)
  const state = { precision: false, manual: false, holds: 0, connects: 0 }
  const keysRef = { current: new Set<string>() }
  const blockedKeysRef = { current: new Set<string>() }
  const document = { activeElement: body }
  const context = {
    Element: FocusElement, HTMLElement: FocusElement,
    document,
    panelRef: { current: Object.assign(panel, { focus: () => { document.activeElement = panel } }) }, keysRef, blockedKeysRef,
    directionsRef: { current: new Set<string>() },
    inputActiveRef: { current: false }, manualModeRef: { current: false },
    clientRef: { current: { hold: () => { state.holds += 1 } } },
    product: 'teleop_avoid', enabled: componentValue('enabled', {
      sseState: { connected }, product: 'teleop_avoid', teleopPath: '/ws/teleop',
    }), connectionReady, resumeRequired: false,
    connectClient: () => { state.connects += 1 },
    onExit: () => {},
    setPrecisionMode: (value: boolean) => { state.precision = value },
    setManualMode: (value: boolean) => { state.manual = value },
    setActiveDirections: () => {},
  }
  const handlers = runInNewContext(keyboardCode, context) as {
    blocksTeleopKeyboard: (target: FocusElement | null, panel: FocusElement) => boolean
    clearInputIntent: () => void
    quiesceForInteraction: (event: { target: FocusElement }) => void
    onKeyDown: (event: ReturnType<typeof keyEvent>) => void
    onKeyUp: (event: ReturnType<typeof keyEvent>) => void
    onShortcutToggle: (event: { currentTarget: FocusElement & { open: boolean } }) => void
  }
  return { ...handlers, ...context, body, panel, state }
}
const legacyMapViewerSource = readFileSync(
  new URL('../../src/gateway/templates/map_viewer.html', import.meta.url),
  'utf8',
)

test('teleop websocket client sends physical velocity samples as ingress-only operator intent', () => {
  const sockets: FakeSocket[] = []
  const acks: TeleopAck[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop',
    clientId: 'operator-a',
    socketFactory: () => {
      const socket = new FakeSocket()
      sockets.push(socket)
      return socket
    },
    onAck: ack => acks.push(ack),
  })

  client.connect()
  sockets[0].open()
  client.move({ deadman: true })
  const inputRequest = JSON.parse(sockets[0].sent[0])
  sockets[0].receive({ type: 'input_ack', request_id: inputRequest.request_id, input_window: 'window-1' })
  const request = client.move({
    vxMps: 0.4,
    vyMps: -0.2,
    yawRps: 0.3,
    deadman: true,
    manualMode: true,
  })
  assert.ok(request)
  const payload = JSON.parse(sockets[0].sent[1]) as Record<string, unknown>
  assert.equal(payload.type, 'velocity')
  assert.equal(payload.deadman, true)
  assert.equal(payload.manual_mode, true)
  assert.equal(payload.vx_mps, 0.4)
  assert.equal(payload.vy_mps, -0.2)
  assert.equal(payload.yaw_rps, 0.3)
  assert.equal(payload.request_id, request)
  assert.equal(payload.input_window, 'window-1')
  assert.equal('sequence' in payload, false)

  sockets[0].receive({
    type: 'ingress_ack',
    action: 'queued',
    request_id: request,
    final_cmd_vel_confirmed: false,
    motor_confirmed: false,
  })
  assert.equal(acks[0].type, 'ingress_ack')
  assert.equal(acks[0].final_cmd_vel_confirmed, false)
  assert.equal(acks[0].motor_confirmed, false)
  client.disconnect()
})

test('hold supersedes pending input and late ACKs never replay a released key', () => {
  const socket = new FakeSocket()
  const client = new TeleopWsClient({ url: '/ws/teleop', socketFactory: () => socket })
  client.connect()
  socket.open()
  try {
    client.move({ vxMps: 0.5, deadman: true })
    const input = JSON.parse(socket.sent[0])
    socket.receive({ type: 'input_ack', request_id: input.request_id, input_window: 'first' })
    const move = client.move({ vxMps: 0.5, deadman: true })
    for (let i = 0; i < 150; i++) client.move({ vxMps: 0.5, deadman: true })
    assert.equal(socket.sent.length, 2)
    const hold = client.hold()
    socket.receive({ type: 'ingress_ack', request_id: move, input_window: 'late' })
    assert.equal(client.move({ vxMps: 0.5, deadman: true }), null)
    socket.receive({ type: 'control_ack', request_id: hold })
    assert.equal(socket.sent.length, 3)
    client.move({ vxMps: 0.2, deadman: true })
    assert.equal(JSON.parse(socket.sent.at(-1)!).type, 'input_request')
  } finally {
    client.disconnect()
  }
})

test('a missing input ACK closes the connection instead of continuing to send motion', async () => {
  const socket = new FakeSocket()
  const acks: TeleopAck[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop', socketFactory: () => socket, inputTimeoutMs: 5, onAck: ack => acks.push(ack),
  })
  client.connect()
  socket.open()
  try {
    client.move({ vxMps: 0.5, deadman: true })
    await new Promise(resolve => setTimeout(resolve, 20))
    assert.equal(client.isOpen(), false)
    assert.equal(acks.at(-1)?.error, 'input_timeout')
    assert.equal(socket.sent.length, 1)
  } finally {
    client.disconnect()
  }
})

test('teleop websocket disconnect sends deadman false zero hold before closing', () => {
  const sockets: FakeSocket[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop',
    socketFactory: () => {
      const socket = new FakeSocket()
      sockets.push(socket)
      return socket
    },
  })

  client.connect()
  sockets[0].open()
  client.disconnect()

  const hold = JSON.parse(sockets[0].sent.at(-1) ?? '{}') as Record<string, unknown>
  assert.equal(hold.type, 'velocity')
  assert.equal(hold.deadman, false)
  assert.equal(hold.vx_mps, 0)
  assert.equal(hold.vy_mps, 0)
  assert.equal(hold.yaw_rps, 0)
})

test('teleop websocket client reconnects after a server-side close', async () => {
    const sockets: FakeSocket[] = []
    const states: string[] = []
    const client = new TeleopWsClient({
      url: '/ws/teleop',
      reconnectDelayMs: 0,
      socketFactory: () => {
        const socket = new FakeSocket()
        sockets.push(socket)
        return socket
      },
      onState: state => states.push(state),
    })

    client.connect()
    sockets[0].open()
    sockets[0].close()
    await new Promise(resolve => setTimeout(resolve, 5))

    assert.equal(sockets.length, 2)
    sockets[1].open()
    assert.deepEqual(states.slice(0, 5), ['connecting', 'open', 'closed', 'connecting', 'open'])
  client.disconnect()
})

test('teleop websocket client retries occupancy without sending motion', async () => {
  const sockets: FakeSocket[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop',
    reconnectDelayMs: 0,
    socketFactory: () => {
      const socket = new FakeSocket()
      sockets.push(socket)
      return socket
    },
  })

  client.connect()
  sockets[0].open()
  sockets[0].receive({ type: 'control_rejected', error: 'control_in_use' })
  sockets[0].close()
  await new Promise(resolve => setTimeout(resolve, 5))

  assert.equal(sockets.length, 2)
  assert.equal(sockets[1].sent.length, 0)
  client.disconnect()
})

test('teleop panel is gated by bootstrap teleop_ws and teleop products', () => {
  assert.match(panelSource, /api\.fetchAppBootstrap\(\)/)
  assert.match(panelSource, /BOOTSTRAP_RETRY_MS/)
  assert.match(panelSource, /teleopPathFromBootstrap/)
  assert.match(panelSource, /new Set<ProductName>\(\['teleop', 'teleop_avoid', 'map'\]\)/)
  assert.match(panelSource, /正在准备遥控/)
  assert.match(panelSource, /teleopLimitsFromBootstrap/)
  assert.match(panelSource, /linear_mps/)
  assert.match(panelSource, /yaw_rad_s/)
  assert.match(sceneViewSource, /<TeleopPanel[\s\S]*sseState=\{sseState\}/)
})

test('mapping admits the actual teleop panel while navigation stays excluded', () => {
  const declarations = keyboardAst.statements.filter(node =>
    (ts.isFunctionDeclaration(node) && node.name?.text === 'currentProduct')
    || (ts.isVariableStatement(node) && node.declarationList.declarations.some(declaration =>
      ts.isIdentifier(declaration.name) && declaration.name.text === 'TELEOP_PRODUCTS')),
  )
  assert.equal(declarations.length, 2)
  const code = ts.transpileModule(
    declarations.map(node => node.getText(keyboardAst)).join('\n') + '\ncurrentProduct;',
    { compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS } },
  ).outputText
  const resolve = runInNewContext(code) as (state: { session?: { product: string } }) => string | null
  for (const product of ['map', 'teleop', 'teleop_avoid']) {
    assert.equal(resolve({ session: { product } }), product)
  }
  assert.equal(resolve({ session: { product: 'nav' } }), null)
  assert.equal(resolve({}), null)
  assert.match(sceneViewSource, /currentProduct === 'map'\) && \([\s\S]*?setTeleopMode/)
  assert.match(panelSource, /建图直接遥控/)
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
  const h = keyboardHarness()
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
      assert.equal(h.state.manual, false)
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
  const h = keyboardHarness()
  const ownButton = new FocusElement('button', h.panel)
  const menuTrigger = new FocusElement('summary', h.body)
  h.onKeyDown(keyEvent('w', ownButton))
  h.onKeyDown(keyEvent('m', ownButton))
  h.quiesceForInteraction({ target: menuTrigger })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.blockedKeysRef.current.has('w'), true)
  assert.equal(h.state.manual, false)
  assert.equal(h.state.holds, 1)
  h.onKeyDown(keyEvent('w', ownButton, true))
  h.onKeyDown(keyEvent('m', ownButton, true))
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.manual, false)
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
  const h = keyboardHarness()
  const menuButton = new FocusElement('button', h.body)
  h.onKeyDown(keyEvent('w', menuButton))
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyUp(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.has('w'), true)
})

test('same-origin websocket URL is resolved without treating queued intent as motor truth', () => {
  const url = resolveTeleopWsUrl('/ws/teleop', 'client x')
  assert.match(url, /^ws:\/\/127\.0\.0\.1\/ws\/teleop\?/)
  assert.match(url, /client_id=client\+x/)
  assert.match(url, /source=web_scene/)
})

test('legacy map viewer cannot bypass the claimed teleop websocket', () => {
  assert.doesNotMatch(legacyMapViewerSource, /\/api\/v1\/cmd_vel/)
  assert.doesNotMatch(legacyMapViewerSource, /addEventListener\(['"]keydown/)
})

test('entering keyboard mode with a key already held cannot start movement', () => {
  const h = keyboardHarness()
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyUp(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.keysRef.current.has('w'), true)
})

test('Escape clears active motion before exiting keyboard mode', () => {
  const h = keyboardHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyDown(keyEvent('Escape', h.panel))
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
})

test('browser shortcuts and IME composition cannot become motion commands', () => {
  for (const modifier of ['ctrlKey', 'metaKey', 'altKey', 'isComposing']) {
    const h = keyboardHarness()
    const event = { ...keyEvent('w', h.panel), [modifier]: true }
    h.onKeyDown(event)
    assert.equal(h.keysRef.current.size, 0, modifier)
    assert.equal(event.defaultPrevented, false, modifier)
    h.onKeyDown(keyEvent('w', h.panel, true))
    assert.equal(h.keysRef.current.size, 0, `${modifier} release cannot resume a held key`)
  }
})

test('pressing a browser modifier while driving immediately clears the held intent', () => {
  const h = keyboardHarness()
  h.onKeyDown(keyEvent('w', h.panel))
  h.onKeyDown({ ...keyEvent('Control', h.panel), ctrlKey: true })
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.state.holds, 1)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.keysRef.current.size, 0)
})

test('a direction pressed before connection only connects and cannot be replayed on open', () => {
  const h = keyboardHarness(true, false)
  h.onKeyDown(keyEvent('w', h.panel))
  assert.equal(h.state.connects, 1)
  assert.equal(h.keysRef.current.size, 0)
  h.onKeyDown(keyEvent('w', h.panel, true))
  assert.equal(h.state.connects, 1)
  assert.equal(h.keysRef.current.size, 0)
})

test('disconnected telemetry blocks motion despite cached teleop configuration and an open socket', () => {
  const h = keyboardHarness(false)
  for (const key of ['w', 'a', 's', 'd', 'q', 'e', 'm']) h.onKeyDown(keyEvent(key, h.panel))
  assert.equal(h.keysRef.current.size, 0)
  assert.equal(h.manualModeRef.current, false)
  assert.equal(h.state.connects, 0)
})

test('teleop status cannot advertise keyboard control while telemetry is disconnected', () => {
  const context = {
    sseState: { connected: false }, enabled: false, connectionReady: true,
    resumeRequired: false, bootstrapError: null, precisionMode: false,
    sessionActive: false, openState: 'open',
  }
  assert.equal(componentValue('controlStatus', context), '连接已断开')
  assert.equal(componentValue('controlStatus', { ...context, resumeRequired: true }), '连接已断开')
  assert.equal(componentValue('controlStatus', {
    ...context, sseState: { connected: true }, enabled: true, connectionReady: false,
  }), '按键开始')
})

test('opening shortcut help clears held motion and requires release before another move', () => {
  const h = keyboardHarness()
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
  const h = keyboardHarness()
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
  const h = keyboardHarness()
  const menu = Object.assign(new FocusElement('details', h.panel), { open: false })
  const otherControl = new FocusElement('summary', h.body)
  h.document.activeElement = otherControl
  h.onShortcutToggle({ currentTarget: menu })
  assert.equal(h.document.activeElement, otherControl)
  h.onKeyDown(keyEvent('w', h.document.activeElement))
  assert.equal(h.keysRef.current.size, 0)
})

test('an ACK blackhole stops sending after 350 ms even while socket close never completes', async () => {
  class BlackholeSocket extends FakeSocket {
    closeCalls = 0
    close(): void {
      this.closeCalls += 1
      this.readyState = 2
    }
  }
  const socket = new BlackholeSocket()
  const acks: TeleopAck[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop', socketFactory: () => socket, onAck: ack => acks.push(ack),
  })
  client.connect()
  socket.open()
  try {
    client.move({ vxMps: 0.5, deadman: true })
    const input = JSON.parse(socket.sent[0])
    socket.receive({ type: 'input_ack', request_id: input.request_id, input_window: 'fresh' })
    const movement = client.move({ vxMps: 0.5, deadman: true })
    for (let i = 0; i < 150; i++) client.move({ vxMps: 0.5, deadman: true })
    assert.equal(socket.sent.length, 2)
    await new Promise(resolve => setTimeout(resolve, 400))
    assert.equal(socket.closeCalls, 1)
    assert.equal(socket.readyState, 2)
    assert.equal(client.isOpen(), false)
    assert.equal(acks.at(-1)?.error, 'input_timeout')
    socket.receive({ type: 'ingress_ack', request_id: movement, input_window: 'late' })
    for (let i = 0; i < 150; i++) client.move({ vxMps: 0.5, deadman: true })
    assert.equal(socket.sent.length, 2, 'neither a delayed ACK nor a held key can send while closing')
  } finally {
    client.disconnect()
  }
})

test('a missing hold ACK blocks new movement and reconnection never replays the previous velocity', async () => {
  const sockets: FakeSocket[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop', inputTimeoutMs: 5, reconnectDelayMs: 0,
    socketFactory: () => { const socket = new FakeSocket(); sockets.push(socket); return socket },
  })
  client.connect()
  sockets[0].open()
  try {
    client.move({ vxMps: 0.5, deadman: true })
    const input = JSON.parse(sockets[0].sent[0])
    sockets[0].receive({ type: 'input_ack', request_id: input.request_id, input_window: 'fresh' })
    const movement = client.move({ vxMps: 0.5, deadman: true })
    client.hold()
    const hold = JSON.parse(sockets[0].sent.at(-1)!)
    assert.equal(hold.deadman, false)
    assert.deepEqual([hold.vx_mps, hold.vy_mps, hold.yaw_rps], [0, 0, 0])
    sockets[0].receive({ type: 'ingress_ack', request_id: movement, input_window: 'obsolete' })
    for (let i = 0; i < 150; i++) client.move({ vxMps: 0.5, deadman: true })
    assert.equal(sockets[0].sent.length, 3)
    await new Promise(resolve => setTimeout(resolve, 25))
    assert.equal(sockets.length, 2)
    sockets[1].open()
    sockets[0].receive({ type: 'control_ack', request_id: hold.request_id })
    assert.equal(sockets[1].sent.length, 0)
    client.move({ vxMps: 0.2, deadman: true })
    assert.equal(JSON.parse(sockets[1].sent[0]).type, 'input_request')
  } finally {
    client.disconnect()
  }
})

test('clearing input on connection failure requires a physical release before movement can restart', () => {
  const h = keyboardHarness()
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
