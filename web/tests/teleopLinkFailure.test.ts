import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

import {
  TeleopWsClient,
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
  keyboardDeclarations.join('\n') + '\n({ blocksTeleopKeyboard, clearInputIntent, quiesceForInteraction, onKeyDown, onKeyUp });',
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

test('occupied during old connection cleanup retries without replaying motion', async () => {
  const sockets: FakeSocket[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop', reconnectDelayMs: 0,
    socketFactory: () => { const socket = new FakeSocket(); sockets.push(socket); return socket },
  })
  client.connect()
  sockets[0].open()
  sockets[0].receive({ type: 'control_rejected', error: 'control_in_use' })
  sockets[0].close()
  await new Promise(resolve => setTimeout(resolve, 10))
  try {
    assert.equal(sockets.length, 2)
    sockets[1].open()
    assert.equal(sockets[1].sent.length, 0)
  } finally { client.disconnect() }
})

test('idle connection probes detect a silent broken cable without sending velocity', async () => {
  const socket = new FakeSocket()
  const client = new TeleopWsClient({ url: '/ws/teleop', inputTimeoutMs: 5, socketFactory: () => socket })
  client.connect()
  socket.open()
  try {
    await new Promise(resolve => setTimeout(resolve, 1100))
    assert.equal(client.isOpen(), false)
    assert.equal(socket.sent.length, 1)
    assert.equal(JSON.parse(socket.sent[0]).type, 'input_request')
  } finally { client.disconnect() }
})

test('reconnect does not wait for a broken socket close handshake', async () => {
  class StalledCloseSocket extends FakeSocket {
    close(): void { this.readyState = 2 }
  }
  const sockets: StalledCloseSocket[] = []
  const client = new TeleopWsClient({
    url: '/ws/teleop', inputTimeoutMs: 5, reconnectDelayMs: 0,
    socketFactory: () => { const socket = new StalledCloseSocket(); sockets.push(socket); return socket },
  })
  client.connect()
  sockets[0].open()
  client.move({ vxMps: 0.2, deadman: true })
  await new Promise(resolve => setTimeout(resolve, 25))
  try {
    assert.equal(sockets.length, 2)
    sockets[1].open()
    sockets[0].onclose?.({ type: 'close' } as CloseEvent)
    assert.equal(client.isOpen(), true)
    assert.equal(sockets[1].sent.length, 0)
  } finally { client.disconnect() }
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
