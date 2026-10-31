import assert from 'node:assert/strict'
import test from 'node:test'

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

for (const ackType of ['input_ack', 'ingress_ack']) {
  test(`${ackType} windows expire while idle without rejecting the next physical key`, t => {
    let now = 1000
    t.mock.method(performance, 'now', () => now)
    const socket = new FakeSocket()
    const acks: TeleopAck[] = []
    const client = new TeleopWsClient({ url: '/ws/teleop', socketFactory: () => socket, onAck: ack => acks.push(ack) })
    client.connect()
    socket.open()
    try {
      client.move({ vxMps: 0.5, deadman: true })
      let request = JSON.parse(socket.sent.at(-1)!)
      if (ackType === 'ingress_ack') {
        socket.receive({ type: 'input_ack', request_id: request.request_id, input_window: 'first' })
        client.move({ vxMps: 0.5, deadman: true })
        request = JSON.parse(socket.sent.at(-1)!)
      }
      // Network time belongs to the same budget; receipt must not restart it.
      now = 1250
      socket.receive({ type: ackType, request_id: request.request_id, input_window: 'idle' })
      now = 1400
      client.move({ vxMps: 0.2, deadman: true })
      const refresh = JSON.parse(socket.sent.at(-1)!)
      assert.equal(refresh.type, 'input_request', 'never send velocity with the expired window')
      const sentBeforeAck = socket.sent.length
      socket.receive({ type: 'input_ack', request_id: refresh.request_id, input_window: 'fresh' })
      assert.equal(socket.sent.length, sentBeforeAck, 'an ACK must not replay velocity')
      client.move({ vxMps: 0.2, deadman: true })
      assert.equal(JSON.parse(socket.sent.at(-1)!).input_window, 'fresh')
      assert.equal(acks.some(ack => ack.type === 'control_rejected'), false)
    } finally { client.disconnect() }
  })
}

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

test('reconnect does not wait for a broken socket close handshake', { timeout: 1000 }, async () => {
  class StalledCloseSocket extends FakeSocket {
    close(): void { this.readyState = 2 }
  }
  const sockets: StalledCloseSocket[] = []
  let onReconnect!: () => void
  const reconnected = new Promise<void>(resolve => { onReconnect = resolve })
  const client = new TeleopWsClient({
    url: '/ws/teleop', inputTimeoutMs: 5, reconnectDelayMs: 0,
    socketFactory: () => {
      const socket = new StalledCloseSocket()
      sockets.push(socket)
      if (sockets.length === 2) onReconnect()
      return socket
    },
  })
  client.connect()
  sockets[0].open()
  client.move({ vxMps: 0.2, deadman: true })
  try {
    await reconnected
    assert.equal(sockets.length, 2)
    sockets[1].open()
    sockets[0].onclose?.({ type: 'close' } as CloseEvent)
    assert.equal(client.isOpen(), true)
    assert.equal(sockets[1].sent.length, 0)
  } finally { client.disconnect() }
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

test('a missing hold ACK blocks new movement and reconnection never replays the previous velocity', { timeout: 1000 }, async () => {
  const sockets: FakeSocket[] = []
  let onReconnect!: () => void
  const reconnected = new Promise<void>(resolve => { onReconnect = resolve })
  const client = new TeleopWsClient({
    url: '/ws/teleop', inputTimeoutMs: 5, reconnectDelayMs: 0,
    socketFactory: () => {
      const socket = new FakeSocket()
      sockets.push(socket)
      if (sockets.length === 2) onReconnect()
      return socket
    },
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
    await reconnected
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

test('same-origin websocket URL is resolved without treating queued intent as motor truth', () => {
  const url = resolveTeleopWsUrl('/ws/teleop', 'client x')
  assert.match(url, /^ws:\/\/127\.0\.0\.1\/ws\/teleop\?/)
  assert.match(url, /client_id=client\+x/)
  assert.match(url, /source=web_scene/)
})
