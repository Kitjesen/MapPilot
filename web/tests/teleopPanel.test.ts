import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
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

const sceneViewSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const panelSource = readFileSync(
  new URL('../src/components/TeleopPanel.tsx', import.meta.url),
  'utf8',
)
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
  const request = client.move({
    vxMps: 0.4,
    vyMps: -0.2,
    yawRps: 0.3,
    deadman: true,
    manualMode: true,
  })
  assert.ok(request)
  const payload = JSON.parse(sockets[0].sent[0]) as Record<string, unknown>
  assert.equal(payload.type, 'velocity')
  assert.equal(payload.deadman, true)
  assert.equal(payload.manual_mode, true)
  assert.equal(payload.vx_mps, 0.4)
  assert.equal(payload.vy_mps, -0.2)
  assert.equal(payload.yaw_rps, 0.3)
  assert.equal(payload.request_id, request)
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

test('teleop websocket client does not retry a connection owned by another browser', async () => {
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

  assert.equal(sockets.length, 1)
  client.disconnect()
})

test('teleop panel is gated by bootstrap teleop_ws and teleop products', () => {
  assert.match(panelSource, /api\.fetchAppBootstrap\(\)/)
  assert.match(panelSource, /BOOTSTRAP_RETRY_MS/)
  assert.match(panelSource, /teleopPathFromBootstrap/)
  assert.match(panelSource, /new Set<ProductName>\(\['teleop', 'teleop_avoid'\]\)/)
  assert.match(panelSource, /bootstrap 未声明 teleop_ws/)
  assert.match(panelSource, /teleopLimitsFromBootstrap/)
  assert.match(panelSource, /linear_mps/)
  assert.match(panelSource, /yaw_rad_s/)
  assert.match(sceneViewSource, /<TeleopPanel[\s\S]*sseState=\{sseState\}/)
})

test('teleop panel uses direct hold-to-move keys and explicitly displays gateway/native ack semantics', () => {
  assert.match(panelSource, /直接按住 W\/S 前后/)
  assert.match(panelSource, /Shift 为 40% 精细模式/)
  assert.match(panelSource, /Space 立即保持/)
  assert.match(panelSource, /const SEND_INTERVAL_MS = 20/)
  assert.match(panelSource, /const PRECISION_SCALE = 0\.4/)
  assert.match(panelSource, /sendHold/)
  assert.match(panelSource, /onClick=\{quiesceInput\}[\s\S]*保持/)
  assert.match(panelSource, /visibilitychange/)
  assert.match(panelSource, /manualMode: manualModeRef\.current/)
  assert.match(panelSource, /按住 M/)
  assert.match(panelSource, /按住脱困/)
  assert.match(panelSource, /final_cmd_vel_confirmed/)
  assert.match(panelSource, /motor_confirmed/)
  assert.match(panelSource, /clearInputIntent\(\)[\s\S]*client\.hold\('rejected_input'\)/)
  assert.match(panelSource, /resume_required/)
  assert.match(panelSource, /api\.resumeNavigation\(\)/)
  assert.match(panelSource, /恢复控制/)
  assert.match(panelSource, /断开/)
  assert.doesNotMatch(panelSource, /LEASE_RENEW_INTERVAL_MS|heartbeat/)
  assert.doesNotMatch(panelSource, /keyboardDeadman/)
})

test('teleop panel clears latched input on focus loss and ignores editable targets', () => {
  assert.match(panelSource, /function isEditableTarget/)
  assert.match(panelSource, /keysRef\.current\.clear\(\)/)
  assert.match(panelSource, /blockedKeysRef\.current\.add\(key\)/)
  assert.match(panelSource, /blockedKeysRef\.current\.has\(key\)/)
  assert.match(panelSource, /blockedKeysRef\.current\.delete\(key\)/)
  assert.match(panelSource, /directionsRef\.current\.clear\(\)/)
  assert.match(panelSource, /setPrecisionMode\(false\)/)
  assert.match(panelSource, /if \(isEditableTarget\(event\.target\)\) return/)
  assert.match(panelSource, /event\.code === 'Space'/)
  assert.match(panelSource, /keysRef\.current\.size === 0 && directionsRef\.current\.size === 0/)
  assert.match(panelSource, /onPointerCancel=/)
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
