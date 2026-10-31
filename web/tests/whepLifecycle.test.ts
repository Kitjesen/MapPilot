import assert from 'node:assert/strict'
import test from 'node:test'

import {
  WHEPConnectionManager,
  type WHEPConnectionCallbacks,
  type WHEPRuntime,
} from '../src/hooks/useWHEP.ts'

class Deferred<T> {
  promise: Promise<T>
  resolve!: (value: T) => void
  reject!: (reason?: unknown) => void

  constructor() {
    this.promise = new Promise<T>((resolve, reject) => {
      this.resolve = resolve
      this.reject = reject
    })
  }
}

class FakeTrack {
  stopped = false
  stop() { this.stopped = true }
}

class FakePeerConnection extends EventTarget {
  connectionState: RTCPeerConnectionState = 'new'
  iceGatheringState: RTCIceGatheringState = 'complete'
  localDescription: RTCSessionDescription | null = null
  remoteDescription: RTCSessionDescription | null = null
  closed = false
  readonly track = new FakeTrack()
  readonly offer = new Deferred<RTCSessionDescriptionInit>()

  addTransceiver() {}
  createOffer() { return this.offer.promise }
  async setLocalDescription(description: RTCLocalSessionDescriptionInit) {
    this.localDescription = { type: 'offer', sdp: description.sdp ?? '' } as RTCSessionDescription
  }
  async setRemoteDescription(description: RTCSessionDescriptionInit) {
    this.remoteDescription = description as RTCSessionDescription
  }
  getReceivers() { return [{ track: this.track }] as RTCRtpReceiver[] }
  close() {
    this.closed = true
    this.connectionState = 'closed'
  }
  resolveOffer(sdp = 'offer') { this.offer.resolve({ type: 'offer', sdp }) }
  setConnectionState(state: RTCPeerConnectionState) {
    this.connectionState = state
    this.dispatchEvent(new Event('connectionstatechange'))
  }
}

type FetchCall = { url: string; init?: RequestInit }

function response(status: number, body = '', location?: string, etag?: string): Response {
  const headers = new Headers()
  if (location) headers.set('location', location)
  if (etag) headers.set('etag', etag)
  return {
    ok: status >= 200 && status < 300,
    status,
    url: 'http://gateway.test/api/v1/webrtc/whep',
    headers,
    text: async () => body,
  } as Response
}

function setup(fetchHandler: (url: string, init?: RequestInit) => Promise<Response>) {
  const peers: FakePeerConnection[] = []
  const calls: FetchCall[] = []
  const runtime: WHEPRuntime = {
    createPeerConnection: () => {
      const pc = new FakePeerConnection()
      peers.push(pc)
      return pc as unknown as RTCPeerConnection
    },
    fetch: async (input, init) => {
      const url = String(input)
      calls.push({ url, init })
      return fetchHandler(url, init)
    },
    setTimeout: (callback, delay) => setTimeout(callback, delay),
    clearTimeout: handle => clearTimeout(handle),
  }
  return { runtime, peers, calls }
}

function callbacks(events: string[]): WHEPConnectionCallbacks {
  return {
    onConnecting: () => events.push('connecting'),
    onStream: () => events.push('stream'),
    onConnected: () => events.push('connected'),
    onError: error => events.push(`error:${error}`),
  }
}

test('an old offer cannot signal or update state after the source changes', async () => {
  const { runtime, peers, calls } = setup(async () => response(201, 'answer'))
  const oldEvents: string[] = []
  const newEvents: string[] = []
  const oldManager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'old', runtime,
    callbacks: callbacks(oldEvents), retryMinMs: 1, retryMaxMs: 1,
  })
  oldManager.start()
  assert.equal(peers.length, 1)

  oldManager.stop()
  const newManager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'new', runtime,
    callbacks: callbacks(newEvents), retryMinMs: 1, retryMaxMs: 1,
  })
  newManager.start()
  peers[1].resolveOffer('new-offer')
  await newManager.settled()

  peers[0].resolveOffer('old-offer')
  await oldManager.settled()

  assert.equal(peers[0].closed, true)
  assert.deepEqual(calls.filter(call => call.init?.method === 'POST').map(call => call.url), [
    '/api/v1/webrtc/whep?src=new',
  ])
  assert.deepEqual(oldEvents, ['connecting'])
  assert.deepEqual(newEvents, ['connecting'])
  newManager.stop()
})

test('a late WHEP creation response is deleted after its source is replaced', async () => {
  const created = new Deferred<Response>()
  let postSignal: AbortSignal | null = null
  const { runtime, peers, calls } = setup(async (_url, init) => {
    if (init?.method === 'DELETE') return response(204)
    postSignal = init?.signal as AbortSignal
    return created.promise
  })
  const manager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'old', runtime,
    callbacks: callbacks([]), retryMinMs: 1, retryMaxMs: 1,
  })
  manager.start()
  peers[0].resolveOffer()
  await new Promise(resolve => setTimeout(resolve, 0))
  await manager.stop()

  assert.equal(postSignal?.aborted, false)
  created.resolve(response(201, 'answer', '/api/v1/webrtc/sessions/late'))
  await new Promise(resolve => setTimeout(resolve, 0))
  assert.ok(calls.some(call =>
    call.url === '/api/v1/webrtc/sessions/late' && call.init?.method === 'DELETE'))
})

test('a failed connection closes promptly and reconnects with bounded backoff', async () => {
  let posts = 0
  const { runtime, peers } = setup(async (_url, init) => {
    if (init?.method === 'POST' && ++posts === 1) return response(500)
    return response(201, 'answer')
  })
  const events: string[] = []
  const manager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'cam', runtime,
    callbacks: callbacks(events), retryMinMs: 1, retryMaxMs: 2,
  })
  manager.start()
  peers[0].resolveOffer()
  await manager.settled()
  assert.equal(peers[0].closed, true)

  await new Promise(resolve => setTimeout(resolve, 5))
  assert.equal(peers.length, 2)
  peers[1].resolveOffer()
  await manager.settled()
  peers[1].setConnectionState('connected')
  assert.ok(events.includes('error:signalling'))
  assert.ok(events.includes('connected'))
  manager.stop()
})

test('a WHEP offer that never receives an answer times out and retries', async () => {
  const { runtime, peers } = setup(async (_url, init) => new Promise<Response>((_resolve, reject) => {
    init?.signal?.addEventListener(
      'abort',
      () => reject(new DOMException('aborted', 'AbortError')),
      { once: true },
    )
  }))
  const events: string[] = []
  const manager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'cam', runtime,
    callbacks: callbacks(events), postTimeoutMs: 5,
    retryMinMs: 50, retryMaxMs: 50,
  })
  manager.start()
  peers[0].resolveOffer()
  await new Promise(resolve => setTimeout(resolve, 15))

  assert.equal(peers[0].closed, true)
  assert.ok(events.includes('error:signalling'))
  assert.equal(peers.length, 1)
  await manager.stop()
})

test('complete server ICE gathering may exceed five seconds without replacing the session', async t => {
  t.mock.timers.enable({ apis: ['setTimeout'] })
  const created = new Deferred<Response>()
  const posted = new Deferred<void>()
  let postSignal: AbortSignal | null = null
  const { runtime, peers } = setup(async (_url, init) => {
    if (init?.method === 'DELETE') return response(204)
    postSignal = init?.signal as AbortSignal
    posted.resolve()
    return created.promise
  })
  const events: string[] = []
  const manager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'cam', runtime,
    callbacks: callbacks(events),
  })
  manager.start()
  peers[0].resolveOffer()
  await posted.promise

  t.mock.timers.tick(6000)
  assert.equal(postSignal?.aborted, false)
  assert.equal(peers[0].closed, false)
  assert.equal(peers.length, 1)
  assert.deepEqual(events, ['connecting'])

  created.resolve(response(201, 'answer', '/api/v1/webrtc/sessions/ready'))
  await manager.settled()
  assert.equal(peers[0].remoteDescription?.sdp, 'answer')
  await manager.stop()
})

test('stop cancels pending reconnects and suppresses later callbacks', async () => {
  const { runtime, peers } = setup(async () => response(500))
  const events: string[] = []
  const manager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'cam', runtime,
    callbacks: callbacks(events), retryMinMs: 20, retryMaxMs: 20,
  })
  manager.start()
  peers[0].resolveOffer()
  await manager.settled()
  manager.stop()
  const countAtStop = events.length

  await new Promise(resolve => setTimeout(resolve, 30))
  assert.equal(peers.length, 1)
  assert.equal(events.length, countAtStop)
  assert.equal(peers[0].track.stopped, true)
})

test('closing an established WHEP session deletes its advertised resource', async () => {
  const { runtime, peers, calls } = setup(async (_url, init) => {
    if (init?.method === 'DELETE') return response(204)
    return response(201, 'answer', '/api/v1/webrtc/whep/session/abc', '"session-abc"')
  })
  const manager = new WHEPConnectionManager({
    whepPath: '/api/v1/webrtc/whep', streamSrc: 'cam', runtime,
    callbacks: callbacks([]), retryMinMs: 1, retryMaxMs: 1,
  })
  manager.start()
  peers[0].resolveOffer()
  await manager.settled()
  await manager.stop()

  assert.ok(calls.some(call =>
    call.url === '/api/v1/webrtc/whep/session/abc'
      && call.init?.method === 'DELETE'
      && new Headers(call.init.headers).get('if-match') === '"session-abc"'))
  assert.equal(peers[0].track.stopped, true)
  assert.equal(peers[0].closed, true)
})
