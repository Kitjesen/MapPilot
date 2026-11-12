import assert from 'node:assert/strict'
import test, { type TestContext } from 'node:test'
import { startBotBlink } from '../src/components/agentConversation/botMotion.ts'

function environment(t: TestContext, run: (env: ReturnType<typeof createEnvironment>) => void) {
  t.mock.timers.enable({ apis: ['setTimeout', 'Date'] })
  t.mock.method(Math, 'random', () => 0)
  const originalDocument = Object.getOwnPropertyDescriptor(globalThis, 'document')
  const originalWindow = Object.getOwnPropertyDescriptor(globalThis, 'window')
  const env = createEnvironment()
  Object.defineProperty(globalThis, 'document', { configurable: true, value: env.document })
  Object.defineProperty(globalThis, 'window', { configurable: true, value: { matchMedia: () => env.media } })
  let cleanup: (() => void) | undefined
  try {
    cleanup = startBotBlink(env.root as unknown as SVGSVGElement, env.lids as unknown as SVGGElement)
    run({ ...env, cleanup })
  } finally {
    cleanup?.()
    if (originalDocument) Object.defineProperty(globalThis, 'document', originalDocument)
    else Reflect.deleteProperty(globalThis, 'document')
    if (originalWindow) Object.defineProperty(globalThis, 'window', originalWindow)
    else Reflect.deleteProperty(globalThis, 'window')
  }
}

function createEnvironment() {
  const document = Object.assign(new EventTarget(), { hidden: false })
  const media = Object.assign(new EventTarget(), { matches: false })
  const root = { dataset: {} as Record<string, string> }
  const blinks: { time: number; frames: Keyframe[]; duration: number; cancelled: boolean }[] = []
  const lids = { animate(frames: Keyframe[], options: { duration: number }) {
    const blink = { time: Date.now(), frames, duration: options.duration, cancelled: false }
    blinks.push(blink)
    return { cancel() { blink.cancelled = true } }
  } }
  return { document, media, root, lids, blinks, cleanup() {} }
}

test('authored phrases keep paired blinks separate from head and gaze', t => {
  environment(t, env => {
    t.mock.timers.tick(2300)
    assert.equal(env.blinks.length, 1)
    assert.equal(env.blinks[0].duration, 240)
    assert.deepEqual(env.blinks[0].frames.map(frame => frame.transform), ['scaleY(1)', 'scaleY(.06)', 'scaleY(1)'])
    t.mock.timers.tick(350)
    assert.equal(env.blinks.length, 2)
    assert.equal(env.blinks[1].time - env.blinks[0].time, 350)
  })
})

test('each deck plays all four phrases without repeating across deck boundaries', t => {
  environment(t, env => {
    const timings: number[][] = []
    // With the deterministic shuffle, deck one is 1,2,3,0 and deck two is 1,2,3,0.
    const phrases = [
      { duration: 9400, blinks: [2300, 2650, 7400] },
      { duration: 10300, blinks: [3800, 8200] },
      { duration: 8700, blinks: [1900, 6300, 6650] },
      { duration: 8100, blinks: [1400, 5700] },
    ]
    for (const phrase of [...phrases, ...phrases]) {
      const start = Date.now()
      const count = env.blinks.length
      let elapsed = 0
      for (const blink of phrase.blinks) {
        t.mock.timers.tick(blink - elapsed)
        elapsed = blink
      }
      timings.push(env.blinks.slice(count).map(blink => blink.time - start))
      t.mock.timers.tick(phrase.duration - elapsed)
    }
    assert.deepEqual(timings, [...phrases, ...phrases].map(phrase => phrase.blinks))
  })
})

test('visibility and reduced motion cancel blinking and resume without a catch-up burst', t => {
  environment(t, env => {
    t.mock.timers.tick(2300)
    env.document.hidden = true
    env.document.dispatchEvent(new Event('visibilitychange'))
    assert.equal(env.root.dataset.paused, 'true')
    assert.equal(env.blinks.at(-1)?.cancelled, true)
    t.mock.timers.tick(60000)
    assert.equal(env.blinks.length, 1)
    env.document.hidden = false
    env.media.matches = true
    env.document.dispatchEvent(new Event('visibilitychange'))
    t.mock.timers.tick(60000)
    assert.equal(env.blinks.length, 1)
    env.media.matches = false
    env.media.dispatchEvent(new Event('change'))
    assert.equal(env.root.dataset.paused, 'false')
    t.mock.timers.tick(100)
    assert.equal(env.blinks.length, 1)
    t.mock.timers.tick(3700)
    assert.equal(env.blinks.length, 2)
  })
})

test('a freshly shuffled deck cannot start with the phrase that just ended', t => {
  environment(t, env => {
    for (const duration of [9400, 10300, 8700]) t.mock.timers.tick(duration)
    t.mock.method(Math, 'random', () => .999)
    t.mock.timers.tick(8100)
    const count = env.blinks.length
    t.mock.timers.tick(1400)
    assert.equal(env.blinks.length, count)
    t.mock.timers.tick(900)
    assert.equal(env.blinks.length, count + 1)
  })
})

test('unmount removes timers and both event listeners', t => {
  environment(t, env => {
    env.cleanup()
    env.document.dispatchEvent(new Event('visibilitychange'))
    env.media.dispatchEvent(new Event('change'))
    t.mock.timers.tick(60000)
    assert.equal(env.blinks.length, 0)
  })
})
