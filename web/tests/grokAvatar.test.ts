import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test, { type TestContext } from 'node:test'
import ts from 'typescript'
import * as engine from '../src/vendor/grok-avatar/src/index.mjs'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/GrokAvatar.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText

class Events extends EventTarget {
  listeners = new Map<string, Set<EventListenerOrEventListenerObject>>()
  override addEventListener(type: string, listener: EventListenerOrEventListenerObject | null, options?: AddEventListenerOptions | boolean) {
    if (listener) {
      if (!this.listeners.has(type)) this.listeners.set(type, new Set())
      this.listeners.get(type)!.add(listener)
    }
    super.addEventListener(type, listener, options)
  }
  override removeEventListener(type: string, listener: EventListenerOrEventListenerObject | null, options?: EventListenerOptions | boolean) {
    if (listener) this.listeners.get(type)?.delete(listener)
    super.removeEventListener(type, listener, options)
  }
  count() { return [...this.listeners.values()].reduce((sum, listeners) => sum + listeners.size, 0) }
}

class SvgNode {
  tagName: string
  ownerDocument: { createElementNS: (ns: string, tag: string) => SvgNode }
  children: SvgNode[] = []
  attributes: { name: string; value: string }[] = []
  html = ''
  constructor(tagName: string, ownerDocument: { createElementNS: (ns: string, tag: string) => SvgNode }) {
    this.tagName = tagName
    this.ownerDocument = ownerDocument
  }
  get firstChild() { return this.children[0] ?? null }
  set innerHTML(html: string) { this.html = html; this.children = [] }
  get innerHTML() { return this.html }
  setAttribute(name: string, value: string) { this.attributes.push({ name, value }) }
  append(node: SvgNode) { this.children.push(node) }
  replaceChildren(...nodes: SvgNode[]) { this.children = nodes; this.html = '' }
  querySelectorAll(): SvgNode[] { return this.children.flatMap(child => [child, ...child.querySelectorAll()]) }
  getBoundingClientRect() { return { left: 0, top: 0, width: 48, height: 48 } }
}

function harness(t: TestContext, initial: Record<string, unknown> = {}) {
  const media = Object.assign(new Events(), { matches: false })
  const window = Object.assign(new Events(), { matchMedia: () => media })
  const document = Object.assign(new Events(), { hidden: false, createElementNS: (_ns: string, tag: string): SvgNode => new SvgNode(tag, document) })
  const root = new SvgNode('span', document)
  const raf = new Map<number, FrameRequestCallback>()
  let rafId = 0
  let time = 0
  let observers = 0
  class Observer {
    constructor() { observers++ }
    observe() {}
    disconnect() { observers-- }
  }
  const globals: Record<string, unknown> = { window, document, performance: { now: () => time },
    requestAnimationFrame: (callback: FrameRequestCallback) => { raf.set(++rafId, callback); return rafId },
    cancelAnimationFrame: (id: number) => raf.delete(id), IntersectionObserver: Observer, ResizeObserver: Observer }
  const original = Object.fromEntries(Object.keys(globals).map(key => [key, Object.getOwnPropertyDescriptor(globalThis, key)]))
  for (const [key, value] of Object.entries(globals)) Object.defineProperty(globalThis, key, { configurable: true, value })
  const slots: Array<{ value?: unknown; deps?: unknown[]; cleanup?: () => void }> = []
  let cursor = 0
  let effects: Array<() => void> = []
  const calls = { creates: [] as engine.AvatarOptions[], updates: [] as engine.AvatarOptions[], faces: [] as (string | null)[], gazes: [] as unknown[], pokes: [] as string[], destroys: 0,
    controllers: [] as engine.AvatarController[] }
  const react = {
    useRef(value: unknown) { const index = cursor++; slots[index] ??= { value: { current: value } }; return slots[index].value },
    useId: () => 'avatar-test',
    useEffectEvent(callback: () => void) {
      const index = cursor++
      slots[index] ??= {}
      slots[index].value = callback
      return () => (slots[index].value as () => void)()
    },
    useEffect(effect: () => (() => void) | undefined, deps: unknown[]) {
      const index = cursor++
      const old = slots[index]
      if (!old?.deps || deps.some((value, i) => !Object.is(value, old.deps?.[i]))) {
        effects.push(() => { old?.cleanup?.(); slots[index] = { deps, cleanup: effect() } })
      }
    },
  }
  const exports: Record<string, unknown> = {}
  const vendor = { ...engine, createAvatar(host: Element, options: engine.AvatarOptions) {
    calls.creates.push(options)
    const avatar = engine.createAvatar(host, options)
    calls.controllers.push(avatar)
    return { ...avatar,
      update(options: engine.AvatarOptions) { calls.updates.push(options); avatar.update(options) },
      setExpression(face: string) { calls.faces.push(face); avatar.setExpression(face) },
      holdExpression(face: string | null) { calls.faces.push(face); avatar.holdExpression(face) },
      gaze(point: [number, number] | null) { calls.gazes.push(point); avatar.gaze(point) },
      poke(name: 'spin' | 'nod' | 'bounce') { calls.pokes.push(name); avatar.poke(name) },
      destroy() { calls.destroys++; avatar.destroy() },
    }
  } }
  runInNewContext(source, { exports, window, document, require: (name: string) => {
    if (name === 'react') return react
    if (name.endsWith('/vendor/grok-avatar/src/index.mjs')) return vendor
    return require(name)
  } })
  const props = { shape: 'heart', fill: '#556677', eyes: '#ffffff', state: 'idle', size: 48,
    animated: true, look: { x: 0, y: 0 }, greeting: false, ...initial }
  const render = (changes: Record<string, unknown> = {}) => {
    Object.assign(props, changes)
    cursor = 0; effects = []
    const view = (exports.GrokAvatar as (props: unknown) => { props: { ref: { current: unknown }; [key: string]: unknown } })(props)
    view.props.ref.current = root
    effects.forEach(effect => effect())
    return view
  }
  const unmount = () => { slots.forEach(slot => { slot.cleanup?.(); slot.cleanup = undefined }) }
  t.after(() => {
    unmount()
    for (const [key, descriptor] of Object.entries(original)) {
      if (descriptor) Object.defineProperty(globalThis, key, descriptor)
      else Reflect.deleteProperty(globalThis, key)
    }
  })
  return { render, unmount, root, calls, media, window, document, raf,
    observers: () => observers,
    tick(ms = 40) { time += ms; const pending = [...raf.values()]; raf.clear(); pending.forEach(callback => callback(time)) },
  }
}

test('26 forms and nine explicit faces render locally with painted eyes and unique defs', () => {
  assert.equal(engine.SHAPES.length, 26)
  assert.equal(engine.STATES.length, 53)
  assert.equal(engine.client.ni.length, 9)
  assert.equal(engine.bodyFor('blob'), 'sphere')
  assert.equal(engine.bodyFor('heart'), 'heartChubby')
  const geometry = new Set<string>()
  for (const shape of engine.SHAPES) {
    const svg = engine.renderAvatarSvg({ shape, id: 'shape-check', state: 'idle', size: 48, fill: .8,
      appearance: engine.paintOf('#556677', '#ffffff'), isStatic: true })
    assert.match(svg, /^<svg/)
    assert.match(svg, /#556677/)
    assert.match(svg, /#ffffff/)
    assert.doesNotMatch(svg, /NaN|undefined/)
    geometry.add(svg)
  }
  assert.equal(geometry.size, 26)
  const faces = engine.client.ni.map(expression => engine.renderAvatarSvg({ shape: 'heart', expression,
    id: 'face-check', state: 'idle', size: 96, fill: .8, appearance: engine.paintOf('#556677', '#fff'), isStatic: true }))
  assert.equal(new Set(faces).size, 9)
  const first = engine.renderAvatarSvg({ shape: 'bean', color: '#123456', id: 'unique-one' })
  const second = engine.renderAvatarSvg({ shape: 'bean', color: '#123456', id: 'unique-two' })
  assert.match(first, /unique-one/)
  assert.doesNotMatch(second, /unique-one/)
})

test('static thumbnails have true size and start no RAF, observers, or listeners', t => {
  const h = harness(t, { animated: false, size: 32 })
  const view = h.render()
  assert.deepEqual({ ...view.props.style as object }, { display: 'inline-block', width: 32, height: 32 })
  assert.equal(view.props['data-family'], 'grok')
  assert.equal(view.props['aria-hidden'], 'true')
  assert.match(h.root.innerHTML, /^<svg/)
  const old = h.root.innerHTML
  h.render({ shape: 'pumpkin', size: 96, fill: '#cc6600' })
  assert.notEqual(h.root.innerHTML, old)
  assert.match(h.root.innerHTML, /#cc6600/)
  assert.equal(h.calls.creates.length, 0)
  assert.equal(h.raf.size + h.observers() + h.media.count() + h.document.count() + h.window.count(), 0)
})

test('body, size, paint and normalized gaze update the existing animated instance', t => {
  const h = harness(t)
  h.render()
  assert.equal(h.calls.creates[0].body, 'heartChubby')
  h.tick()
  h.render({ shape: 'star', fill: '#112233', eyes: '#abcdef', size: 64, look: { x: .5, y: -.3 } })
  assert.equal(h.calls.creates.length, 1)
  const update = h.calls.updates.at(-1)!
  assert.equal(update.body, 'star6')
  assert.equal(update.size, 64)
  assert.equal(update.appearance?.inkPaint, '#112233')
  assert.equal(update.appearance?.spotPaint, '#abcdef')
  assert.equal(update.appearance?.cutout, false)
  assert.equal(JSON.stringify(h.calls.gazes.at(-1)), '[0.5,-0.3]')
  h.tick()
  assert(h.root.querySelectorAll().some(node => node.attributes.some(attr => attr.value === '#112233')))
})

test('all business states are valid; activity and completion preserve the selected body', t => {
  const h = harness(t, { shape: 'flower' })
  h.render()
  for (const state of ['processing', 'working', 'writing', 'waiting', 'error', 'unavailable', 'done', 'idle']) {
    const view = h.render({ state })
    const options = h.calls.updates.at(-1)!
    assert(engine.STATES.includes(options.state!))
    assert.equal(options.body, 'starFlower')
    assert.equal(view.props['data-state'], state)
  }
  h.render({ state: 'done' })
  assert.equal(h.calls.updates.at(-1)!.state, 'idle')
  assert.equal(h.calls.faces.at(-1), 'happy')
  const faceCount = h.calls.faces.length
  h.render({ look: { x: .1, y: 0 } })
  assert.equal(h.calls.faces.length, faceCount)
  h.render({ activity: 'radar' })
  assert.equal(h.calls.updates.at(-1)!.state, 'radar')
  h.render({ activity: undefined })
  assert.equal(h.calls.updates.at(-1)!.state, 'idle')
  assert.equal(h.calls.updates.at(-1)!.body, 'starFlower')
  assert(h.calls.updates.every(options => options.state !== 'celebrate' && options.state !== 'finished'))
})

test('greeting is one nod and expression is an independent Studio preview', t => {
  const h = harness(t)
  h.render()
  h.render({ greeting: true })
  h.render({ size: 64 })
  assert.deepEqual(h.calls.pokes, ['nod'])
  h.render({ expression: 'confused' })
  assert.equal(h.calls.faces.at(-1), 'confused')
  assert.equal(h.calls.updates.at(-1)!.state, 'idle')
  h.render({ expression: undefined, greeting: false })
  assert.equal(h.calls.faces.at(-1), null)
})

test('an explicit face survives activity changes and auto releases the face track', t => {
  const h = harness(t)
  h.render()
  assert.deepEqual(h.calls.faces, [null])
  h.render({ expression: 'happy' })
  const count = h.calls.faces.length
  h.render({ activity: 'radar' })
  assert.equal(h.calls.faces.length, count + 1)
  assert.equal(h.calls.faces.at(-1), 'happy')
  for (let index = 0; index < 300; index++) h.tick()
  h.render({ expression: undefined })
  assert.equal(h.calls.faces.at(-1), null)
  h.render({ activity: undefined })
  assert.equal(h.calls.faces.at(-1), null)
  for (let index = 0; index < 30; index++) h.tick()
  assert.equal(h.raf.size, 1)
})

test('reduced motion and hidden tabs tear down motion; resume creates a fresh clock without replay', t => {
  const h = harness(t, { greeting: true })
  h.render()
  assert.equal(h.raf.size, 1)
  assert.equal(h.window.count(), 1)
  h.media.matches = true
  h.media.dispatchEvent(new Event('change'))
  assert.equal(h.calls.destroys, 1)
  assert.equal(h.raf.size + h.observers() + h.window.count(), 0)
  assert.match(h.root.innerHTML, /^<svg/)
  h.media.matches = false
  h.media.dispatchEvent(new Event('change'))
  assert.equal(h.calls.creates.length, 2)
  assert.equal(h.calls.pokes.length, 1)
  h.document.hidden = true
  h.document.dispatchEvent(new Event('visibilitychange'))
  assert.equal(h.raf.size + h.observers() + h.window.count(), 0)
  h.tick(60000)
  h.document.hidden = false
  h.document.dispatchEvent(new Event('visibilitychange'))
  assert.equal(h.calls.creates.length, 3)
  assert.equal(h.calls.pokes.length, 1)
  assert.equal(h.raf.size, 1)
  h.unmount()
  assert.equal(h.raf.size + h.observers() + h.window.count() + h.media.count() + h.document.count(), 0)
})

test('initial reduced motion can switch to live motion; animated false detaches everything', t => {
  const h = harness(t)
  h.media.matches = true
  h.render()
  assert.equal(h.calls.creates.length, 0)
  h.media.matches = false
  h.media.dispatchEvent(new Event('change'))
  assert.equal(h.calls.creates.length, 1)
  h.render({ animated: false })
  assert.equal(h.raf.size + h.observers() + h.window.count() + h.media.count() + h.document.count(), 0)
})

test('native blink and pointer movement reuse the one animation clock', t => {
  const h = harness(t)
  h.render()
  h.calls.controllers[0].blink()
  h.window.dispatchEvent(Object.assign(new Event('pointermove'), { clientX: 140, clientY: 35 }))
  assert.equal(h.raf.size, 1)
  for (let index = 0; index < 10; index++) h.tick()
  assert.equal(h.raf.size, 1)
  h.unmount()
  assert.equal(h.raf.size + h.window.count() + h.observers(), 0)
})

test('native morph previews return to the selected body after activity ends', () => {
  type Driver = { advance: (seconds: number) => void; frame: () => { body: string }; setBody: (body: string) => void }
  type Machine = { advance: (seconds: number) => void; setState: (state: string) => void }
  const driver = (engine.client.eo as (options: { body: string }) => Driver)({ body: engine.bodyFor('flower') })
  const machine = (engine.client.go as (driver: Driver, state: string) => Machine)(driver, 'idle')
  for (const activity of ['orbit', 'radar', 'progress', 'writing', 'sending', 'receiving', 'uploading', 'dictating', 'alerting', 'bouncing', 'turntable', 'showcase']) {
    machine.setState(activity)
    for (let index = 0; index < 40; index++) { machine.advance(.1); driver.advance(.1) }
    machine.setState('idle')
    for (let index = 0; index < 40; index++) { machine.advance(.1); driver.advance(.1) }
    assert.equal(driver.frame().body, 'starFlower', activity)
  }
  driver.setBody(engine.bodyFor('heart'))
  for (let index = 0; index < 40; index++) { machine.advance(.1); driver.advance(.1) }
  assert.equal(driver.frame().body, 'heartChubby')
})
