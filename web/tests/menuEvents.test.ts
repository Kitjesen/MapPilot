import assert from 'node:assert/strict'
import test, { type TestContext } from 'node:test'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

type Handler = (event: Record<string, unknown>) => void
const source = ts.transpileModule(readFileSync(new URL('../src/components/menuEvents.ts', import.meta.url), 'utf8'), {
  compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022 },
}).outputText

function setup(t: TestContext, enabled = true) {
  t.mock.timers.enable({ apis: ['setTimeout'] })
  const rootHandlers = new Map<string, Handler>()
  const documentHandlers = new Map<string, Handler>()
  const summaryChild = {}
  const control = {}
  let focused = 0
  const summary = { contains: (target: unknown) => target === summary || target === summaryChild,
    focus() { focused++ } }
  const menu = { open: false, hasAttribute: () => true,
    contains: (target: unknown) => target === menu || summary.contains(target) || target === control,
    querySelector: () => summary }
  const menus = [menu]
  const root = { current: {
    querySelectorAll: () => menus,
    addEventListener: (name: string, handler: Handler) => rootHandlers.set(name, handler),
    removeEventListener: (name: string) => rootHandlers.delete(name),
  } }
  let cleanup: (() => void) | undefined
  const exports: { useMenuDismiss?: (root: object, enabled: boolean) => void } = {}
  runInNewContext(source, { exports,
    require: () => ({ useEffect: (effect: () => (() => void) | undefined) => { cleanup = effect() } }),
    setTimeout, clearTimeout,
    document: {
      addEventListener: (name: string, handler: Handler) => documentHandlers.set(name, handler),
      removeEventListener: (name: string) => documentHandlers.delete(name),
    },
  })
  exports.useMenuDismiss!(root, enabled)
  t.after(() => cleanup?.())
  return { menu, menus, control, summary, rootHandlers, documentHandlers, cleanup: () => cleanup?.(),
    focused: () => focused,
    pointer(type: string, relatedTarget: unknown = null, target: unknown = summary, pointerType = 'mouse') {
      rootHandlers.get(type)!({ type, target, relatedTarget, pointerType })
    },
    click(detail = 1) {
      let prevented = false
      rootHandlers.get('click')!({ target: summaryChild, detail, preventDefault() { prevented = true } })
      if (!prevented) menu.open = !menu.open
      return prevented
    },
  }
}

test('hover waits for intent and cancels a quick pass', t => {
  const ui = setup(t)
  ui.pointer('pointerover')
  t.mock.timers.tick(219)
  assert.equal(ui.menu.open, false)
  ui.pointer('pointerout')
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, false)
  ui.pointer('pointerover')
  t.mock.timers.tick(220)
  assert.equal(ui.menu.open, true)
})

test('moving into menu controls or across the panel gap keeps it open', t => {
  const ui = setup(t)
  ui.pointer('pointerover')
  t.mock.timers.tick(220)
  ui.pointer('pointerout', ui.control)
  ui.pointer('pointerover', ui.summary, ui.control)
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, true)
  ui.pointer('pointerout')
  t.mock.timers.tick(100)
  ui.pointer('pointerover', null, ui.control)
  t.mock.timers.tick(220)
  assert.equal(ui.menu.open, true)
  ui.pointer('pointerout', null, ui.control)
  t.mock.timers.tick(179)
  assert.equal(ui.menu.open, true)
  t.mock.timers.tick(1)
  assert.equal(ui.menu.open, false)
})

test('mouse click after hover stays open and the next click can close', t => {
  const ui = setup(t)
  ui.pointer('pointerover')
  t.mock.timers.tick(220)
  assert.equal(ui.click(), true)
  assert.equal(ui.menu.open, true)
  assert.equal(ui.click(), false)
  assert.equal(ui.menu.open, false)
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, false)
})

test('click before the delay uses native toggle and cancels delayed reopening', t => {
  const ui = setup(t)
  ui.pointer('pointerover')
  t.mock.timers.tick(100)
  assert.equal(ui.click(), false)
  assert.equal(ui.menu.open, true)
  ui.click()
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, false)
})

test('touch, pen and keyboard keep native activation', t => {
  const ui = setup(t)
  for (const pointerType of ['touch', 'pen', '']) {
    ui.pointer('pointerover', null, ui.summary, pointerType)
    t.mock.timers.tick(300)
    assert.equal(ui.menu.open, false)
    ui.documentHandlers.get('pointerdown')!({ target: ui.summary, pointerType })
    assert.equal(ui.click(), false)
    assert.equal(ui.menu.open, true)
    ui.click()
  }
  ui.pointer('pointerover')
  t.mock.timers.tick(220)
  assert.equal(ui.click(0), false)
  assert.equal(ui.menu.open, false)
  assert.equal(ui.click(0), false)
  assert.equal(ui.menu.open, true)
})

test('outside and Escape dismiss menus and cancel pending hover', t => {
  const ui = setup(t)
  ui.menu.open = true
  ui.documentHandlers.get('pointerdown')!({ target: ui.control, pointerType: 'mouse' })
  assert.equal(ui.menu.open, true)
  ui.documentHandlers.get('keydown')!({ key: 'Escape', defaultPrevented: true })
  assert.equal(ui.menu.open, true)
  ui.documentHandlers.get('keydown')!({ key: 'Escape', defaultPrevented: false })
  assert.equal(ui.menu.open, false)
  assert.equal(ui.focused(), 1)
  ui.pointer('pointerover')
  ui.documentHandlers.get('pointerdown')!({ target: {}, pointerType: 'mouse' })
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, false)
  ui.pointer('pointerover')
  ui.documentHandlers.get('keydown')!({ key: 'Escape', defaultPrevented: false })
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, false)
})

test('cleanup cancels timers and removes all listeners', t => {
  const ui = setup(t)
  ui.pointer('pointerover')
  ui.cleanup()
  t.mock.timers.tick(300)
  assert.equal(ui.menu.open, false)
  assert.equal(ui.rootHandlers.size, 0)
  assert.equal(ui.documentHandlers.size, 0)
})

test('pinning a menu cancels the effect of an already scheduled hover close', t => {
  const ui = setup(t)
  ui.pointer('pointerover')
  t.mock.timers.tick(220)
  ui.pointer('pointerout')
  ui.menu.hasAttribute = () => false
  t.mock.timers.tick(180)
  assert.equal(ui.menu.open, true)
})

test('disabled menus install no listeners', t => {
  const ui = setup(t, false)
  assert.equal(ui.rootHandlers.size, 0)
  assert.equal(ui.documentHandlers.size, 0)
})

test('Escape restores the outer summary when nested menus close', t => {
  const ui = setup(t)
  let nestedFocused = 0
  const nestedSummary = { contains: () => false, focus() { nestedFocused++ } }
  const nested = { ...ui.menu, open: true, querySelector: () => nestedSummary }
  ui.menus.push(nested)
  ui.menu.open = true
  ui.documentHandlers.get('keydown')!({ key: 'Escape', defaultPrevented: false })
  assert.equal(ui.menu.open, false)
  assert.equal(nested.open, false)
  assert.equal(ui.focused(), 1)
  assert.equal(nestedFocused, 0)
})
