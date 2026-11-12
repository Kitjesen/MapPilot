import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const require = createRequire(import.meta.url)

interface ModalView {
  props: {
    [key: string]: unknown
    children: ModalView[]
    footer: ModalView
    ref: { current: unknown }
    onClose: () => void
    onCancel: (event: Record<string, unknown>) => void
    onClick: (event?: Record<string, unknown>) => void
    onKeyDown: (event: Record<string, unknown>) => void
    disabled: boolean
  }
}

function modal() {
  const effects: Array<() => void | (() => void)> = []
  const updates: unknown[] = []
  class Element {
    isConnected = true
    focused = 0
    focus() { this.focused += 1 }
  }
  const opener = new Element()
  const exports: Record<string, (props: Record<string, unknown>) => ModalView> = {}
  const source = readFileSync(new URL('../src/components/Modal.tsx', import.meta.url), 'utf8')
  runInNewContext(ts.transpileModule(`${source}\nexport { ModalShell }`, { compilerOptions: {
    target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX,
  } }).outputText, { exports, HTMLElement: Element, document: { body: {}, activeElement: opener },
    require: (name: string) => {
      if (name === 'react') return {
        useRef: (current: unknown) => ({ current }), useId: () => 'modal-title',
        useEffect: (effect: () => void) => effects.push(effect),
        useState: (value: unknown) => [value, (next: unknown) => updates.push(next)],
      }
      if (name === 'react-dom') return { createPortal: (node: unknown) => node }
      if (name.endsWith('.css')) return { default: {} }
      return require(name)
    },
  })
  return { ...exports, effects, updates, opener, Element }
}

test('native modal opens with conservative focus and restores its opener on cleanup', () => {
  const { ModalShell, effects, opener, Element } = modal()
  const cancel = new Element()
  let opened = 0, closed = 0
  const title = require('react').createElement('span', null, '继续导航')
  const dialog = ModalShell({ open: true, title, onClose() {} })
  dialog.props.ref.current = { showModal() { opened += 1 }, close() { closed += 1 }, querySelector: () => cancel }
  const cleanup = effects[0]()
  assert.equal(opened, 1)
  assert.equal(cancel.focused, 1)
  assert.equal(dialog.props['aria-labelledby'], dialog.props.children[0].props.children[1].props.id)
  assert.equal(dialog.props.children[0].props.children[1].props.children, title)
  assert.equal(typeof cleanup, 'function')
  if (cleanup) cleanup()
  assert.equal(closed, 1)
  assert.equal(opener.focused, 1)
})

test('prompt focus uses its input and a closed modal does not open', () => {
  const { ModalShell, effects, Element } = modal()
  const input = new Element()
  const dialog = ModalShell({ open: true, title: '保存', initialFocus: { current: input }, onClose() {} })
  dialog.props.ref.current = { showModal() {}, close() {} }
  effects[0]()
  assert.equal(input.focused, 1)
  assert.equal(ModalShell({ open: false, title: '关闭', onClose() {} }), null)
  assert.equal(effects[1](), undefined)
})

test('Escape and backdrop dismiss while dialog padding and child clicks do not', () => {
  const { ModalShell } = modal()
  let closes = 0, prevented = 0, stopped = 0
  const dialog = ModalShell({ open: true, title: '确认', onClose() { closes += 1 } })
  dialog.props.onCancel({ preventDefault() { prevented += 1 }, stopPropagation() { stopped += 1 } })
  assert.equal(closes, 1)
  assert.equal(prevented, 1)
  assert.equal(stopped, 1)
  const currentTarget = { getBoundingClientRect: () => ({ left: 20, top: 20, right: 200, bottom: 200 }) }
  dialog.props.onClick({ currentTarget, target: currentTarget, clientX: 24, clientY: 24 })
  dialog.props.onClick({ currentTarget, target: {}, clientX: 0, clientY: 0 })
  assert.equal(closes, 1)
  dialog.props.onClick({ currentTarget, target: currentTarget, clientX: 0, clientY: 0 })
  assert.equal(closes, 2)
})

test('busy confirmation blocks dismissal and actions; admission can disable only confirm', () => {
  const { ConfirmModal } = modal()
  let cancels = 0
  const props = { open: true, title: '继续导航', message: '确认后可能开始移动', onConfirm() {},
    onCancel() { cancels += 1 }, cancelLabel: '暂不继续', icon: 'route' }
  const busy = ConfirmModal({ ...props, busy: true })
  busy.props.onClose()
  assert.equal(cancels, 0)
  assert.ok(busy.props.footer.props.children.every(button => button.props.disabled))
  const blocked = ConfirmModal({ ...props, confirmDisabled: true })
  const [cancel, confirm] = blocked.props.footer.props.children
  assert.equal(cancel.props.disabled, false)
  assert.equal(cancel.props.children, '暂不继续')
  assert.equal(confirm.props.disabled, true)
  assert.equal(blocked.props.icon, 'route')
  blocked.props.onClose()
  assert.equal(cancels, 1)
})

test('prompt retains trimming, validation and Enter submission', () => {
  const valid = modal()
  const received: string[] = []
  const prompt = valid.PromptModal({ open: true, title: '保存', initialValue: '  room  ',
    onCancel() {}, onConfirm: (value: string) => received.push(value) })
  const input = prompt.props.children[1]
  let prevented = false
  input.props.onKeyDown({ key: 'Enter', preventDefault() { prevented = true } })
  assert.equal(prevented, true)
  assert.deepEqual(received, ['room'])
  const invalid = modal()
  const rejected = invalid.PromptModal({ open: true, title: '保存', initialValue: 'room',
    validate: () => '名称重复', onCancel() {}, onConfirm: () => assert.fail('invalid input submitted') })
  rejected.props.footer.props.children[1].props.onClick()
  assert.deepEqual(invalid.updates, ['名称重复'])
})
