import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/agentConversation/FullscreenViewer.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText

function render(props: Record<string, unknown>) {
  const exports: Record<string, unknown> = {}
  runInNewContext(source, { exports, document: {}, require: (name: string) => {
    if (name === 'react') return require('react')
    if (name === 'lucide-react') return { Maximize2: () => null, X: () => null }
    if (name.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
    return require(name)
  } })
  const react = require('react')
  return require('react-dom/server').renderToStaticMarkup(react.createElement(exports.FullscreenViewer, props)) as string
}

test('renders every viewer kind with accessible title and content', () => {
  const image = render({ open: true, title: '现场照片', kind: 'image', src: '/photo.jpg', onClose() {} })
  assert.match(image, /role="dialog"/)
  assert.match(image, /aria-label="现场照片"/)
  assert.match(image, /src="\/photo.jpg"/)

  const video = render({ open: true, title: '巡检录像', kind: 'video', src: '/clip.mp4', onClose() {} })
  assert.match(video, /<video[^>]*src="\/clip.mp4"/)
  assert.match(video, /controls=""/)

  const table = render({ open: true, title: '测量结果', kind: 'table', table: { columns: ['点位', '数值'], rows: [['A-01', 12.4], ['A-02', null]] }, onClose() {} })
  assert.match(table, /<th[^>]*>点位<\/th>/)
  assert.match(table, /12\.4/)
  assert.match(table, /—/)

  const map = render({ open: true, title: '地图预览', kind: 'map', children: 'MAP_SCENE', onClose() {} })
  assert.match(map, /MAP_SCENE/)
})

test('closed viewer renders no overlay', () => {
  assert.equal(render({ open: false, title: '隐藏', kind: 'image', src: '/x', onClose() {} }), '')
})
