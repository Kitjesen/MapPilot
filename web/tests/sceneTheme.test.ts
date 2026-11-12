import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import * as THREE from 'three'

const source = ts.createSourceFile('Scene3D.tsx', readFileSync(
  new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8',
), ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)

test('scene background follows light and dark themes without rebuilding the scene', () => {
  let callback = ''
  function visit(node: ts.Node) {
    if (ts.isVariableDeclaration(node) && node.name.getText(source) === 'applyTheme') {
      callback = node.initializer!.getText(source)
    }
    ts.forEachChild(node, visit)
  }
  visit(source)
  assert.ok(callback)
  const css = readFileSync(new URL('../src/App.css', import.meta.url), 'utf8')
  const scene = new THREE.Scene()
  const grid = { material: { opacity: 0 } }
  const document = { documentElement: { dataset: { theme: 'light' } } }
  let background = ''
  const apply = runInNewContext(ts.transpileModule(`(${callback})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022 },
  }).outputText, {
    THREE, scene, grid, document, mount: {},
    getComputedStyle: () => ({ getPropertyValue: () => background }),
  }) as () => void
  for (const theme of ['light', 'dark', 'light']) {
    const selector = theme === 'light' ? /:root\s*\{([^}]+)\}/ : /html\[data-theme="dark"\]\s*\{([^}]+)\}/
    background = css.match(selector)![1].match(/--bg-layer-1:\s*([^;]+);/)![1].trim()
    document.documentElement.dataset.theme = theme
    apply()
    assert.equal((scene.background as THREE.Color).getHexString(), background.slice(1))
    assert.equal(grid.material.opacity, theme === 'dark' ? .18 : .55)
  }
})
