import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import { createElement } from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import ts from 'typescript'
import type { Octomap } from '../src/services/octomap.ts'
import * as surfaceMap from '../src/services/surfaceMap.ts'
import * as heightColors from '../src/services/cloudHeightColor.ts'

const require = createRequire(import.meta.url)
const noGoRegions = [{ id: 'door', min: [1, 2, 0], max: [2, 3, 2] }]
const code = ts.transpileModule(readFileSync(new URL('../src/components/SavedMapVoxels.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText

function render(state: { status: 'loading' | 'error'; message?: string } | { status: 'ready'; map: Octomap }) {
  const exports: Record<string, unknown> = {}
  const queries: unknown[][] = []
  let scene: Record<string, unknown> | undefined
  runInNewContext(code, { exports, Float32Array, require: (name: string) => {
    if (name.endsWith('/useOctomap.ts')) return { useOctomap: (...args: unknown[]) => { queries.push(args); return state } }
    if (name.endsWith('/surfaceMap.ts')) return surfaceMap
    if (name.endsWith('/useSurfaceMap.ts')) return { useSurfaceMap: () => ({}) }
    if (name.endsWith('/useMapSettings.ts')) return { useMapSettings: () => null }
    if (name.endsWith('/MapProgress.tsx')) return { MapProgress: () => null }
    if (name.endsWith('/useNoGo.ts')) return { useNoGo: () => ({ map: { regions: noGoRegions } }) }
    if (name.endsWith('/Scene3D.tsx')) return { Scene3D: (props: Record<string, unknown>) => { scene = props; return null } }
    if (name.endsWith('/cloudHeightColor.ts')) return heightColors
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  const component = exports.SavedMapVoxels as (props: { mapName: string; contentEpoch: number }) => ReturnType<typeof createElement>
  const html = renderToStaticMarkup(createElement(component, { mapName: 'test6_repaired', contentEpoch: 42 }))
  return { html, queries, scene }
}

test('saved OctoMap viewing uses the selected content version without inventing a live robot or floor', () => {
  const map: Octomap = { resolution: .1, cells: new Float32Array([1, 2, 3, .1, 30, 2, 3, .1]),
    bounds: { min: [0, 0, 0], max: [31, 3, 4] } }
  const result = render({ status: 'ready', map })
  assert.deepEqual(result.queries, [['test6_repaired', 42, undefined, true]])
  assert.equal(result.scene?.navigationMap, map)
  assert.equal(result.scene?.navigationMapRadius, Infinity)
  assert.equal(result.scene?.noGoRegions, noGoRegions)
  assert.equal(result.scene?.robotValid, false)
  assert.equal((result.scene?.layers as { grid: boolean }).grid, false)
  assert.equal((result.scene?.cloud as { count: number }).count, 0)
  assert.match(result.html, /10 cm.*2 个占据格/)
  assert.match(result.html, /空白可能是自由或未知空间/)
  assert.match(result.html, /aria-label="高度色标"/)
  assert.match(result.html, /3\.00 m/)
})

test('isolated extreme voxels do not compress the saved-map color scale', () => {
  const heights = [-100, ...Array.from({ length: 100 }, (_, i) => i / 25), 100]
  const map: Octomap = { resolution: .1, cells: new Float32Array(heights.flatMap(z => [0, 0, z, .1])),
    bounds: { min: [0, 0, -100], max: [1, 1, 100] } }
  const result = render({ status: 'ready', map })
  const range = result.scene?.cloudColorRange as { min: number; max: number }
  assert.ok(range.min >= 0 && range.max <= 4)
  assert.equal(result.scene?.navigationMap, map)
  assert.equal(map.cells.length / 4, heights.length)
  assert.match(result.html, /linear-gradient/)
})

test('loading and artifact failures are visible without displaying old map geometry', () => {
  const loading = render({ status: 'loading' })
  assert.match(loading.html, /role="status".*正在读取导航体素/)
  assert.equal(loading.scene, undefined)
  const failed = render({ status: 'error', message: '地图内容已更新，请刷新地图库后重试' })
  assert.match(failed.html, /role="alert".*地图内容已更新/)
  assert.equal(failed.scene, undefined)
})
