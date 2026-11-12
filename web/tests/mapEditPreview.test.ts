import assert from 'node:assert/strict'
import test from 'node:test'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import { previewEditedMap, saveEditedMap } from '../src/services/api.ts'
import { isMapCrop, applyMapEditOperations, readMapEditRecipe } from '../src/services/mapEdit.ts'
import type { MapEditOperation } from '../src/types/mapEdit.ts'

const source = { name: 'office', contentEpoch: 4, frameId: 'floor' }
const operations: MapEditOperation[] = [{ operation: 'voxel', voxel_size: .1 }]
const preview = { success: true, source_content_epoch: 4, frame_id: 'floor',
  point_count: 123456, removed_points: 2345, preview_sample: true,
  preview_points: [1, 2, 3], preview_removed: [4, 5, 6] }

test('chat preview decodes the exact source and ordered wire operations', () => {
  const bounds = { min_x: 0, min_y: 1, min_z: 2, max_x: 3, max_y: 4, max_z: 5 }
  const recipe = { frame_id: 'floor', preview_request: { map_id: 'office', expected_source_content_epoch: 4,
    operations: [{ operation: 'delete_inside', bounds }, ...operations] } }
  assert.deepEqual(readMapEditRecipe(recipe), { source, operations: [
    { operation: 'delete_inside', bounds: { min: { x: 0, y: 1, z: 2 }, max: { x: 3, y: 4, z: 5 } } }, ...operations,
  ] })
  for (const invalid of [{ preview_request: null }, {}, { ...recipe, frame_id: '' },
    { ...recipe, preview_request: { ...recipe.preview_request, expected_source_content_epoch: 0 } },
    { ...recipe, preview_request: { ...recipe.preview_request, operations: [null] } },
    { ...recipe, preview_request: { ...recipe.preview_request, operations: [{ operation: 'delete_inside', bounds: {} }] } },
  ]) assert.throws(() => readMapEditRecipe(invalid as never), /预览/)
})

test('preview and save submit the same ordered cleaning history and exact counts stay separate from samples', async () => {
  const previous = globalThis.fetch
  const calls: Array<{ url: string; init?: RequestInit }> = []
  const controller = new AbortController()
  try {
    globalThis.fetch = async (input, init) => { calls.push({ url: String(input), init }); return new Response(JSON.stringify(preview)) }
    const result = await previewEditedMap(source, operations, controller.signal)
    await saveEditedMap(source, 'office-edited', operations)
    assert.equal(result.point_count, 123456)
    assert.equal(result.preview_points.length, 3)
    assert.equal(calls[0].url, '/api/v1/maps/office/edit_preview')
    assert.equal(calls[0].init?.signal, controller.signal)
    assert.deepEqual(JSON.parse(String(calls[0].init?.body)), { expected_source_content_epoch: 4, operations })
    assert.deepEqual(JSON.parse(String(calls[1].init?.body)), { name: 'office-edited', expected_source_content_epoch: 4, operations })
  } finally { globalThis.fetch = previous }
})

test('an empty full result remains an inspectable preview', async () => {
  const previous = globalThis.fetch
  try {
    globalThis.fetch = async () => new Response(JSON.stringify({ ...preview, point_count: 0,
      removed_points: 125801, preview_points: [], preview_removed: [4, 5, 6] }))
    const result = await previewEditedMap(source, operations)
    assert.equal(result.point_count, 0)
    assert.deepEqual(result.preview_points, [])
    assert.equal(result.removed_points, 125801)
  } finally { globalThis.fetch = previous }
})

test('preview rejects a changed source and surfaces native processing failure', async () => {
  const previous = globalThis.fetch
  try {
    for (const data of [
      { ...preview, source_content_epoch: 5 }, { ...preview, frame_id: 'other' },
      { ...preview, preview_points: [1, 2] }, { ...preview, point_count: -1 },
    ]) {
      globalThis.fetch = async () => new Response(JSON.stringify(data))
      await assert.rejects(previewEditedMap(source, operations), /当前地图不匹配/)
    }
    globalThis.fetch = async () => new Response(JSON.stringify({ success: false, message: 'no stable plane' }))
    await assert.rejects(previewEditedMap(source, operations), /no stable plane/)
  } finally { globalThis.fetch = previous }
})

const editorSource = readFileSync(new URL('../src/components/MapEditor.tsx', import.meta.url), 'utf8')
const editorFile = ts.createSourceFile('MapEditor.tsx', editorSource, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)

test('every default cleaning parameter satisfies its HTML input minimum and step', () => {
  const values = new Map<string, unknown>()
  for (const statement of editorFile.statements) {
    if (!ts.isVariableStatement(statement)) continue
    for (const declaration of statement.declarationList.declarations) {
      const name = declaration.name.getText(editorFile)
      if (!declaration.initializer || !['filters', 'filterFields'].includes(name)) continue
      const code = ts.transpileModule(`(${declaration.initializer.getText(editorFile)})`, {
        compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
      }).outputText
      values.set(name, runInNewContext(code))
    }
  }
  const filters = values.get('filters') as Array<Record<string, string | number>>
  const fields = values.get('filterFields') as Record<string, { min: number; step: number }>
  assert.ok(filters.length)
  for (const filter of filters) for (const [key, value] of Object.entries(filter)) {
    if (key === 'operation') continue
    const field = fields[key]
    assert.ok(typeof value === 'number' && value >= field.min, `${filter.operation}.${key} minimum`)
    const steps = (Number(value) - field.min) / field.step
    assert.ok(Math.abs(steps - Math.round(steps)) < 1e-9, `${filter.operation}.${key} step mismatch`)
  }
})

function previewEffect(context: Record<string, unknown>): () => void {
  let expression: ts.Expression | undefined
  const visit = (node: ts.Node) => {
    if (ts.isCallExpression(node) && node.expression.getText(editorFile) === 'useEffect'
      && node.arguments[0]?.getText(editorFile).includes('api.previewEditedMap')) expression = node.arguments[0]
    ts.forEachChild(node, visit)
  }
  visit(editorFile)
  assert.ok(expression)
  return runInNewContext(ts.transpileModule(`(${expression.getText(editorFile)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, context)
}

test('undo or a newer operation cancels the old preview delivery', async () => {
  let deliver: (value: unknown) => void = () => {}
  let signal: AbortSignal | undefined
  const results: unknown[] = []
  const context = { nativePreview: true, recovery: null, pendingSave: null, saved: null,
    sourceName: 'office', contentEpoch: 4, frameId: 'floor', operations, AbortController,
    api: { previewEditedMap: (_source: unknown, _ops: unknown, value: AbortSignal) => {
      signal = value
      return new Promise(resolve => { deliver = resolve })
    } }, setPreviewState: (value: unknown) => results.push(value) }
  const cleanup = previewEffect(context)()
  cleanup()
  assert.equal(signal?.aborted, true)
  deliver(preview)
  await Promise.resolve()
  assert.deepEqual(results, [])
  context.nativePreview = false
  assert.equal(previewEffect(context)(), undefined)
})

test('draft uses native kept points while original view and crop-only undo use source data', () => {
  const sceneText = readFileSync(new URL('../src/components/Scene3D.tsx', import.meta.url), 'utf8')
  const file = ts.createSourceFile('Scene3D.tsx', sceneText, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
  let expression: ts.Expression | undefined
  const visit = (node: ts.Node) => {
    if (ts.isVariableDeclaration(node) && node.name.getText(file) === 'displayMapFlat'
      && node.initializer && ts.isCallExpression(node.initializer)) expression = node.initializer.arguments[0]
    ts.forEachChild(node, visit)
  }
  visit(file)
  assert.ok(expression)
  const context = { savedMapFlat: [7, 8, 9], draftOperations: operations as readonly MapEditOperation[] | undefined,
    editPreview: preview, isMapCrop, applyMapEditOperations }
  const compute = runInNewContext(ts.transpileModule(`(${expression.getText(file)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, context)
  assert.equal(compute(), preview.preview_points)
  context.draftOperations = undefined
  assert.equal(compute(), context.savedMapFlat)
  context.draftOperations = []
  assert.equal(compute(), context.savedMapFlat)
})
