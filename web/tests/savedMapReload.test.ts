import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const source = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
const ast = ts.createSourceFile('SceneView.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
let effect: ts.CallExpression | undefined
function find(node: ts.Node) {
  if (ts.isCallExpression(node) && node.expression.getText(ast) === 'useEffect'
      && node.arguments[0]?.getText(ast).includes('api.fetchSavedMapPointCloud(activeMapName)')) effect = node
  ts.forEachChild(node, find)
}
find(ast)
assert.ok(effect)
const code = ts.transpileModule(`(${effect.arguments[0].getText(ast)})()`, {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
}).outputText
const flush = async () => { await Promise.resolve(); await Promise.resolve(); await Promise.resolve() }

function harness(fetch: () => Promise<unknown>) {
  const timers = new Map<number, () => void>()
  const loaded: unknown[] = []
  let sequence = 0
  const cleanup = runInNewContext(code, {
    showSavedMapInScene: true, activeMapName: '903room_v4', savedMapCloud: undefined,
    savedMapFlat: undefined, cloud: { frameId: 'map', epoch: 2 }, sseState: { connected: true },
    setSavedMapLoadError: () => {}, api: { fetchSavedMapPointCloud: fetch }, setSavedMapCloud: (value: unknown) => loaded.push(value),
    setTimeout: (callback: () => void) => { timers.set(++sequence, callback); return sequence },
    clearTimeout: (id: number) => timers.delete(id),
  }) as () => void
  return { timers, loaded, cleanup }
}

test('a transient saved-map failure retries and displays the successful response', async () => {
  let attempts = 0
  const map = { mapName: '903room_v4', points: [1, 2, 3], epoch: 2 }
  const h = harness(async () => { if (++attempts === 1) throw new Error('scene not bound'); return map })
  await flush()
  assert.equal(h.timers.size, 1)
  const retry = [...h.timers.values()][0]; h.timers.clear(); retry()
  await flush()
  assert.equal(attempts, 2)
  assert.equal(h.loaded[0], map)
  h.cleanup()
})

test('leaving a scene cancels retry and ignores a delayed old response', async () => {
  const failed = harness(async () => { throw new Error('offline') })
  await flush(); failed.cleanup()
  assert.equal(failed.timers.size, 0)
  let resolve!: (value: unknown) => void
  const delayed = harness(() => new Promise(done => { resolve = done }))
  delayed.cleanup(); resolve({ mapName: 'old' }); await flush()
  assert.equal(delayed.loaded.length, 0)
})

test('connection and product session changes restart saved-map loading', () => {
  const dependencies = effect!.arguments[1].getText(ast)
  assert.match(dependencies, /sseState\.connected/)
  assert.match(dependencies, /session\?\.product_session_id/)
})
