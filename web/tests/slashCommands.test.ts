import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'
import type { SSEState } from '../src/types/index.ts'
import * as navigationPreview from '../src/services/navigationPreview.ts'

function harness() {
  const candidates: Array<Record<string, unknown>> = []
  const goals: Array<{ x: number; y: number; options: Record<string, unknown> }> = []
  const exports: Record<string, unknown> = {}
  const code = ts.transpileModule(readFileSync(new URL('../src/utils/slashCommands.ts', import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  runInNewContext(code, { exports, require: (name: string) => {
    if (name.endsWith('/api')) return {
      constructGoalCandidate: async (request: Record<string, unknown>) => {
        candidates.push(request)
        return { ok: true, preview: { feasible: true } }
      },
      sendGoal: async (x: number, y: number, options: Record<string, unknown>) => {
        goals.push({ x, y, options })
        return { accepted: true }
      },
      formatCommandAck: (_response: unknown, label: string) => label,
      formatCommandError: (error: unknown) => String(error),
    }
    if (name.endsWith('/navigationPreview')) return navigationPreview
    if (name.endsWith('/navigationStatus')) return {}
    throw new Error(`Unexpected module: ${name}`)
  } })
  const execute = exports.executeSlashCommand as (command: string, state: SSEState) => Promise<string>
  return { candidates, goals, execute }
}

test('/go previews and submits the same measured body height on every floor', async () => {
  for (const z of [0, .435, 2.255524]) {
    const h = harness()
    await h.execute('/go 3.5 -1.2', { odometry: { x: 7, y: 0, z } } as SSEState)
    assert.equal(h.candidates.length, 1)
    assert.equal(h.candidates[0].z, z)
    assert.equal(h.goals.length, 1)
    assert.equal(h.goals[0].options.z, z)
    assert.equal(h.goals[0].x, 3.5)
    assert.equal(h.goals[0].y, -1.2)
  }
})

test('/go does not silently submit z=0 when the body height is absent or invalid', async () => {
  for (const odometry of [null, {}, { z: null }, { z: NaN }, { z: Infinity }]) {
    const h = harness()
    const result = await h.execute('/go 3.5 -1.2', { odometry } as SSEState)
    assert.match(result, /等待有效定位高度/)
    assert.equal(h.candidates.length, 0)
    assert.equal(h.goals.length, 0)
  }
})
