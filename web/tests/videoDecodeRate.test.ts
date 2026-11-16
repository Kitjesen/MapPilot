import test from 'node:test'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

test('decode rate clears on disconnect or stream replacement', () => {
  let sample: unknown = null
  let effect: () => (() => void) | undefined
  let frame: (now: number) => void
  const exports: Record<string, unknown> = {}
  const source = readFileSync(new URL('../src/hooks/useVideoDecodeRate.ts', import.meta.url), 'utf8')
  runInNewContext(ts.transpileModule(source, { compilerOptions: {
    target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS,
  } }).outputText, { exports, performance: { now: () => 0 }, require: () => ({
    useState: () => [sample, (value: unknown) => { sample = value }],
    useEffect: (value: typeof effect) => { effect = value },
  }) })
  const stream = {}
  const video = { srcObject: null as object | null,
    requestVideoFrameCallback(next: typeof frame) { frame = next; return 1 },
    cancelVideoFrameCallback() {},
  }
  const ref = { current: video }
  const rate = exports.useVideoDecodeRate as (ref: object, stream: object | null, enabled?: boolean) => number | null
  assert.equal(rate(ref, stream), null)
  const cleanup = effect!()
  frame!(1000)
  assert.equal(rate(ref, stream), 1)
  assert.equal(rate(ref, stream, false), null)
  assert.equal(rate(ref, null), null)
  assert.equal(rate(ref, {}), null)
  cleanup?.()
  assert.equal(video.srcObject, null)
})

test('video decode rate is displayed only for WebRTC playback', async () => {
  const feed = await import('node:fs/promises').then(fs => fs.readFile('src/components/CameraFeed.tsx', 'utf8'))
  const scene = await import('node:fs/promises').then(fs => fs.readFile('src/components/SceneView.tsx', 'utf8'))
  assert.match(feed, /useVideoDecodeRate\(videoRef, whepStream, whepConnected\)/)
  assert.match(feed, /videoFps=\{videoFps\}/)
  assert.match(scene, /cameraDecodeFps/)
})
